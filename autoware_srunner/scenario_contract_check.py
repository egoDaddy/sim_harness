#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scenario_contract_check: 跨仓契约校验 (多仓分离第三刀).

背景: harness(本仓) / 场景库 / 故障库 / autoware(被测物) 各自版本化后,
"同一份代码配同一份数据"的隐式保证消失。错配不会在编译期暴露, 只会在跑批
10 分钟后以 NO_CRITERIA / 注入链静默退化 / ego 附着超时 的形式出现, 被误读成
"代码回归"。本脚本做纯文件检查 (无 ROS / 无 CARLA / 无 GPU, 秒级), 在 CI 的
contract stage 把错配提前拦下。

契约组:
  C1 配置自洽   : 资产根存在; ego 角色名在各消费者参数里可见
  C2 场景清单   : xosc_list 每条 file 可解析; name 唯一 (run_summary 可归因)
  C5 故障库     : matrix 的 presets_file / preset / sensor / scenario 全可解析
  C3 场景结构   : xosc 可解析 / ego ScenarioObject 名 = ego_role_name /
                  ControllerCatalog 存在且 sensor_kit_entry 条目在其中 /
                  ego_trajectory_ref 的同名 json 存在 / 有 criteria_ 判据
  C4 传感器三侧 : yaml use_*_sensor_mapping 标签 == ControllerCatalog 条目后缀
                  == autoware 侧 sensor_mapping_<tag>.yaml; 且 catalog 的
                  sensor_id 覆盖 mapping 的 id (interface 发现式 listen 的键)
  C6 结果目录   : 残留 json 会被 monitor 按 mtime 当成本次判定 (CI 假通过风险)
  C7 版本照片   : 场景库/故障库/autoware 的实际 commit 与 versions.lock.yaml
                  一致; scenario_runner 的本地补丁 sha256 未被覆盖

退出码: 0 = 无 FAIL (WARN 允许); 1 = 存在 FAIL。
用法:
  python3 scenario_contract_check.py                # 用包内/源码树 config
  scenario_contract_check --scenario-root /path/scenario_lib --strict
  scenario_contract_check --lock versions.lock.yaml --strict   # 连版本一起对
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml

CATALOG_FILENAME = 'ControllerCatalog.xosc'   # scenario_runner 硬编码约定名
KIT_ENTRY_PREFIX = 'ego_sensor_kit'           # Controller 条目名 = 前缀_标签
PARAM_KIT = 'sensor_kit_entry'
PARAM_TRAJ = 'ego_trajectory_ref'
CRITERIA_PREFIX = 'criteria_'

LEVEL_ORDER = {'FAIL': 0, 'WARN': 1, 'PASS': 2}


class Report(object):
    """契约检查结果收集器 (按契约组输出, 组内 FAIL 在前)."""

    def __init__(self):
        self.rows = []

    def add(self, level, cid, msg):
        self.rows.append({'id': cid, 'level': level, 'message': msg})

    def pass_(self, cid, msg):
        self.add('PASS', cid, msg)

    def warn(self, cid, msg):
        self.add('WARN', cid, msg)

    def fail(self, cid, msg):
        self.add('FAIL', cid, msg)

    def counts(self):
        out = {'PASS': 0, 'WARN': 0, 'FAIL': 0}
        for r in self.rows:
            out[r['level']] += 1
        return out

    def dump(self):
        for r in sorted(self.rows, key=lambda x: (x['id'], LEVEL_ORDER[x['level']])):
            print('[%-4s] %s %s' % (r['level'], r['id'], r['message']))
        c = self.counts()
        print('-' * 72)
        print('契约校验: PASS %d / WARN %d / FAIL %d' % (c['PASS'], c['WARN'], c['FAIL']))


# --------------------------------------------------------------------- 工具函数
def resolve_file(value, roots):
    """与 monitor._resolve_input_path 同规则: 绝对/相对 cwd 命中直接用,
    否则在 roots 下按同名查找; 找不到返回 None (不抛异常, 便于批量报告)."""
    value = os.path.expanduser(str(value or '')).strip()
    if not value:
        return None
    if os.path.isfile(value):
        return value
    for d in roots:
        if not d:
            continue
        cand = os.path.join(d, value)
        if os.path.isfile(cand):
            return cand
    return None


def rel_tag(path, root):
    """报告里用相对资产根的路径, 避免 CI 日志出现 checkout 前缀噪音."""
    try:
        return os.path.relpath(path, root)
    except ValueError:
        return path


def param_value(root, name):
    """取 ParameterDeclaration 的值 (场景级默认参数)."""
    for pd in root.iter('ParameterDeclaration'):
        if pd.get('name') == name:
            return pd.get('value', '')
    return ''


def read_catalog_entries(catalog_path):
    """ControllerCatalog.xosc -> [(条目名, [sensor_id, ...]), ...]."""
    tree = ET.parse(catalog_path)
    out = []
    for ctrl in tree.getroot().iter('Controller'):
        ids = []
        for prop in ctrl.iter('Property'):
            if (prop.get('name') or '').startswith('sensors.'):
                ids.append((prop.get('value') or '').split('|')[0])
        out.append((ctrl.get('name'), ids))
    return out


def catalog_for_xosc(root, xosc_path):
    """按 xosc 的 CatalogLocations/ControllerCatalog/Directory 解析 catalog 路径
    (相对场景文件所在目录); 未声明则回退场景库根目录."""
    for cc in root.iter('ControllerCatalog'):
        d = cc.find('Directory')
        if d is None:
            continue
        base = d.get('path') or '.'
        cand = os.path.normpath(os.path.join(
            os.path.dirname(os.path.abspath(xosc_path)), base, CATALOG_FILENAME))
        return cand
    return None


# ------------------------------------------------------------------ C1 配置自洽
def check_c1(ctx, rep):
    cfg = ctx['cfg']
    ego = cfg.get('ego_role_name', 'ego_vehicle')
    rep.pass_('C1', 'ego_role_name=%r (yaml 单一源头)' % ego)

    # 消费侧 1/2: 两个"用角色名在白名单里找 ego"的节点
    for sec in ('npc_ready_trigger', 'npc_markers'):
        names = list((cfg.get(sec) or {}).get('ego_role_names') or [])
        if not names:
            rep.warn('C1', '%s.ego_role_names 缺失 -> 用节点内置默认值, 可能与'
                           ' ego_role_name 不一致' % sec)
        elif ego not in names:
            rep.fail('C1', '%s.ego_role_names=%s 不含 ego_role_name=%r -> 该节点'
                           '永远匹配不到 ego (就绪信号/标记永不产生)'
                     % (sec, names, ego))
        else:
            rep.pass_('C1', '%s.ego_role_names 覆盖 %r' % (sec, ego))

    if not os.path.isdir(ctx['scenario_root']):
        rep.fail('C1', 'scenario_root 不是目录: %s' % ctx['scenario_root'])
    else:
        rep.pass_('C1', 'scenario_root=%s' % ctx['scenario_root'])

    fi = cfg.get('sensor_fault_injector') or {}
    supported = list(fi.get('supported_sensors') or [])
    if not supported:
        rep.fail('C1', 'sensor_fault_injector.supported_sensors 缺失 -> 故障矩阵'
                       ' sensor 校验退化为内置默认, 拼错也不报错')
    else:
        rep.pass_('C1', 'supported_sensors=%s' % supported)


# ------------------------------------------------------------------ C2 场景清单
def check_c2(ctx, rep):
    xl = ctx['xosc_list']
    mx = ctx['fault_matrix']
    if not xl and not mx:
        rep.fail('C2', 'xosc_list 与 fault_matrix 均未配置 -> auto 模式启动即抛'
                       ' RuntimeError (需求 15 互斥/必填)')
        return

    if not xl:
        rep.pass_('C2', '未配置 xosc_list (矩阵驱动), 场景清单由 C5 提供')
        return

    path = resolve_file(xl, [ctx['scenario_root']])
    if not path:
        rep.fail('C2', 'xosc_list 不可解析: %r (scenario_root=%s)'
                 % (xl, ctx['scenario_root']))
        return
    try:
        with open(path) as f:
            data = json.load(f)
    except ValueError as e:
        rep.fail('C2', 'xosc_list JSON 解析失败 (%s): %s' % (path, e))
        return

    entries = data.get('scenarios') or []
    if not entries:
        rep.fail('C2', 'xosc_list 场景列表为空: %s' % path)
        return

    seen = {}
    for i, s in enumerate(entries):
        name = s.get('name') or '#%d' % i
        raw = s.get('file') or ''
        if os.path.isabs(raw):
            rep.warn('C2', 'xosc_list[%d] %s: file 是绝对路径 (%s) -> 场景库换'
                           'checkout 路径即断' % (i, name, raw))
        full = resolve_file(raw, [ctx['scenario_root']])
        if not full:
            rep.fail('C2', 'xosc_list[%d] %s: file 不可解析: %r'
                     % (i, name, raw))
            continue
        if name in seen:
            rep.fail('C2', '场景名重复 %r (xosc_list) -> run_summary 无法归因'
                     % name)
        seen[name] = full
        ctx['scenario_files'].append((name, full))
        ctx['names'][name] = full

    rep.pass_('C2', 'xosc_list 解析出 %d 个场景 (%s)'
              % (len(ctx['scenario_files']), rel_tag(path, ctx['scenario_root'])))


# ------------------------------------------------------------------ C5 故障库
def check_c5(ctx, rep):
    """矩阵驱动模式: 条目 scenario 也进 C3 检查; preset/sensor 在这里校验."""
    mx = ctx['fault_matrix']
    if not mx:
        rep.pass_('C5', '未配置故障矩阵, 跳过')
        return

    path = resolve_file(mx, [ctx['fault_root'],
                             os.path.join(ctx['fault_root'], 'matrix')])
    if not path:
        rep.fail('C5', '故障矩阵不可解析: %r (fault_root=%s)'
                 % (mx, ctx['fault_root']))
        return
    try:
        with open(path) as f:
            matrix = json.load(f)
    except ValueError as e:
        rep.fail('C5', '故障矩阵 JSON 解析失败 (%s): %s' % (path, e))
        return

    presets_file = matrix.get('presets_file') or ''
    pfull = resolve_file(presets_file, [ctx['fault_root']])
    if not pfull:
        rep.fail('C5', 'presets_file 不可解析: %r (fault_root=%s) -> monitor '
                       '加载矩阵即抛错' % (presets_file, ctx['fault_root']))
        return
    with open(pfull) as f:
        presets = (yaml.safe_load(f) or {}).get('presets') or {}
    if not presets:
        rep.fail('C5', 'preset 库为空: %s' % pfull)
        return
    rep.pass_('C5', 'preset 库 %d 条 (%s)'
              % (len(presets), rel_tag(pfull, ctx['fault_root'])))

    supported = list((ctx['cfg'].get('sensor_fault_injector') or {})
                     .get('supported_sensors') or [])
    entries = matrix.get('matrix') or []
    if not entries:
        rep.fail('C5', '故障矩阵条目为空: %s' % path)
        return

    bad = 0
    for idx, entry in enumerate(entries):
        eid = entry.get('id') or '#%d' % idx
        groups = entry.get('fault')
        if not isinstance(groups, list) or not groups:
            rep.fail('C5', '条目 %s: fault 必须为非空数组' % eid)
            bad += 1
            continue
        for g in groups:
            sensor = (g.get('sensor') or '').strip()
            preset = g.get('preset')
            if sensor not in supported:
                rep.fail('C5', '条目 %s: sensor %r 不在 supported_sensors=%s'
                         % (eid, sensor, supported))
                bad += 1
            if preset not in presets:
                rep.fail('C5', '条目 %s: preset %r 不在 preset 库' % (eid, preset))
                bad += 1
        scn = entry.get('scenario')
        if not scn:
            rep.fail('C5', '条目 %s: scenario 字段缺失 -> 无法确定执行哪个场景'
                     % eid)
            bad += 1
            continue
        if str(scn).endswith('.xosc'):
            full = resolve_file(scn, [ctx['scenario_root']])
            if not full:
                rep.fail('C5', '条目 %s: scenario 不可解析: %r' % (eid, scn))
                bad += 1
                continue
        else:
            full = ctx['names'].get(scn)
            if not full:
                rep.fail('C5', '条目 %s: scenario=%r 不在 xosc_list 中 (按 name '
                               '引用时 xosc_list 必须可用)' % (eid, scn))
                bad += 1
                continue
        if full not in [p for _, p in ctx['scenario_files']]:
            ctx['scenario_files'].append((eid, full))

    if bad == 0:
        rep.pass_('C5', '矩阵 %d 条: preset/sensor/scenario 全部可解析'
                  % len(entries))


# ------------------------------------------------------------------ C3 场景结构
def check_c3(ctx, rep):
    cfg = ctx['cfg']
    ego = cfg.get('ego_role_name', 'ego_vehicle')
    root_dir = ctx['scenario_root']
    cache = {}

    if not ctx['scenario_files']:
        rep.warn('C3', '没有可检查的场景文件 (清单为空/全部解析失败)')
        return

    for name, path in ctx['scenario_files']:
        tag = rel_tag(path, root_dir)
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as e:
            rep.fail('C3', '%s: xosc XML 解析失败 (%s)' % (tag, e))
            continue

        # (1) ego 角色名: 场景资产侧
        objs = [o.get('name') for o in root.iter('ScenarioObject') if o.get('name')]
        if not objs:
            rep.fail('C3', '%s: 没有 ScenarioObject -> scenario_runner 不会生成'
                           ' ego/NPC' % tag)
        elif ego not in objs:
            rep.fail('C3', '%s: ScenarioObject 里没有 ego_role_name=%r (实际 %s)'
                           ' -> interface 按 yaml 角色名找不到 ego, 附着会超时'
                     % (tag, ego, objs))
        else:
            rep.pass_('C3', '%s: ego ScenarioObject=%r' % (tag, ego))

        # (2) catalog 与 sensor_kit_entry 条目
        cat = catalog_for_xosc(root, path) or os.path.join(root_dir, CATALOG_FILENAME)
        if cat not in cache:
            if os.path.isfile(cat):
                try:
                    cache[cat] = read_catalog_entries(cat)
                except ET.ParseError as e:
                    cache[cat] = None
                    rep.fail('C3', '%s: catalog 解析失败 (%s)'
                             % (rel_tag(cat, root_dir), e))
            else:
                cache[cat] = None
        cat_entries = cache[cat]
        kit = param_value(root, PARAM_KIT)
        if cat_entries is None:
            rep.fail('C3', '%s: ControllerCatalog 不可用 (%s) -> scenario_runner'
                           ' 无法附着传感器' % (tag, rel_tag(cat, root_dir)))
        elif not kit:
            rep.fail('C3', '%s: 缺 sensor_kit_entry 声明 -> 传感器套件无法被'
                           ' e2e 参数覆盖' % tag)
        elif kit not in [n for n, _ in cat_entries]:
            rep.fail('C3', '%s: sensor_kit_entry=%r 不在 catalog 条目 %s'
                     % (tag, kit, [n for n, _ in cat_entries]))
        else:
            rep.pass_('C3', '%s: sensor_kit_entry=%r 命中 catalog' % (tag, kit))

        # (3) ego 轨迹引用: 声明了就必须有同名 json, 否则 monitor 静默降级
        ref = param_value(root, PARAM_TRAJ)
        if not ref:
            rep.warn('C3', '%s: 未声明 ego_trajectory_ref -> 走 autoware 完整'
                           '规划链路 (注入链只做 set_route), 属预期配置则忽略' % tag)
        else:
            full = os.path.normpath(os.path.join(
                os.path.dirname(os.path.abspath(path)), ref))
            js = os.path.splitext(full)[0] + '.json'
            if not os.path.isfile(js):
                rep.fail('C3', '%s: ego_trajectory_ref=%r 的同名 json 不存在 (%s)'
                               ' -> monitor 静默跳过 trajectory_publisher, ego 无'
                               '轨迹注入且不报错' % (tag, ref, js))
            else:
                rep.pass_('C3', '%s: ego_trajectory_ref -> %s'
                          % (tag, rel_tag(js, root_dir)))

        # (4) 判据: 无 criteria_ 前缀 -> verdict 恒 NO_CRITERIA (CI 只能拿到 3)
        crit = [c.get('name') for c in root.iter('Condition')
                if (c.get('name') or '').startswith(CRITERIA_PREFIX)]
        if not crit:
            rep.warn('C3', '%s: 无 criteria_ 前缀判据 -> scenario_runner 不落判定'
                           ' JSON, CI 只能拿到 exit 3 (结论不可靠)' % tag)
        else:
            rep.pass_('C3', '%s: 判据 %s' % (tag, crit))


# ------------------------------------------------------ C4 传感器套件三侧一致
def _kit_tag_from_yaml(cfg):
    """与 launch / parse_sensor_kit.resolve_kit_tag 同规则: 值为 true 的
    use_<tag>_sensor_mapping -> tag; 没有 -> full."""
    for key, value in (cfg.get('autoware') or {}).items():
        if key.startswith('use_') and key.endswith('_sensor_mapping') \
                and str(value).lower() == 'true':
            return key[len('use_'):-len('_sensor_mapping')]
    return 'full'


def check_c4(ctx, rep):
    cfg = ctx['cfg']
    tag = _kit_tag_from_yaml(cfg)
    entry = '%s_%s' % (KIT_ENTRY_PREFIX, tag)
    root_dir = ctx['scenario_root']

    catalog = resolve_file(ctx['catalog'], [root_dir])
    if not catalog:
        rep.fail('C4', 'ControllerCatalog 不可解析: %r' % ctx['catalog'])
        return
    try:
        entries = dict(read_catalog_entries(catalog))
    except ET.ParseError as e:
        rep.fail('C4', 'catalog 解析失败 (%s): %s' % (catalog, e))
        return

    # 侧 1(yaml 标签) vs 侧 2(场景库 catalog 条目)
    if entry not in entries:
        rep.fail('C4', 'yaml 选中的套件 %r 不在 catalog 条目 %s -> e2e 透传'
                       ' sensor_kit_entry 覆盖后指向不存在的 Controller, 传感器'
                       ' 附着失败' % (entry, sorted(entries)))
    else:
        rep.pass_('C4', 'yaml 标签 %r -> catalog 条目 %r' % (tag, entry))

    # 侧 3(autoware 侧 sensor mapping): 目录不可达则跳过 (CI 未 checkout 被测物)
    mdir = ctx['mapping_dir']
    if not mdir or not os.path.isdir(mdir):
        rep.warn('C4', 'mapping_dir 不可达 (%r) -> 跳过 sensor_id 一致性检查'
                       ' (可用 --mapping-dir 指向被测物 config 目录)' % (mdir or '',
                                                                       ))
        return
    fname = 'sensor_mapping.yaml' if tag == 'full' \
        else 'sensor_mapping_%s.yaml' % tag
    mfile = os.path.join(mdir, fname)
    if not os.path.isfile(mfile):
        rep.fail('C4', 'autoware 侧缺 %s (mapping_dir=%s) -> 该 kit 在 interface'
                       ' 侧不存在, 只有 catalog 有' % (fname, mdir))
        return
    with open(mfile) as f:
        mapping = yaml.safe_load(f) or {}
    mappings = mapping.get('sensor_mappings') or {}
    enabled = list(mapping.get('enabled_sensors') or [])
    ids = []
    for sname in enabled:
        mid = (mappings.get(sname) or {}).get('id')
        if not mid:
            rep.fail('C4', '%s: enabled_sensors 里的 %r 无 id 字段 -> interface'
                           ' 无法用 role_name 发现该传感器' % (fname, sname))
            continue
        ids.append(mid)
    if ids and entry in entries:
        missing = [i for i in ids if i not in entries[entry]]
        if missing:
            rep.fail('C4', 'catalog 条目 %r 缺 sensor_id %s (mapping 已启用) ->'
                           ' interface 发现不到该传感器, 需重跑 parse_sensor_kit'
                           '.py' % (entry, missing))
        else:
            rep.pass_('C4', '%s 的 id %s 全在 catalog 条目 %r 中'
                      % (fname, ids, entry))


# ------------------------------------------------------------ C6 结果目录卫生
def check_c6(ctx, rep):
    rd = ctx['results_root']
    if not rd:
        rep.warn('C6', 'results_root 未配置 -> run_summary.json 不落盘, CI 无'
                       '判定来源')
        return
    if not os.path.isdir(rd):
        rep.warn('C6', 'results_root 不存在: %s -> CI 需在跑批前创建' % rd)
        return
    stale = sorted(f for f in os.listdir(rd) if f.endswith('.json'))
    if stale:
        rep.warn('C6', 'results_root 有 %d 个残留 json (如 %s) -> monitor 按 mtime'
                       ' 取最新判定, 会串上次结果; CI 跑批前必须清空 %s'
                 % (len(stale), stale[0], rd))
    else:
        rep.pass_('C6', 'results_root 干净 (无残留 json)')


# ---------------------------------------------------------------- C7 版本照片
def _git(path, *args):
    """只读 git 查询; 任何失败都返回 (rc<0, ''), 不抛异常 (日志要能继续)."""
    try:
        p = subprocess.run(['git', '-C', path] + list(args),
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           timeout=20)
        return p.returncode, p.stdout.decode('utf-8', 'replace').strip()
    except (OSError, subprocess.SubprocessError):
        return -1, ''


def _sha256(path):
    """文件指纹 (补丁被覆盖/重装丢失时能看出来)."""
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def _lock_path(base, raw):
    """lock 里的 path: 相对路径以 lock 文件所在目录为基准."""
    raw = os.path.expanduser(str(raw or '')).strip()
    if not raw:
        return ''
    return raw if os.path.isabs(raw) else os.path.normpath(os.path.join(base, raw))


def check_c7(ctx, rep):
    """拿 versions.lock.yaml 这份照片与机器上的实际版本对账.

    判定语义 (谁变了谁负责):
      - commit 不符 -> FAIL: 跑的不是照片里那份, 结论与历史不可比
      - 目录缺失 / 不是 git 仓 -> FAIL: 无版本可对
      - 有未提交改动 -> allow_dirty 时 WARN (开发期), 否则 FAIL
    """
    lock_path = ctx.get('lock') or ''
    if not lock_path or not os.path.isfile(lock_path):
        rep.warn('C7', '无 versions.lock.yaml -> 版本未锁, "昨天绿今天红" 无法归因')
        return
    try:
        with open(lock_path) as f:
            lock = yaml.safe_load(f) or {}
    except (OSError, ValueError) as e:
        rep.fail('C7', 'lock 文件不可解析 (%s): %s' % (lock_path, e))
        return

    base = os.path.dirname(os.path.abspath(lock_path))
    rep.pass_('C7', 'lock=%s (schema=%s)' % (lock_path, lock.get('schema')))

    for name in ('scenario_lib', 'fault_lib', 'autoware'):
        ent = lock.get(name) or {}
        path = _lock_path(base, ent.get('path'))
        want = str(ent.get('commit') or '').strip()
        if not path:
            rep.fail('C7', '%s.path 未声明 -> 该类资产无版本可对' % name)
            continue
        if not os.path.isdir(path) and name != 'autoware':
            # 安装态下 lock 可能从 share/build 目录被读到, 相对基准会漂;
            # 此时回退到编排配置里的资产根 (它们才是运行时真正读的位置)
            fb = ctx.get('scenario_root' if name == 'scenario_lib' else 'fault_root')
            if fb and os.path.isdir(fb):
                rep.warn('C7', 'lock 中 %s.path 不可达 (%s), 改用配置根 %s'
                         % (name, path, fb))
                path = fb
        if not os.path.isdir(path):
            rep.fail('C7', '%s 目录不存在: %s -> 照片与机器不是同一份'
                     % (name, path))
            continue
        rc, head = _git(path, 'rev-parse', 'HEAD')
        if rc != 0 or not head:
            rep.fail('C7', '%s 不是 git 仓: %s -> 先 git init + commit 才有版本'
                     % (name, path))
            continue
        if want and not head.startswith(want):
            rep.fail('C7', '%s 版本不符: lock=%s 实际=%s -> 要么改回照片那次,'
                     ' 要么在新版本上重跑全量并更新 lock'
                     % (name, want, head[:12]))
            continue
        _, dirty = _git(path, 'status', '--porcelain')
        if dirty:
            msg = ('%s @ %s 有未提交改动 (%d 行) -> 实质跑的与照片不完全一致'
                   % (name, head[:12], len(dirty.splitlines())))
            if ent.get('allow_dirty'):
                rep.warn('C7', msg)
            else:
                rep.fail('C7', msg + '; 本类资产要求精确匹配')
        else:
            rep.pass_('C7', '%s @ %s (干净)' % (name, head[:12]))

    sr = lock.get('scenario_runner') or {}
    if sr:
        path = _lock_path(base, sr.get('path'))
        ver = str(sr.get('version') or '')
        if not path or not os.path.isdir(path):
            rep.fail('C7', 'scenario_runner 目录不存在: %s' % (path or '(未声明)'))
            return
        if ver and ver not in os.path.basename(path):
            rep.warn('C7', 'scenario_runner 目录名不含 version=%r (%s) -> 仅提示'
                     % (ver, os.path.basename(path)))
        for item in (sr.get('patched') or []):
            fpath = os.path.join(path, str(item.get('file') or ''))
            want = str(item.get('sha256') or '').lower()
            if not os.path.isfile(fpath):
                rep.fail('C7', 'scenario_runner 补丁文件不存在: %s -> 该版本被'
                               '替换/未打补丁' % fpath)
                continue
            got = _sha256(fpath)
            if want and got != want:
                rep.fail('C7', 'scenario_runner 补丁指纹不符: %s\n'
                               '         lock=%s\n         实际=%s\n'
                               '         -> 补丁丢失/被覆盖 (丢了 SYNC_FLAG 补丁会与'
                               ' monitor 抢同步, 表现为随机 NO_CRITERIA)'
                         % (os.path.relpath(fpath, path), want, got))
            else:
                rep.pass_('C7', 'scenario_runner %s 补丁指纹匹配 (%s)'
                          % (ver, os.path.basename(fpath)))


# ----------------------------------------------------------------------- 入口
def default_lock():
    """lock 定位: ASR_LOCK > 源码树 harness 仓根 > ament share."""
    cands = [os.environ.get('ASR_LOCK', ''),
             os.path.normpath(os.path.join(
                 os.path.dirname(os.path.abspath(__file__)),
                 '..', 'versions.lock.yaml'))]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(get_package_share_directory('autoware_srunner'),
                                  'versions.lock.yaml'))
    except Exception:  # pylint: disable=broad-except
        pass
    for c in cands:
        if c and os.path.isfile(c):
            return c
    return ''


def default_config():
    """配置定位顺序: ASR_CONFIG > 源码树(../config, build 前也可跑) > ament share."""
    cands = [os.environ.get('ASR_CONFIG', ''),
             os.path.normpath(os.path.join(
                 os.path.dirname(os.path.abspath(__file__)),
                 '..', 'config', 'autoware_srunner.yaml'))]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(get_package_share_directory('autoware_srunner'),
                                  'config', 'autoware_srunner.yaml'))
    except Exception:  # pylint: disable=broad-except
        pass
    for c in cands:
        if c and os.path.isfile(c):
            return c
    return ''


def build_parser():
    ap = argparse.ArgumentParser(
        description='跨仓契约校验 (场景库/故障库/autoware/harness 四仓错配提前拦)')
    ap.add_argument('--config', default='', help='编排配置 yaml (默认自动定位)')
    ap.add_argument('--scenario-root', default='', help='场景库根 (覆盖 paths.scenario_root)')
    ap.add_argument('--fault-root', default='', help='故障库根 (覆盖 paths.fault_root)')
    ap.add_argument('--results-root', default='', help='结果根 (覆盖 paths.results_root)')
    ap.add_argument('--xosc-list', default='', help='场景清单 (覆盖 lifecycle.cycle)')
    ap.add_argument('--fault-matrix', default='', help='故障矩阵 (覆盖 lifecycle.cycle)')
    ap.add_argument('--catalog', default='', help='ControllerCatalog.xosc 路径')
    ap.add_argument('--mapping-dir', default='',
                    help='autoware 侧 sensor_mapping*.yaml 所在目录 (不可达则跳过 C4 侧 3)')
    ap.add_argument('--lock', default='',
                    help='版本照片 versions.lock.yaml (默认自动定位; 传 none 跳过 C7)')
    ap.add_argument('--strict', action='store_true', help='WARN 也视为失败')
    ap.add_argument('--json', default='', help='把结果写成 JSON (CI artifact)')
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)

    cfg_path = args.config or default_config()
    if not cfg_path or not os.path.isfile(cfg_path):
        print('[FAIL] C0 找不到编排配置 (试过 --config / ASR_CONFIG / 源码树 /'
              ' ament share)')
        return 1
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f) or {}

    ws = (cfg.get('workspace') or {}).get('root', '')
    paths = cfg.get('paths') or {}
    cycle = (cfg.get('lifecycle') or {}).get('cycle') or {}
    scenario_root = os.path.expanduser(args.scenario_root) or \
        paths.get('scenario_root') or os.path.join(ws, 'scenarios')
    fault_root = os.path.expanduser(args.fault_root) or \
        paths.get('fault_root') or os.path.join(ws, 'fault')
    results_root = os.path.expanduser(args.results_root) or \
        paths.get('results_root') or os.path.join(ws, 'scenarios', 'results')
    mapping_dir = os.path.expanduser(args.mapping_dir) or \
        paths.get('sensor_mapping_dir') or os.environ.get('ASR_SENSOR_MAPPING_DIR', '')

    lock = '' if args.lock.lower() == 'none' else (args.lock or default_lock())

    ctx = {
        'cfg': cfg,
        'cfg_path': cfg_path,
        'lock': lock,
        'scenario_root': scenario_root,
        'fault_root': fault_root,
        'results_root': results_root,
        'mapping_dir': mapping_dir,
        'xosc_list': args.xosc_list or os.environ.get('ASR_XOSC_LIST', '')
                     or cycle.get('xosc_list', ''),
        'fault_matrix': args.fault_matrix or os.environ.get('ASR_FAULT_MATRIX', '')
                        or cycle.get('fault_matrix', ''),
        'catalog': args.catalog or os.path.join(scenario_root, CATALOG_FILENAME),
        'scenario_files': [],
        'names': {},
    }

    print('config        : %s' % cfg_path)
    print('scenario_root : %s' % scenario_root)
    print('fault_root    : %s' % fault_root)
    print('results_root  : %s' % results_root)
    print('xosc_list     : %s' % (ctx['xosc_list'] or '(空)'))
    print('fault_matrix  : %s' % (ctx['fault_matrix'] or '(空)'))
    print('lock          : %s' % (lock or '(无, 跳过 C7)'))
    print('-' * 72)

    rep = Report()
    # C5 先于 C3: 矩阵驱动模式下场景清单由矩阵提供, 需先展开再逐个查结构
    for fn in (check_c1, check_c2, check_c5, check_c3, check_c4, check_c6,
               check_c7):
        try:
            fn(ctx, rep)
        except Exception as e:  # pylint: disable=broad-except
            rep.fail('CX', '%s 未预期异常: %r' % (fn.__name__, e))
    rep.dump()

    counts = rep.counts()
    ok = counts['FAIL'] == 0 and not (args.strict and counts['WARN'])
    if args.json:
        payload = {'ok': ok, 'counts': counts, 'rows': rep.rows,
                   'scenario_root': scenario_root, 'fault_root': fault_root,
                   'config': cfg_path, 'lock': lock}
        try:
            with open(args.json, 'w') as f:
                json.dump(payload, f, indent=2, ensure_ascii=False)
            print('JSON 结果 -> %s' % args.json)
        except OSError as e:
            print('[WARN] JSON 落盘失败: %s' % e)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())