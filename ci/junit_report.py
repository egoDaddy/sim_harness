#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""junit_report: 把自研 JSON 结果转成 JUnit XML (GitLab Tests 页面展示用)。

为什么需要: judge_run_summary.py 只往终端打印, contract.json / run_summary.json
都是自研结构, GitLab 页面读不到, 于是看不到"用例级别的通过率"。

边界 (重要): 本脚本**只做格式转换, 不参与判定**。结论仍然只由
  - contract: scenario_contract_check.py 的退出码 (FAIL 计数)
  - sim     : run_summary.json 的 summary.exit_code (judge_run_summary.py)
决定。转换失败只打印 warn 并返回 0, 绝不改变 job 结论, 避免出现两个判定源。

用法:
  junit_report.py --contract ci_out/contract/contract.json --out ci_out/contract/junit.xml
  junit_report.py --sim ci_out/sim/run_summary.json --out ci_out/sim/junit.xml

退出码恒为 0。
"""
import argparse
import json
import os
import xml.etree.ElementTree as ET

# XML 1.0 不允许的控制字符 (保留 \t \n \r)
_ILLEGAL = {c: None for c in range(0x20) if c not in (0x09, 0x0A, 0x0D)}


def _clean(value):
    return str('' if value is None else value).translate(_ILLEGAL)


def _short(value, limit=100):
    text = _clean(value).replace('\n', ' ').replace('\r', ' ').strip()
    return text if len(text) <= limit else text[:limit] + '...'


def _new_suite(root, name):
    return ET.SubElement(root, 'testsuite', name=name, tests='0', failures='0',
                         errors='0', skipped='0', time='0')


def _add_case(suite, classname, name, time_s=0.0):
    """建 testcase 并同步 tests 计数 (统计口径在这里统一收口).

    time_s 必须是真实耗时: 之前固定 time='0' 导致 GitLab Tests 页面里每条用例
    都是 0s (只有 suite 汇总时长真实)。
    """
    suite.set('tests', str(int(suite.get('tests')) + 1))
    try:
        elapsed = max(0.0, float(time_s or 0.0))
    except (TypeError, ValueError):
        elapsed = 0.0
    return ET.SubElement(suite, 'testcase', classname=classname, name=name,
                         time='%.3f' % elapsed)


def _bump(suite, attr):
    suite.set(attr, str(int(suite.get(attr)) + 1))


# ------------------------------------------------------------------ contract
def _fill_contract(suite, path):
    """contract.json: {ok, counts, rows:[{id, level, message}]} -> 每条一个用例."""
    if not os.path.isfile(path):
        case = _add_case(suite, 'contract', 'contract.json 缺失')
        _bump(suite, 'skipped')
        ET.SubElement(case, 'skipped',
                      message='未产出 contract.json (契约步骤可能提前失败)')
        return
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError) as e:
        case = _add_case(suite, 'contract', 'contract.json 解析失败')
        _bump(suite, 'errors')
        ET.SubElement(case, 'error', message=_short('解析失败: %s' % e, 200))
        return

    rows = data.get('rows') or []
    for i, row in enumerate(rows):
        level = str((row or {}).get('level') or 'PASS').upper()
        cid = _clean((row or {}).get('id') or 'CX')
        msg = _clean((row or {}).get('message') or '')
        case = _add_case(suite, 'contract.%s' % cid,
                         '%s-%02d %s' % (cid, i + 1, _short(msg, 80)))
        if level == 'FAIL':
            _bump(suite, 'failures')
            node = ET.SubElement(case, 'failure', message=_short(msg, 200))
            node.text = msg
        elif level == 'WARN':
            # WARN = 已执行但有风险, 不是"未执行". 映射成 <skipped> 会让 GitLab
            # 显示为 skipped, 误导成"校验被跳过". 保持用例通过, 告警写进
            # system-out (只展示, 判定仍由 contract 退出码决定).
            out = ET.SubElement(case, 'system-out')
            out.text = _clean('[WARN] ' + msg)


# ----------------------------------------------------------------------- sim
_VERDICT_KIND = {
    'SUCCESS': None,       # 通过
    'FAILURE': 'failure',  # 判据不满足 (测试失败)
    'ERROR': 'error',      # scenario_runner 异常退出, 判定未知
    'NO_CRITERIA': 'error',
    'UNKNOWN': 'error',
}


def _criteria_lines(criteria):
    """把 criteria 明细压成可读多行 (写进 system-out)."""
    lines = []
    for c in criteria or []:
        if not isinstance(c, dict):
            continue
        lines.append('%s(actor=%s) expected=%s actual=%s success=%s'
                     % (c.get('name', '?'), c.get('actor', '-'),
                        c.get('expected'), c.get('actual'), c.get('success')))
    return lines


def _fill_sim(suite, path):
    """run_summary.json -> 每个场景一个用例; 判据明细进 failure/system-out."""
    if not os.path.isfile(path):
        case = _add_case(suite, 'sim', 'run_summary.json 缺失')
        _bump(suite, 'errors')
        ET.SubElement(case, 'error',
                      message='未产出 run_summary.json (monitor 未收尾, 见 launch.log)')
        return
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError) as e:
        case = _add_case(suite, 'sim', 'run_summary.json 解析失败')
        _bump(suite, 'errors')
        ET.SubElement(case, 'error', message=_short('解析失败: %s' % e, 200))
        return

    run = data.get('run') or {}
    summary = data.get('summary') or {}
    try:
        elapsed = max(0.0, float(run.get('end_time')) - float(run.get('start_time')))
    except (TypeError, ValueError):
        elapsed = 0.0
    suite.set('time', '%.3f' % elapsed)
    # 编排异常信息写在 suite 级别, 页面顶部就能看到
    props = ET.SubElement(suite, 'system-out')
    props.text = _clean('orchestration=%s abort_reason=%s exit_code=%s total=%s'
                        % (run.get('orchestration', '?'),
                           run.get('abort_reason') or '-',
                           summary.get('exit_code'), summary.get('total')))

    for sc in data.get('scenarios') or []:
        if not isinstance(sc, dict):
            continue
        verdict_obj = sc.get('verdict') or {}
        result = str(verdict_obj.get('result') or 'UNKNOWN').upper()
        criteria = verdict_obj.get('criteria') or []
        bad = [str(c.get('name', '?')) for c in criteria
               if isinstance(c, dict) and c.get('success') is False]
        name = _short(sc.get('name') or '?', 120)
        # 单条墙钟耗时由 monitor 落盘 (scenarios[].wall_time_s); 缺失时退回 0
        case = _add_case(suite, 'sim.scenario', name,
                         sc.get('wall_time_s', 0.0))

        kind = _VERDICT_KIND.get(result, 'error')
        if kind == 'failure':
            _bump(suite, 'failures')
            node = ET.SubElement(case, 'failure',
                                 message=_short('failed=' + ','.join(bad), 200))
            node.text = _clean('\n'.join(_criteria_lines(criteria)))
        elif kind == 'error':
            _bump(suite, 'errors')
            node = ET.SubElement(case, 'error',
                                 message=_short('结论不可靠: %s' % result, 200))
            node.text = _clean('execution.result=%s abort_reason=%s'
                               % ((sc.get('execution') or {}).get('result'),
                                  sc.get('abort_reason') or '-'))

        out = ET.SubElement(case, 'system-out')
        try:
            wall = max(0.0, float(sc.get('wall_time_s') or 0.0))
        except (TypeError, ValueError):
            wall = 0.0
        out.text = _clean('wall_time_s=%.3f\n%s'
                          % (wall, '\n'.join(_criteria_lines(criteria)) or 'criteria=-'))


# ---------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description='自研 JSON -> JUnit XML (仅展示, 不判定)')
    ap.add_argument('--contract', default='', help='contract.json 路径')
    ap.add_argument('--sim', default='', help='run_summary.json 路径')
    ap.add_argument('--out', required=True, help='输出 junit.xml 路径')
    args = ap.parse_args(argv)

    if not args.contract and not args.sim:
        print('[warn] 未给 --contract / --sim, 不生成 JUnit')
        return 0

    root = ET.Element('testsuites', name='autoware_srunner')
    if args.contract:
        _fill_contract(_new_suite(root, 'contract'), args.contract)
    if args.sim:
        _fill_sim(_new_suite(root, 'sim'), args.sim)

    out_dir = os.path.dirname(os.path.abspath(args.out))
    try:
        os.makedirs(out_dir, exist_ok=True)
        ET.ElementTree(root).write(args.out, encoding='utf-8', xml_declaration=True)
    except OSError as e:
        print('[warn] JUnit 落盘失败 (不影响判定): %s' % e)
        return 0

    for suite in root:
        print('[junit] %-10s tests=%-3s failures=%-3s errors=%-3s skipped=%-3s -> %s'
              % (suite.get('name'), suite.get('tests'), suite.get('failures'),
                 suite.get('errors'), suite.get('skipped'), args.out))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())