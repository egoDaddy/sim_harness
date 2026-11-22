#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""collect_attempts: 汇总 attempt 台账 -> 第一批指标 (attempts / flaky / 重复失败).

背景: job retry 之后 GitLab 只认最后一次尝试, 第一次失败被"盖掉" —— 页面变绿、
artifacts 覆盖、也没有 attempt 级计数。record_attempt.py 在每次尝试结束时往台账写
一份 result.json, 本脚本在 report 阶段统一汇总, 产出 Prometheus 文本
(artifacts:reports:metrics), 让"第几次才过"可被统计。

指标 (第一批; 通过率/时长等留给后续):
  ci_sim_attempts_total           本 pipeline 的尝试次数
  ci_sim_attempts_failed_total    其中失败的次数
  ci_sim_attempt_failed{seq}      每次尝试是否失败 (1/0)
  ci_sim_attempt_exit_code{seq}   run_summary 的 exit_code (-1 = 未落盘)
  ci_sim_flaky                    最后一次通过但前面失败过 -> 1
  ci_sim_deterministic_repeat     相邻两次失败且失败判据完全相同 -> 1 (配置自检:
                                  若长期为 1, 说明 when 列表误收了确定性失败)
  ci_sim_attempts_api             GitLab API 对账值 (可选, 缺省不输出)

用法:
  collect_attempts.py --ledger "$CI_OUT/attempts/$CI_PIPELINE_ID" \
                      --attempts-root "$CI_OUT/attempts" --keep 10 \
                      --out "$CI_OUT/metrics/attempts.txt" --job-name "sim:smoke"
"""
import argparse
import json
import os
import shutil
import urllib.request


def _lbl(value):
    """Prometheus label 值转义 (反斜杠/双引号/换行)."""
    return str(value).replace('\\', '\\\\').replace('"', '\\"').replace('\n', ' ')


def _load(ledger):
    """读 attempt_NN/result.json, 按 seq 排序; 缺失的尝试直接跳过并提示."""
    if not ledger or not os.path.isdir(ledger):
        print('[collect] 台账目录不存在: %s (sim 可能被 rules 跳过)' % ledger)
        return []
    rows = []
    for name in sorted(os.listdir(ledger)):
        path = os.path.join(ledger, name, 'result.json')
        if not os.path.isfile(path):
            print('[collect] 跳过 %s (无 result.json, 该尝试可能在判定前中断)' % name)
            continue
        try:
            with open(path) as f:
                rec = json.load(f)
        except (OSError, ValueError) as e:
            print('[collect] 跳过 %s (解析失败: %s)' % (name, e))
            continue
        rec['_dir'] = name
        rows.append(rec)
    return rows


def _prune(root, keep, protected):
    """只保留最近 keep 个 pipeline 台账目录 (按 mtime), 防止长期累积占空间."""
    if not root or not os.path.isdir(root):
        return []
    protected = os.path.abspath(protected or '')
    dirs = [os.path.join(root, d) for d in os.listdir(root)
            if os.path.isdir(os.path.join(root, d))]
    dirs.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    removed = []
    for path in dirs[keep:]:
        if os.path.abspath(path) == protected:
            continue
        try:
            shutil.rmtree(path)
            removed.append(os.path.basename(path))
        except OSError as e:
            print('[collect] 清理 %s 失败: %s' % (path, e))
    return removed


def _reconcile(api_url, project_id, pipeline_id, token, job_name):
    """用 CI_JOB_TOKEN 查本 pipeline 的 job 列表, 对账台账条数 (失败不阻塞)."""
    if not (api_url and project_id and pipeline_id and token):
        return None, '参数不全, 跳过对账'
    url = '%s/projects/%s/pipelines/%s/jobs?per_page=100' % (
        api_url.rstrip('/'), project_id, pipeline_id)
    req = urllib.request.Request(url, headers={'JOB-TOKEN': token})
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            jobs = json.loads(resp.read().decode('utf-8'))
    except Exception as e:  # pylint: disable=broad-except
        return None, '对账请求失败(不影响指标): %r' % e
    same = [j for j in jobs if (j or {}).get('name') == job_name]
    return len(same), 'API 记录 %s 共 %d 条 (status=%s)' % (
        job_name, len(same), ','.join(str(j.get('status')) for j in same) or '-')


def main(argv=None):
    ap = argparse.ArgumentParser(description='汇总 attempt 台账为 Prometheus 指标')
    ap.add_argument('--ledger', required=True, help='本 pipeline 的台账目录')
    ap.add_argument('--attempts-root', default='', help='台账根 (用于清理旧 pipeline)')
    ap.add_argument('--keep', type=int, default=10, help='保留最近多少个 pipeline 台账')
    ap.add_argument('--out', required=True, help='输出 Prometheus 文本路径')
    ap.add_argument('--job-name', default='sim:smoke', help='被观察的 job 名')
    ap.add_argument('--api-url', default='', help='CI_API_V4_URL')
    ap.add_argument('--project-id', default='', help='CI_PROJECT_ID')
    ap.add_argument('--pipeline-id', default='', help='CI_PIPELINE_ID')
    ap.add_argument('--token', default='', help='CI_JOB_TOKEN')
    args = ap.parse_args(argv)

    rows = _load(args.ledger)
    pipeline = args.pipeline_id or os.path.basename(os.path.normpath(args.ledger))
    job = args.job_name
    base = 'pipeline="%s",job="%s"' % (_lbl(pipeline), _lbl(job))

    failed = [r for r in rows if int(r.get('judge_rc') or 0) != 0]
    # flaky: 最终通过, 但之前有过失败 (典型的"第二次才过")
    flaky = 1 if (len(rows) >= 2 and int(rows[-1].get('judge_rc') or 0) == 0
                  and failed) else 0
    # 确定性重复: 相邻两次都失败且失败判据完全相同 -> 说明这条失败不该被重试
    repeat = 0
    for prev, cur in zip(rows, rows[1:]):
        if int(prev.get('judge_rc') or 0) == 0 or int(cur.get('judge_rc') or 0) == 0:
            continue
        if sorted(prev.get('failed_criteria') or []) == \
                sorted(cur.get('failed_criteria') or []):
            repeat = 1
            break

    lines = [
        '# HELP ci_sim_attempts_total 本 pipeline 中 %s 的执行次数' % job,
        '# TYPE ci_sim_attempts_total gauge',
        'ci_sim_attempts_total{%s} %d' % (base, len(rows)),
        '# HELP ci_sim_attempts_failed_total 其中失败的次数',
        '# TYPE ci_sim_attempts_failed_total gauge',
        'ci_sim_attempts_failed_total{%s} %d' % (base, len(failed)),
        '# HELP ci_sim_attempt_failed 每次尝试是否失败 (1/0)',
        '# TYPE ci_sim_attempt_failed gauge',
    ]
    for r in rows:
        seq = _lbl(r.get('seq') or r.get('_dir'))
        lines.append('ci_sim_attempt_failed{%s,seq="%s"} %d'
                     % (base, seq, 1 if int(r.get('judge_rc') or 0) != 0 else 0))
    lines += ['# HELP ci_sim_attempt_exit_code 每次尝试的 run_summary exit_code (-1=未落盘)',
              '# TYPE ci_sim_attempt_exit_code gauge']
    for r in rows:
        seq = _lbl(r.get('seq') or r.get('_dir'))
        code = r.get('exit_code')
        lines.append('ci_sim_attempt_exit_code{%s,seq="%s"} %s'
                     % (base, seq, -1 if code is None else int(code)))
    lines += ['# HELP ci_sim_flaky 重试后才通过 (1=本次 pipeline 是 flaky)',
              '# TYPE ci_sim_flaky gauge',
              'ci_sim_flaky{%s} %d' % (base, flaky),
              '# HELP ci_sim_deterministic_repeat 相邻两次失败且判据完全相同',
              '# TYPE ci_sim_deterministic_repeat gauge',
              'ci_sim_deterministic_repeat{%s} %d' % (base, repeat)]

    api_count, api_msg = _reconcile(args.api_url, args.project_id,
                                    args.pipeline_id, args.token, job)
    print('[collect] %s' % api_msg)
    if api_count is not None:
        lines += ['# HELP ci_sim_attempts_api GitLab 侧同名 job 条数 (对账用)',
                  '# TYPE ci_sim_attempts_api gauge',
                  'ci_sim_attempts_api{%s} %d' % (base, api_count)]
        if api_count != len(rows):
            print('[collect][warn] 台账 %d 条 != API %d 条, 检查是否有尝试未写 result.json'
                  % (len(rows), api_count))

    removed = _prune(args.attempts_root, args.keep, args.ledger)
    if removed:
        print('[collect] 清理旧台账: %s' % ', '.join(removed))

    out_dir = os.path.dirname(os.path.abspath(args.out))
    try:
        os.makedirs(out_dir, exist_ok=True)
        with open(args.out, 'w') as f:
            f.write('\n'.join(lines) + '\n')
    except OSError as e:
        print('[collect][warn] 指标落盘失败: %s' % e)
        return 0

    print('[collect] attempts=%d failed=%d flaky=%d repeat=%d -> %s'
          % (len(rows), len(failed), flaky, repeat, args.out))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())