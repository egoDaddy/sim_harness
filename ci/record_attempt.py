#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""record_attempt: 把"本次尝试"的判定结论写进 attempt 台账 (result.json)。

为什么需要: GitLab 的 job retry 会让最终状态取最后一次尝试, 第一次失败被"盖掉"
(页面变绿、artifacts 覆盖、也没有 attempt 级计数)。本脚本每次尝试结束都落一份
结构化结论, 供 collect_attempts.py 汇总成 attempts / flaky / 确定重复 指标。

台账位置 (由 CI 传入 --attempt-dir):
  $CI_OUT/attempts/<pipeline_id>/attempt_NN/result.json

边界: 本脚本**不改变判定**, 退出码恒为 0; job 的结论仍由
judge_run_summary.py 的退出码决定 (`--judge-rc` 只是被记录。
"""
import argparse
import json
import os
import time


def _read_seq(attempt_dir):
    """meta.env 里的 seq (没有就返回 None, 不影响记录)."""
    path = os.path.join(attempt_dir, 'meta.env')
    try:
        with open(path) as f:
            for line in f:
                if line.startswith('seq='):
                    return line.split('=', 1)[1].strip()
    except OSError:
        pass
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(description='记录单次尝试结论 (result.json)')
    ap.add_argument('--attempt-dir', required=True, help='本次尝试的台账目录')
    ap.add_argument('--summary', required=True, help='run_summary.json 路径')
    ap.add_argument('--judge-rc', type=int, required=True, help='judge 的退出码')
    args = ap.parse_args(argv)

    # 空目录会让 result.json 落到当前工作目录 (os.path.join('', 'x') == 'x'),
    # 直接跳过比在工作区里留垃圾安全
    if not args.attempt_dir.strip():
        print('[warn] --attempt-dir 为空, 跳过台账记录')
        return 0

    rec = {
        'seq': _read_seq(args.attempt_dir),
        'judge_rc': args.judge_rc,
        'finished_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'summary_exists': os.path.isfile(args.summary),
        'exit_code': None,
        'total': None,
        'abort_reason': None,
        'failed_criteria': [],
    }

    if rec['summary_exists']:
        try:
            with open(args.summary) as f:
                data = json.load(f)
            summary = data.get('summary') or {}
            rec['exit_code'] = summary.get('exit_code')
            rec['total'] = summary.get('total')
            rec['abort_reason'] = (data.get('run') or {}).get('abort_reason')
            # 记"哪个场景的哪条判据没过", 用于区分"确定性失败"和"环境抖动"
            for sc in data.get('scenarios') or []:
                if not isinstance(sc, dict):
                    continue
                for c in ((sc.get('verdict') or {}).get('criteria') or []):
                    if isinstance(c, dict) and c.get('success') is False:
                        rec['failed_criteria'].append(
                            '%s/%s' % (sc.get('name', '?'), c.get('name', '?')))
        except (OSError, ValueError) as e:
            rec['parse_error'] = str(e)

    out = os.path.join(args.attempt_dir, 'result.json')
    try:
        with open(out, 'w') as f:
            json.dump(rec, f, indent=2, ensure_ascii=False)
    except OSError as e:
        print('[warn] 台账落盘失败 (不影响判定): %s' % e)
        return 0

    print('[attempt] seq=%s judge_rc=%s exit_code=%s 失败判据=%s -> %s'
          % (rec['seq'], rec['judge_rc'], rec['exit_code'],
             ','.join(rec['failed_criteria']) or '-', out))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())