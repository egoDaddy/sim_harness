#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""judge_run_summary: 按 run_summary.json 判定 CI 结论 (需求 5 的 CI 消费端)。

为什么需要它: ros2 launch 的返回码不可信 —— monitor 在 auto 模式下 exit 1
(编排异常) 会刻意不 Shutdown 保留现场, launch 也可能被 timeout 强杀。因此
CI 的唯一判定来源是 scenario_monitor._write_run_summary() 落盘的
run_summary.json:

  summary.exit_code  0 = 全部通过
                     1 = 编排异常 (e2e 死/服务拒收/墙钟超时/信号中止)
                     2 = 测试失败 (存在 FAILURE 场景)
                     3 = 结论不可靠 (NO_CRITERIA / TIMEOUT / UNKNOWN)

额外守一条: summary.total == 0 不算通过 —— 清单为空/全部未执行
时 exit_code 也是 0, 这是最容易出现的"假绿"。

用法:
  judge_run_summary.py --summary out/run_summary.json
  judge_run_summary.py --summary out/run_summary.json --expect-total 6
退出码: 0 = 通过; 1 = 不通过 (含 summary 缺失)。
"""
import argparse
import json
import os
import sys

# exit_code -> (CI 级别, 人类可读原因)
VERDICT = {
    0: ('PASS', '全部场景通过'),
    1: ('FAIL', '编排异常 (e2e 死 / 服务拒收 / 墙钟超时 / 信号中止)'),
    2: ('FAIL', '测试失败 (存在 FAILURE 场景)'),
    3: ('FAIL', '结论不可靠 (NO_CRITERIA / TIMEOUT / UNKNOWN)'),
}

# 逐场景 verdict -> 明细里要突出的异常级别
ABNORMAL = ('FAILURE', 'ERROR', 'NO_CRITERIA', 'TIMEOUT', 'UNKNOWN')


def _fmt_metrics(metrics):
    """挑几个关键指标单行展示 (缺失就留空, 不报错)."""
    if not isinstance(metrics, dict) or not metrics:
        return ''
    keys = ('collision_speed_kph', 'min_ttc_s', 'max_acc_kph')
    parts = ['%s=%s' % (k, metrics[k]) for k in keys if metrics.get(k) is not None]
    return ('  ' + ' '.join(parts)) if parts else ''


def _fmt_criteria(verdict):
    """criteria 是 Criterion 列表: 只列失败的项名, 别把整段 dict 打出来."""
    crit = verdict.get('criteria')
    if not isinstance(crit, list) or not crit:
        return 'criteria=-'
    bad = [str(c.get('name', '?')) for c in crit
           if isinstance(c, dict) and c.get('success') is False]
    if bad:
        return 'failed=' + ','.join(bad)
    return 'criteria=ok(%d)' % len(crit)


def main(argv=None):
    ap = argparse.ArgumentParser(description='按 run_summary.json 判定 CI 结论')
    ap.add_argument('--summary', required=True, help='run_summary.json 路径')
    ap.add_argument('--expect-total', type=int, default=0,
                    help='>0 时校验场景数 (矩阵子集回归用), 不符判失败')
    args = ap.parse_args(argv)

    if not os.path.isfile(args.summary):
        print('[FAIL] 找不到 %s' % args.summary)
        print('       可能原因: monitor 未正常收尾 (launch 被 kill / results_dir 落错),')
        print('       或路径写错 —— CI 里应为 ci_out/sim/run_summary.json')
        return 1

    try:
        with open(args.summary) as f:
            data = json.load(f)
    except (OSError, ValueError) as e:
        print('[FAIL] run_summary.json 解析失败: %s' % e)
        return 1

    summary = data.get('summary') or {}
    run = data.get('run') or {}

    print('run: orchestration=%s  abort=%s'
          % (run.get('orchestration', '?'), run.get('abort_reason') or '-'))
    print('summary: total=%s success=%s failure=%s error=%s no_criteria=%s'
          % (summary.get('total'), summary.get('success'),
             summary.get('failure'), summary.get('error'),
             summary.get('no_criteria_or_unknown')))

    scenarios = data.get('scenarios') or []
    for sc in scenarios:
        verdict_obj = sc.get('verdict') or {}
        verdict = verdict_obj.get('result', '?')
        execu = sc.get('execution') or {}
        flag = '  <<' if verdict in ABNORMAL else ''
        print('  %-30s %-10s %-8s %s%s%s'
              % (str(sc.get('name', '?'))[:30], verdict,
                 execu.get('status', '?'),
                 _fmt_criteria(verdict_obj),
                 _fmt_metrics(sc.get('metrics')), flag))

    code = summary.get('exit_code')
    try:
        code = int(code)
    except (TypeError, ValueError):
        code = -1
    level, why = VERDICT.get(code, ('FAIL', '未知 exit_code=%r' % summary.get('exit_code')))
    print('-' * 66)
    print('%s exit_code=%s (%s)' % (level, code, why))

    if level != 'PASS':
        return 1

    total = summary.get('total') or 0
    if total == 0:
        print('[FAIL] 场景数为 0 -> 清单为空或全部未执行, 不能当通过')
        return 1
    if args.expect_total and total != args.expect_total:
        print('[FAIL] 期望 %d 个场景, 实际 %d' % (args.expect_total, total))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
