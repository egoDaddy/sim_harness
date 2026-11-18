#!/usr/bin/env python3
"""call_execute_scenario: 调用 /scenario_runner/execute_scenario 服务, 拉动 scenario_runner 执行 xosc.

用法: call_execute_scenario.py <scenario_name> <scenario_file> [--timeout 120]
"""
import argparse
import sys

import rclpy
from rclpy.node import Node

from carla_ros_scenario_runner_types.srv import ExecuteScenario


def main():
    p = argparse.ArgumentParser(description='调用 execute_scenario 服务')
    p.add_argument('name', help='场景名')
    p.add_argument('scenario_file', help='xosc 路径')
    p.add_argument('--timeout', type=float, default=120.0, help='服务等待/响应超时秒')
    args = p.parse_args()

    rclpy.init()
    node = Node('call_execute_scenario')
    cli = node.create_client(ExecuteScenario, '/scenario_runner/execute_scenario')

    if not cli.wait_for_service(args.timeout):
        node.get_logger().error('等待服务 /scenario_runner/execute_scenario 超时')
        sys.exit(1)

    req = ExecuteScenario.Request()
    req.scenario.name = args.name
    req.scenario.scenario_file = args.scenario_file
    node.get_logger().info('请求执行场景: %s (%s)' % (args.name, args.scenario_file))
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=args.timeout)

    ok = fut.done() and fut.result() is not None and fut.result().result
    node.get_logger().info('execute_scenario 结果: %s' % ('成功' if ok else '失败/超时'))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
