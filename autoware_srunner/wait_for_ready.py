#!/usr/bin/env python3
"""wait_for_ready: set_route.py / trajectory_publisher.py 的前置等待脚本 (需求 3c 的触发信号消费方).

依次等待:
  1) 指定 ROS 服务全部就绪 (e2e 起稳, 如 /api/routing/set_route_points);
  2) latch 话题 /carla/srunner/npc_ready 收到 True (场景 Init 完成, NPC 就位)。
全部满足后退出码 0, 否则超时退出码 1。
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool


def wait_services(node, services, deadline):
    if not services:
        return True
    while time.monotonic() < deadline:
        try:
            names = [n for n, _ in node.get_service_names_and_types()]
        except Exception:  # pylint: disable=broad-except
            names = []
        missing = [s for s in services if s not in names]
        if not missing:
            node.get_logger().info('服务全部就绪: %s' % ', '.join(services))
            return True
        node.get_logger().info('等待服务: %s' % ', '.join(missing), throttle_duration_sec=10.0)
        time.sleep(1.0)
    node.get_logger().error('等待服务超时: %s' % ', '.join(services))
    return False


def wait_topic(node, topic, deadline):
    got = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(Bool, topic, lambda m: got.append(m.data), qos)
    node.get_logger().info('等待触发信号 %s (NPC 就位) ...' % topic)
    while time.monotonic() < deadline:
        if any(got):
            node.get_logger().info('收到触发信号 %s=True' % topic)
            node.destroy_subscription(sub)
            return True
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    node.get_logger().error('等待触发信号超时: %s' % topic)
    return False


def main():
    p = argparse.ArgumentParser(description='等待 NPC 就位触发信号与 e2e 服务就绪')
    p.add_argument('--topic', default='/carla/srunner/npc_ready', help='触发信号 latch 话题')
    p.add_argument('--timeout', type=float, default=300.0, help='总超时秒')
    p.add_argument('--wait-service', nargs='+', default=[],
                   help='同时等待的 ROS 服务名列表 (全部就绪才放行)')
    args = p.parse_args()

    rclpy.init()
    node = Node('wait_for_ready')
    deadline = time.monotonic() + args.timeout

    ok = wait_services(node, args.wait_service, deadline) and \
        wait_topic(node, args.topic, deadline)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
