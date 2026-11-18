#!/usr/bin/env python3
"""set_route: 从当前 ego 位姿向目标点规划并设置 route (mission_planning)，并验证 route 已发布。

用途: 轨迹注入前的必要步骤——scenario_selector 在无 route 时停在 EMPTY 状态,
会丢弃 lane_driving 轨迹, 导致 velocity_smoother/controller 无输入、车辆不动。
由 trajectory_publisher.py 在启动时自动调用 (也可单独运行)。

版本陷阱 (已验证): 本版 mission_planner 的 set_lanelet_route 服务 (/api/routing/set_route)
只复制调用方传入的 segments、不做规划, ADAPI 传空 segments 时永远返回
"The planned route is empty"。真正会跑规划器的是 set_waypoint_route 服务
(/api/routing/set_route_points): points = [ego, waypoints..., goal] -> planner_->plan()。
因此本脚本必须使用 /api/routing/set_route_points。

已知陷阱 (自动处理): ego 停在交叉口重叠区 (如本项目 spawn 点位于停止线以南) 时,
起点吸附到交叉口/转弯 lanelet, 规划结果为 "The planned route is empty"。
此时脚本自动把 ego 沿航向后移 --nudge-back-m 米 (默认 6m, 退到接近段) 后重试。
后移经 /initialpose 发布, 由 autoware_carla_interface 转换并 set_transform 到 CARLA。

用法:
  source <autoware_ws>/install/setup.bash
  python3 set_route.py --goal 334.8,-80.0 --goal-yaw-deg -90.0

约定:
  - goal 为 ROS map 右手系坐标; --goal-yaw-deg 必须与道路方向一致 (南行车道为 -90),
    否则 mission_planner 的 goal 车辆轮廓校验 (footprint exceeds lane) 会拒绝 goal。
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from autoware_adapi_v1_msgs.srv import SetRoutePoints
from autoware_planning_msgs.msg import LaneletRoute


def take_odom(node, timeout=5.0):
    """订阅 /localization/kinematic_state 取一帧 ego 位姿, 返回 (x, y, z, yaw) 或 None。"""
    got = []
    sub = node.create_subscription(Odometry, "/localization/kinematic_state",
                                   lambda m: got.append(m), QoSProfile(depth=10))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not got:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if not got:
        return None
    m = got[-1]
    q = m.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z, yaw)


def nudge_back(node, dist_m):
    """把 ego 沿当前航向后移 dist_m 米 (经 /initialpose -> 接口 set_transform), 并验证落地。"""
    ego = take_odom(node)
    if ego is None:
        node.get_logger().error("后移前无法获取 ego 位姿")
        return False
    x, y, z, yaw = ego
    tx = x - dist_m * math.cos(yaw)   # 航向反方向
    ty = y - dist_m * math.sin(yaw)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = tx
    msg.pose.pose.position.y = ty
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)   # 保持原朝向
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
    node.get_logger().warn("ego 后移 %.1f m: (%.2f, %.2f) -> (%.2f, %.2f), 等待落地 ..."
                           % (dist_m, x, y, tx, ty))
    # 等待订阅者匹配 (RELIABLE QoS 下匹配前发布的消息会被丢弃, ros2 topic pub 默认带 --wait-matching)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.get_logger().error("无节点订阅 /initialpose, 无法后移 ego")
        node.destroy_publisher(pub)
        return False
    pub.publish(msg)
    # 瞬移有延迟 (CARLA 同步模式下一 tick 才落地), 循环等验证而非固定睡一次
    deadline = time.monotonic() + 10.0
    err = 1e9
    while time.monotonic() < deadline:
        time.sleep(0.5)
        ego2 = take_odom(node)
        if ego2 is None:
            continue
        err = math.hypot(ego2[0] - tx, ego2[1] - ty)
        if err <= 1.5:
            break
    node.destroy_publisher(pub)
    if err > 1.5:
        node.get_logger().error("后移未生效: 目标 (%.2f, %.2f), 偏差 %.2f m" % (tx, ty, err))
        return False
    node.get_logger().info("ego 已就位: (%.2f, %.2f) (偏差 %.2f m)" % (ego2[0], ego2[1], err))
    return True


def call_set_route(node, cli, gx, gy, yaw_deg):
    req = SetRoutePoints.Request()
    req.header.frame_id = "map"
    req.goal.position.x = gx
    req.goal.position.y = gy
    req.goal.position.z = 0.0
    yaw = math.radians(yaw_deg)
    req.goal.orientation.z = math.sin(yaw / 2.0)
    req.goal.orientation.w = math.cos(yaw / 2.0)
    req.option.allow_goal_modification = False
    # waypoints 留空: points = [ego, goal], 由 mission_planner 规划器规划
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    if not future.done():
        node.get_logger().error("set_route_points 服务无响应 (超时)")
        return None
    return future.result()


def main():
    p = argparse.ArgumentParser(description="规划并设置 mission_planning route, 验证发布")
    p.add_argument("--goal", required=True, help='目标点 map 系坐标 "x,y" (右手系, 与 trajectory.json 同系)')
    p.add_argument("--goal-yaw-deg", type=float, default=0.0,
                   help="goal 朝向 (度, 默认 0); 必须与道路方向一致, 否则 goal 校验失败")
    p.add_argument("--timeout", type=float, default=20.0, help="route 发布验证超时秒 (默认 20)")
    p.add_argument("--nudge-back-m", type=float, default=6.0,
                   help="route 为空时 ego 沿航向后移距离 m (默认 6, 0=关闭)")
    cli_args = p.parse_args()
    try:
        gx, gy = [float(v) for v in cli_args.goal.split(",")]
    except ValueError:
        sys.exit('--goal 格式错误, 应为 "x,y" (如 334.8,-238.8)')

    rclpy.init()
    node = Node("set_route_client")

    # 1) 调 /api/routing/set_route_points 服务 (失败时后移 ego 重试一次)
    cli = node.create_client(SetRoutePoints, "/api/routing/set_route_points")
    if not cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("等待服务 /api/routing/set_route_points 超时, 确认 e2e 已启动")
        sys.exit(1)

    # route 验证订阅提前建立: TRANSIENT_LOCAL 可收到"已存在的 route"消息
    got = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(LaneletRoute, "/planning/mission_planning/route",
                                   lambda m: got.append(m), qos)

    resp = None
    for attempt in range(10):
        resp = call_set_route(node, cli, gx, gy, cli_args.goal_yaw_deg)
        if resp is not None and resp.status.success:
            break
        if resp is not None and not resp.status.success and resp.status.code == 1:
            # code=1 有两种可能: 1) route 真已设置过(幂等) 2) mission_planner 未 ready
            # (state 还是 UNKNOWN, 例: odometry 未到)。用 route 话题消息区分。
            rclpy.spin_once(node, timeout_sec=0.5)   # 收集可能已存在的 transient route
            if got:
                node.get_logger().info("route 已存在 (code=1), 视为已满足, 无需重设")
                resp.status.success = True
                break
            node.get_logger().warn(
                "code=1 但 route 未发布 (planner 可能未 ready), 2s 后重试 (%d/10) ..."
                % (attempt + 1))
            time.sleep(2.0)
            continue
        if resp is not None and not resp.status.success and cli_args.nudge_back_m > 0.0:
            node.get_logger().warn("route 为空 (%s); ego 可能处于交叉口重叠区, 后移重试"
                                   % resp.status.message)
            if nudge_back(node, cli_args.nudge_back_m):
                continue
        break
    if resp is None or not resp.status.success:
        node.get_logger().error(
            "set_route_points 失败: %s" % ("服务超时" if resp is None
                                          else "code=%d, %s" % (resp.status.code, resp.status.message)))
        sys.exit(1)
    node.get_logger().info("set_route_points 服务成功 (goal=(%.1f, %.1f))" % (gx, gy))

    # 2) 验证 route 话题确实发布 (服务成功但 route 空的情况会被抓出)
    deadline = time.monotonic() + cli_args.timeout
    while time.monotonic() < deadline and not got:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not got:
        node.get_logger().error(
            "服务成功但 %.0fs 内未见 route 话题发布, 请检查 mission_planner 日志" % cli_args.timeout)
        sys.exit(1)

    route = got[0]
    node.get_logger().info(
        "route 已发布: %d 个 segment, goal=(%.1f, %.1f) — scenario_selector 即将解锁"
        % (len(route.segments), route.goal_pose.position.x, route.goal_pose.position.y))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
