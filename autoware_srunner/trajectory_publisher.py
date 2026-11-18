#!/usr/bin/env python3
"""trajectory_publisher: 读取 trajectory.json, 10Hz 整帧发布到 MVP 注入话题。

用法 (需在 Autoware 同一 ROS2 环境内运行, 保证 autoware_planning_msgs 可用):
  source <autoware_ws>/install/setup.bash
  python3 trajectory_publisher.py \
      --file /home/carla_disk/carla_controller_ws/scenarios/generated_scenarios/trajectory.json \
      --ros-args -p use_sim_time:=true     # 可选: header.stamp 用仿真时钟 (与 /clock 对齐)

  # 循环执行模式 (需求 2/3): 用 --xosc 指定场景文件, 解析其 ParameterDeclarations
  # 中的 ego_trajectory_ref 声明, 加载同名 .json 轨迹 (替代 --file):
  python3 trajectory_publisher.py --xosc /path/to/npc_accel_multistep.xosc --no-route ...

行为:
  - 启动后先调用 set_route.py 设置 route (默认 goal=轨迹末点; --route-goal 可覆盖, --no-route 跳过)。
    这一步是必要的: 无 route 时 scenario_selector 停在 EMPTY, 会丢弃注入轨迹, 车辆不动;
  - 等待 TF (map -> base_link, 由 carla_state_publisher 广播) 可用;
  - 取到 ego 位姿后, 对整条轨迹做一次刚体变换, 使首点与 ego 当前位置+朝向重合 (首点锚定);
  - 之后按 --rate (默认 10Hz) 周期整帧发布, header.stamp 刷新为当前时刻, 锚定只做一次;
  - 轨迹点 {x, y, z, yaw, v} 为 map 右手系 (drive_straight.py 的输出口径)。

注入通道 (两条同时发, 缺一不可):
  - 主话题 (--topic, 默认 path_optimizer/trajectory): 经 scenario_selector ->
    /planning/scenario_planning/trajectory, 喂给 topic_state_monitor 保持 planning 诊断 OK;
  - controller 话题 (--controller-topic, 默认 velocity_smoother/trajectory):
    velocity_smoother 节点被注释时此话题无人发布, 直接注入可同时喂给
    controller_node_exe (其 input/reference_trajectory remap 到此) 和 planning_validator
    (其输出 /planning/trajectory 供 transition_manager 使用)。

自动切 autonomous:
  - 订阅 /system/operation_mode/availability, 一旦 autonomous 可用即调用
    /api/operation_mode/change_to_autonomous 服务 (只调一次, 失败退避重试),
    避免 operation mode 卡在 STOP 导致 pid 停在 STOPPED 输出驻车制动、车辆不动;
  - --no-autonomous 可关闭自动切换。

注意:
  - 发布前需停用 path_optimizer 节点 (motion_planning.launch.xml 注释掉 load_composable_node),
    避免双发布者;
  - 若需在运行中重新锚定, 加 --reanchor (仅调试用, 每周期重锚会导致轨迹跟随 ego 平移,
    最近点永远停在首点, 车辆无法前进)。
"""

"""
- 理论上（spawn_point = json 输入坐标、转换链正确）锚定量应为 0。但实际运行时几乎必然存在偏差：
实际 spawn 位置 ≠ 理论位置
CARLA spawn 时车辆从略高处落到地面（hybrid physics），落地位置与参数坐标有微小偏差；
之前调试中的真实数据：理论 map 系 (334.8, -14.2)，ego 实际停在 (334.8, -12.78)，差了 1.4 m（停止线/交叉口吸附问题）；
spawn 后车辆还可能被物理模拟轻微滚动。
转换链的配置漂移
trajectory.json 是静态文件，spawn_point 是 launch 参数，两者"一致"完全靠人工保证：drive_straight 输入改一个、e2e launch 改另一个没同步，就会错位；
y 翻转/yaw 取反的转换链上任何一环出错，轨迹会整体偏离。
ego 可能已经移动
publisher 的启动时机在 ego spawn 之后，若 ego 已移动（前一轮跑过、被推过），位置就偏离出生点。
锚定的本质：以运行时实测的 ego 位姿为准做对齐。偏差为 0 时锚定量就是 0（幂等无害）；有偏差时自动修正，系统对上述漂移免疫

- 刚性变换的具体操作：旋转 + 平移，不只是平移
即一个 SE(2) 刚体变换（绕首点旋转 + 平移），效果：
首点落到 ego 实际位置（x, y 对齐）
整条轨迹朝向与 ego 实际 yaw 对齐（旋转部分）
轨迹形状、点间距离、速度谱完全不变（刚体变换保距）
注意旋转是绕轨迹首点（先 p - p0 再旋转），不是绕坐标系原点——所以首点被精确固定在 ego 位姿上。
另外有个配套细节（L168-171）：锚定后首点恰在 ego 脚下，而 json 首点 v=0，controller 按最近点取目标速度会拿到 0 → 输出急刹、车辆不动。所以代码把速度谱前移一格（首点速度取原第二点的值），让车辆起步即有非零目标速度。
"""

import json
import math
import os
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformListener

from autoware_adapi_v1_msgs.msg import OperationModeState as AdapiOperationModeState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from tier4_system_msgs.msg import OperationModeAvailability

DEFAULT_FILE = "/home/carla_disk/carla_controller_ws/scenarios/generated_scenarios/trajectory.json"
DEFAULT_TOPIC = "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory"
DEFAULT_CONTROLLER_TOPIC = "/planning/scenario_planning/velocity_smoother/trajectory"


def resolve_trajectory_from_xosc(xosc_path):
    """解析场景 xosc 的 ego_trajectory_ref 声明 (需求 2/3), 返回同名 .json 轨迹路径.

    声明形式 (npc_accel_multistep.xosc ParameterDeclarations):
        <ParameterDeclaration name="ego_trajectory_ref" parameterType="string"
                              value="../ego/ego_trajectory_accel_multistep.xosc"/>
    声明仅作参考, scenario_runner 不执行; 本函数读它把 .xosc 换成同名 .json,
    路径相对场景 xosc 所在目录解析; 未声明返回 None.
    """
    import xml.etree.ElementTree as ET
    if not xosc_path or not os.path.isfile(xosc_path):
        return None
    try:
        root = ET.parse(xosc_path).getroot()
    except ET.ParseError:
        return None
    for pd in root.iter('ParameterDeclaration'):
        if pd.get('name') == 'ego_trajectory_ref':
            ref = pd.get('value', '')
            if not ref:
                return None
            full = os.path.normpath(os.path.join(
                os.path.dirname(os.path.abspath(xosc_path)), ref))
            return os.path.splitext(full)[0] + '.json'
    return None


def ensure_route(goal_xy, yaw_deg):
    """调用 set_route.py 设置 route (解锁 scenario_selector), 失败则终止。"""
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "set_route.py")
    cmd = [sys.executable, script, "--goal", "%.3f,%.3f" % goal_xy,
           "--goal-yaw-deg", "%.1f" % yaw_deg]
    print("[route] 运行: %s" % " ".join(cmd))
    ret = subprocess.run(cmd).returncode
    if ret != 0:
        sys.exit("set_route.py 失败 (exit %d), 终止发布; 可改用 --no-route 跳过" % ret)
    print("[route] route 已就绪, 等待 2s 让 scenario_selector 解锁 ...")
    time.sleep(2.0)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class TrajectoryPublisher(Node):
    def __init__(self, args):
        super().__init__("trajectory_publisher")
        self.args = args
        if not os.path.exists(args.file):
            sys.exit("找不到轨迹文件: %s" % args.file)
        with open(args.file) as f:
            raw = json.load(f)
        if not raw or not all(k in raw[0] for k in ("x", "y", "z", "yaw", "v")):
            sys.exit("trajectory.json 格式错误, 需要 {{x, y, z, yaw, v}} 数组")
        self.pts = [(p["x"], p["y"], p["z"], p["yaw"], p["v"]) for p in raw]

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Trajectory, args.topic, qos)
        self.pub_ctrl = None
        if not args.no_controller_topic:
            self.pub_ctrl = self.create_publisher(Trajectory, args.controller_topic, qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.anchor = None          # 刚体锚定变换 (dx, dy, dyaw), 只算一次
        self.wait_logged = False
        self.create_timer(1.0 / args.rate, self.tick)

        # 自动切 autonomous: 订阅 availability, 可用时调 change_to_autonomous 服务
        self.mode_client = self.create_client(
            ChangeOperationMode, "/api/operation_mode/change_to_autonomous")
        self.auto_requested = False
        self.auto_failed_count = 0
        self.last_attempt = None
        self.mode_reported = {}
        self.sub_availability = self.create_subscription(
            OperationModeAvailability, "/system/operation_mode/availability",
            self.on_availability, 1)
        self.sub_mode = self.create_subscription(
            AdapiOperationModeState, "/system/operation_mode/state", self.on_mode, 1)

        self.get_logger().info("加载 %d 个轨迹点 (总长 %.1f m)" % (
            len(self.pts), sum(math.hypot(
                self.pts[i][0] - self.pts[i - 1][0],
                self.pts[i][1] - self.pts[i - 1][1]) for i in range(1, len(self.pts)))))
        self.get_logger().info("发布: %s @ %.1f Hz (frame=map, reliable QoS)" % (args.topic, args.rate))

    def get_ego_pose(self):
        """TF 查 ego 位姿 (map -> base_link), 失败返回 None。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.parent_frame, self.args.child_frame, rclpy.time.Time())
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z, yaw_from_quat(q))

    def set_anchor(self):
        """首点锚定: 计算把轨迹首点平移到 ego 位姿的刚体变换, 只执行一次。"""
        ego = self.get_ego_pose()
        if ego is None:
            if not self.wait_logged:
                self.get_logger().warn("等待 TF %s -> %s ..." % (
                    self.args.parent_frame, self.args.child_frame))
                self.wait_logged = True
            return False
        x0, y0, _, yaw0, _ = self.pts[0]
        self.anchor = (ego[0] - x0, ego[1] - y0,
                       math.atan2(math.sin(ego[3] - yaw0), math.cos(ego[3] - yaw0)))
        self.get_logger().info(
            "锚定完成: ego=(%.2f, %.2f, yaw=%.1fdeg) -> 轨迹首点平移 (%.2f, %.2f) 旋转 %.2fdeg"
            % (ego[0], ego[1], math.degrees(ego[3]), self.anchor[0], self.anchor[1],
               math.degrees(self.anchor[2])))
        return True

    def build_msg(self):
        msg = Trajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.trajectory_frame
        dx, dy, dyaw = self.anchor
        c, s = math.cos(dyaw), math.sin(dyaw)
        t_acc = 0.0
        prev = None
        n = len(self.pts)
        for i, (x, y, z, yaw, v) in enumerate(self.pts):
            # 速度谱前移一格: 首点 v=0 (静止起步点) 经锚定后恰在 ego 脚下,
            # controller 按最近点取目标速度会得到 0 -> 输出急刹、车辆不动。
            # 前移后首点速度 = 原第二点速度 (>0), 车辆立即获得非零目标速度起步。
            v_eff = self.pts[i + 1][4] if i + 1 < n else v
            pt = TrajectoryPoint()
            # 刚体变换: 先按首点旋转平移, 再叠加 ego 位移
            px = x - self.pts[0][0]
            py = y - self.pts[0][1]
            pt.pose.position.x = px * c - py * s + self.pts[0][0] + dx
            pt.pose.position.y = px * s + py * c + self.pts[0][1] + dy
            pt.pose.position.z = z
            pt.pose.orientation = quat_from_yaw(yaw + dyaw)
            pt.longitudinal_velocity_mps = float(v_eff)
            if prev is not None:
                ds = math.hypot(pt.pose.position.x - prev.pose.position.x,
                                pt.pose.position.y - prev.pose.position.y)
                v_mean = max(0.5 * (v_eff + prev.longitudinal_velocity_mps), 0.5)
                t_acc += ds / v_mean
            prev = pt
            pt.time_from_start = Duration(sec=int(t_acc), nanosec=int((t_acc % 1) * 1e9))
            msg.points.append(pt)
        return msg

    def on_availability(self, msg):
        if self.args.no_autonomous or self.auto_requested:
            return
        if not msg.autonomous:
            return
        if self.last_attempt is not None and \
                (self.get_clock().now() - self.last_attempt).nanoseconds < 2e9:
            return   # 失败后退避 2s, 避免刷服务
        if not self.mode_client.wait_for_service(0.0):
            self.get_logger().warn("change_to_autonomous 服务未就绪, 稍后重试 ...")
            return
        self.auto_requested = True
        self.last_attempt = self.get_clock().now()
        req = ChangeOperationMode.Request()
        self.get_logger().info("autonomous 可用, 请求 change_to_autonomous ...")
        self.mode_client.call_async(req).add_done_callback(self.on_change_result)

    def on_change_result(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error("change_to_autonomous 调用失败: %s" % e)
            self.auto_requested = False
            return
        if not resp.status.success:
            self.auto_failed_count += 1
            self.auto_requested = False   # 允许 availability 回调退避重试
            if self.auto_failed_count % 10 == 1:
                self.get_logger().error(
                    "change_to_autonomous 被拒 (第 %d 次): %s"
                    % (self.auto_failed_count, resp.status.message))
            return
        self.get_logger().info("change_to_autonomous 成功, 等待 transition_manager 切模式 ...")

    def on_mode(self, msg):
        names = {0: "UNKNOWN", 1: "STOP", 2: "AUTONOMOUS", 3: "LOCAL", 4: "REMOTE"}
        if self.mode_reported.get(msg.mode) != msg.is_autoware_control_enabled:
            self.mode_reported[msg.mode] = msg.is_autoware_control_enabled
            self.get_logger().info(
                "operation mode = %s (enabled=%s)"
                % (names.get(msg.mode, str(msg.mode)), msg.is_autoware_control_enabled))

    def tick(self):
        if self.args.reanchor:
            self.anchor = None
        if self.anchor is None and not self.set_anchor():
            return
        msg = self.build_msg()
        self.pub.publish(msg)
        if self.pub_ctrl is not None:
            self.pub_ctrl.publish(msg)


def main():
    import argparse
    p = argparse.ArgumentParser(description="trajectory.json -> MVP 注入话题发布器")
    p.add_argument("--file", default=DEFAULT_FILE, help="trajectory.json 路径")
    p.add_argument("--xosc", default=None,
                   help="场景 xosc 路径: 解析其 ego_trajectory_ref 声明加载同名 "
                        ".json 轨迹 (需求 2/3, 优先于 --file)")
    p.add_argument("--topic", default=DEFAULT_TOPIC, help="注入话题 (MVP 输入)")
    p.add_argument("--controller-topic", default=DEFAULT_CONTROLLER_TOPIC,
                   help="controller 直注话题 (velocity_smoother 被注释时用)")
    p.add_argument("--no-controller-topic", action="store_true",
                   help="不向 controller 话题发布 (仅 --topic)")
    p.add_argument("--no-autonomous", action="store_true",
                   help="不自动请求 change_to_autonomous")
    p.add_argument("--rate", type=float, default=10.0, help="发布频率 Hz (默认 10)")
    p.add_argument("--parent-frame", default="map", help="TF 父帧 (默认 map)")
    p.add_argument("--child-frame", default="base_link", help="TF ego 帧 (默认 base_link)")
    p.add_argument("--trajectory-frame", default="map", help="轨迹消息 frame_id (默认 map)")
    p.add_argument("--reanchor", action="store_true",
                   help="每周期重新锚定 (仅调试, 正常使用勿开)")
    p.add_argument("--route-goal", default=None,
                   help='route 目标点 map 系 "x,y" (默认取轨迹前方约 100m 处)')
    p.add_argument("--no-route", action="store_true",
                   help="跳过 set_route (route 已设置好时用)")
    global args
    # 先剥离 ROS 参数 (--ros-args ...), 再交给 argparse; rclpy.init() 会自行消费 sys.argv 应用参数覆盖
    args = p.parse_args(remove_ros_args(sys.argv[1:]))

    # 需求 3: --xosc 解析场景声明 (ego_trajectory_ref) -> 同名 .json 轨迹文件
    if args.xosc:
        resolved = resolve_trajectory_from_xosc(args.xosc)
        if not resolved or not os.path.isfile(resolved):
            sys.exit("无法从场景声明解析轨迹: %s (ego_trajectory_ref 未声明或同名 json 不存在)"
                     % args.xosc)
        print("[trajectory] xosc 声明 -> 轨迹文件: %s" % resolved)
        args.file = resolved

    # 注入前先设置 route, 解锁 scenario_selector (无 route 时其丢弃轨迹, 车辆不动)
    if not args.no_route:
        if not os.path.exists(args.file):
            sys.exit("找不到轨迹文件: %s" % args.file)
        with open(args.file) as f:
            raw = json.load(f)
        yaw_deg = math.degrees(raw[0]["yaw"])   # goal 朝向需与道路一致, 取轨迹首点朝向
        if args.route_goal:
            gx, gy = [float(v) for v in args.route_goal.split(",")]
        else:
            # 默认 goal = 轨迹前方约 100m 处: route 只用于解锁 selector, goal 不必是终点;
            # 终点可能超出 lanelet 覆盖或跨多个路口, 近处点吸附更稳
            target_s = 100.0
            acc = 0.0
            gx, gy = raw[-1]["x"], raw[-1]["y"]
            for i in range(1, len(raw)):
                acc += math.hypot(raw[i]["x"] - raw[i - 1]["x"],
                                 raw[i]["y"] - raw[i - 1]["y"])
                if acc >= target_s:
                    gx, gy = raw[i]["x"], raw[i]["y"]
                    break
        ensure_route((gx, gy), yaw_deg)

    rclpy.init()
    node = TrajectoryPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
