#!/usr/bin/env python3
"""npc_ready_trigger: 判定 scenario_runner 场景 Init 完成 (NPC 就位), 发布 latch 触发信号.

循环执行模式: 本节点常驻, 每个场景触发一次, 由编排器 (scenario_monitor) 的
/carla/srunner/cycle 信号复位推进:
  - 场景就位 -> 发布 npc_ready=True (注入链 wait_for_ready 放行);
  - cycle="reset" -> 发布 npc_ready=False 并复位 triggered,
    等待下一场景 execute_scenario 后的新 NPC.

判据 (任一满足即触发):
  1) /scenario_runner/status 进入 RUNNING (ScenarioManager 已开跑 -> Init 已结束, NPC 已生成);
  2) CARLA 世界轮询到非 ego 的 vehicle.*/walker.* actor (不依赖 ROS 状态话题, 兜底).

发布 /carla/srunner/npc_ready (std_msgs/Bool, TRANSIENT_LOCAL latch),
作为 set_route.py / trajectory_publisher.py 的触发信号 (需求 3c).
注意: 本节点用 wall-clock timer (不依赖 /clock), e2e 启动早期即可轮询.
"""
import rclpy
import threading

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String

from carla_ros_scenario_runner_types.msg import CarlaScenarioRunnerStatus

import carla


class NpcReadyTrigger(Node):
    def __init__(self):
        super().__init__('npc_ready_trigger')
        self.declare_parameter('status_topic', '/scenario_runner/status')
        self.declare_parameter('ready_topic', '/carla/srunner/npc_ready')
        self.declare_parameter('poll_rate_hz', 2.0)
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('ego_role_names', ['ego_vehicle', 'hero', 'ego'])
        self.declare_parameter('cycle_topic', '/carla/srunner/cycle')

        self.ego_role_names = set(self.get_parameter('ego_role_names').value)
        self.triggered = False
        # 退出事件: 与 scenario_monitor 同因, callback 内直接 rclpy.shutdown()
        # 会与 global executor 互相等待而死锁, 改为事件 + spin_once 循环退出
        self.finish_event = threading.Event()

        latch = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(
            Bool, self.get_parameter('ready_topic').value, latch)

        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CarlaScenarioRunnerStatus,
                                 self.get_parameter('status_topic').value,
                                 self.on_status, status_qos)

        # 编排器循环信号: reset -> 复位触发状态并发布 False, 进入下一轮等待
        self.create_subscription(String,
                                 self.get_parameter('cycle_topic').value,
                                 self.on_cycle, 1)

        try:
            self.client = carla.Client(self.get_parameter('host').value,
                                       self.get_parameter('port').value)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
        except Exception as e:  # pylint: disable=broad-except
            self.get_logger().error('CARLA 连接失败: %s' % e)
            raise

        rate = float(self.get_parameter('poll_rate_hz').value)
        self.create_timer(1.0 / rate, self.poll_actors)
        self.get_logger().info('npc_ready_trigger 启动: 等待场景 Init 完成 (NPC 就位) ...')

    def on_status(self, msg):
        if msg.status == CarlaScenarioRunnerStatus.RUNNING:
            self.fire('scenario_runner status=RUNNING')

    def on_cycle(self, msg):
        """编排器循环信号: reset 时复位触发状态并发布 False (下一轮注入链需重新等 True)."""
        if msg.data == 'reset':
            self.triggered = False
            reset = Bool()
            reset.data = False
            self.ready_pub.publish(reset)
            self.get_logger().info('收到编排 reset: 复位触发状态, 等待下一场景 NPC 就位')

    def poll_actors(self):
        if self.triggered:
            return
        try:
            actors = self.world.get_actors()
            for actor in list(actors.filter('vehicle.*')) + list(actors.filter('walker.*')):
                if not actor.is_alive:
                    continue
                role = actor.attributes.get('role_name', '')
                if role in self.ego_role_names:
                    continue
                self.fire('CARLA 检测到 NPC actor id=%d type=%s role=%s'
                          % (actor.id, actor.type_id, role or '-'))
                break
        except Exception as e:  # pylint: disable=broad-except
            self.get_logger().debug('轮询 actors 失败: %s' % e)

    def fire(self, reason):
        if self.triggered:
            return
        self.triggered = True
        self.get_logger().info('场景 Init 完成 (NPC 就位): %s' % reason)
        msg = Bool()
        msg.data = True
        self.ready_pub.publish(msg)
        # 循环模式: 发布后常驻, 等待编排器 cycle="reset" 后进入下一轮


def main(args=None):
    rclpy.init(args=args)
    node = NpcReadyTrigger()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.finish_event.is_set():
            executor.spin_once(timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
