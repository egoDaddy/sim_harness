#!/usr/bin/env python3
"""metrics_collector: 联合仿真 metrics 采集节点.

目前唯一指标: collision_speed_kph (ego 首次碰撞瞬间时速, km/h).

数据源:
  - 碰撞传感器: ego 传感器 kit 的标配 (mapping yaml sensor_mappings 声明,
    parse_sensor_kit.py 无条件追加进 ControllerCatalog.xosc, scenario_runner
    spawn ego 时附着, role_name=collision). 本节点发现式 listen (与 interface
    passive attach 同模式), 不自 spawn 不自 destroy.
    注: 系统中无 carla_ros_bridge actor 桥接, /carla/{ego}/collision topic
    无人发布, 因此不能走 ROS 订阅, 必须直连 CARLA listen.
  - 碰撞瞬间速度: 直连 CARLA 读 ego actor 瞬时速度向量换算 km/h, 只记录
    首次碰撞 (撞击瞬间, 后续弹跳不覆盖).

对外: 发布 /carla/srunner/metrics (std_msgs/String, JSON, TRANSIENT_LOCAL
latch), scenario_monitor 编排器在每场景完成时读取, 并入 run_summary.json 的
scenarios[i].metrics 字段.

循环模式复位: cycle="reset" 时清空指标 (monitor 收尾时已销毁 ego 及附着
传感器), 等下一场景 ego 出现重新发现 listen.

注意: 本节点用 wall-clock timer (不依赖 /clock), 与 npc_ready_trigger 同理.
"""
import json
import math
import threading

import carla
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String


class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_collector')
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('ego_role_name', 'ego_vehicle')
        self.declare_parameter('poll_rate_hz', 2.0)
        self.declare_parameter('cycle_topic', '/carla/srunner/cycle')
        self.declare_parameter('metrics_topic', '/carla/srunner/metrics')

        self.ego_role_name = self.get_parameter('ego_role_name').value
        self._metrics = {}
        self._lock = threading.Lock()
        self._ego = None
        self._collision_sensor = None
        # 退出事件: 与 scenario_monitor 同因, callback 内直接 rclpy.shutdown()
        # 会与 global executor 互相等待而死锁, 改为事件 + spin_once 循环退出
        self.finish_event = threading.Event()

        latch = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.metrics_pub = self.create_publisher(
            String, self.get_parameter('metrics_topic').value, latch)

        # 编排器循环信号: reset -> 清空本场景指标, 等下一场景 ego
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
        self.create_timer(1.0 / rate, self.poll_ego)
        self.get_logger().info('metrics_collector 启动: 等待 ego 出现以发现 kit 碰撞传感器 ...')

    # ------------------------------------------------------------ 指标采集
    def on_collision(self, event):
        """CARLA 碰撞回调 (CARLA 线程): 记录首次碰撞瞬间 ego 时速 (km/h)."""
        with self._lock:
            if 'collision_speed_kph' in self._metrics:
                return  # 只记录首次碰撞 (撞击瞬间)
            sensor = self._collision_sensor
            if sensor is None or not sensor.is_alive:
                return
            ego = sensor.parent
            if ego is None or not ego.is_alive:
                return
            try:
                vel = ego.get_velocity()
            except Exception:  # pylint: disable=broad-except
                return
            kph = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2) * 3.6
            self._metrics['collision_speed_kph'] = round(kph, 2)
            other = getattr(event, 'other_actor', None)
            self.get_logger().info(
                '记录碰撞: collision_speed_kph=%.2f (other_actor=%s)'
                % (kph, other.id if other else '?'))
        self._publish_metrics()

    def _publish_metrics(self):
        with self._lock:
            data = json.dumps(dict(self._metrics))
        msg = String()
        msg.data = data
        self.metrics_pub.publish(msg)

    # ------------------------------------------------------------ 循环复位
    def on_cycle(self, msg):
        """编排器循环信号: reset 时清空本场景指标 (ego/传感器已被 monitor 销毁)."""
        if msg.data == 'reset':
            with self._lock:
                self._metrics = {}
            self._collision_sensor = None
            self._ego = None
            self.get_logger().info('收到编排 reset: 清空本场景指标, 等待下一场景 ego')
            self._publish_metrics()

    # ------------------------------------------------------------ ego/传感器发现
    def poll_ego(self):
        """轮询发现 ego 及其 kit 标配碰撞传感器, 发现后 listen (passive attach).

        已 listen 的传感器死亡 (场景收尾被 destroy 或随 ego 销毁) 后自动
        重新发现; 碰撞传感器由 scenario_runner 按 catalog spawn, 本节点
        只 listen 不 spawn (与 interface 发现式 listen 同模式).
        """
        if self._collision_sensor is not None and self._collision_sensor.is_alive:
            return
        self._collision_sensor = None
        try:
            actors = self.world.get_actors()
            ego = None
            for v in actors.filter('vehicle.*'):
                if v.is_alive and v.attributes.get('role_name') == self.ego_role_name:
                    ego = v
                    break
            if ego is None:
                self._ego = None
                return
            self._ego = ego
            for s in actors.filter('sensor.other.collision'):
                if not s.is_alive:
                    continue
                parent = getattr(s, 'parent', None)
                if parent is not None and parent.id == ego.id:
                    s.listen(self.on_collision)
                    self._collision_sensor = s
                    self.get_logger().info('已 listen kit 碰撞传感器 (sensor id=%d, ego id=%d)'
                                           % (s.id, ego.id))
                    return
        except Exception as e:  # pylint: disable=broad-except
            self.get_logger().debug('轮询 actors 失败: %s' % e)


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollector()
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
