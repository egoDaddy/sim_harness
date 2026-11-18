#!/usr/bin/env python3
"""auto_ready_trigger: 判定 autoware auto available, 写入门控标志文件.

方案 A 门控: scenario_runner 行为树的 Act StartTrigger 挂 WaitForFileFlag 原子,
每 tick 检查标志文件; 本节点在 "auto available" 时:
  1) 写入 SRUNNER_FLAG_DIR/auto_ready.flag (触发 Act 开闸, NPC 起步);
  2) 发布 /carla/srunner/auto_ready (std_msgs/Bool, TRANSIENT_LOCAL latch) 供观测.

循环执行模式: 本节点常驻, 每个场景写一次标志, 由编排器 (scenario_monitor) 的
/carla/srunner/cycle 信号复位推进:
  - cycle="reset"  -> 清 auto_ready.flag、复位触发状态、armed=False
                      (旧 autoware 已被编排器关闭, 此后不再采信 mode/engage 消息);
  - cycle="e2e_up"  -> 新 autoware 已启动就绪, armed=True 恢复监听;
  - armed 且 mode=AUTONOMOUS -> 写 flag (场景 N+1 开闸).

判据 (OR, 任一满足即触发):
  - 主判据: operation mode = AUTONOMOUS 且 autoware control enabled
    (订阅 /system/operation_mode/state; 由注入链的 trajectory_publisher 自动
     调 change_to_autonomous 切换到 AUTONOMOUS, 无需人工操作).
  - 备选判据: /autoware/engage 变 true (部分集成中由 rviz 手动 engage).

注意: 本集成中 ego 无 engage 也能起步 (控制不经 vehicle_cmd_gate 的 engage
门), /autoware/engage 可能永远不发 true, 故主判据用 AUTONOMOUS 而非 engage.

flag 目录经环境变量 SRUNNER_FLAG_DIR 与 scenario_runner 子进程共享
(默认 /tmp/srunner/flags); 本节点启动时清理目录内残留 *.flag, 防上一轮
残留导致本轮场景提前开闸 (scenario_runner 场景加载前也会再清一次, 双保险).

注意: 本节点只依赖 rclpy, 不 import carla, 无需 CARLA python 环境.
"""
import os
import threading

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from autoware_adapi_v1_msgs.msg import OperationModeState


class AutoReadyTrigger(Node):
    def __init__(self):
        super().__init__('auto_ready_trigger')
        self.declare_parameter('engage_topic', '/autoware/engage')
        self.declare_parameter('mode_state_topic', '/system/operation_mode/state')
        self.declare_parameter('ready_topic', '/carla/srunner/auto_ready')
        self.declare_parameter('cycle_topic', '/carla/srunner/cycle')

        self.flag_dir = os.environ.get('SRUNNER_FLAG_DIR', '/tmp/srunner/flags')
        self.flag_file = os.path.join(self.flag_dir, 'auto_ready.flag')

        # 启动即清理残留 flag (防上一轮残留导致本轮场景提前开闸)
        os.makedirs(self.flag_dir, exist_ok=True)
        for fname in os.listdir(self.flag_dir):
            if fname.endswith('.flag'):
                try:
                    os.remove(os.path.join(self.flag_dir, fname))
                except OSError:
                    pass

        latch = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(
            Bool, self.get_parameter('ready_topic').value, latch)

        self.create_subscription(Bool, self.get_parameter('engage_topic').value,
                                 self.on_engage, 1)
        self.create_subscription(
            OperationModeState, self.get_parameter('mode_state_topic').value,
            self.on_mode_state, 1)

        # 编排器循环信号: reset 复位并停止监听, e2e_up 恢复监听 (见 docstring)
        self.create_subscription(String, self.get_parameter('cycle_topic').value,
                                 self.on_cycle, 1)

        self.triggered = False
        self.armed = True   # 首轮场景默认监听 (尚无历史 reset)
        # 与 npc_ready_trigger 同因: callback 内直接 rclpy.shutdown() 会与
        # global executor 互相等待而死锁, 改为事件 + spin_once 循环退出
        self.finish_event = threading.Event()
        self.get_logger().info(
            'auto_ready_trigger 启动: 等待 %s=AUTONOMOUS 或 %s=true ...'
            % (self.get_parameter('mode_state_topic').value,
               self.get_parameter('engage_topic').value))

    def on_cycle(self, msg):
        """编排器循环信号: reset 复位并清 flag, e2e_up 恢复监听.

        reset 时旧 autoware 已被编排器关闭 (先杀 e2e 再发 reset), 此后不会再有
        旧 mode/engage 消息; armed 门控保证只采信新 autoware 的状态,
        防止残留 AUTONOMOUS 消息在新 autoware 就绪前提前写 flag 开闸.
        """
        if msg.data == 'reset':
            self.triggered = False
            self.armed = False
            try:
                if os.path.exists(self.flag_file):
                    os.remove(self.flag_file)
            except OSError as e:
                self.get_logger().warn('reset 清 flag 失败: %s' % e)
            ready = Bool()
            ready.data = False
            self.ready_pub.publish(ready)
            self.get_logger().info('收到编排 reset: 清门控标志, 停止监听 (等待 e2e_up)')
        elif msg.data == 'e2e_up':
            self.armed = True
            self.get_logger().info('收到编排 e2e_up: 新 autoware 就绪, 恢复监听 AUTONOMOUS')

    def on_mode_state(self, msg):
        # OperationModeState: mode 0=UNKNOWN 1=STOP 2=AUTONOMOUS 3=LOCAL 4=REMOTE
        if not self.armed:
            return
        if msg.mode == 2 and msg.is_autoware_control_enabled:
            self._trigger('operation mode = AUTONOMOUS')

    def on_engage(self, msg):
        if not self.armed:
            return
        if msg.data:
            self._trigger('engage=true')

    def _trigger(self, reason):
        if self.triggered:
            return
        self.triggered = True
        self.get_logger().info('auto available (%s) -> 写门控标志 %s'
                               % (reason, self.flag_file))
        try:
            with open(self.flag_file, 'w') as f:
                f.write('1')
        except OSError as e:  # pylint: disable=broad-except
            self.get_logger().error('写标志文件失败: %s' % e)
            self.triggered = False
            return
        ready = Bool()
        ready.data = True
        self.ready_pub.publish(ready)
        # 循环模式: 写 flag 后常驻, 等待编排器 cycle="reset" 后进入下一轮


def main(args=None):
    rclpy.init(args=args)
    node = AutoReadyTrigger()
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
