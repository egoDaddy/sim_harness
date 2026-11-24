#!/usr/bin/env python3
"""CarlaLink: 带后台重试的 CARLA 连接 (旁路节点共用).

carla.Client 的构造函数内部自带一个重连窗口 (默认 10s), 服务器还在加载地图时
抛 "time-out of 10000ms while waiting for the simulator"。launch 与 CARLA 服务器
启动常常并行, 启动早期连不上是常态; 一次性 raise 会让节点退出 1, 其中
npc_ready_trigger 退出会使 /carla/srunner/npc_ready 永不 latch ->
注入链 wait_for_ready 干等到超时 -> 全程不注入轨迹 (ego 一动不动)。
改为后台线程重试, 与 scenario_monitor._connect_carla 的容错口径一致。
"""
import threading

import carla


class CarlaLink:
    """后台线程连接 CARLA; world 为 None 表示尚未连上 (调用方跳过本轮即可)."""

    def __init__(self, node, host='localhost', port=2000,
                 retry_s=2.0, timeout_s=10.0):
        self._node = node
        self.host = host
        self.port = int(port)
        self.retry_s = float(retry_s)
        self.timeout_s = float(timeout_s)
        self.client = None
        self.world = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='carla-link',
                                        daemon=True)

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()

    def _run(self):
        while not self._stop.is_set() and self.world is None:
            try:
                client = carla.Client(self.host, self.port)
                client.set_timeout(self.timeout_s)
                world = client.get_world()
            except Exception as e:  # pylint: disable=broad-except
                self._node.get_logger().warn(
                    'CARLA 连接失败, %.0fs 后重试: %s' % (self.retry_s, e))
                self._stop.wait(self.retry_s)
                continue
            self.client, self.world = client, world
            self._node.get_logger().info(
                'CARLA 连接成功 (%s:%d)' % (self.host, self.port))