#!/usr/bin/env python3
"""npc_marker_publisher: 把 CARLA 世界中的 NPC (vehicle.* / walker.*) 可视化到 Autoware RViz.

- 只读 CARLA 状态, 绝不 tick: 同步模式 tick 单主 = autoware_carla_interface, 本节点为纯旁观者;
- CARLA -> ROS 坐标转换: y 取反 / yaw 取反 (与接口转换链一致);
- 发布 visualization_msgs/MarkerArray 到 /carla/npc_markers (frame_id=map), 与 lanelet2 同画面;
- 每帧 DELETEALL 后整帧重发, NPC 销毁/重生自然收敛;
- 造型参照 ego 车辆观感 (车身 + 车窗 + 车头指示 + 四轮), 颜色与 ego 区分 (默认橙色).
"""
import math

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from autoware_srunner.carla_link import CarlaLink


def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def rgba(values):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = [float(v) for v in values[:4]]
    return c


class NpcMarkerPublisher(Node):
    def __init__(self):
        super().__init__('npc_marker_publisher')
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('topic', '/carla/npc_markers')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('ego_role_names', ['ego_vehicle', 'hero', 'ego'])
        self.declare_parameter('vehicle_color', [1.0, 0.45, 0.10, 0.90])
        self.declare_parameter('vehicle_window_color', [0.10, 0.14, 0.20, 0.95])
        self.declare_parameter('vehicle_light_color', [1.0, 0.95, 0.60, 1.0])
        self.declare_parameter('wheel_color', [0.08, 0.08, 0.08, 1.0])
        self.declare_parameter('walker_color', [0.25, 0.85, 0.35, 0.90])

        self.frame_id = self.get_parameter('frame_id').value
        self.ego_role_names = set(self.get_parameter('ego_role_names').value)
        self.color_vehicle = rgba(self.get_parameter('vehicle_color').value)
        self.color_window = rgba(self.get_parameter('vehicle_window_color').value)
        self.color_light = rgba(self.get_parameter('vehicle_light_color').value)
        self.color_wheel = rgba(self.get_parameter('wheel_color').value)
        self.color_walker = rgba(self.get_parameter('walker_color').value)

        # CARLA 连接放后台线程重试 (启动早期服务器还在加载地图, 见 carla_link)
        self.link = CarlaLink(self, self.get_parameter('host').value,
                              self.get_parameter('port').value).start()

        self.pub = self.create_publisher(MarkerArray, self.get_parameter('topic').value, 10)
        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self.publish_markers)
        self.get_logger().info('npc_marker_publisher 启动: %s @ %.1f Hz (只读不 tick)'
                               % (self.get_parameter('topic').value, rate))

    @staticmethod
    def carla_to_ros(location, rotation):
        """CARLA(左手) -> ROS map(右手): y 取反, yaw 取反 (弧度)."""
        return (location.x, -location.y, location.z, -math.radians(rotation.yaw))

    def publish_markers(self):
        world = self.link.world
        if world is None:  # 尚未连上 CARLA (carla_link 后台重试中)
            return
        try:
            actors = world.get_actors()
        except Exception as e:  # pylint: disable=broad-except
            self.get_logger().warn('get_actors 失败: %s' % e)
            return

        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = self.frame_id
        delete_all.header.stamp = now
        delete_all.action = Marker.DELETEALL
        arr.markers.append(delete_all)

        for actor in list(actors.filter('vehicle.*')) + list(actors.filter('walker.*')):
            if not actor.is_alive:
                continue
            role = actor.attributes.get('role_name', '')
            if role in self.ego_role_names:
                continue
            base_id = actor.id * 16
            if 'vehicle' in actor.type_id:
                self.append_vehicle_markers(arr, actor, base_id, now)
            else:
                self.append_walker_markers(arr, actor, base_id, now)

        self.pub.publish(arr)

    def _base_marker(self, ns, mid, mtype, x, y, z, yaw, sx, sy, sz, col, now):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = now
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation = quat_from_yaw(yaw)
        m.scale.x = sx
        m.scale.y = sy
        m.scale.z = sz
        m.color = col
        return m

    def append_vehicle_markers(self, arr, vehicle, base_id, now):
        t = vehicle.get_transform()
        x, y, z, yaw = self.carla_to_ros(t.location, t.rotation)
        bb = vehicle.bounding_box
        length, width, height = 2.0 * bb.extent.x, 2.0 * bb.extent.y, 2.0 * bb.extent.z
        zc = z + bb.location.z   # bbox 中心高度 (CARLA 车辆 transform 位于地面)
        c, s = math.cos(yaw), math.sin(yaw)

        # 车身
        arr.markers.append(self._base_marker('npc_vehicles', base_id, Marker.CUBE,
                                             x, y, zc, yaw, length, width, height,
                                             self.color_vehicle, now))
        # 车窗 (深色, 略窄, 贴顶)
        arr.markers.append(self._base_marker('npc_vehicles', base_id + 1, Marker.CUBE,
                                             x, y, zc + height * 0.32, yaw,
                                             length * 0.5, width * 0.9, height * 0.38,
                                             self.color_window, now))
        # 车头指示块 (车前)
        fx = x + c * (length * 0.5 - 0.45)
        fy = y + s * (length * 0.5 - 0.45)
        arr.markers.append(self._base_marker('npc_vehicles', base_id + 2, Marker.CUBE,
                                             fx, fy, zc, yaw, 0.35, width * 0.55, 0.28,
                                             self.color_light, now))
        # 四轮
        part = base_id + 3
        for dx, dy in ((-(length * 0.5 - 0.8), -(width * 0.5 + 0.02)),
                       (-(length * 0.5 - 0.8), +(width * 0.5 + 0.02)),
                       (+(length * 0.5 - 0.8), -(width * 0.5 + 0.02)),
                       (+(length * 0.5 - 0.8), +(width * 0.5 + 0.02))):
            wx = x + c * dx - s * dy
            wy = y + s * dx + c * dy
            arr.markers.append(self._base_marker('npc_vehicles', part, Marker.CUBE,
                                                 wx, wy, zc - height * 0.5 + 0.3, yaw,
                                                 0.75, 0.22, 0.6, self.color_wheel, now))
            part += 1

    def append_walker_markers(self, arr, walker, base_id, now):
        t = walker.get_transform()
        x, y, z, yaw = self.carla_to_ros(t.location, t.rotation)
        bb = walker.bounding_box
        height = 2.0 * bb.extent.z
        radius = max(bb.extent.x, bb.extent.y, 0.2)
        zc = z + bb.location.z

        # 身体 (圆柱) + 头 (球)
        arr.markers.append(self._base_marker('npc_walkers', base_id, Marker.CYLINDER,
                                             x, y, zc - height * 0.15, 0.0,
                                             radius * 2.0, radius * 2.0, height * 0.7,
                                             self.color_walker, now))
        arr.markers.append(self._base_marker('npc_walkers', base_id + 1, Marker.SPHERE,
                                             x, y, zc + height * 0.5 - 0.05, 0.0,
                                             0.36, 0.36, 0.36, self.color_walker, now))


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(NpcMarkerPublisher())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
