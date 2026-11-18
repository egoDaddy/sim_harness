#!/usr/bin/env python3
"""sensor_fault_injector: 传感器故障注入常驻节点 (点云话题中间件).

架构 (算法层 / 编排层分离):
  算法层: 4 种故障数学模型 (移植 kitti fault_injector.cpp, numpy 向量化):
      dropout / noise / dead_zone / intensity, 逐点独立变换 (统计可交换).
  编排层: 多实例状态机——每组注入实例独立管理 trigger (speed 门控) 与
      duration (仿真时钟计时), 每帧对 active 实例依次应用, 各自独立 RNG.

数据链路 (话题顶替, 感知侧零改动):
  interface 发布 <topic_base>/pointcloud_before_sync_raw
      -> 本节点订阅 raw, 篡改或透传
      -> 本节点顶替发布 <topic_base>/pointcloud_before_sync (原话题名)
      -> crop_box_filter_self / 感知链 (订阅原话题名, 无感知)

configure 服务 (/fault_injection/configure): 一次调用原子替换全部实例;
groups 为空 = 关闭全部故障 (纯透传). 校验失败拒绝整个请求并保持现有实例.

参数校验统一补全 (kitti 缺失 noise/dead_zone 范围校验的历史教训):
  dropout_rate/intensity_rate ∈ [0,1]; noise_stddev >= 0; dead_zone min<=max;
  sensor 必填且受支持; duration >= 0.
"""
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from autoware_srunner_msgs.msg import FaultGroup
from autoware_srunner_msgs.srv import ConfigureFault

FAULT_TYPES = ('dropout', 'noise', 'dead_zone', 'intensity')
TRIGGER_TYPES = ('none', 'speed')

# intensity 字段 datatype (sensor_msgs/PointField): UINT8=2, FLOAT32=7
_DT_UINT8 = 2
_DT_FLOAT32 = 7


class _Instance:
    """单个故障注入实例 (编排层最小单元): 参数 + trigger/duration 状态 + 独立 RNG."""

    def __init__(self, group, rng):
        self.name = group.name
        self.type = group.type
        self.sensor = group.sensor
        self.trigger_type = group.trigger_type
        self.trigger_value = group.trigger_value
        self.duration = group.duration
        self.rng = rng
        # 参数快照 (按 type 取用)
        self.dropout_rate = group.dropout_rate
        self.noise_mean = group.noise_mean
        self.noise_stddev = group.noise_stddev
        self.dead_zone = (group.dead_zone_x_min, group.dead_zone_x_max,
                          group.dead_zone_y_min, group.dead_zone_y_max,
                          group.dead_zone_z_min, group.dead_zone_z_max)
        self.intensity_rate = group.intensity_rate
        # trigger none -> configure 即 active; speed -> 等速度门控
        self.state = 'active' if group.trigger_type == 'none' else 'armed'
        self.t0 = None          # 触发时刻 (仿真时钟秒), duration 计时基准


class SensorFaultInjector(Node):
    def __init__(self):
        super().__init__('sensor_fault_injector')
        self.declare_parameter('raw_topic',
                               '/sensing/lidar/top/pointcloud_before_sync_raw')
        self.declare_parameter('faulted_topic',
                               '/sensing/lidar/top/pointcloud_before_sync')
        self.declare_parameter('configure_service', '/fault_injection/configure')
        self.declare_parameter('kinematic_state_topic', '/localization/kinematic_state')
        self.declare_parameter('supported_sensors', ['top'])
        self.declare_parameter('base_seed', 42)
        self.declare_parameter('qos_depth', 10)
        # use_sim_time 无需显式声明: rcl 层每个节点自动声明该参数
        # (决定 clock 数据源), launch params-file 传入的值在节点构造时已生效;
        # 显式再声明会抛 ParameterAlreadyDeclaredException

        raw_topic = self.get_parameter('raw_topic').value
        faulted_topic = self.get_parameter('faulted_topic').value
        self._supported = list(self.get_parameter('supported_sensors').value)
        self._base_seed = int(self.get_parameter('base_seed').value)
        depth = int(self.get_parameter('qos_depth').value)

        # QoS 与 interface 发布侧对齐 (mapping qos_profile: best_effort)
        qos = QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self._sub = self.create_subscription(
            PointCloud2, raw_topic, self._on_pointcloud, qos)
        self._pub = self.create_publisher(PointCloud2, faulted_topic, qos)
        self._cfg_srv = self.create_service(
            ConfigureFault, self.get_parameter('configure_service').value,
            self._on_configure)

        # 速度门控数据源 (e2e 每场景重启, 订阅常驻; 无消息时实例停在 armed)
        self._kinematic_sub = self.create_subscription(
            Odometry, self.get_parameter('kinematic_state_topic').value,
            self._on_kinematic, QoSProfile(depth=1))
        self._latest_speed = 0.0

        # 编排状态: 实例列表 (configure 原子替换) + 实例序号 (RNG seed 派生)
        self._instances = []
        self._instance_seq = 0

        self.get_logger().info(
            '故障注入节点启动: %s -> %s (透传中, 等待 configure)'
            % (raw_topic, faulted_topic))

    # ------------------------------------------------------------ ROS 回调
    def _on_kinematic(self, msg):
        v = msg.twist.twist.linear
        self._latest_speed = float(np.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2))

    def _on_pointcloud(self, msg):
        """每帧: 编排实例状态 -> 依次应用 active 实例 -> 发布."""
        now = self._now_sec()
        if self._instances:
            self._tick_instances(now)
        if not self._instances or not any(i.state == 'active'
                                          for i in self._instances):
            # 无实例或全部处于 armed (未触发): 纯透传 (header 原样)
            self._pub.publish(msg)
            return
        try:
            out = self._inject(msg)
        except Exception as e:  # pylint: disable=broad-except
            # 篡改失败不阻断数据流: 原样透传 (与 kitti 一致)
            self.get_logger().error('故障注入失败, 透传原帧: %s' % e)
            self._pub.publish(msg)
            return
        out.header = msg.header
        self._pub.publish(out)

    def _on_configure(self, req, resp):
        """原子替换全部实例; 任一校验失败拒绝整个请求且保持现有实例."""
        if not req.groups:
            n = len(self._instances)
            self._instances = []
            resp.success = True
            resp.message = '关闭全部故障 (off), 清除 %d 个实例' % n
            self.get_logger().info(resp.message)
            return resp
        new_instances = []
        for i, group in enumerate(req.groups):
            err = self._validate(group)
            if err is not None:
                resp.success = False
                resp.message = '第 %d 组 (%s) 校验失败: %s; 请求整体拒绝' % (
                    i, group.name or group.type, err)
                self.get_logger().warn(resp.message)
                return resp
            rng = np.random.default_rng(self._base_seed + self._instance_seq)
            new_instances.append(_Instance(group, rng))
            self._instance_seq += 1
        self._instances = new_instances
        resp.success = True
        resp.message = '配置 %d 个注入实例' % len(new_instances)
        self.get_logger().info(resp.message)
        return resp

    # ------------------------------------------------------------ 编排层
    def _now_sec(self):
        """当前仿真时间 (秒); 节点设 use_sim_time 时即 /clock."""
        t = self.get_clock().now()
        return t.nanoseconds / 1e9

    def _tick_instances(self, now):
        """推进实例状态: armed 按速度门控转 active; active 按 duration 关断.

        duration 计时基准: trigger none 的实例以首次帧回调时刻为 t0
        (configure 后第一帧, 误差 < 一个帧周期); speed 触发实例以触发时刻为 t0。
        """
        alive = []
        for inst in self._instances:
            if inst.state == 'armed':
                if (inst.trigger_type == 'speed'
                        and self._latest_speed > inst.trigger_value):
                    inst.state = 'active'
                    inst.t0 = now
                    self.get_logger().info(
                        '实例 %s 触发 (speed %.2f > %.2f m/s)'
                        % (inst.name, self._latest_speed, inst.trigger_value))
            elif inst.duration > 0.0:
                if inst.t0 is None:
                    inst.t0 = now  # active 且未计时: 以首帧为 duration 起点
                elif now - inst.t0 >= inst.duration:
                    self.get_logger().info(
                        '实例 %s 关断 (duration %.1fs 到期)'
                        % (inst.name, inst.duration))
                    continue  # 到期关断: 移出实例列表
            alive.append(inst)
        self._instances = alive

    # ------------------------------------------------------------ 算法层
    def _inject(self, msg):
        """对当前帧依次应用全部 active 实例 (输入输出均为 PointCloud2)."""
        buf, field_off = _pc2_to_buf(msg)
        for inst in self._instances:
            if inst.state != 'active':
                continue
            buf = _apply_fault(buf, field_off, inst)
        return _buf_to_pc2(buf, field_off, msg)

    # ------------------------------------------------------------ 校验层
    def _validate(self, g):
        if not g.type or g.type not in FAULT_TYPES:
            return '未知故障类型: %r' % g.type
        if not g.sensor or g.sensor.strip() == '':
            return 'sensor 字段缺失'
        if g.sensor not in self._supported:
            return 'sensor %r 不受支持 (支持列表: %s)' % (g.sensor, self._supported)
        if g.trigger_type not in TRIGGER_TYPES:
            return '未知 trigger_type: %r' % g.trigger_type
        if g.trigger_type == 'speed' and g.trigger_value <= 0.0:
            return 'trigger_type=speed 时 trigger_value 必须 > 0'
        if g.duration < 0.0:
            return 'duration 必须 >= 0'
        if g.type == 'dropout' and not 0.0 <= g.dropout_rate <= 1.0:
            return 'dropout_rate 必须在 [0,1]'
        if g.type == 'noise' and g.noise_stddev < 0.0:
            return 'noise_stddev 必须 >= 0'
        if g.type == 'dead_zone':
            pairs = ((g.dead_zone_x_min, g.dead_zone_x_max, 'x'),
                     (g.dead_zone_y_min, g.dead_zone_y_max, 'y'),
                     (g.dead_zone_z_min, g.dead_zone_z_max, 'z'))
            for lo, hi, ax in pairs:
                if lo > hi:
                    return 'dead_zone %s 边界 min > max' % ax
        if g.type == 'intensity' and not 0.0 <= g.intensity_rate <= 1.0:
            return 'intensity_rate 必须在 [0,1]'
        return None


# ------------------------------------------------------------------ 点云存取
def _pc2_to_buf(msg):
    """PointCloud2 -> (N, point_step) uint8 buffer + 字段偏移表.

    字段表: {'x': (offset, FLOAT32), 'y': ..., 'z': ..., 'intensity': (offset, dt) 或 None}
    """
    fields = {}
    for f in msg.fields:
        fields[f.name] = (f.offset, f.datatype)
    need = ['x', 'y', 'z']
    missing = [n for n in need if n not in fields]
    if missing:
        raise ValueError('点云缺少字段: %s' % missing)
    n = msg.width * msg.height
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
    return buf, fields


def _buf_to_pc2(buf, fields, src_msg):
    """buffer -> 新 PointCloud2 (保留原 is_dense/layout, 更新 width/row_step/data)."""
    out = PointCloud2()
    out.header = src_msg.header
    out.height = 1
    out.width = buf.shape[0]
    out.fields = src_msg.fields
    out.is_bigendian = src_msg.is_bigendian
    out.point_step = src_msg.point_step
    out.row_step = buf.shape[0] * src_msg.point_step
    out.is_dense = src_msg.is_dense
    out.data = np.ascontiguousarray(buf).tobytes()
    return out


def _xyz_of(buf, fields):
    """提取 (N,3) float32 xyz 视图拷贝."""
    n = buf.shape[0]
    xyz = np.empty((n, 3), dtype=np.float32)
    for i, ax in enumerate(('x', 'y', 'z')):
        off = fields[ax][0]
        xyz[:, i] = buf[:, off:off + 4].copy().view(np.float32).reshape(-1)
    return xyz


def _set_xyz(buf, fields, xyz):
    """xyz 写回 buffer 对应字段."""
    for i, ax in enumerate(('x', 'y', 'z')):
        off = fields[ax][0]
        buf[:, off:off + 4] = np.ascontiguousarray(
            xyz[:, i], dtype=np.float32).view(np.uint8).reshape(-1, 4)


def _apply_fault(buf, fields, inst):
    """对一帧应用单个实例的故障模型 (逐点独立变换, 统计可交换)."""
    if inst.type == 'dropout':
        n = buf.shape[0]
        keep = inst.rng.random(n) > inst.dropout_rate
        return np.ascontiguousarray(buf[keep])

    if inst.type == 'noise':
        xyz = _xyz_of(buf, fields)
        n = xyz.shape[0]
        xyz += inst.rng.normal(inst.noise_mean, inst.noise_stddev,
                               size=(n, 3)).astype(np.float32)
        _set_xyz(buf, fields, xyz)
        return buf

    if inst.type == 'dead_zone':
        xyz = _xyz_of(buf, fields)
        (xmin, xmax, ymin, ymax, zmin, zmax) = inst.dead_zone
        inside = ((xyz[:, 0] >= xmin) & (xyz[:, 0] <= xmax)
                  & (xyz[:, 1] >= ymin) & (xyz[:, 1] <= ymax)
                  & (xyz[:, 2] >= zmin) & (xyz[:, 2] <= zmax))
        return np.ascontiguousarray(buf[~inside])

    # intensity: 以 rate 概率替换强度 (UINT8 -> U(0,256); FLOAT32 -> U(0,1))
    if 'intensity' in fields:
        off, dt = fields['intensity']
        n = buf.shape[0]
        hit = inst.rng.random(n) < inst.intensity_rate
        if not hit.any():
            return buf
        if dt == _DT_UINT8:
            col = buf[:, off:off + 1]
            col[hit] = inst.rng.integers(0, 256, size=hit.sum(),
                                         dtype=np.uint8).reshape(-1, 1)
        elif dt == _DT_FLOAT32:
            col = buf[:, off:off + 4].copy().view(np.float32).reshape(-1, 1)
            col[hit] = inst.rng.random(hit.sum()).astype(np.float32).reshape(-1, 1)
            buf[:, off:off + 4] = np.ascontiguousarray(
                col, dtype=np.float32).view(np.uint8).reshape(-1, 4)
        else:
            raise ValueError('intensity 字段 datatype %d 不受支持' % dt)
        return buf
    raise ValueError('intensity 故障但点云无 intensity 字段')


def main(args=None):
    rclpy.init(args=args)
    node = SensorFaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
