#!/usr/bin/env python3
"""scenario_monitor: 循环执行模式编排器 (需求 1/5/6/7 的编排中枢).

职责 (单进程串行编排整个多场景循环):
  1) 读场景列表 (auto: fault_matrix 非空时以矩阵条目驱动, 否则
     scenarios/xosc_list.json; manual: 初始为空, execute_batch 服务按需入队)
     与 autoware_srunner.yaml (编排配置);
  2) 每场景用 subprocess 拉起 autoware e2e (需求 7: autoware 每场景 relaunch);
  3) e2e 路由服务就绪后发布 /carla/srunner/cycle="e2e_up" (循环触发器恢复监听);
  4) 拉起 ego 注入链子进程 (wait_for_ready -> set_route -> trajectory_publisher);
  5) 故障注入: 队列项故障组经 /fault_injection/configure 注入 (场景开始前),
     场景收尾 configure 空组 off (防跨场景残留);
  6) 调用 /scenario_runner/execute_scenario 服务执行当前 xosc
     (scenario_runner 进程常驻, 每轮服务调用 = 一个完整 xosc 生命周期, 需求 1);
  7) 订阅 /scenario_runner/scenario_finished (逐场景完成状态通道, 需求 6):
     场景完成 -> 杀注入链 -> SIGINT e2e 等退出 -> 故障 off -> 销毁 CARLA actors ->
     cycle="reset" -> 下一场景;
  8) auto:   全部场景执行完毕退出进程 (exit 0/2/3 分层), launch 侧
     OnProcessExit 触发 Shutdown;
     manual: 队列执行完回 idle 常驻 (monitor 不退出), 等待 execute_batch
     指令 (需求 21 on_demand); 单个队列项出错记 ERROR 行后继续队列;
     仅 SIGINT/SIGTERM 时收尾退出;
  9) 收到 SIGINT/SIGTERM (手动 Ctrl+C) 同样先杀子进程再销毁 actors 后退出.

时序保证 (单线程状态机, 每 0.2s tick 一步, 全部步骤串行无竞态):
  起 e2e -> 路由服务就绪 -> e2e_up -> 起注入链 -> execute_scenario ->
  场景完成 (scenario_finished) -> 杀注入链 -> 杀 e2e -> destroy actors ->
  reset -> 下一队列项 / 回 idle -> ...

trigger 循环模式配套 (npc_ready_trigger / auto_ready_trigger):
  cycle="reset"  -> npc: 复位并发布 npc_ready=False; auto: 清 flag + 停止监听;
  cycle="e2e_up" -> auto: 恢复监听新 autoware 的 AUTONOMOUS.
"""
import ctypes
import json
import os
import signal
import subprocess
import sys
import threading
import time

import carla
import rclpy
import yaml
from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from carla_ros_scenario_runner_types.msg import CarlaScenarioRunnerScenarioFinished
from carla_ros_scenario_runner_types.srv import ExecuteScenario
from autoware_srunner_msgs.msg import FaultGroup
from autoware_srunner_msgs.srv import ConfigureFault, ExecuteBatch

# 编排状态机
S_IDLE = 'idle'
S_START_E2E = 'start_e2e'
S_WAIT_E2E_READY = 'wait_e2e_ready'
S_WAIT_FINISH = 'wait_finish'
S_KILL_INJECT = 'kill_inject'
S_KILL_E2E = 'kill_e2e'
# 旧 e2e 的 routing 服务从 DDS 消失前不得开下一条: 否则 _routing_service_ready()
# 会被残留服务骗过, e2e 没起来就 execute_scenario -> result=4 秒退
S_WAIT_E2E_GONE = 'wait_e2e_gone'
S_DESTROY = 'destroy'
S_NEXT = 'next'


def _pdeathsig():
    """子进程随 monitor 死亡 (PDEATHSIG=SIGKILL): launch 强杀 monitor 时,
    注入链/e2e 不残留孤儿 (bash 非交互模式会忽略 SIGINT, killpg 不可靠).
    preexec_fn 内仅调用 prctl(异步信号安全), 多线程进程 fork 风险可控."""
    try:
        ctypes.CDLL(None, use_errno=True).prctl(1, 9)  # PR_SET_PDEATHSIG=1, SIGKILL=9
    except Exception:  # pylint: disable=broad-except
        pass


class ScenarioMonitor(Node):
    def __init__(self):
        super().__init__('scenario_monitor')
        self.declare_parameter('asr_config', '')
        self.declare_parameter('e2e_pythonpath', '')
        self.declare_parameter('e2e_ld_preload', '')
        self.declare_parameter('results_dir', '')
        self.declare_parameter('metrics_topic', '/carla/srunner/metrics')
        # 3a: 编排控制参数 (launch 已按 CLI > yaml 覆盖语义解析为最终值,
        # 本节点不再回退 yaml; 直接 ros2 run 时为空则走都空报错)
        self.declare_parameter('scenario_root', '')
        self.declare_parameter('fault_root', '')
        self.declare_parameter('xosc_list', '')
        self.declare_parameter('fault_matrix', '')
        self.declare_parameter('orchestration', 'auto')

        # e2e 子进程环境恢复值 (launch 侧传入的用户 shell 原始环境,
        # 使 e2e 子进程环境与改造前 IncludeLaunchDescription 继承的环境一致,
        # 避免 monitor 自身 GroupAction 注入的 CARLA/anaconda 路径污染 autoware)
        self.e2e_pythonpath = self.get_parameter('e2e_pythonpath').value
        self.e2e_ld_preload = self.get_parameter('e2e_ld_preload').value

        # ---- 读编排配置 (yaml + xosc_list.json) ----
        cfg_file = self.get_parameter('asr_config').value
        if not cfg_file or not os.path.isfile(cfg_file):
            raise RuntimeError('编排配置文件不存在: %s' % cfg_file)
        with open(cfg_file) as f:
            cfg = yaml.safe_load(f)
        self.ws = cfg['workspace']['root']
        self.env_cfg = cfg['environment']
        self.aw = cfg['autoware']
        self.inj = cfg['ego_injection']
        self.lc = cfg['lifecycle']
        self.cyc = self.lc.get('cycle', {})
        self.ego_role_name = cfg.get('ego_role_name', 'ego_vehicle')
        # 故障注入配置 (矩阵加载/校验与 configure 服务共用, 必须先于矩阵加载赋值)
        self.fi_cfg = cfg.get('sensor_fault_injector', {})

        # 资产根: harness 对场景库/故障库的唯一入口 (CLI > yaml > 旧位置)
        p_cfg = cfg.get('paths', {}) or {}
        self.scenario_root = (
            str(self.get_parameter('scenario_root').value or '').strip()
            or p_cfg.get('scenario_root')
            or os.path.join(self.ws, 'scenarios'))
        self.fault_root = (
            str(self.get_parameter('fault_root').value or '').strip()
            or p_cfg.get('fault_root')
            or os.path.join(self.ws, 'fault'))
        # 结果根: 判定 JSON 与 run_summary.json 的落盘目录 (CI 从这里取结论)
        self.results_dir = (
            str(self.get_parameter('results_dir').value or '').strip()
            or p_cfg.get('results_root')
            or os.path.join(self.ws, 'scenarios', 'results'))

        # 3a: orchestration 模式校验 (需求 21: auto=auto_loop, manual=on_demand;
        # 值由 launch 参数传入, CLI > yaml 覆盖语义在 launch 侧解析)
        orch = str(self.get_parameter('orchestration').value).strip()
        if orch not in ('auto', 'manual'):
            raise RuntimeError(
                'sensor_fault_injector.orchestration=%r 未实现'
                ' (仅支持 auto=auto_loop / manual=on_demand)' % orch)
        self.orch = orch

        # 3a: 场景列表来源互斥校验 (需求 15): 二选一互斥, auto 模式都空报错.
        # 参数值由 launch 按 CLI > yaml 覆盖语义解析为最终值 (此处不回退 yaml);
        # 校验发生在任何子进程拉起之前 -> monitor 退出即触发 launch Shutdown,
        # 不存在 e2e/trajectory_publisher 残留 (强于需求 '先关闭再退出')
        xosc_list_path = str(self.get_parameter('xosc_list').value or '').strip()
        fault_matrix_path = str(self.get_parameter('fault_matrix').value or '').strip()
        if xosc_list_path and fault_matrix_path:
            raise RuntimeError(
                'xosc_list 与 fault_matrix 不能同时配置 (scenario 来源互斥): '
                'xosc_list=%r fault_matrix=%r' % (xosc_list_path, fault_matrix_path))
        # 相对路径一律按资产根解析 (多仓分离: 场景列表/矩阵随场景库一起搬走)
        xosc_list_path = self._resolve_input_path(
            xosc_list_path, [self.scenario_root]) if xosc_list_path else ''
        fault_matrix_path = self._resolve_input_path(
            fault_matrix_path,
            [self.fault_root, os.path.join(self.fault_root, 'matrix')]) \
            if fault_matrix_path else ''
        # 已解析绝对路径 (矩阵条目按 name 反查 xosc_list 时复用)
        self._xosc_list_path = xosc_list_path

        # 执行队列 (batch 项 = {name, file, id, fault_groups[FaultGroup 消息列表]}):
        #   auto   启动时把静态列表 (xosc_list / fault_matrix) 全量装入, 与改造前一致;
        #   manual 初始为空, execute_batch 服务回调按需入队 (S_IDLE 等待)
        self._batch = []
        self._batch_idx = -1
        if self.orch == 'auto':
            if not xosc_list_path and not fault_matrix_path:
                raise RuntimeError(
                    'orchestration=auto 时必须配置 xosc_list 或 fault_matrix (需求 15)')
            if fault_matrix_path:
                scenarios, fault_by_idx = self._load_fault_matrix(fault_matrix_path)
            else:
                with open(xosc_list_path) as f:
                    data = json.load(f)
                scenarios = data['scenarios']
                if not scenarios:
                    raise RuntimeError('xosc_list.json 场景列表为空')
                # file 相对 scenario_root 解析 (绝对路径仍兼容)
                for s in scenarios:
                    s['file'] = self._resolve_input_path(
                        s['file'], [self.scenario_root])
                fault_by_idx = {}
            for i, s in enumerate(scenarios):
                groups = fault_by_idx.get(i)
                self._batch.append({
                    'name': s['name'],
                    'file': s['file'],
                    'id': s.get('id'),
                    'fault_groups': self._build_fault_groups(groups) if groups else None,
                })
        # manual: 队列由 on_execute_batch 填充

        # ---- 编排状态 ----
        self.state = S_IDLE if self.orch == 'manual' else S_START_E2E
        self.results = []
        self._run_start_ts = time.time()
        self.done = False
        self.exit_code = 0
        self.abort_reason = None
        self._e2e_proc = None
        self._inject_proc = None
        self._exec_future = None
        self._deadline = 0.0
        self._progress_next = 0.0    # 场景执行中的进度日志节拍 (区分卡死/慢)
        self._interrupted = False
        self._kill_start = None
        self._kill_deadline = 0.0
        self._kill_pgid = None
        self._finish_event = threading.Event()
        self._finish_result = 0
        self._latest_metrics = {}
        # 单条墙钟起点: run_summary/junit 的 per-case 时长 (仿真时长 ≠ 墙钟)
        self._scn_start_ts = None
        # 注入链夭折的条级处理 (不牵连整批): 见 S_WAIT_FINISH / S_NEXT
        self._item_error_reason = None
        # 注入链是否应常驻 (有 trajectory_publisher): 无 ego_trajectory 声明时
        # wait_for_ready && set_route 正常完成即 exit 0, 不能当夭折
        self._inject_expect_linger = True
        self._consecutive_item_errors = 0
        try:
            self._item_error_limit = int(
                self.cyc.get('consecutive_item_error_abort', 3))
        except (TypeError, ValueError):
            self._item_error_limit = 3
        # e2e 就绪判定的双保险: 新 e2e 至少运行 N 秒才认 routing 服务 (避开
        # 上一条残留服务的 DDS 窗口), _e2e_start_ts 在 S_START_E2E 记录
        self._e2e_start_ts = 0.0
        try:
            self._e2e_ready_min_s = float(
                self.cyc.get('e2e_ready_min_s', 5.0))
        except (TypeError, ValueError):
            self._e2e_ready_min_s = 5.0
        # 收尾后等旧 routing 服务从 DDS 消失的上限 (S_WAIT_E2E_GONE)
        self._gone_deadline = 0.0
        try:
            self._e2e_gone_wait_s = float(
                self.cyc.get('e2e_gone_wait_s', 10.0))
        except (TypeError, ValueError):
            self._e2e_gone_wait_s = 10.0
        self.world = None
        # 启动清场: CARLA 常驻时, 上次运行异常退出可能留下 ego/npc 车辆,
        # 占住 xosc 的 spawn 点 -> scenario_runner 报 "Cannot spawn actor" ->
        # ApplicationStatus.ERROR(4) -> 场景 result=4 假失败
        # (实测 location (334.8, 14.2) 被残留 npc1 占用). 连接成功后先清一次.
        self._clean_on_start = bool(self.lc.get('clean_on_start', True))
        self._start_clean_done = False

        # ---- ROS 接口 ----
        fin_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            CarlaScenarioRunnerScenarioFinished,
            self.cyc.get('scenario_finished_topic', '/scenario_runner/scenario_finished'),
            self.on_scenario_finished, fin_qos)
        self._cycle_pub = self.create_publisher(
            String, self.cyc.get('cycle_topic', '/carla/srunner/cycle'),
            QoSProfile(depth=1))
        self._exec_cli = self.create_client(
            ExecuteScenario, '/scenario_runner/execute_scenario')
        # 故障注入 configure 客户端 (sensor_fault_injector 常驻节点服务;
        # fi_cfg 已在编排配置读取时赋值, 供矩阵加载校验使用)
        self._cfg_cli = self.create_client(
            ConfigureFault,
            self.fi_cfg.get('configure_service', '/fault_injection/configure'))
        # manual 模式指令服务 (需求 21 on_demand): 外部按需入队执行批次
        self._batch_srv = self.create_service(
            ExecuteBatch,
            self.cyc.get('batch_service', '/carla/srunner/execute_batch'),
            self.on_execute_batch)
        self._cfg_future = None     # 注入 configure 异步请求
        self._off_future = None     # 收尾 off 异步请求
        self._cfg_active = False    # 当前场景是否已注入 (S_DESTROY 决定是否 off)
        # metrics 通道 (metrics_collector 发布, latch): 场景完成时并入 summary
        self.create_subscription(
            String, self.get_parameter('metrics_topic').value,
            self.on_metrics, fin_qos)

        # ---- CARLA (收尾销毁 actors 用) ----
        self.create_timer(5.0, self._retry_connect)
        self._connect_carla()

        signal.signal(signal.SIGINT, self.on_signal)
        signal.signal(signal.SIGTERM, self.on_signal)
        self.get_logger().info(
            'scenario_monitor 编排器启动: orchestration=%s, 初始队列 %d 项'
            % (self.orch, len(self._batch)))

    # ------------------------------------------------------------------ 信号
    def on_signal(self, signum, frame):
        if self.done:
            return
        self._interrupted = True

    def on_scenario_finished(self, msg):
        """逐场景完成状态通道 (需求 6): 校验场景文件后置位完成事件."""
        cur = self._cur()
        if cur is None:
            return
        if msg.scenario_file != cur['file']:
            return
        self._finish_result = msg.result
        self._finish_event.set()

    def on_metrics(self, msg):
        """metrics_collector 发布的当前场景指标 (latch): 保存最新 JSON."""
        try:
            self._latest_metrics = json.loads(msg.data)
        except (TypeError, ValueError) as e:  # pylint: disable=broad-except
            self.get_logger().warn('metrics JSON 解析失败: %s' % e)

    # ------------------------------------------------------------ 执行队列 (manual)
    def _cur(self):
        """当前执行项 (batch[_batch_idx]), 无则 None."""
        if 0 <= self._batch_idx < len(self._batch):
            return self._batch[self._batch_idx]
        return None

    def on_execute_batch(self, request, response):
        """manual 指令服务: 展开请求为队列项追加到执行队列 (需求 21 on_demand).

        队列项与 auto 单场景迭代同构 (name/file/id/fault_groups);
        clear_queue 丢弃未开始执行的项 (保留已执行记录与正在执行的当前项);
        任一来源解析/校验失败 -> 拒绝整个请求 (fail-fast, 队列不变).
        """
        items, err = self._expand_batch_request(request)
        if err:
            response.accepted = False
            response.message = err
            response.queued_count = 0
            self.get_logger().warn('execute_batch 拒绝: %s' % err)
            return response
        if request.clear_queue:
            # _batch_idx 为当前正在执行项的索引 (-1 = 未开始);
            # 保留 [0 .. _batch_idx] (已执行 + 正在执行), 丢弃其后未开始项
            self._batch = self._batch[:self._batch_idx + 1]
        self._batch.extend(items)
        response.accepted = True
        response.message = '入队 %d 项 (队列总长 %d)' % (len(items), len(self._batch))
        response.queued_count = len(items)
        self.get_logger().info('execute_batch 受理: %s' % response.message)
        return response

    def _expand_batch_request(self, request):
        """展开 ExecuteBatch 请求 -> (items, err).

        入队顺序: scenario_files -> list_file -> matrix_file;
        fault_groups 附加给 scenario_files 与 list_file 展开项 (matrix 项
        用矩阵自带故障组); entry_ids 过滤 matrix 条目 (空 = 全矩阵).
        """
        try:
            if not (request.scenario_files or request.list_file
                    or request.matrix_file):
                return [], '请求为空 (scenario_files/list_file/matrix_file 均未提供)'
            items = []
            fg = list(request.fault_groups) if request.fault_groups else None
            for f in (request.scenario_files or []):
                f = f.strip()
                if not f:
                    continue
                # 相对路径按 scenario_root 解析 (与 auto 模式一致)
                f = self._resolve_input_path(f, [self.scenario_root])
                items.append({'name': os.path.splitext(os.path.basename(f))[0],
                              'file': f, 'id': None, 'fault_groups': fg})
            if request.list_file:
                lp = self._resolve_input_path(request.list_file.strip(),
                                              [self.scenario_root])
                with open(lp) as f:
                    try:
                        data = json.load(f)
                    except ValueError as e:
                        return [], 'xosc_list 解析失败: %s' % e
                for s in data.get('scenarios', []):
                    items.append({
                        'name': s['name'],
                        'file': self._resolve_input_path(s['file'],
                                                         [self.scenario_root]),
                        'id': None, 'fault_groups': fg})
            if request.matrix_file:
                scenarios, fault_by_idx = self._load_fault_matrix(
                    self._resolve_input_path(
                        request.matrix_file.strip(),
                        [self.fault_root, os.path.join(self.fault_root, 'matrix')]))
                want = {s.strip() for s in (request.entry_ids or []) if s.strip()}
                for i, s in enumerate(scenarios):
                    if want and s['id'] not in want:
                        continue
                    groups = fault_by_idx.get(i)
                    items.append({'name': s['name'], 'file': s['file'],
                                  'id': s['id'],
                                  'fault_groups': self._build_fault_groups(groups)
                                  if groups else None})
            if not items:
                return [], '展开后队列项为空'
            return items, ''
        except RuntimeError as e:
            return [], str(e)
        except Exception as e:  # pylint: disable=broad-except
            return [], '展开失败: %r' % e

    @staticmethod
    def _fault_to_dicts(groups):
        """FaultGroup 消息列表 -> 可 JSON 序列化 dict 列表 (run_summary 用)."""
        return [{
            'name': g.name, 'type': g.type, 'sensor': g.sensor,
            'dropout_rate': g.dropout_rate, 'noise_mean': g.noise_mean,
            'noise_stddev': g.noise_stddev, 'intensity_rate': g.intensity_rate,
            'dead_zone_x_min': g.dead_zone_x_min, 'dead_zone_x_max': g.dead_zone_x_max,
            'dead_zone_y_min': g.dead_zone_y_min, 'dead_zone_y_max': g.dead_zone_y_max,
            'dead_zone_z_min': g.dead_zone_z_min, 'dead_zone_z_max': g.dead_zone_z_max,
            'trigger_type': g.trigger_type, 'trigger_value': g.trigger_value,
            'duration': g.duration,
        } for g in groups]

    # -------------------------------------------------------------- 编排状态机
    def tick_orchestrator(self):
        if self.done:
            return
        if self._interrupted:
            self._interrupted = False
            self.abort_reason = 'signal (SIGINT/SIGTERM)'
            self.get_logger().warn('收到中断信号, 先收尾子进程与 actors 再退出 ...')
            self._start_teardown()

        st = self.state
        if st == S_IDLE:
            # manual: 队列空等待指令 (monitor 常驻, 不退出)
            if self._batch_idx + 1 < len(self._batch):
                self.state = S_START_E2E

        elif st == S_START_E2E:
            self._batch_idx += 1
            self._scn_start_ts = None
            self._item_error_reason = None
            scn = self._cur()
            self.get_logger().info('========== 场景 %d/%d: %s =========='
                                   % (self._batch_idx + 1, len(self._batch), scn['name']))
            self._e2e_proc = self._spawn_e2e()
            self._e2e_start_ts = time.monotonic()
            self._deadline = time.monotonic() + float(
                self.cyc.get('e2e_ready_wait_s', 180.0))
            self.state = S_WAIT_E2E_READY

        elif st == S_WAIT_E2E_READY:
            rc = self._e2e_proc.poll()
            if rc is not None:
                self.abort_reason = 'autoware e2e 提前退出 (exit %s)' % rc
                self._start_teardown()
                return
            if self._routing_service_ready() and (
                    time.monotonic() - self._e2e_start_ts >= self._e2e_ready_min_s):
                if self._inject_proc is None:
                    self._publish_cycle('e2e_up')
                    self._inject_proc = self._spawn_inject(self._cur()['file'])
                # 故障注入: 测例有故障组时先 configure (非阻塞状态化),
                # 成功后才开场景; 无故障组直接执行
                if self._configure_fault(self._cur()):
                    if self._call_execute_scenario(self._cur()):
                        if self._scn_start_ts is None:
                            self._scn_start_ts = time.time()
                        self._deadline = time.monotonic() + float(
                            self.cyc.get('scenario_timeout_s', 900.0))
                        self.state = S_WAIT_FINISH
            elif time.monotonic() > self._deadline:
                self.abort_reason = '等待 e2e 路由服务就绪超时'
                self._start_teardown()

        elif st == S_WAIT_FINISH:
            rc = self._e2e_proc.poll()
            if rc is not None:
                self.abort_reason = 'autoware e2e 提前退出 (exit %s)' % rc
                self._start_teardown()
                return
            # 注入链 (wait_for_ready -> set_route -> trajectory_publisher) 夭折即失败:
            # 没有 route/轨迹 ego 不可能动, 继续等到 scenario_timeout 纯属空耗
            # (job=93 实证: set_route 15 次重试耗尽后仍空等 ~750s 才超时).
            # trajectory_publisher 是常驻发布器 (rclpy.spin), 正常不会主动退出;
            # 因此这里的退出只可能是链路失败 -> 立即收尾.
            # 但要区分两种性质:
            #   e2e 也死了 -> 真编排异常, 整批中止;
            #   e2e 还活着 -> 多半是注入故障本身打瘫了被测栈 (如 lidar 100% 丢点
            #     导致定位起不来), 属于该条用例的结论, 记 ERROR 后继续跑后续矩阵项.
            #     否则一条不可跑用例会吞掉后面全部覆盖 (pipeline#40 实证: 50 条
            #     只跑到第 10 条 lidar_dropout_r100 就整批退出).
            if self._inject_proc is not None and self._inject_proc.poll() is not None:
                irc = self._inject_proc.returncode
                self._inject_proc = None
                # 无 ego_trajectory 声明的注入链 (wait_for_ready && set_route) 任务完成
                # 就会 exit 0, 场景由 autoware 自规划驱动 -> 不是夭折, 继续等 finish.
                finished_ok = (irc == 0 and not self._inject_expect_linger)
                # finish 信号已到: 场景已完成, 注入链退出属正常收尾,
                # 交给下面的 finish 分支结算, 不能误记成条级 ERROR.
                if not finished_ok and not self._finish_event.is_set():
                    e2e_rc = self._e2e_proc.poll() if self._e2e_proc is not None else -1
                    reason = 'ego 注入链提前退出 (exit %s, 见 launch.log)' % irc
                    if e2e_rc is not None:
                        self.abort_reason = (reason +
                                             '; autoware e2e 同时退出 (exit %s)' % e2e_rc)
                        self.get_logger().error(self.abort_reason)
                    else:
                        self._item_error_reason = reason
                        self.get_logger().error(
                            '%s -> 本条记 ERROR 并继续队列 (e2e 仍存活)' % reason)
                    self._start_teardown()
                    return
            if self._exec_future is not None and self._exec_future.done():
                resp = self._exec_future.result()
                self._exec_future = None
                if resp is None or not resp.result:
                    self.abort_reason = 'execute_scenario 服务拒绝当前场景 (文件不存在?)'
                    self._start_teardown()
                    return
            if self._finish_event.is_set():
                self._finish_event.clear()
                scn = self._cur()
                verdict, criteria = self._load_scenario_verdict(self._finish_result)
                result_row = {
                    'name': scn['name'],
                    'file': scn['file'],
                    'result': self._finish_result,
                    'verdict': verdict,
                    'criteria': criteria,
                    'metrics': dict(self._latest_metrics),
                    'wall_time_s': self._scn_wall_s(),
                }
                if scn.get('id'):
                    result_row['id'] = scn['id']
                if scn.get('fault_groups'):
                    result_row['fault'] = self._fault_to_dicts(scn['fault_groups'])
                if self._finish_result != 0:
                    # result!=0 是 ApplicationStatus 语义 (只有 ERROR=4 会走到这里):
                    # scenario_runner.py 子进程非零退出, 场景根本没加载/未跑完,
                    # 与 criteria 无关. 最常见原因: CARLA 中残留 actor 占住 spawn 点
                    # (报 Cannot spawn actor, 见 launch.log 的 [SC] 段).
                    reason = ('scenario_runner 子进程异常退出 (result=%s), '
                              '详见 launch.log 的 [SC] 段 (常见: 残留 actor 占位 -> '
                              'Cannot spawn actor)' % self._finish_result)
                    result_row['abort_reason'] = reason
                    # 条级不可执行: 归 exit 3 (结论不可靠), 不算整批编排异常
                    result_row['item_level_error'] = True
                    self.get_logger().error(reason)
                self.results.append(result_row)
                if self._finish_result != 0:
                    # result!=0 = 这条没跑成 (scenario_runner 崩溃), 计入连续夭折:
                    # CARLA 死/残留 actor/TM 端口占用时每条都是 4, 不计数就会
                    # 在几分钟内"跑完"50 条假结果, 把真问题掩盖成大面积失败。
                    self._consecutive_item_errors += 1
                    if (self._item_error_limit > 0 and
                            self._consecutive_item_errors >= self._item_error_limit):
                        self.abort_reason = (
                            '连续 %d 条 scenario_runner 异常退出 (result!=0, 上限 %d) '
                            '-> 疑似环境级故障' % (self._consecutive_item_errors,
                                                   self._item_error_limit))
                        self.get_logger().error('循环中止: %s' % self.abort_reason)
                        self._finalize(1)
                        return
                else:
                    # 跑完一条 (无论通过/失败) 就打断"连续中止"计数
                    self._consecutive_item_errors = 0
                self.get_logger().info('场景完成 (result=%s, verdict=%s, wall=%.1fs): %s'
                                       % (self._finish_result, verdict,
                                          result_row['wall_time_s'], scn['file']))
                self._start_teardown()
            elif time.monotonic() > self._deadline:
                self.abort_reason = '场景执行超时 (%.0fs)'
                self.abort_reason = self.abort_reason % float(
                    self.cyc.get('scenario_timeout_s', 900.0))
                self._start_teardown()
            elif time.monotonic() >= self._progress_next:
                # 进度节拍: 场景未结束但也没失败时, 每 30s 打一行剩余兜底时间,
                # 便于从 launch.log 直接判断是"仿真跑得慢"还是"彻底卡死"
                self._progress_next = time.monotonic() + 30.0
                self.get_logger().info(
                    '场景执行中: 剩余兜底 %.0fs (注入链 %s)'
                    % (max(0.0, self._deadline - time.monotonic()),
                       'alive' if self._inject_proc else 'none'))

        elif st == S_KILL_INJECT:
            if self._kill_proc(self._inject_proc, 'ego 注入链',
                               float(self.cyc.get('inject_kill_wait_s', 10.0))):
                self._inject_proc = None
                self.state = S_KILL_E2E

        elif st == S_KILL_E2E:
            if self._kill_proc(self._e2e_proc, 'autoware e2e',
                               float(self.cyc.get('e2e_shutdown_wait_s', 60.0))):
                self._e2e_proc = None
                self._gone_deadline = time.monotonic() + self._e2e_gone_wait_s
                self.state = S_WAIT_E2E_GONE

        elif st == S_WAIT_E2E_GONE:
            # e2e 主进程退出不等于节点已全部下线: 注入链夭折时 (定位挂了)
            # component_container 里的节点可能卡住不退, 它们的
            # /api/routing/set_route_points 服务仍在 DDS 里 -> 下一条
            # S_WAIT_E2E_READY 会 0.2s 就判 ready -> e2e 没起来就调
            # execute_scenario -> scenario_runner result=4 秒退
            # (pipeline#41 实测: 第 11/12/13 条 wall 0.7~0.9s)。
            # 必须等到服务真的从服务列表消失, 才能开下一条。
            if not self._routing_service_ready():
                self.state = S_DESTROY
            elif time.monotonic() > self._gone_deadline:
                self.get_logger().warn(
                    '旧 e2e 的 routing 服务 %.0fs 内未从 DDS 消失, 继续下一条'
                    ' (可能有残留进程, 看 launch.log 的 SIGKILL 记录)'
                    % self._e2e_gone_wait_s)
                self.state = S_DESTROY

        elif st == S_DESTROY:
            # 先故障 off (configure 空组, 防跨场景残留; 服务不可达容忍跳过),
            # 再销毁 actors
            if self._cfg_active:
                if self._off_future is None:
                    if self._cfg_cli.wait_for_service(1.0):
                        req = ConfigureFault.Request()
                        req.groups = []
                        self.get_logger().info('故障 off: configure 空组 (清全部实例)')
                        self._off_future = self._cfg_cli.call_async(req)
                        return
                    self.get_logger().warn('configure 服务不可达, 跳过故障 off')
                    self._cfg_active = False
                elif not self._off_future.done():
                    return
                else:
                    resp = self._off_future.result()
                    self._off_future = None
                    self._cfg_active = False
                    if resp is None or not resp.success:
                        self.get_logger().warn('故障 off 失败: %s' % (
                            resp.message if resp else '无响应'))
            self.destroy_actors()
            self._publish_cycle('reset')
            self.state = S_NEXT

        elif st == S_NEXT:
            if self.abort_reason is not None:
                if self.abort_reason.startswith('signal') or self.orch == 'auto':
                    # 信号中止 (任何模式) / auto 编排异常: 收尾后退出
                    self.get_logger().error('循环中止: %s' % self.abort_reason)
                    self._finalize(1)
                    return
                # manual: 当前项执行出错 -> 记 ERROR 行, 继续队列 (monitor 常驻)
                self.get_logger().error('场景中止: %s (记 ERROR, 继续队列)'
                                        % self.abort_reason)
                self._append_abort_row(self.abort_reason)
                self.abort_reason = None
            if self._item_error_reason is not None:
                # 单条用例夭折 (e2e 仍存活): 记 ERROR 行后继续, 不牺牲后续覆盖.
                # 连续 N 条都夭折才升级为编排异常, 防止 CARLA/DDS 真故障空转整批.
                reason = self._item_error_reason
                self._item_error_reason = None
                self._append_abort_row(reason)
                self._consecutive_item_errors += 1
                self.get_logger().error('场景中止: %s (记 ERROR, 继续队列 %d/%d)'
                                        % (reason, self._batch_idx + 1, len(self._batch)))
                if (self._item_error_limit > 0 and
                        self._consecutive_item_errors >= self._item_error_limit):
                    self.abort_reason = ('连续 %d 条用例夭折 (注入链提前退出, 上限 %d) -> '
                                         '疑似环境级故障' % (
                                             self._consecutive_item_errors,
                                             self._item_error_limit))
                    self.get_logger().error('循环中止: %s' % self.abort_reason)
                    self._finalize(1)
                    return
            if self._batch_idx + 1 < len(self._batch):
                self.state = S_START_E2E
            elif self.orch == 'manual':
                self.get_logger().info('========== 队列执行完毕, 回 idle 等待指令 ==========')
                self.state = S_IDLE
            else:
                self.get_logger().info('========== 全部 %d 个场景执行完毕 =========='
                                       % len(self._batch))
                for r in self.results:
                    self.get_logger().info('  场景 result=%s verdict=%s  %s  (%s)'
                                           % (r['result'], r['verdict'], r['name'], r['file']))
                self.get_logger().info(
                    '编排器退出, 触发 launch Shutdown (CARLA 保持运行)')
                self._finalize(self._compute_exit_code())

    def _start_teardown(self):
        self._kill_start = None
        self.state = S_KILL_INJECT

    def _kill_proc(self, proc, name, timeout):
        """状态化收尾: SIGINT 整组 -> 等整组清空 -> 超时 SIGKILL。

        只等主进程不够: ros2 launch 主进程退出后, component_container 里的节点
        可能仍存活并继续提供 /api/routing/set_route_points。它们会让下一条的
        _routing_service_ready() 0.2s 就为 True -> e2e 没起来就 execute_scenario
        -> scenario_runner result=4 秒退 (pipeline#41 实测第 11/12/13 条 wall 0.7~0.9s)。
        因此必须等到进程组为空 (killpg(pgid, 0) 抛 ProcessLookupError)。
        """
        if proc is None:
            return True
        if self._kill_start is None:
            try:
                self._kill_pgid = os.getpgid(proc.pid)
            except ProcessLookupError:
                self.get_logger().warn(
                    '收尾: %s 进程组已不存在 (主进程已被回收)' % name)
                return True
            self.get_logger().info('收尾: SIGINT -> %s (pid %d)' % (name, proc.pid))
            self._kill_start = time.monotonic()
            self._kill_deadline = self._kill_start + timeout
            try:
                os.killpg(self._kill_pgid, signal.SIGINT)
            except ProcessLookupError:
                return True
            return False
        try:
            proc.wait(timeout=0.1)   # 先 reap 主进程 (避免僵尸被当成存活)
        except subprocess.TimeoutExpired:
            pass
        group_alive = True
        try:
            os.killpg(self._kill_pgid, 0)
        except ProcessLookupError:
            group_alive = False
        if proc.poll() is not None and not group_alive:
            self._kill_start = None
            self._kill_pgid = None
            return True
        if time.monotonic() > self._kill_deadline:
            self.get_logger().warn(
                '收尾: %s SIGINT 超时 (主进程%s), 升级 SIGKILL 整组'
                % (name, '已退' if proc.poll() is not None else '未退'))
            try:
                os.killpg(self._kill_pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass
            self._kill_start = None
            self._kill_pgid = None
            return True
        return False

    def _finalize(self, code):
        if self.done:
            return
        self.done = True
        self.exit_code = code
        self._write_run_summary()

    def _scn_wall_s(self):
        """当前场景已耗时 (墙钟秒); 场景尚未启动时返回 0."""
        if self._scn_start_ts is None:
            return 0.0
        return round(max(0.0, time.time() - self._scn_start_ts), 3)

    def _append_abort_row(self, reason):
        """当前项执行中止 (manual 任意原因 / auto 注入链夭折): 记 verdict=ERROR 行,
        不退出, 继续队列."""
        cur = self._cur()
        if cur is None:
            return
        result_row = {
            'name': cur['name'],
            'file': cur['file'],
            'result': -1,
            'verdict': 'ERROR',
            'criteria': [],
            'metrics': dict(self._latest_metrics),
            'wall_time_s': self._scn_wall_s(),
            'item_level_error': True,
            'abort_reason': reason,
        }
        if cur.get('id'):
            result_row['id'] = cur['id']
        if cur.get('fault_groups'):
            result_row['fault'] = self._fault_to_dicts(cur['fault_groups'])
        self.results.append(result_row)

    # ------------------------------------------------------------ 故障矩阵加载
    @staticmethod
    def _resolve_input_path(value, search_dirs):
        """场景列表输入路径解析 (需求 15 '在各自路径下搜索文件'):
        值本身是存在的文件 (绝对路径或相对 cwd) 直接用; 否则在 search_dirs
        下按同名文件查找; 仍找不到报错 (fail-fast, 不残留子进程).
        CLI 传入 ~ 前缀路径时 bash 不展开, 此处补 expanduser."""
        value = os.path.expanduser(value)
        if os.path.isfile(value):
            return value
        for d in search_dirs:
            cand = os.path.join(d, value)
            if os.path.isfile(cand):
                return cand
        raise RuntimeError('输入文件不存在: %s (搜索目录: %s)' % (value, search_dirs))

    def _load_fault_matrix(self, matrix_path):
        """加载 fault_matrix.json (schema v2.0) 并全量校验展开.

        返回 (scenarios, fault_by_idx):
          scenarios:   [{name, file, id}] (name=条目 id, file=解析的 xosc 路径)
          fault_by_idx: {索引: [展开组 dict]}
                        (每组含 preset/type/sensor/params/trigger/duration)
        校验规则 (fail-fast, 任一失败拒绝所有条目执行):
          fault 必须为非空数组; 每组 sensor 必填且受支持; preset 必须在库中;
          条目 scenario 必须可解析出 xosc 文件.
        """
        if not os.path.isfile(matrix_path):
            raise RuntimeError('故障矩阵不存在: %s' % matrix_path)
        with open(matrix_path) as f:
            try:
                matrix = json.load(f)
            except ValueError as e:
                raise RuntimeError('故障矩阵 JSON 解析失败: %s' % e)
        presets_file = matrix.get('presets_file', '') or ''
        if not presets_file:
            raise RuntimeError('故障矩阵缺少 presets_file 字段')
        # preset 库随故障库一起搬: 相对路径按 fault_root 解析
        presets_file = self._resolve_input_path(presets_file, [self.fault_root])
        with open(presets_file) as f:
            presets_data = yaml.safe_load(f)
        presets = presets_data.get('presets', {}) \
            if isinstance(presets_data, dict) else {}
        supported = self.fi_cfg.get('supported_sensors', ['top'])
        scenarios, fault_by_idx = [], {}
        entries = matrix.get('matrix', [])
        if not entries:
            raise RuntimeError('故障矩阵条目为空: %s' % matrix_path)
        for idx, entry in enumerate(entries):
            eid = entry.get('id') or '#%d' % idx
            groups = entry.get('fault')
            if not isinstance(groups, list) or not groups:
                raise RuntimeError(
                    '矩阵条目 %s fault 必须为非空数组' % eid)
            expanded = []
            for g in groups:
                if not isinstance(g, dict):
                    raise RuntimeError(
                        '矩阵条目 %s 注入组格式错误 (非对象)' % eid)
                sensor = (g.get('sensor') or '').strip()
                if not sensor:
                    raise RuntimeError(
                        '矩阵条目 %s 注入组 sensor 字段缺失, 拒绝所有条目执行' % eid)
                if sensor not in supported:
                    raise RuntimeError(
                        '矩阵条目 %s sensor %r 不受支持 (%s)'
                        % (eid, sensor, supported))
                pname = g.get('preset')
                if pname not in presets:
                    raise RuntimeError(
                        '矩阵条目 %s preset %r 不在库中' % (eid, pname))
                params = presets[pname]
                expanded.append({
                    'preset': pname,
                    'type': params.get('type', ''),
                    'sensor': sensor,
                    'params': params,
                    'trigger': g.get('trigger'),
                    'duration': float(g.get('duration', 0.0) or 0.0),
                })
            scn_file = self._resolve_scenario_file(entry.get('scenario'))
            scenarios.append({
                'name': entry.get('id') or 'matrix_%03d' % idx,
                'file': scn_file,
                'id': entry.get('id'),
            })
            fault_by_idx[idx] = expanded
        return scenarios, fault_by_idx

    def _resolve_scenario_file(self, scenario):
        """matrix 条目 scenario -> xosc 文件绝对路径:
        带 .xosc 后缀按 scenario_root 解析; 否则查已解析 xosc_list 的 name->file 字典."""
        if not scenario:
            raise RuntimeError('矩阵条目 scenario 字段缺失')
        if scenario.endswith('.xosc'):
            return self._resolve_input_path(scenario, [self.scenario_root])
        xosc_list = self._xosc_list_path
        if not xosc_list or not os.path.isfile(xosc_list):
            raise RuntimeError(
                'scenario %r 需经 xosc_list 解析, 但 xosc_list 不可用' % scenario)
        with open(xosc_list) as f:
            try:
                data = json.load(f)
            except ValueError as e:
                raise RuntimeError('xosc_list.json 解析失败: %s' % e)
        for s in data.get('scenarios', []):
            if s.get('name') == scenario:
                return self._resolve_input_path(s.get('file', ''),
                                                [self.scenario_root])
        raise RuntimeError('scenario %r 不在 xosc_list 中' % scenario)

    def _build_fault_groups(self, groups):
        """展开组 dict -> FaultGroup 消息列表 (平铺参数, 供 configure 请求)."""
        req_groups = []
        for g in groups:
            fg = FaultGroup()
            fg.name = g['preset']
            fg.type = g['type']
            fg.sensor = g['sensor']
            p = g['params']
            fg.dropout_rate = float(p.get('dropout_rate', 0.0))
            fg.noise_mean = float(p.get('noise_mean', 0.0))
            fg.noise_stddev = float(p.get('noise_stddev', 0.0))
            fg.dead_zone_x_min = float(p.get('x_min', 0.0))
            fg.dead_zone_x_max = float(p.get('x_max', 0.0))
            fg.dead_zone_y_min = float(p.get('y_min', 0.0))
            fg.dead_zone_y_max = float(p.get('y_max', 0.0))
            fg.dead_zone_z_min = float(p.get('z_min', 0.0))
            fg.dead_zone_z_max = float(p.get('z_max', 0.0))
            fg.intensity_rate = float(p.get('intensity_rate', 0.0))
            trig = g.get('trigger') or {}
            fg.trigger_type = str(trig.get('type', 'none'))
            fg.trigger_value = float(trig.get('value', 0.0))
            fg.duration = float(g.get('duration', 0.0))
            req_groups.append(fg)
        return req_groups

    # ------------------------------------------------------------ 判定结果汇总
    def _load_scenario_verdict(self, finish_result):
        """解析场景判定 JSON (scenario_runner --json 落盘), 返回 (verdict, criteria).

        finish_result=0 (STOPPED) 时读本场景落盘的 JSON (按 mtime 过滤到
        _scn_start_ts 之后, 排除 run_summary.json);finish_result!=0 (ERROR)
        时子进程异常退出, 判定未知, 不读文件。
        无本场景判据 (xosc 无 criteria_ 条件 / 场景没跑起来) -> NO_CRITERIA。
        """
        if finish_result != 0:
            return 'ERROR', []
        results_dir = self.results_dir
        if not results_dir or not os.path.isdir(results_dir):
            return 'NO_CRITERIA', []
        latest = None
        try:
            files = [os.path.join(results_dir, f) for f in os.listdir(results_dir)
                     if f.endswith('.json') and f != 'run_summary.json']
            if not files:
                return 'NO_CRITERIA', []
            # 只接受本场景开始之后落盘的判定: 目录里混着历史场景的文件,
            # 若本场景根本没跑起来 (application 秒退), 直接取 mtime 最新会
            # 把上一条的结论偷过来 (pipeline#41 实测: wall=0.7s 却报 FAILURE)。
            # 2s 容差覆盖文件系统时间粒度与首帧写盘抖动。
            cutoff = (self._scn_start_ts or 0.0) - 2.0
            fresh = [f for f in files if os.path.getmtime(f) >= cutoff]
            if not fresh:
                newest = max(files, key=os.path.getmtime)
                self.get_logger().warn(
                    '本场景未产出判定 JSON (最新 %s 早于场景开始), 判 NO_CRITERIA'
                    % os.path.basename(newest))
                return 'NO_CRITERIA', []
            latest = max(fresh, key=os.path.getmtime)
            with open(latest) as f:
                data = json.load(f)
        except (OSError, ValueError) as e:  # pylint: disable=broad-except
            self.get_logger().warn('判定 JSON 解析失败 (%s): %s' % (latest, e))
            return 'UNKNOWN', []
        criteria = data.get('criteria', [])
        success = bool(data.get('success', False))
        return ('SUCCESS' if success else 'FAILURE'), criteria

    def _compute_exit_code(self):
        """exit code 分层 (优先级从高到低):
        1 = 编排异常 (e2e 死/服务拒收/墙钟超时/信号, 由 abort_reason 路径保证);
        2 = 测试失败 (任一场景 verdict=FAILURE);
        3 = 结论不可靠 (NO_CRITERIA/TIMEOUT/UNKNOWN, 或条级 ERROR: 该用例
            自身跑不起来, 例如 lidar 100% 去点把定位打瘫);
        0 = 全部通过。

        条级 ERROR (item_level_error) 刻意不算 1: 它不牵连整批, 如果把整批判成
        "编排异常" 会盖掉"9 条碰撞 FAILURE"这类真实结论。"""
        if any(r['verdict'] == 'ERROR' and not r.get('item_level_error')
               for r in self.results):
            return 1
        for r in self.results:
            if r['verdict'] == 'FAILURE':
                return 2
        for r in self.results:
            if r['verdict'] in ('NO_CRITERIA', 'TIMEOUT', 'UNKNOWN') or (
                    r['verdict'] == 'ERROR' and r.get('item_level_error')):
                return 3
        return 0

    def _write_run_summary(self):
        """汇总判定结果落盘 run_summary.json (CI 消费, 需求 5)。"""
        results_dir = self.results_dir
        if not results_dir:
            self.get_logger().warn('results_dir 未配置, 跳过 run summary 落盘')
            return
        try:
            os.makedirs(results_dir, exist_ok=True)
        except OSError as e:  # pylint: disable=broad-except
            self.get_logger().error('results_dir 创建失败: %s' % e)
            return
        counts = {'SUCCESS': 0, 'FAILURE': 0, 'ERROR': 0, 'OTHER': 0}
        for r in self.results:
            v = r['verdict']
            if v == 'SUCCESS':
                counts['SUCCESS'] += 1
            elif v == 'FAILURE':
                counts['FAILURE'] += 1
            elif v == 'ERROR':
                counts['ERROR'] += 1
            else:
                counts['OTHER'] += 1
        summary = {
            'schema_version': '1.0',
            'run': {
                'start_time': self._run_start_ts,
                'end_time': time.time(),
                'orchestration': self.orch,
                'xosc_list': self.cyc.get('xosc_list', ''),
                'fault_matrix': self.cyc.get('fault_matrix', ''),
                'scenario_count': len(self.results),
                'queue_total': len(self._batch),
                'output_dir': results_dir,
                'abort_reason': self.abort_reason,
            },
            'summary': {
                'total': len(self.results),
                'success': counts['SUCCESS'],
                'failure': counts['FAILURE'],
                'error': counts['ERROR'],
                'no_criteria_or_unknown': counts['OTHER'],
                'exit_code': self.exit_code,
            },
            'scenarios': [
                {
                    'name': r['name'],
                    'file': r['file'],
                    'id': r.get('id', r['name']),
                    'execution': {
                        'status': 'STOPPED' if r['result'] == 0 else 'ERROR',
                        'result': r['result'],
                    },
                    'verdict': {'result': r['verdict'], 'criteria': r['criteria']},
                    'metrics': r.get('metrics', {}),
                    'wall_time_s': r.get('wall_time_s', 0.0),
                    'fault': r.get('fault'),
                    'abort_reason': r.get('abort_reason'),
                    'item_level_error': bool(r.get('item_level_error')),
                }
                for r in self.results
            ],
        }
        summary_path = os.path.join(results_dir, 'run_summary.json')
        try:
            with open(summary_path, 'w') as f:
                json.dump(summary, f, indent=2)
            self.get_logger().info('run summary 落盘: %s (exit_code=%d)'
                                   % (summary_path, self.exit_code))
        except OSError as e:  # pylint: disable=broad-except
            self.get_logger().error('run summary 写入失败: %s' % e)

    # ------------------------------------------------------------ 子进程管理
    @staticmethod
    def _sanitize_pythonpath(path_str):
        """剔除 PYTHONPATH 中的 anaconda/conda 段 (cv2 undefined symbol 历史教训).

        e2e 内 autoware_carla_interface 构造 CvBridge 时 import cv2; 用户 shell 的
        anaconda (carla_py310) site-packages 若进入 e2e 环境, cv2 会加载 anaconda 的
        libpangoft2 -> FcConfigSetDefaultSubstitute undefined symbol -> ImportError,
        节点启动后 ~1s 崩掉并作为 required 节点拖垮整个 e2e. 保留 carla egg 与
        autoware/ROS 安装路径 (interface import carla 依赖它们).
        """
        return ':'.join(p for p in path_str.split(':')
                        if p and 'anaconda' not in p and 'conda' not in p)

    def _spawn_e2e(self):
        """拉起 autoware e2e 子进程 (每场景 relaunch, 需求 7).

        环境恢复为用户 shell 原始值 (e2e_pythonpath/e2e_ld_preload 由 launch 传入),
        与改造前 IncludeLaunchDescription 继承的环境一致; 但剔除 anaconda 路径,
        否则 autoware_carla_interface 的 cv2 导入崩溃 (见 _sanitize_pythonpath).
        """
        e2e_args = {
            'map_path': self.aw['map_path'],
            'vehicle_model': self.aw.get('vehicle_model', 'sample_vehicle'),
            'sensor_model': self.aw.get('sensor_model', 'carla_sensor_kit'),
            'simulator_type': self.aw.get('simulator_type', 'carla'),
            'ego_vehicle_role_name': self.ego_role_name,
            'attach_timeout_s': str(self.aw.get('attach_timeout_s', 120.0)),
            'use_light_weight_sensor_mapping': str(
                self.aw.get('use_light_weight_sensor_mapping', True)).lower(),
            # 循环模式 e2e 每场景重启, rviz 每轮重开窗口会频繁闪断, 默认关 (yaml cycle.rviz)
            'rviz': str(self.cyc.get('rviz', self.aw.get('rviz', False))).lower(),
            'use_sim_time': str(self.aw.get('use_sim_time', True)).lower(),
            'perception_mode': self.aw.get('perception_mode', 'lidar'),
        }
        args_str = ' '.join('%s:=%s' % kv for kv in e2e_args.items())
        pkg = self.aw.get('launch_package', 'autoware_launch')
        launch_file = self.aw.get('launch_file', 'e2e_simulator.launch.xml')
        pkg_share = get_package_share_directory(pkg)
        cmd = 'ros2 launch %s %s %s' % (pkg, launch_file, args_str)
        env = os.environ.copy()
        env['PYTHONPATH'] = self._sanitize_pythonpath(self.e2e_pythonpath)
        env['LD_PRELOAD'] = self.e2e_ld_preload
        self.get_logger().info('拉起 autoware e2e: %s (%s)' % (
            os.path.join(pkg_share, 'launch', launch_file), args_str))
        return subprocess.Popen(['bash', '-c', cmd], env=env,
                                start_new_session=True, preexec_fn=_pdeathsig)

    def _spawn_inject(self, scn_file):
        """拉起 ego 注入链子进程 (每场景重启).

        wait_for_ready (等 npc_ready latch) -> set_route -> trajectory_publisher
        (--xosc 解析场景声明加载同名 json 轨迹). 三个工具均为本包 console_scripts,
        经 install 空间调用, 不依赖源码目录形状/场景库目录结构.
        3a (需求 19): 场景 xosc 未声明 ego_trajectory_ref 时跳过 trajectory_publisher
        (autoware 走完整规划链路), 仅 wait_for_ready + set_route 解锁 selector.
        """
        services = ' '.join('--wait-service %s' % s
                            for s in self.inj.get('wait_services', []))
        # console_scripts 绝对路径: 本工作区 install 未把 lib/<pkg> 写入 PATH
        # (裸命令不可见), 用 ament prefix 解析 -> 不依赖源码目录形状与 PATH
        bin_dir = os.path.join(get_package_prefix('autoware_srunner'),
                               'lib', 'autoware_srunner')
        # 本包 install 必须一并 source: console_scripts 装在 install/<pkg>/lib/<pkg>/
        cmd_prefix = ('unset PYTHONPATH LD_PRELOAD && '
                      'source /opt/ros/humble/setup.bash && '
                      'source %s/setup.bash && '
                      'source %s/setup.bash && '
                      % (self.env_cfg['autoware_install'],
                         self.env_cfg.get('srunner_install',
                                          os.path.join(self.ws, 'install'))))
        cmd_ready = ('%s/wait_for_ready --topic %s --timeout %.0f %s'
                     % (bin_dir, self.inj['ready_topic'],
                        float(self.inj.get('wait_timeout_s', 300.0)), services))
        cmd_route = ('%s/set_route --goal "%s" --goal-yaw-deg %s --nudge-back-m %.1f'
                     % (bin_dir, self.inj['set_route_goal'],
                        self.inj.get('set_route_goal_yaw_deg', 0.0),
                        float(self.inj.get('nudge_back_m', 6.0))))

        if not self._resolve_ego_trajectory(scn_file):
            # 无 ego_trajectory 声明: 跳过 trajectory_publisher (需求 19),
            # autoware 自身规划链驱动; set_route 仍执行 (解锁 scenario_selector)
            # 本链完成后即退出, 故不常驻 (S_WAIT_FINISH 据此区分"完成"与"夭折")
            self._inject_expect_linger = False
            cmd = cmd_prefix + cmd_ready + ' && ' + cmd_route
            self.get_logger().info(
                '拉起 ego 注入链 (无 ego_trajectory 声明, 跳过 trajectory_publisher)')
            return subprocess.Popen(['bash', '-c', cmd],
                                    start_new_session=True, preexec_fn=_pdeathsig)

        ctrl_topic = str(self.inj.get('controller_topic', '') or '').strip()
        ctrl_arg = ('--controller-topic %s' % ctrl_topic) if ctrl_topic else \
            '--no-controller-topic'
        # trajectory_publisher 是常驻发布器, 正常只会在 SIGINT 时退出
        self._inject_expect_linger = True
        # controller 加速上限覆盖 (每场景新 autoware 参数复位, 需重设):
        # 后台等待 controller_node_exe 上线后 ros2 param set
        max_acc = self.inj.get('max_acc_override', 6.0)
        override = ''
        if max_acc:
            override = (
                '{ ( until ros2 param set /control/trajectory_follower/'
                'controller_node_exe max_acc %s; do sleep 3; done; '
                'echo "[ego_injection_chain] controller max_acc -> %s '
                '(ros2 param set)"; ) & } && ' % (max_acc, max_acc))
        cmd = (cmd_prefix + override + cmd_ready + ' && ' + cmd_route + ' && ' +
               '%s/trajectory_publisher --xosc %s --no-route --rate %.1f --topic %s %s '
               '--ros-args -p use_sim_time:=%s'
               % (bin_dir, scn_file, float(self.inj.get('rate', 10.0)),
                  self.inj.get('trajectory_topic',
                               '/planning/scenario_planning/lane_driving/motion_planning/'
                               'path_optimizer/trajectory'),
                  ctrl_arg, str(self.inj.get('use_sim_time', True)).lower()))
        self.get_logger().info('拉起 ego 注入链 (trajectory --xosc %s)' % scn_file)
        return subprocess.Popen(['bash', '-c', cmd],
                                start_new_session=True, preexec_fn=_pdeathsig)

    @staticmethod
    def _resolve_ego_trajectory(scn_file):
        """检查场景 xosc 是否声明 ego_trajectory_ref (需求 19).

        与 trajectory_publisher.resolve_trajectory_from_xosc 同规则:
        ParameterDeclaration name="ego_trajectory_ref" 存在且其引用的同名
        .json 存在 -> 返回 json 路径; 否则 None (编排器据此决定开关).
        """
        import xml.etree.ElementTree as ET
        if not scn_file or not os.path.isfile(scn_file):
            return None
        try:
            root = ET.parse(scn_file).getroot()
        except ET.ParseError:
            return None
        for pd in root.iter('ParameterDeclaration'):
            if pd.get('name') != 'ego_trajectory_ref':
                continue
            ref = pd.get('value', '')
            if not ref:
                return None
            full = os.path.normpath(os.path.join(
                os.path.dirname(os.path.abspath(scn_file)), ref))
            json_path = os.path.splitext(full)[0] + '.json'
            return json_path if os.path.isfile(json_path) else None
        return None

    # ------------------------------------------------------------ ROS 交互
    def _routing_service_ready(self):
        try:
            names = [n for n, _ in self.get_service_names_and_types()]
        except Exception:  # pylint: disable=broad-except
            return False
        return '/api/routing/set_route_points' in names

    def _publish_cycle(self, val):
        msg = String()
        msg.data = val
        self._cycle_pub.publish(msg)
        self.get_logger().info('发布编排信号 cycle=%s' % val)

    def _configure_fault(self, cur):
        """当前测例故障注入 configure (非阻塞状态化).

        cur['fault_groups'] 为 FaultGroup 消息列表 (auto 装载/manual 入队时
        已展开); 无故障组 -> True (跳过); 有组: 发 configure -> 等响应 ->
        success=True 置 _cfg_active (S_DESTROY 据此 off); 失败 -> 中止编排.
        """
        groups = cur['fault_groups'] if cur is not None else None
        if not groups:
            return True
        if self._cfg_future is None:
            if not self._cfg_cli.wait_for_service(1.0):
                self.get_logger().warn('configure 服务未就绪, 1s 后重试 ...')
                return False
            req = ConfigureFault.Request()
            req.groups = groups
            self.get_logger().info('故障注入 configure: %s'
                                   % ', '.join(g.name for g in groups))
            self._cfg_future = self._cfg_cli.call_async(req)
            return False
        if not self._cfg_future.done():
            return False
        resp = self._cfg_future.result()
        self._cfg_future = None
        if resp is None or not resp.success:
            self.abort_reason = 'configure 故障注入失败: %s' % (
                resp.message if resp else '无响应')
            self._start_teardown()
            return False
        self._cfg_active = True
        return True

    def _call_execute_scenario(self, scn):
        """调用 execute_scenario 服务 (非阻塞), 返回 True=请求已发出."""
        if self._exec_future is not None and not self._exec_future.done():
            return True
        if not self._exec_cli.wait_for_service(1.0):
            self.get_logger().warn('execute_scenario 服务未就绪, 1s 后重试 ...')
            return False
        req = ExecuteScenario.Request()
        req.scenario.name = scn['name']
        req.scenario.scenario_file = scn['file']
        self.get_logger().info('请求执行场景: %s (%s)' % (scn['name'], scn['file']))
        self._exec_future = self._exec_cli.call_async(req)
        return True

    # ------------------------------------------------------------ CARLA 收尾
    def _connect_carla(self):
        if self.world is not None:
            return
        try:
            client = carla.Client(self.lc.get('host', 'localhost'),
                                  self.lc.get('port', 2000))
            client.set_timeout(5.0)
            self.world = client.get_world()
            self.get_logger().info('CARLA 连接成功')
            if self._clean_on_start and not self._start_clean_done:
                self._start_clean_done = True
                self.get_logger().info(
                    '启动清场: 清除上次运行残留的车辆/传感器 actors (lifecycle.clean_on_start)')
                self.destroy_actors()
        except Exception as e:  # pylint: disable=broad-except
            self.get_logger().warn('CARLA 连接失败, 5 秒后重试: %s' % e)

    def _retry_connect(self):
        self._connect_carla()

    def destroy_actors(self):
        """销毁 CARLA 世界中的车辆与传感器 actor (每场景收尾 + 兜底)."""
        if self.world is None:
            self.get_logger().warn('CARLA 未连接, 跳过 actor 销毁')
            return
        try:
            actors = self.world.get_actors()
            n = 0
            for actor in list(actors.filter('vehicle.*')) + list(actors.filter('sensor.*')):
                if not actor.is_alive:
                    continue
                actor.destroy()
                n += 1
            self.get_logger().info('CARLA actors 清理完成 (%d 个)' % n)
        except Exception as e:  # pylint: disable=broad-except
            self.get_logger().error('销毁 actors 失败: %s' % e)


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioMonitor()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.2)
            node.tick_orchestrator()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 兜底: 无论以何种路径退出 (异常/信号打断), 硬杀残留子进程组,
        # 防止注入链/e2e 孤儿化 (正常收尾路径已把 proc 置 None, 此处无操作)
        for proc in (node._e2e_proc, node._inject_proc):
            if proc is not None and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:  # pylint: disable=broad-except
                    pass
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == '__main__':
    sys.exit(main())
