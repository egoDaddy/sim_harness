#!/usr/bin/env python3
"""autoware_srunner 联合仿真主控 (tick 主从反转 + 循环执行模式):

  a. autoware e2e 与 ego 注入链由 scenario_monitor 编排器接管: 每场景用 subprocess
     relaunch e2e (需求 7), 场景间经 /carla/srunner/cycle 信号驱动触发器复位;
  b. carla_ros_scenario_runner 常驻 (注入 SCENARIO_RUNNER_SYNC_FLAG=true:
     scenario_runner 成为 ego spawn 者 + CARLA tick 主), 编排器逐场景调用
     execute_scenario 服务 (循环执行模式, 需求 1);
  c. npc_ready_trigger / auto_ready_trigger 常驻循环模式 (需求 5): 每场景重新触发,
     由编排器 cycle 信号复位;
  d. scenario_monitor 监听逐场景完成状态通道 /scenario_runner/scenario_finished
     (需求 6): 收尾 (杀注入链 -> SIGINT e2e -> 销毁 CARLA actors) 后推进下一场景;
     auto 模式全部场景完成后退出触发 Shutdown, CARLA 服务器保持运行;
     manual 模式 monitor 常驻 (idle 等待 execute_batch 指令入队执行),
     信号中止收尾后 Shutdown.
"""
import os
import re
import sys
import subprocess

import yaml

from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, Shutdown)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _path_list(value):
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    return list(value)


def _resolve_cli_inputs(xosc_arg, fault_arg, orch_arg, cyc, fi_cfg):
    """CLI 编排控制参数解析 (CLI > yaml 覆盖语义, 需求 15/21).

    xosc_list/fault_matrix: 任一 CLI 值非空 -> CLI 为唯一来源 (yaml cycle 段
    忽略, 免改 yaml 即切换模式); 双传互斥报错; 全空回退 yaml cycle 段.
    orchestration: CLI 值非空用之, 否则回退 yaml sensor_fault_injector 段.
    返回 (xosc_final, fault_final, orch_final).
    """
    if xosc_arg and fault_arg:
        raise RuntimeError(
            'CLI 参数 xosc_list 与 fault_matrix 互斥, 不能同时传入')
    if xosc_arg or fault_arg:
        xosc_final, fault_final = xosc_arg, fault_arg
    else:
        xosc_final = str(cyc.get('xosc_list', '') or '').strip()
        fault_final = str(cyc.get('fault_matrix', '') or '').strip()
    orch_final = (orch_arg or '').strip() or str(fi_cfg.get('orchestration', 'auto'))
    return xosc_final, fault_final, orch_final

_destroy_once_done = False


def _make_monitor_exit_guard(orch):
    """monitor 收尾守卫 (闭包捕获编排模式; humble launch 0.19 的
    OnProcessExit 无 returncode 参数, 但 on_exit 支持 callable(event, context)).

    monitor 分层退出码 (_compute_exit_code):
      0 = 全部通过; 1 = 编排异常 (真异常); 2 = 测试失败 (有场景 FAILURE);
      3 = 结论不可靠 (NO_CRITERIA/TIMEOUT/UNKNOWN)
    0/2/3 都是跑批完整结束 (结论已落盘 run_summary.json) -> 正常 Shutdown;
    exit 1 按模式分流: auto = 编排异常, 不级联关停保留现场可调试;
    manual = 信号中止收尾完成 (monitor 常驻, 唯一正常退出路径即 Ctrl+C
    信号收尾) -> 属正常操作, Shutdown 关停联合仿真."""
    def _guard(event, context):
        rc = getattr(event, 'returncode', None)
        if rc == 0:
            reason = '全部场景执行完毕: 全部通过'
        elif rc == 2:
            reason = '全部场景执行完毕: 存在 FAILURE 场景 (测试失败结论)'
        elif rc == 3:
            reason = '全部场景执行完毕: 存在 NO_CRITERIA/TIMEOUT/UNKNOWN (结论不可靠)'
        elif rc == 1 and orch == 'manual':
            reason = 'manual 模式信号中止: 收尾完成 (summary 已落盘)'
        else:
            print('[autoware_srunner] scenario_monitor 异常退出 (exit %s),'
                  ' 不关停联合仿真 (现场保留可调试)' % rc)
            return []
        return [Shutdown(reason=reason + ', 关闭联合仿真 (CARLA 保持运行)')]
    return _guard


def _destroy_actors_once(host, port):
    """shutdown 兜底销毁 actors (全局互斥, 免疫 OnShutdown 多次触发).

    destroy_actors 走 console_scripts 入口 (install 布局), 不拼源码目录路径.
    """
    global _destroy_once_done
    if _destroy_once_done:
        return []
    _destroy_once_done = True
    exe = os.path.join(get_package_prefix('autoware_srunner'),
                       'lib', 'autoware_srunner', 'destroy_actors')
    try:
        subprocess.run([exe, '--host', host, '--port', str(port)], timeout=90)
    except Exception as e:  # pylint: disable=broad-except
        print('[autoware_srunner] destroy_actors 兜底执行失败: %s' % e)
    return []

def _carla_env_actions(env_cfg):
    """CARLA python 环境注入 actions (每次调用新建实例, 只作用于包裹它的 Group)."""
    acts = []
    extra_paths = _path_list(env_cfg.get('pythonpath_extra'))
    if extra_paths:
        acts.append(SetEnvironmentVariable('PYTHONPATH', [
            ':'.join(extra_paths), ':',
            EnvironmentVariable('PYTHONPATH', default_value='')]))
    if env_cfg.get('ld_preload'):
        acts.append(SetEnvironmentVariable('LD_PRELOAD', [
            env_cfg['ld_preload'], ':',
            EnvironmentVariable('LD_PRELOAD', default_value='')]))
    return acts

def _launch_setup(context):
    # 编排配置来源: CLI asr_config > 包内 config/autoware_srunner.yaml
    cfg_arg = os.path.expanduser(
        (LaunchConfiguration('asr_config').perform(context) or '').strip())
    cfg_file = cfg_arg or os.path.join(
        get_package_share_directory('autoware_srunner'),
        'config', 'autoware_srunner.yaml')
    if not os.path.isfile(cfg_file):
        raise RuntimeError('配置文件不存在: %s' % cfg_file)
    with open(cfg_file) as f:
        cfg = yaml.safe_load(f)

    # CLI 编排控制参数 (需求 15/21): 不传则回退 yaml 对应段
    xosc_arg = (LaunchConfiguration('xosc_list').perform(context) or '').strip()
    fault_arg = (LaunchConfiguration('fault_matrix').perform(context) or '').strip()
    orch_arg = (LaunchConfiguration('orchestration').perform(context) or '').strip()
    root_arg = (LaunchConfiguration('scenario_root').perform(context) or '').strip()
    froot_arg = (LaunchConfiguration('fault_root').perform(context) or '').strip()
    rdir_arg = (LaunchConfiguration('results_dir').perform(context) or '').strip()

    ws = cfg['workspace']['root']
    # 资产根 (多仓分离边界): CLI > yaml paths > 改造前位置
    p_cfg = cfg.get('paths', {}) or {}
    scenario_root = os.path.expanduser(root_arg) or p_cfg.get('scenario_root') \
        or os.path.join(ws, 'scenarios')
    fault_root = os.path.expanduser(froot_arg) or p_cfg.get('fault_root') \
        or os.path.join(ws, 'fault')
    results_dir = os.path.expanduser(rdir_arg) or p_cfg.get('results_root') \
        or cfg['scenario_runner'].get('output_dir') \
        or os.path.join(ws, 'scenarios', 'results')
    env_cfg = cfg['environment']
    aw = cfg['autoware']
    sr = cfg['scenario_runner']
    # ego role_name 单一源头 (yaml/xosc/interface 三侧一致)
    ego_role_name = cfg.get('ego_role_name', 'ego_vehicle')
    inj = cfg['ego_injection']
    mk = cfg['npc_markers']
    trig = cfg['npc_ready_trigger']
    atrig = cfg.get('auto_ready_trigger', {})
    mt = cfg.get('metrics', {})
    lc = cfg['lifecycle']
    fi = cfg.get('sensor_fault_injector', {})
    # CLI > yaml 覆盖语义解析 (互斥/回退), monitor 拿最终值
    xosc_final, fault_final, orch_final = _resolve_cli_inputs(
        xosc_arg, fault_arg, orch_arg, lc.get('cycle', {}), fi)

    # e2e 子进程要恢复的用户 shell 原始环境, 必须在这里 (OpaqueFunction 顶层)
    # 就取好: monitor 节点被包在 _carla_env_actions 的 Group 里, 组内的
    # SetEnvironmentVariable 会先把 PYTHONPATH/LD_PRELOAD 改写掉。
    # 且必须用 ParameterValue(value_type=str) 绕过 launch 的 yaml.safe_load:
    # 形如 "/xxx/libcudart.so:" 的值会被 yaml 解析成 {"/xxx/libcudart.so": None}
    # 而报 TypeError (Got <class 'dict'>), 在 CI 干净环境 (LD_PRELOAD 为空)
    # 拼出结尾冒号时必现。
    orig_pythonpath = perform_substitutions(
        context, [EnvironmentVariable('PYTHONPATH', default_value='')])
    orig_ld_preload = perform_substitutions(
        context, [EnvironmentVariable('LD_PRELOAD', default_value='')])

    actions = []

    # ---- CARLA python 环境: 仅注入给需要 carla 的进程组, 严禁污染 autoware 节点 ----
    # (全局注入曾导致 autoware_carla_interface 加载 anaconda cv2 -> undefined symbol 崩溃)

    # ---- a. autoware e2e 由 scenario_monitor 编排器接管 (循环执行模式) ----
    # 每场景 relaunch (需求 7): monitor 读 yaml autoware 段 + lifecycle.cycle 段,
    # 用 subprocess 拉起 e2e 子进程, 场景结束后 SIGINT 收尾; e2e 子进程环境由
    # e2e_pythonpath/e2e_ld_preload 参数恢复为用户 shell 原始值 (见 monitor 节点)

    # ---- b. carla_ros_scenario_runner 节点 (注入 sync_flag_override=true: scenario_runner
    #          成为 tick 主; 官方行为: 节点退出即收尾) ----
    # 传感器 kit 运行时选择: 从 use_$$$_sensor_mapping (值为 true) 提取 kit 标签,
    # 与 parse_sensor_kit.py 的标签定义一致 (没有 -> full); 经环境变量透传给
    # scenario_runner_runner -> scenario_runner.py openscenarioparams 参数,
    # 覆盖主场景 xosc ParameterDeclaration sensor_kit_entry 默认值,
    # 与 interface 侧 use_$$$_sensor_mapping 选择的 mapping 保持一致
    kit_tag = 'full'
    for _key, _value in aw.items():
        _m = re.match(r'^use_(.+)_sensor_mapping$', _key)
        if _m and str(_value).lower() == 'true':
            kit_tag = _m.group(1)
            break
    srunner_node = Node(
        package=sr['package'],
        executable=sr['executable'],
        name=sr['executable'],
        output='screen',
        emulate_tty=True,
        on_exit=Shutdown(reason='carla_ros_scenario_runner 退出, 联合仿真终止 (兜底)'),
        additional_env={
            'SCENARIO_RUNNER_SYNC_FLAG': str(sr.get('sync_flag_override', False)).lower(),
            'SRUNNER_FLAG_DIR': atrig.get('flag_dir', '/tmp/srunner/flags'),
            'SCENARIO_RUNNER_OPENSCENARIO_PARAMS':
                'sensor_kit_entry:ego_sensor_kit_{}'.format(kit_tag),
            'SCENARIO_RUNNER_OUTPUT_DIR': results_dir,
        },
        parameters=[{
            'host': sr.get('host', 'localhost'),
            'port': sr.get('port', 2000),
            'role_name': ego_role_name,
            'scenario_runner_path': sr['scenario_runner_path'],
            'wait_for_ego': str(sr.get('wait_for_ego', True)).lower(),
        }],
    )
    actions.append(GroupAction(_carla_env_actions(env_cfg) + [srunner_node]))

    # b2. execute_scenario 服务调用由 scenario_monitor 编排器负责 (每场景调用一次,
    #     scenario_runner 随即 spawn ego + teleport NPC, 无需等待 interface spawn ego)

    # ---- c. NPC 就位触发信号节点 (wall-clock timer, 不依赖 /clock; 需要 carla) ----
    # 循环模式 (需求 5): 常驻, 每场景由编排器 cycle="reset" 复位后重新触发
    actions.append(GroupAction(_carla_env_actions(env_cfg) + [Node(
        package='autoware_srunner',
        executable='npc_ready_trigger',
        name='npc_ready_trigger',
        output='screen',
        parameters=[{
            'status_topic': trig.get('status_topic', '/scenario_runner/status'),
            'ready_topic': trig.get('ready_topic', inj['ready_topic']),
            'poll_rate_hz': trig.get('poll_rate_hz', 2.0),
            'host': trig.get('host', 'localhost'),
            'port': trig.get('port', 2000),
            'ego_role_names': trig.get('ego_role_names', ['ego_vehicle', 'hero', 'ego']),
            'cycle_topic': trig.get('cycle_topic', '/carla/srunner/cycle'),
        }],
    )]))

    # c0. auto ready 门控触发节点 (方案 A: AUTONOMOUS 或 engage=true -> 写 auto_ready.flag -> Act 开闸, NPC 起步)
    # 循环模式 (需求 5): 常驻, reset 清 flag + 停监听, e2e_up 恢复监听新 autoware
    actions.append(Node(
        package='autoware_srunner',
        executable='auto_ready_trigger',
        name='auto_ready_trigger',
        output='screen',
        additional_env={'SRUNNER_FLAG_DIR': atrig.get('flag_dir', '/tmp/srunner/flags')},
        parameters=[{
            'engage_topic': atrig.get('engage_topic', '/autoware/engage'),
            'mode_state_topic': atrig.get(
                'mode_state_topic', '/system/operation_mode/state'),
            'ready_topic': atrig.get('ready_topic', '/carla/srunner/auto_ready'),
            'cycle_topic': atrig.get('cycle_topic', '/carla/srunner/cycle'),
        }],
    ))

    # c2. ego 注入链 (wait_for_ready -> set_route -> trajectory_publisher) 由
    # scenario_monitor 编排器接管: 每场景重新拉起, trajectory_publisher 用
    # --xosc 解析场景声明加载同名 json 轨迹 (需求 3)

    # ---- d. NPC marker 发布节点 (需求 4, 需要 carla) ----
    actions.append(GroupAction(_carla_env_actions(env_cfg) + [Node(
        package='autoware_srunner',
        executable='npc_marker_publisher',
        name='npc_marker_publisher',
        output='screen',
        parameters=[{'use_sim_time': aw.get('use_sim_time', True), **mk}],
    )]))

    # ---- d0. metrics 采集节点 (collision_speed_kph, 需要 carla): 直连 CARLA 给
    #      ego 挂碰撞传感器, 发布 metrics JSON latch, monitor 并入 run_summary ----
    actions.append(GroupAction(_carla_env_actions(env_cfg) + [Node(
        package='autoware_srunner',
        executable='metrics_collector',
        name='metrics_collector',
        output='screen',
        parameters=[{
            'host': mt.get('host', 'localhost'),
            'port': mt.get('port', 2000),
            'ego_role_name': ego_role_name,
            'poll_rate_hz': mt.get('poll_rate_hz', 2.0),
            'cycle_topic': mt.get('cycle_topic', '/carla/srunner/cycle'),
            'metrics_topic': mt.get('metrics_topic', '/carla/srunner/metrics'),
        }],
    )]))

    # ---- d1. 传感器故障注入节点 (常驻): 顶替发布 pointcloud_before_sync,
    #      monitor 经 configure 服务按测例注入, 场景收尾 off 透传;
    #      不需要 CARLA env (纯 ROS 点云中间件) ----
    actions.append(Node(
        package='autoware_srunner',
        executable='sensor_fault_injector',
        name='sensor_fault_injector',
        output='screen',
        parameters=[{'use_sim_time': aw.get('use_sim_time', True), **fi}],
    ))

    # ---- d1.5 最小 default_adapi (operation_mode API 服务提供者) ----
    # 裁剪版 autoware 的 e2e 里官方 default_adapi.launch.py (19 节点) 的容器
    # 顺序加载到 manual/local 时 load_node 响应不返回 -> 加载链卡死, 排在其后
    # 的 operation_mode 永不加载 -> /api/operation_mode/change_to_autonomous
    # 服务缺失 -> ego 注入链 wait_for_ready 卡满超时 (ego 不动)。故 harness
    # 侧用独立容器只加载 operation_mode (与编排器同生命周期, 场景间不随 e2e
    # 重启), 详见 launch/default_adapi_mini.launch.py; 有常驻 adapi 的环境
    # 可 launch_default_adapi:=false 关闭。
    adapi_arg = (LaunchConfiguration('launch_default_adapi').perform(context)
                 or '').strip().lower()
    if adapi_arg in ('', 'true', '1'):
        try:
            get_package_share_directory('autoware_default_adapi_universe')
        except PackageNotFoundError:
            print('[autoware_srunner] autoware_default_adapi_universe 未安装, '
                  '跳过 default_adapi mini (change_to_autonomous 需外部提供)')
        else:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('autoware_srunner'),
                    'launch', 'default_adapi_mini.launch.py'))))

    # ---- d2. 循环编排器 (scenario_monitor): 逐场景编排 e2e/注入链/execute_scenario,
    #      全部场景完成后退出 -> 触发 Shutdown (需求 1/5/6/7) ----
    # e2e_pythonpath/e2e_ld_preload: 用户 shell 原始环境 (launch 进程继承值),
    # monitor 用其重建 e2e 子进程环境, 隔离本节点 GroupAction 注入的 CARLA 环境
    monitor_node = Node(
        package='autoware_srunner',
        executable='scenario_monitor',
        name='scenario_monitor',
        output='screen',
        parameters=[{
            'asr_config': cfg_file,
            'e2e_pythonpath': ParameterValue(orig_pythonpath, value_type=str),
            'e2e_ld_preload': ParameterValue(orig_ld_preload, value_type=str),
            'results_dir': results_dir,
            'metrics_topic': mt.get('metrics_topic', '/carla/srunner/metrics'),
            # 编排控制参数 (CLI > yaml 覆盖语义已在本 launch 解析为最终值)
            'scenario_root': scenario_root,
            'fault_root': fault_root,
            'xosc_list': xosc_final,
            'fault_matrix': fault_final,
            'orchestration': orch_final,
        }],
    )
    actions.append(GroupAction(_carla_env_actions(env_cfg) + [monitor_node]))

    if lc.get('auto_shutdown', True):
        # returncode 过滤经 on_exit callable 实现 (humble launch 0.19 的
        # OnProcessExit 无 returncode 关键字): 0/2/3 (auto/manual) 与
        # manual 的 exit 1 (信号中止收尾完成) 触发 Shutdown;
        # auto 的 exit 1 (编排异常) 保持现场可调试
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=monitor_node,
            on_exit=_make_monitor_exit_guard(orch_final))))

    # 手动 Ctrl+C 兜底: shutdown 流程中再跑一次幂等 destroy (先清理 actor 再杀接口进程)
    actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=lambda ctx: _destroy_actors_once(
            lc.get('host', 'localhost'), lc.get('port', 2000)))])))

    return actions


def generate_launch_description():
    # CLI 编排控制参数 (3a): 覆盖 yaml 对应段, 不传则回退 yaml;
    # asr_config/资产根暴露为 CLI -> CI 可注入不同配置与场景库 checkout 路径
    return LaunchDescription([
        DeclareLaunchArgument(
            'asr_config', default_value='',
            description='编排配置文件路径 (CLI 优先; 空则用包内 config/autoware_srunner.yaml)'),
        DeclareLaunchArgument(
            'scenario_root', default_value='',
            description='场景库根目录 (相对路径据此解析; 空则用 yaml paths.scenario_root)'),
        DeclareLaunchArgument(
            'fault_root', default_value='',
            description='故障库根目录 (空则用 yaml paths.fault_root)'),
        DeclareLaunchArgument(
            'results_dir', default_value='',
            description='结果输出目录 (空则用 yaml paths.results_root)'),
        DeclareLaunchArgument(
            'xosc_list', default_value='',
            description='xosc_list.json 路径 (CLI 优先; 空则用 yaml lifecycle.cycle 段)'),
        DeclareLaunchArgument(
            'fault_matrix', default_value='',
            description='fault_matrix.json 路径 (CLI 优先; 空则用 yaml lifecycle.cycle 段;'
                        ' 与 xosc_list 互斥)'),
        DeclareLaunchArgument(
            'orchestration', default_value='',
            description='注入编排模式 auto/manual (CLI 优先; 空则用 yaml'
                        ' sensor_fault_injector 段)'),
        DeclareLaunchArgument(
            'launch_default_adapi', default_value='true',
            description='是否由 harness 拉起最小 default_adapi (operation_mode 节点, '
                        '提供 /api/operation_mode/change_to_autonomous; 裁剪版 autoware'
                        ' 官方 adapi 容器会卡死在 manual/local, 故独立加载)'),
        OpaqueFunction(function=_launch_setup),
    ])
