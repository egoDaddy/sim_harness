#!/usr/bin/env python3
"""最小 default_adapi: 只加载 operation_mode 节点.

背景: 裁剪版 autoware 的 e2e (tier4_autoware_api_launch/autoware_api.launch.xml)
会 include 官方 default_adapi.launch.py (19 个 ComposableNode, 单容器顺序加载),
实测加载到 manual/local 节点时 /adapi/container 的 load_node 服务响应不返回,
launch_ros.actions.load_composable_nodes 一直等 -> 排在其后的 operation_mode
(第 12 个) 永不加载 -> /api/operation_mode/change_to_autonomous 服务缺失 ->
ego 注入链 wait_for_ready 卡满超时.

本文件用独立命名的容器只加载 operation_mode, 与 harness 编排器同生命周期
(场景间不随 e2e 重启), 规避官方全量容器; 由 autoware_srunner_launch.py 在
autoware_default_adapi_universe 包存在时 include (launch_default_adapi:=false
可关, 供已有常驻 adapi 的环境使用).

闭环说明: operation_mode 订阅 /system/operation_mode/state + availability
(operation_mode_transition_manager 发布), 经 trajectory_publisher 调用
/api/operation_mode/change_to_autonomous -> transition_manager 发布
/autoware/engage=true -> vehicle_cmd_gate 兼容订阅更新 engage ->
/api/autoware/get/engage=true -> AUTONOMOUS 闭环成立 (e2e 侧节点齐备).
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('autoware_default_adapi_universe'),
        'config/default_adapi.param.yaml'])
    container = ComposableNodeContainer(
        namespace='adapi',
        # 与 e2e 侧官方容器 /adapi/container 区分 (其加载链已卡死, 不复用)
        name='srunner_adapi_container',
        package='rclcpp_components',
        executable='component_container_mt',
        ros_arguments=['--log-level', 'adapi.srunner_adapi_container:=WARN'],
        parameters=[{'use_sim_time': True}],
        composable_node_descriptions=[
            ComposableNode(
                namespace='adapi/node',
                name='operation_mode',
                package='autoware_default_adapi_universe',
                plugin='autoware::default_adapi::OperationModeNode',
                parameters=[ParameterFile(config), {'use_sim_time': True}],
            ),
        ],
    )
    return LaunchDescription([container])
