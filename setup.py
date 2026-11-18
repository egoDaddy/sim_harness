from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'autoware_srunner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['launch', 'launch.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), ['launch/autoware_srunner_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hcx',
    maintainer_email='wangxuhua110@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'npc_marker_publisher = autoware_srunner.npc_marker_publisher:main',
            'npc_ready_trigger = autoware_srunner.npc_ready_trigger:main',
            'auto_ready_trigger = autoware_srunner.auto_ready_trigger:main',
            'metrics_collector = autoware_srunner.metrics_collector:main',
            'scenario_monitor = autoware_srunner.scenario_monitor:main',
            'sensor_fault_injector = autoware_srunner.sensor_fault_injector:main',
            'destroy_actors = autoware_srunner.destroy_actors:main',
            'wait_for_ready = autoware_srunner.wait_for_ready:main',
            'call_execute_scenario = autoware_srunner.call_execute_scenario:main',
            # ego 注入链工具 (原 scenarios/ego/ 下裸脚本, 属执行器不属场景资产):
            # 注册为 console_scripts 后由 install 空间调用, 不依赖源码目录形状
            'set_route = autoware_srunner.set_route:main',
            'trajectory_publisher = autoware_srunner.trajectory_publisher:main',
            # 跨仓契约校验 (纯文件检查, 不 import rclpy; CI contract stage 用)
            'scenario_contract_check = autoware_srunner.scenario_contract_check:main',
        ],
    },
)
