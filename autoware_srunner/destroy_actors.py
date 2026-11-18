#!/usr/bin/env python3
"""destroy_actors: 销毁 CARLA 世界中的 ego 车辆与传感器 actor (CARLA 服务器保持运行).

联调停止约定: 杀 autoware_carla_interface / ros2 launch 进程之前, 必须先销毁 actor,
否则残留 ego/传感器孤儿会污染下一次启动。本脚本幂等, 可反复执行。

手动执行前需保证 PYTHONPATH 含 CARLA 库:
  export PYTHONPATH=/home/carla_disk/carla/PythonAPI/carla:/home/xuhua/anaconda3/envs/carla_py310/lib/python3.10/site-packages:$PYTHONPATH
  python3 destroy_actors.py --host localhost --port 2000
"""
import argparse

import carla


def main():
    p = argparse.ArgumentParser(description='销毁 CARLA actors (vehicle/sensor), 服务器保持运行')
    p.add_argument('--host', default='localhost')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--timeout', type=float, default=15.0)
    p.add_argument('--filters', default='vehicle.*,sensor.*',
                   help='逗号分隔的 actor 类型过滤器')
    p.add_argument('--keep-roles', default='',
                   help='逗号分隔的 role_name 白名单, 命中则保留不销毁')
    args = p.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()

    keep = set(filter(None, [s.strip() for s in args.keep_roles.split(',')]))
    destroyed = 0
    for pattern in args.filters.split(','):
        pattern = pattern.strip()
        if not pattern:
            continue
        for actor in world.get_actors().filter(pattern):
            if not actor.is_alive:
                continue
            role = actor.attributes.get('role_name', '')
            if keep and role in keep:
                continue
            actor.destroy()
            destroyed += 1
            print('destroyed actor id=%d type=%s role=%s' % (actor.id, actor.type_id, role))

    print('destroy_actors done: %d 个 actor 已销毁, CARLA 服务器保持运行' % destroyed)
    return 0


if __name__ == '__main__':
    main()
