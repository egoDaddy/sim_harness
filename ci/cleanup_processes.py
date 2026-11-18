#!/usr/bin/env python3
"""CI 跑批前清场: 杀残留 e2e / autoware / 场景进程, 并释放 Traffic Manager 端口.

背景与失败模式:
- 残留 e2e 会持续提供 /api/routing/set_route_points, 使 scenario_monitor 的
  _routing_service_ready() 误判 "e2e ready" 而秒开跑; 但残留集群不完整
  (缺 /api/operation_mode/change_to_autonomous), 注入链 wait_for_ready
  卡满超时 -> 无 set_route / 无 engage -> ego 不动 -> 全 criteria FAILURE。
- scenario_runner.py 孤儿 (carla_ros_scenario_runner 被 SIGINT 后其 python
  子进程不退出) 会继续持有 Traffic Manager 的 RPC 端口 8000, 使下一轮场景
  在 "trying to create rpc server for traffic manager" 处直接 ERROR。
- 本脚本用 /proc 扫描 + 端口 inode 反查, 不依赖 pgrep/ss 等外部工具,
  并排除自身祖先链, 避免误杀 CI job 的 shell 进程树。
"""
import glob
import os
import signal

# cmdline 匹配模式 (字节串); 命中即杀 (自身祖先链除外)
PATTERNS = [
    b"/home/xuhua/autoware/install",
    b"e2e_simulator.launch.xml",
    b"autoware_srunner_launch.py",
    b"lib/autoware_srunner/scenario_monitor",
    b"carla_ros_scenario_runner",
    b"lib/autoware_srunner/npc_marker_publisher",
    b"rclcpp_components/component_container",
    b"rviz2",
    b"scenario_runner-0.9.16/scenario_runner.py",
]

# Traffic Manager RPC 端口 (8000 起), 监听者同样清掉
TM_PORTS = (8000, 8001, 8002, 8003)


def ancestor_chain(pid):
    """返回 pid 的所有祖先 (含自身), 用于跳过本 job 进程树."""
    chain = set()
    while pid > 1:
        if pid in chain:
            break
        chain.add(pid)
        try:
            pid = int(open("/proc/%d/stat" % pid).read().split()[3])
        except Exception:
            break
    return chain


def tcp_listen_inodes():
    """解析 /proc/net/tcp{,6} 的 LISTEN (st=0A) 行, 返回 {inode: port}."""
    inodes = {}
    for f in ("/proc/net/tcp", "/proc/net/tcp6"):
        try:
            lines = open(f).read().splitlines()[1:]
        except Exception:
            continue
        for ln in lines:
            parts = ln.split()
            if len(parts) < 10 or parts[3] != "0A":
                continue
            port = int(parts[1].split(":")[1], 16)
            if port in TM_PORTS:
                inodes[parts[9]] = port
    return inodes


def pid_for_inode(inode):
    """扫描 /proc/*/fd 找持有该 socket inode 的进程 pid."""
    for fd in glob.glob("/proc/[0-9]*/fd/*"):
        try:
            target = os.readlink(fd)
        except Exception:
            continue
        if "socket:[%s]" % inode in target:
            return int(fd.split("/")[2])
    return None


def main():
    skip = {os.getpid(), os.getppid()} | ancestor_chain(os.getppid())
    killed = 0

    # 1) cmdline pattern 清场
    for p in glob.glob("/proc/[0-9]*/cmdline"):
        try:
            raw = open(p, "rb").read().replace(b"\0", b" ")
            pid = int(p.split("/")[2])
        except Exception:
            continue
        if pid in skip or not any(k in raw for k in PATTERNS):
            continue
        try:
            os.kill(pid, signal.SIGKILL)
            killed += 1
            print("  killed %d: %s" % (pid, raw[:100].decode("utf8", "replace")))
        except ProcessLookupError:
            pass

    # 2) TM RPC 端口占用者清场
    for inode, port in tcp_listen_inodes().items():
        pid = pid_for_inode(inode)
        if pid is None or pid in skip:
            continue
        try:
            os.kill(pid, signal.SIGKILL)
            killed += 1
            print("  freed :%d (pid %d)" % (port, pid))
        except ProcessLookupError:
            pass

    print("  清场完成, 共杀 %d 个残留进程" % killed)


if __name__ == "__main__":
    main()
