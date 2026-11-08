#!/usr/bin/env python3
"""按键 → 机械臂1 预设路径。

按一下物理按钮，就把对应的预设轨迹发给 arm_controller 执行。

链路
----
    物理按钮 → HID 模块（0483:5750）→ 本脚本 → FollowJointTrajectory action
                                            → arm_controller
                                            → ros2_control → 臂1

为什么要独占抓取（EVIOCGRAB）
-----------------------------
把按键映射成数字/字母时，这些键会同时传给其他窗口（终端里打字、编辑器插字）。
本脚本对模块的 event 设备做 EVIOCGRAB，之后**只有本进程能看到这些按键**，
其他程序完全收不到——用数字键也绝对安全。

配置文件
--------
见 tools/paths.yaml：

    keys:
      "1": p1
      "2": p2
      "3": p3
      "4": STOP

    paths:
      p1:
        points:
          - {t: 0.0, q: [0, 0, 0, 0, 0, 0]}
          - {t: 3.0, q: [0.2, 0.5, -1.0, 0, 0, 0]}

用法
----
    python3 tools/path_runner.py --config tools/paths.yaml
    python3 tools/path_runner.py --dry-run          # 只打印不动作
"""

from __future__ import annotations

import argparse
import fcntl
import glob
import os
import select
import struct
import sys
import time

import yaml

EVENT_FMT = "llHHi"
EVENT_SIZE = struct.calcsize(EVENT_FMT)
EV_KEY = 1

# ioctl: EVIOCGRAB = _IOW('E', 0x90, int)
EVIOCGRAB = 0x40044590

# 常见键码 → 名字（配置里用名字或数字都可以）
CODE_TO_NAME = {
    2: "1", 3: "2", 4: "3", 5: "4", 6: "5", 7: "6", 8: "7", 9: "8", 10: "9", 11: "0",
    59: "F1", 60: "F2", 61: "F3", 62: "F4", 63: "F5", 64: "F6",
    65: "F7", 66: "F8", 67: "F9", 68: "F10", 87: "F11", 88: "F12",
    1: "ESC", 57: "SPACE", 28: "ENTER",
}
NAME_TO_CODE = {v: k for k, v in CODE_TO_NAME.items()}


def find_module_events() -> list[str]:
    """找出 0483:5750 的所有 input 事件设备。"""
    out = []
    for path in sorted(glob.glob("/dev/input/event*")):
        ev = os.path.basename(path)
        try:
            name = open(f"/sys/class/input/{ev}/device/name", encoding="utf-8").read().strip()
        except OSError:
            continue
        if "0483:5750" in name:
            out.append(path)
    return out


def build_trajectory(joints, points):
    """把配置里的点转成 JointTrajectory。"""
    from trajectory_msgs.msg import JointTrajectoryPoint

    traj_points = []
    for p in points:
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in p["q"]]
        pt.time_from_start.sec = int(p.get("t", 0.0))
        pt.time_from_start.nanosec = int((float(p.get("t", 0.0)) % 1.0) * 1e9)
        traj_points.append(pt)
    return traj_points


def main() -> int:
    ap = argparse.ArgumentParser(description="按键 → 预设路径")
    ap.add_argument("--config", default=os.path.join(os.path.dirname(__file__), "paths.yaml"))
    ap.add_argument("--joints", default="J0,J1,J2,J3,J4,J5")
    ap.add_argument("--action", default="/arm_controller/follow_joint_trajectory")
    ap.add_argument("--dry-run", action="store_true", help="只打印，不真的发轨迹")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config, encoding="utf-8"))
    keymap = {str(k): str(v) for k, v in (cfg.get("keys") or {}).items()}
    paths = cfg.get("paths") or {}
    joints = [j.strip() for j in args.joints.split(",")]

    # 键名统一成键码
    code_to_path = {}
    stop_paths = set()
    for k, target in keymap.items():
        code = NAME_TO_CODE.get(k.upper())
        if code is None:
            try:
                code = int(k)
            except ValueError:
                print(f"⚠️ 配置里的键 {k!r} 不认识，跳过", file=sys.stderr)
                continue
        if target.upper() == "STOP":
            stop_paths.add(code)
        else:
            if target not in paths:
                print(f"⚠️ 键 {k} 指向的路径 {target!r} 不存在", file=sys.stderr)
                continue
            code_to_path[code] = target

    devs = find_module_events()
    if not devs:
        print("找不到 0483:5750 的输入设备（按键模块插好了吗？）", file=sys.stderr)
        return 1

    fds = {}
    grabbed = []
    for d in devs:
        try:
            fd = os.open(d, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as e:
            print(f"打不开 {d}: {e}", file=sys.stderr)
            return 1
        try:
            fcntl.ioctl(fd, EVIOCGRAB, 1)   # 独占：其他程序再也收不到这些按键
            grabbed.append(d)
        except OSError as e:
            print(f"⚠️ 无法独占 {d}: {e}（按键仍会传给其他程序）", file=sys.stderr)
        fds[fd] = d

    print("已独占抓取:", ", ".join(grabbed) if grabbed else "(失败)")
    print("按键映射:")
    for code, p in sorted(code_to_path.items()):
        print(f"  {CODE_TO_NAME.get(code, code)} → {p}")
    if stop_paths:
        print("  停止键: " + ", ".join(str(CODE_TO_NAME.get(c, c)) for c in stop_paths))

    client = None
    node = None
    if not args.dry_run:
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from rclpy.node import Node

        rclpy.init()
        node = Node("path_runner")
        client = ActionClient(node, FollowJointTrajectory, args.action)
        if not client.wait_for_server(timeout_sec=5.0):
            print(f"⚠️ {args.action} 服务不在（arm_controller 没起来？）", file=sys.stderr)
        goal_handle = None

    print("\n等待按键…（Ctrl-C 退出）")
    try:
        while True:
            ready, _, _ = select.select(list(fds), [], [], 0.3)
            for fd in ready:
                data = os.read(fd, EVENT_SIZE * 64)
                for i in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
                    _, _, etype, code, value = struct.unpack(EVENT_FMT, data[i:i + EVENT_SIZE])
                    if etype != EV_KEY or value != 1:      # 只处理"按下"
                        continue

                    if code in stop_paths:
                        print("🛑 停止键 → 取消当前轨迹")
                        if not args.dry_run and goal_handle is not None:
                            goal_handle.cancel_goal_async()
                            goal_handle = None
                        continue

                    name = code_to_path.get(code)
                    if name is None:
                        continue

                    pts = paths[name].get("points") or []
                    print(f"▶️  {name}（{len(pts)} 个点）")
                    if args.dry_run:
                        for p in pts:
                            print("     t=%.1fs q=%s" % (p.get("t", 0.0), p["q"]))
                        continue

                    if not client.server_is_ready():
                        print("   ⚠️ action 服务不可用，跳过")
                        continue

                    goal = FollowJointTrajectory.Goal()
                    goal.trajectory.joint_names = joints
                    goal.trajectory.points = build_trajectory(joints, pts)
                    goal_handle = client.send_goal_async(goal)
                    time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
    finally:
        for fd in fds:
            try: fcntl.ioctl(fd, EVIOCGRAB, 0)
            except OSError: pass
            os.close(fd)
        if node is not None:
            node.destroy_node()
        if not args.dry_run:
            import rclpy
            rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
