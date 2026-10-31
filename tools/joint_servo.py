#!/usr/bin/env python3
"""订阅 /joint_cmd 驱动真机：位置伺服 + 重力补偿（达妙 MIT 帧）。

    τ = kp·(q_cmd − q) + kd·(0 − q̇) + g(q)·gain

把 /joint_cmd 的关节角当作目标，限速后下发。
同时把实测角发到 /joint_states，RViz 里看到的是真机。

安全：
  * 必须 --send 才使能；前面有 3 秒倒计时
  * 目标限速（默认 30°/s），滑条拉再快手臂也按匀速走
  * 力矩限幅、丢帧急停、指令超时保持、Ctrl-C 零力矩+失能

用法:
    # 终端1：看数据（不使能）
    python3 joint_servo.py
    # 终端2：发目标（弧度）
    ros2 topic pub --once /joint_cmd sensor_msgs/msg/JointState \
      "{name: [J0,J1,J2,J3,J4,J5], position: [0.5,0,0,0,0,0]}"
"""

from __future__ import annotations

import argparse
import errno
import math
import pathlib
import socket
import struct
import sys
import threading
import time

import numpy as np
import pinocchio as pin
import yaml

AF_CAN = getattr(socket, "AF_CAN", 29)
CAN_RAW = getattr(socket, "CAN_RAW", 1)
FRAME = struct.Struct("=IB3x8s")
RID_MST, RID_PMAX, RID_VMAX, RID_TMAX, RID_P_M, RID_XOUT = 7, 21, 22, 23, 80, 81
JOINTS = ["J0", "J1", "J2", "J3", "J4", "J5"]
IDS = [1, 2, 3, 4, 5, 6]
ENABLE = bytes([0xFF] * 7 + [0xFC])
DISABLE = bytes([0xFF] * 7 + [0xFD])

ROOT = pathlib.Path(__file__).resolve().parent.parent
DEF_URDF = ROOT / "src/robotarm/urdf/robotarm.urdf"
DEF_CAL = ROOT / "src/robotarm/config/joint_offsets.yaml"
DEF_CFG = ROOT / "src/robotarm/config/joint_servo.yaml"


def f2u(x, lo, hi, bits):
    return int((min(max(x, lo), hi) - lo) * ((1 << bits) - 1) / (hi - lo))


def u2f(v, lo, hi, bits):
    return v * (hi - lo) / ((1 << bits) - 1) + lo


def load_yaml(path):
    try:
        return yaml.safe_load(pathlib.Path(path).read_text(encoding="utf-8")) or {}
    except Exception as e:
        print("⚠️ 读 %s 失败: %s" % (path, e))
        return {}


class Bus:
    def __init__(self, ch):
        self.s = socket.socket(AF_CAN, socket.SOCK_RAW, CAN_RAW)
        self.s.bind((ch,))
        self.s.setsockopt(101, 4, 0)
        self.s.settimeout(0.0)

    def send(self, cid, data):
        try:
            self.s.send(FRAME.pack(cid & 0x7FF, len(data), bytes(data)))
            return True
        except OSError:
            return False

    def drain(self):
        while True:
            try:
                self.s.recv(16)
            except Exception:
                return

    def read_reg(self, mid, rid, listen=0.02):
        self.drain()
        for _ in range(2):
            self.send(0x7FF, [mid, 0, 0x33, rid, 0, 0, 0, 0])
            time.sleep(0.002)
        end = time.time() + listen
        while time.time() < end:
            try:
                raw = self.s.recv(16)
            except BlockingIOError:
                time.sleep(0.0003); continue
            except Exception:
                break
            if len(raw) < 16:
                continue
            cid, _, d = FRAME.unpack(raw)
            if (cid & 0x7FF) == 0x7FF or d[2] != 0x33 or d[3] != rid:
                continue
            if (d[0] & 0x0F) != mid:
                continue
            return struct.unpack("<I", d[4:8])[0]
        return None


def as_float(u):
    return struct.unpack("<f", struct.pack("<I", u))[0]


def mit_frame(p, kp, kd, t, pmax, vmax, tmax):
    pi = f2u(p, -pmax, pmax, 16)
    vi = f2u(0.0, -vmax, vmax, 12)
    kpi = f2u(kp, 0.0, 500.0, 12)
    kdi = f2u(kd, 0.0, 5.0, 12)
    ti = f2u(t, -tmax, tmax, 12)
    return bytes([pi >> 8, pi & 0xFF, vi >> 4, ((vi & 0xF) << 4) | (kpi >> 8),
                  kpi & 0xFF, kdi >> 4, ((kdi & 0xF) << 4) | (ti >> 8), ti & 0xFF])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--urdf", default=str(DEF_URDF))
    ap.add_argument("--cal", default=str(DEF_CAL))
    ap.add_argument("--cfg", default=str(DEF_CFG))
    ap.add_argument("--send", action="store_true", help="真正使能并出力")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--max-deg-s", type=float, default=30.0, help="目标限速 度/秒")
    ap.add_argument("--limit", type=float, default=None, help="力矩限幅 N·m（默认读 yaml）")
    ap.add_argument("--kp", type=float, default=None)
    ap.add_argument("--kd", type=float, default=None)
    ap.add_argument("--ramp", type=float, default=None)
    ap.add_argument("--cmd-timeout", type=float, default=2.0,
                    help="超过多久没收到新目标就停止跟随（秒）")
    ap.add_argument("--no-ros", action="store_true", help="不启 ROS，纯 CAN 测试")
    args = ap.parse_args()

    # ---------------- 配置
    cfg = load_yaml(pathlib.Path(args.cfg))
    common = cfg.get("common", {}) or {}
    gains_y = cfg.get("gains", {}) or {}
    kp_y = cfg.get("kp", {}) or {}
    kd_y = cfg.get("kd", {}) or {}
    limit = args.limit if args.limit is not None else float(common.get("limit", 6.0))
    ramp_t = args.ramp if args.ramp is not None else float(common.get("ramp", 4.0))
    gains = {j: float(gains_y.get(j, 1.0)) for j in JOINTS}

    # 逐关节 kp/kd；命令行给了 --kp/--kd 就统一覆盖六个轴
    kp_map = {j: (float(args.kp) if args.kp is not None
                  else float(kp_y.get(j, 0.0))) for j in JOINTS}
    kd_map = {j: (float(args.kd) if args.kd is not None
                  else float(kd_y.get(j, 0.0))) for j in JOINTS}
    if all(v <= 0 for v in kp_map.values()):
        print("⚠️ 六个轴 kp 都是 0，位置伺服没有意义 → 全部按 5.0 处理")
        kp_map = {j: 5.0 for j in JOINTS}

    print("配置: %s" % pathlib.Path(args.cfg).name)
    print("  关节   gains      kp      kd")
    for j in JOINTS:
        print("   %-4s  %5.2f   %6.2f  %6.2f" % (j, gains[j], kp_map[j], kd_map[j]))

    cal = load_yaml(pathlib.Path(args.cal))
    offs = {j: float((cal.get("offsets") or {}).get(j, 0.0)) for j in JOINTS}
    dirs = {j: int((cal.get("directions") or {}).get(j, 1)) for j in JOINTS}

    model = pin.buildModelFromUrdf(args.urdf)
    data = model.createData()
    if [n for n in model.names if n != "universe"] != JOINTS:
        print("❌ URDF 关节顺序不符"); return 1

    if not open("/sys/class/net/%s/operstate" % args.channel).read().strip() == "up":
        print("❌ %s 不是 UP 状态" % args.channel); return 2

    bus = Bus(args.channel)
    cfg_m = {}
    for j, mid in zip(JOINTS, IDS):
        mst = bus.read_reg(mid, RID_MST)
        pmax = bus.read_reg(mid, RID_PMAX)
        vmax = bus.read_reg(mid, RID_VMAX)
        tmax = bus.read_reg(mid, RID_TMAX)
        r_pm = bus.read_reg(mid, RID_P_M)
        r_xo = bus.read_reg(mid, RID_XOUT)
        pm_to_xout = (as_float(r_xo) - as_float(r_pm)) if (r_pm and r_xo) else 0.0
        cfg_m[j] = dict(mid=mid,
                        mst=int(mst) if mst else 0x10 | mid,
                        pmax=as_float(pmax) if pmax else 12.5,
                        vmax=as_float(vmax) if vmax else 30.0,
                        tmax=as_float(tmax) if tmax else 10.0,
                        pm2xo=pm_to_xout)

    def read_q():
        enc = {}
        for j, mid in zip(JOINTS, IDS):
            u = bus.read_reg(mid, RID_XOUT)
            if u is None:
                return None
            enc[j] = as_float(u)
        return np.array([dirs[j] * enc[j] + offs[j] for j in JOINTS])

    q_now = read_q()
    if q_now is None:
        print("❌ 读角度失败"); return 2

    # 反馈帧解析：非阻塞，把最新一帧写进 q_meas
    q_meas = q_now.copy()
    fb_count = [0]

    def parse_feedback():
        got = 0
        while True:
            try:
                raw = bus.s.recv(16)
            except BlockingIOError:
                break
            except Exception:
                break
            if len(raw) < 16:
                continue
            cid, _, d = FRAME.unpack(raw)
            id_ = cid & 0x7FF
            for i, j in enumerate(JOINTS):
                if id_ != cfg_m[j]["mst"]:
                    continue
                pmax = cfg_m[j]["pmax"]
                p_i = (d[1] << 8) | d[2]
                p_m = u2f(p_i, -pmax, pmax, 16)
                enc = p_m + cfg_m[j]["pm2xo"]        # 修回输出轴
                q_meas[i] = dirs[j] * enc + offs[j]
                got += 1
                break
        fb_count[0] += got
        return got>0
    print("当前: " + " ".join("%s=%+.1f°" % (j, math.degrees(q_now[i]))
                              for i, j in enumerate(JOINTS)))

    # ---------------- 目标（ROS 或固定）
    target = q_now.copy()
    lock = threading.Lock()
    last_cmd_t = [0.0]
    ros_ok = False
    if not args.no_ros:
        try:
            import rclpy
            from sensor_msgs.msg import JointState
            rclpy.init()
            node = rclpy.create_node("joint_servo")
            pub = node.create_publisher(JointState, "/joint_states", 10)

            def cb(msg):
                if not msg.name:
                    return
                with lock:
                    for n, p in zip(msg.name, msg.position):
                        if n in JOINTS:
                            target[JOINTS.index(n)] = float(p)
                    last_cmd_t[0] = time.time()

            node.create_subscription(JointState, "/joint_cmd", cb, 10)
            threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
            ros_ok = True
            print("ROS 已启动：订阅 /joint_cmd，发布 /joint_states")
        except Exception as e:
            print("⚠️ ROS 启动失败（继续纯 CAN 模式）: %s" % e)

    armed = args.send
    if armed:
        print("\n⚠️  3 秒后使能出力：limit=%.1f ramp=%.1fs 限速 %.0f°/s"
              % (limit, ramp_t, args.max_deg_s))
        try:
            for k in (3, 2, 1):
                print("   %d…" % k); time.sleep(1)
        except KeyboardInterrupt:
            return 0
        for j, mid in zip(JOINTS, IDS):
            bus.send(mid, ENABLE)
        time.sleep(0.05)
        bus.drain()
    else:
        print("\n【只读模式】不使能、不出力；只读角度并发布 /joint_states")
        print("  加 --send 才真正驱动真机")

    period = 1.0 / args.rate
    t_start = time.time()
    q_target = q_now.copy()          # 限速后的实际下发目标
    fails = 0
    last_print = 0.0
    max_step = math.radians(args.max_deg_s) * period

    try:
        while True:
            t0 = time.time()
            ramp = min((t0 - t_start) / max(ramp_t, 1e-3), 1.0)
            # 先收上一周期发出的帧的回复（非阻塞）
            if not parse_feedback():
                fails += 1
                if fails > 200:
                    print("❌ 长时间收不到反馈，急停"); break
            else:
                fails = 0
            q = q_meas.copy()

            # 目标限速
            with lock:
                tgt = target.copy()
                fresh = (time.time() - last_cmd_t[0]) < args.cmd_timeout
            if fresh:
                d = np.clip(tgt - q_target, -max_step, max_step)
                q_target = q_target + d

            g = pin.computeGeneralizedGravity(model, data, q)

            for i, (j, mid) in enumerate(zip(JOINTS, IDS)):
                tff = (dirs[j] * g[i] * gains[j] * ramp) if armed else 0.0
                tff = math.copysign(min(abs(tff), limit), tff)
                kp_j, kd_j = (kp_map[j], kd_map[j]) if armed else (0.0, 0.0)
                p_des = dirs[j] * (q_target[i] - offs[j])
                bus.send(mid, mit_frame(p_des, kp_j, kd_j, tff,
                                        cfg_m[j]["pmax"], cfg_m[j]["vmax"], cfg_m[j]["tmax"]))

            if ros_ok:
                m = JointState()
                m.header.stamp = node.get_clock().now().to_msg()
                m.name = JOINTS
                m.position = [float(x) for x in q]
                pub.publish(m)

            if t0 - last_print > 1.0:
                last_print = t0
                err = " ".join("%s%+.1f°" % (j, math.degrees(q_target[i] - q[i]))
                               for i, j in enumerate(JOINTS))
                print("  t=%4.1fs ramp=%.2f  err: %s" % (t0 - t_start, ramp, err))
            rest = period - (time.time() - t0)
            if rest > 0:
                time.sleep(rest)
    except KeyboardInterrupt:
        print("\nCtrl-C")
    finally:
        if not armed:
            bus.s.close()
            if ros_ok:
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
            print("只读模式退出")
            return 0
        print("零力矩 + 失能…")
        for _ in range(20):
            for j, mid in zip(JOINTS, IDS):
                bus.send(mid, mit_frame(0, 0, 0, 0, cfg_m[j]["pmax"],
                                        cfg_m[j]["vmax"], cfg_m[j]["tmax"]))
            time.sleep(0.01)
        for j, mid in zip(JOINTS, IDS):
            bus.send(mid, DISABLE)
        bus.s.close()
        if ros_ok:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        print("已停止")
    return 0


if __name__ == "__main__":
    sys.exit(main())
