#!/usr/bin/env python3
"""6 轴 robotarm 重力补偿（Pinocchio g(q) + 达妙 MIT 力矩帧）。

原理
----
    τ_joint = g(q)                      引脚诺 g(q)，抵消重力所需的关节力矩
    τ_motor = dir * τ_joint             映射回电机侧（dir 见 joint_offsets.yaml）
    MIT 帧:  p_des = 当前位置, v_des = 0, kp, kd, t_ff = τ_motor

安全
----
* 默认 **dry-run**：只读角度、只打印 g(q)，不使能、不出力
* --send 才真正使能并下发；力矩从 0 按 --ramp 秒线性渐入
* 退出（含 Ctrl-C）先发 0.5 s 零力矩，再失能
* 任一台电机报错状态 或 CAN 连续丢帧 → 立刻零力矩 + 失能
* 力矩限幅 --limit（默认 ±3 N·m，实测全臂重力矩峰值约 3.6）

用法
----
    python3 gravity_comp.py                          # 试算，打印各关节 g(q)
    python3 gravity_comp.py --send                   # 真出力（仅重力，软如无重力）
    python3 gravity_comp.py --send --kp 3 --kd 0.5   # 带位置保持
    python3 gravity_comp.py --send --scale 0.5 --limit 2
"""

from __future__ import annotations

import argparse
import errno
import math
import pathlib
import socket
import struct
import sys
import time

import numpy as np
import pinocchio as pin

AF_CAN = getattr(socket, "AF_CAN", 29)
CAN_RAW = getattr(socket, "CAN_RAW", 1)
FRAME = struct.Struct("=IB3x8s")
RID_MST, RID_PMAX, RID_VMAX, RID_TMAX, RID_XOUT = 7, 21, 22, 23, 81
JOINTS = ["J0", "J1", "J2", "J3", "J4", "J5"]
IDS = [1, 2, 3, 4, 5, 6]
DEFAULT_URDF = pathlib.Path(__file__).resolve().parent.parent / \
    "src/robotarm/urdf/robotarm.urdf"
DEFAULT_YAML = pathlib.Path(__file__).resolve().parent.parent / \
    "src/robotarm/config/joint_offsets.yaml"
DEFAULT_CFG = pathlib.Path(__file__).resolve().parent.parent / \
    "src/robotarm/config/gravity_comp.yaml"
ENABLE = bytes([0xFF] * 7 + [0xFC])
DISABLE = bytes([0xFF] * 7 + [0xFD])


def load_cal(path):
    offs = {j: 0.0 for j in JOINTS}
    dirs = {j: 1 for j in JOINTS}
    block = None
    for line in path.read_text(encoding="utf-8").splitlines():
        s = line.strip()
        if s.startswith("offsets:"):
            block = "o"; continue
        if s.startswith("directions:"):
            block = "d"; continue
        if s.startswith("can_ids:"):
            block = None; continue
        if not s or s.startswith("#") or ":" not in s or block is None:
            continue
        k, v = s.split(":", 1)
        k = k.strip()
        if k not in JOINTS:
            continue
        try:
            if block == "o":
                offs[k] = float(v)
            else:
                dirs[k] = int(float(v))
        except ValueError:
            pass
    return offs, dirs


def f2u(x, lo, hi, bits):
    return int((min(max(x, lo), hi) - lo) * ((1 << bits) - 1) / (hi - lo))


def u2f(v, lo, hi, bits):
    return v * (hi - lo) / ((1 << bits) - 1) + lo


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
    ap.add_argument("--urdf", default=str(DEFAULT_URDF))
    ap.add_argument("--yaml", default=str(DEFAULT_YAML))
    ap.add_argument("--config", default=None,
                    help="重力补偿参数 yaml（默认 src/robotarm/config/gravity_comp.yaml）")
    ap.add_argument("--send", action="store_true", help="真正使能并出力")
    ap.add_argument("--scale", type=float, default=None, help="重力补偿系数 0~1（默认读 yaml）")
    ap.add_argument("--limit", type=float, default=None, help="力矩限幅 N·m（默认读 yaml）")
    ap.add_argument("--kp", type=float, default=None)
    ap.add_argument("--kd", type=float, default=None)
    ap.add_argument("--ramp", type=float, default=None, help="渐入秒数（默认读 yaml）")
    ap.add_argument("--rate", type=float, default=None)
    ap.add_argument("--gain", default=None,
                    help="逐关节重力补偿增益，如 J1=1.2,J2=1.1（默认全 1.0）；"
                         "某轴下坠就加大，往上飘就减小")
    ap.add_argument("--hold", action="store_true",
                    help="锁存使能瞬间的角度作为目标，配合 --kp 做位置保持")
    ap.add_argument("--only", default=None,
                    help="只对指定关节做重力补偿（如 J1 或 J1,J2），"
                         "其余关节不使能、不出力")
    args = ap.parse_args()

    # ---------------- 读 yaml 配置（命令行显式给出的优先）
    cfg_path = pathlib.Path(args.config) if args.config else DEFAULT_CFG
    ycfg = {}
    if cfg_path.exists():
        try:
            import yaml as _yaml
            ycfg = _yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
        except Exception as exc:
            print("⚠️ 读取 %s 失败: %s" % (cfg_path, exc))
    common_cfg = ycfg.get("common", {}) or {}
    gains_yaml = ycfg.get("gains", {}) or {}
    kp_yaml = ycfg.get("kp", {}) or {}
    kd_yaml = ycfg.get("kd", {}) or {}

    args.scale = float(common_cfg.get("scale", 1.0)) if args.scale is None else args.scale
    args.limit = float(common_cfg.get("limit", 3.0)) if args.limit is None else args.limit
    args.ramp = float(common_cfg.get("ramp", 3.0)) if args.ramp is None else args.ramp
    args.rate = float(common_cfg.get("rate", 100.0)) if args.rate is None else args.rate
    if args.only is None and common_cfg.get("active"):
        args.only = ",".join(str(x) for x in common_cfg["active"])
    gains = {j: float(gains_yaml.get(j, 1.0)) for j in JOINTS}
    if args.gain:
        for item in args.gain.split(","):
            if "=" not in item:
                print("❌ --gain 格式应为 J1=1.2,J2=1.1"); return 1
            k, v = item.split("=", 1)
            k = k.strip().upper()
            if k not in JOINTS:
                print("❌ 未知关节 %s" % k); return 1
            gains[k] = float(v)

    # 逐关节 kp/kd；命令行若给了 --kp/--kd，则统一覆盖六个轴
    kp_map = {j: (float(args.kp) if args.kp is not None
                  else float(kp_yaml.get(j, 0.0))) for j in JOINTS}
    kd_map = {j: (float(args.kd) if args.kd is not None
                  else float(kd_yaml.get(j, 0.0))) for j in JOINTS}

    print("参数: scale=%.2f limit=%.1f ramp=%.1fs rate=%.0fHz  (来自 %s)"
          % (args.scale, args.limit, args.ramp, args.rate,
             cfg_path.name if cfg_path.exists() else "命令行"))
    print("  关节   gains      kp      kd")
    for j in JOINTS:
        print("   %-4s  %5.2f   %6.2f  %6.2f" % (j, gains[j], kp_map[j], kd_map[j]))

    active = set(JOINTS) if not args.only else {x.strip() for x in args.only.split(",")}
    bad = active - set(JOINTS)
    if bad:
        print("❌ 未知关节: %s（可选 %s）" % (bad, JOINTS)); return 1
    print("补偿关节: %s   （其余不使能）" % ",".join(j for j in JOINTS if j in active))

    offs, dirs = load_cal(pathlib.Path(args.yaml))
    model = pin.buildModelFromUrdf(args.urdf)
    data = model.createData()
    names = [n for n in model.names if n != "universe"]
    if names != JOINTS:
        print("❌ URDF 关节顺序不符: %s" % names); return 1
    print("pinocchio 模型: nq=%d  重力 |g|=%.2f m/s²" % (model.nq, np.linalg.norm(model.gravity.linear)))

    bus = Bus(args.channel)
    cfg = {}
    for j, mid in zip(JOINTS, IDS):
        mst = bus.read_reg(mid, RID_MST)
        pmax = bus.read_reg(mid, RID_PMAX)
        vmax = bus.read_reg(mid, RID_VMAX)
        tmax = bus.read_reg(mid, RID_TMAX)
        cfg[j] = dict(mst=int(mst) if mst else 0x10 | mid,
                      pmax=as_float(pmax) if pmax else 12.5,
                      vmax=as_float(vmax) if vmax else 30.0,
                      tmax=as_float(tmax) if tmax else 10.0)

    def read_q():
        enc = {}
        for j, mid in zip(JOINTS, IDS):
            u = bus.read_reg(mid, RID_XOUT)
            if u is None:
                return None
            enc[j] = as_float(u)
        return np.array([dirs[j] * enc[j] + offs[j] for j in JOINTS])

    q = read_q()
    if q is None:
        print("❌ 读角度失败"); return 2
    g = pin.computeGeneralizedGravity(model, data, q)
    tau_m = np.array([dirs[j] * g[i] for i, j in enumerate(JOINTS)])

    print("\n%-4s %9s %10s %10s %9s %9s" %
          ("关节", "q(rad)", "q(°)", "g(q) N·m", "τ_motor", "限幅后"))
    for i, j in enumerate(JOINTS):
        if j in active:
            t_g = tau_m[i] * args.scale * gains[j]
            lim = math.copysign(min(abs(t_g), args.limit), t_g)
            mark = ""
        else:
            lim = 0.0
            mark = "  ← 不补偿"
        print("%-4s %+9.3f %+10.1f %+10.3f %+9.3f %+9.3f  (TMAX=%.0f, gain=%.2f)%s" %
              (j, q[i], math.degrees(q[i]), g[i], tau_m[i], lim, cfg[j]["tmax"],
               gains[j], mark))
    print("\n合计 |τ| = %.2f N·m，最大单轴 %.2f N·m，限幅 ±%.1f" %
          (np.abs(tau_m).sum(), np.abs(tau_m).max(), args.limit))

    if not open("/sys/class/net/%s/operstate" % args.channel).read().strip() == "up":
        print("❌ %s 不是 UP 状态（配置已打印，先修总线）" % args.channel); return 2

    if not args.send:
        print("\n（dry-run：未使能、未出力。确认数值和符号合理后加 --send）")
        return 0

    # ---------------------------------------------------------- 出力
    print("\n⚠️  即将使能并出力：scale=%.2f ramp=%.1fs" % (args.scale, args.ramp))
    print("   kp/kd 逐轴：" + "  ".join(
        "%s=%.1f/%.2f" % (j, kp_map[j], kd_map[j]) for j in JOINTS))
    print("   3 秒后开始，Ctrl-C 随时退出（会先零力矩再失能）")
    try:
        for k in range(3, 0, -1):
            print("   %d…" % k); time.sleep(1)
    except KeyboardInterrupt:
        return 0

    # 锁存保持目标（使能前最后读一次）
    q_hold = read_q()
    if q_hold is None:
        print("❌ 读角度失败，放弃使能"); return 2
    if args.hold:
        print("保持目标: " + "  ".join("%s=%+.1f°" % (j, math.degrees(q_hold[i]))
                                       for i, j in enumerate(JOINTS) if j in active))

    for j, mid in zip(JOINTS, IDS):
        if j in active:
            bus.send(mid, ENABLE)
    time.sleep(0.05)
    bus.drain()

    period = 1.0 / args.rate
    t_start = time.time()
    fails = 0
    last_print = 0.0
    try:
        while True:
            t0 = time.time()
            el = t0 - t_start
            ramp = min(el / max(args.ramp, 1e-3), 1.0)
            q = read_q()
            if q is None:
                fails += 1
                if fails > 5:
                    print("❌ CAN 连续丢帧，紧急停止"); break
                time.sleep(period); continue
            fails = 0
            g = pin.computeGeneralizedGravity(model, data, q)
            bus.drain()
            for i, (j, mid) in enumerate(zip(JOINTS, IDS)):
                if j in active:
                    tau = dirs[j] * g[i] * gains[j] * args.scale * ramp
                    tau = math.copysign(min(abs(tau), args.limit), tau)
                    kp_j, kd_j = kp_map[j], kd_map[j]
                else:
                    tau, kp_j, kd_j = 0.0, 0.0, 0.0
                # MIT 帧的位置字段是电机侧角度，需要从关节角反算
                q_ref = q_hold[i] if args.hold else q[i]
                p_des = dirs[j] * (q_ref - offs[j])
                bus.send(mid, mit_frame(p_des, kp_j, kd_j, tau,
                                        cfg[j]["pmax"], cfg[j]["vmax"], cfg[j]["tmax"]))
            if t0 - last_print > 1.0:
                last_print = t0
                msg = "  t=%4.1fs ramp=%.2f  " % (el, ramp)
                parts = []
                for i, j in enumerate(JOINTS):
                    if j not in active:
                        continue
                    tff = dirs[j] * g[i] * gains[j] * args.scale * ramp
                    err = (q_hold[i] - q[i]) if args.hold else 0.0
                    tkp = kp_map[j] * err
                    tot = dirs[j] * (g[i] * gains[j] * args.scale * ramp + tkp)
                    tot = math.copysign(min(abs(tot), args.limit), tot)
                    parts.append("%s: t_ff%+.2f + kp%+.2f = %+.2f%s" %
                                 (j, tff, dirs[j] * tkp, tot,
                                  "  [限幅]" if abs(tot) >= args.limit * 0.999 else ""))
                    if args.hold:
                        parts[-1] += "  err%+.1f°" % math.degrees(-err)
                msg += "   ".join(parts)
                print(msg)
            rest = period - (time.time() - t0)
            if rest > 0:
                time.sleep(rest)
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
    finally:
        print("零力矩 + 失能…")
        for _ in range(20):
            for j, mid in zip(JOINTS, IDS):
                bus.send(mid, mit_frame(0.0, 0.0, 0.0, 0.0,
                                        cfg[j]["pmax"], cfg[j]["vmax"], cfg[j]["tmax"]))
            time.sleep(0.01)
        for j, mid in zip(JOINTS, IDS):
            if j in active:
                bus.send(mid, DISABLE)
        bus.s.close()
        print("已停止")
    return 0


if __name__ == "__main__":
    sys.exit(main())
