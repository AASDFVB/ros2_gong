#!/usr/bin/env python3
"""达妙电机 MIT 反馈帧实时监控。

反馈帧 8 字节（CAN ID = MST_ID）:
    D[0] = ERR<<4 | ID      D[1:2] = POS   D[3:4] = VEL
    D[4:5] = TORQUE         D[6]   = T_MOS D[7]   = T_Rotor

安全性：全程只发 p_des=当前位置, v=0, kp=0, kd=0, t=0 的**零力矩帧**，
电机不使能（状态应为"禁用"）、输出力矩恒为 0。

用法:
    python3 live_feedback.py                  # 20 Hz 刷新
    python3 live_feedback.py --rate 40        # 40 Hz
    python3 live_feedback.py --raw            # 显示原始 8 字节
    python3 live_feedback.py --csv fb.csv     # 记录
    python3 live_feedback.py --count 5        # 打印 5 帧退出（非交互）
"""

from __future__ import annotations

import argparse
import errno
import math
import socket
import struct
import sys
import time
import unicodedata

AF_CAN = getattr(socket, "AF_CAN", 29)
CAN_RAW = getattr(socket, "CAN_RAW", 1)
FRAME = struct.Struct("=IB3x8s")
RID_MST, RID_PMAX, RID_VMAX, RID_TMAX, RID_XOUT = 7, 21, 22, 23, 81
DEFAULT_JOINTS = ["J0", "J1", "J2", "J3", "J4", "J5"]
DEFAULT_IDS = [1, 2, 3, 4, 5, 6]
ERR = {0x0: "禁用", 0x1: "使能", 0x8: "过压", 0x9: "欠压", 0xA: "过流",
       0xB: "MOS超温", 0xC: "线圈超温", 0xD: "通信丢失", 0xE: "过载"}


def iface_up(ch):
    try:
        return open("/sys/class/net/%s/operstate" % ch).read().strip() == "up"
    except OSError:
        return False


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
        except OSError as exc:
            if getattr(exc, "errno", None) == errno.ENOBUFS:
                time.sleep(0.001)
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
            self.send(0x7FF, [mid, 0x00, 0x33, rid, 0, 0, 0, 0])
            time.sleep(0.002)
        end = time.time() + listen
        while time.time() < end:
            try:
                raw = self.s.recv(16)
            except Exception:
                break
            if len(raw) < 16:
                continue
            cid, _, d = FRAME.unpack(raw)
            if (cid & 0x7FF) == 0x7FF or d[2] != 0x33 or d[3] != rid:
                continue
            if (d[0] & 0x0F) != mid:
                continue
            return struct.unpack("<I", d[4:8])[0]    # 原始 4 字节，由调用方决定怎么解
        return None


def as_float(u32):
    return struct.unpack("<f", struct.pack("<I", u32))[0]


def zero_torque_frame(p_des, pmax, vmax, tmax):
    p = f2u(p_des, -pmax, pmax, 16)
    v = f2u(0.0, -vmax, vmax, 12)
    kp = f2u(0.0, 0.0, 500.0, 12)
    kd = f2u(0.0, 0.0, 5.0, 12)
    t = f2u(0.0, -tmax, tmax, 12)
    return bytes([p >> 8, p & 0xFF, v >> 4, ((v & 0xF) << 4) | (kp >> 8),
                  kp & 0xFF, kd >> 4, ((kd & 0xF) << 4) | (t >> 8), t & 0xFF])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--ids", default=",".join(str(i) for i in DEFAULT_IDS))
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--count", type=int, default=0)
    ap.add_argument("--raw", action="store_true")
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    if not iface_up(args.channel):
        print("❌ %s 不是 UP。先执行：" % args.channel)
        print("   sudo ip link set %s down" % args.channel)
        print("   sudo ip link set %s type can bitrate 1000000" % args.channel)
        print("   sudo ip link set %s up" % args.channel)
        return 2

    ids = [int(x) for x in args.ids.split(",")]
    joints = DEFAULT_JOINTS[:len(ids)]
    bus = Bus(args.channel)

    # 启动时逐台读限幅和 MST_ID —— 解码必须用各自的值
    # 注意: MST_ID/ESC_ID 是整数寄存器，PMAX/VMAX/TMAX/xout 是 float
    cfg = {}
    for mid in ids:
        mst = bus.read_reg(mid, RID_MST)
        pmax = bus.read_reg(mid, RID_PMAX)
        vmax = bus.read_reg(mid, RID_VMAX)
        tmax = bus.read_reg(mid, RID_TMAX)
        xout = bus.read_reg(mid, RID_XOUT)
        cfg[mid] = {
            "mst": int(mst) if mst else (0x10 | mid),
            "pmax": as_float(pmax) if pmax else 12.5,
            "vmax": as_float(vmax) if vmax else 30.0,
            "tmax": as_float(tmax) if tmax else 10.0,
            "p": as_float(xout) if xout else 0.0,
        }
    print("已读取限幅: " + "  ".join(
        "%s(P=%.4g V=%.4g T=%.4g MST=0x%02X)" %
        (j, cfg[m]["pmax"], cfg[m]["vmax"], cfg[m]["tmax"], cfg[m]["mst"])
        for j, m in zip(joints, ids)))

    tty = sys.stdout.isatty()
    tw = 86

    def _w(s):
        return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in str(s))

    def col(s, width, right=False):
        s = str(s)
        pad = max(width - _w(s), 0)
        return (" " * pad + s) if right else (s + " " * pad)

    last = {mid: None for mid in ids}
    nframe = 0
    t_start = time.time()
    period = 1.0 / max(args.rate, 0.5)
    csv_f = open(args.csv, "a", encoding="utf-8") if args.csv else None
    if csv_f and csv_f.tell() == 0:
        csv_f.write("t,id,pos,vel,torque,t_mos,t_rotor,err\n")

    lines_prev = 0
    t_loop_prev = None
    try:
        while True:
            t0 = time.time()
            bus.drain()
            for mid in ids:
                bus.send(mid, zero_torque_frame(cfg[mid]["p"], cfg[mid]["pmax"],
                                                cfg[mid]["vmax"], cfg[mid]["tmax"]))
            end = time.time() + 0.018
            while time.time() < end:
                try:
                    raw = bus.s.recv(16)
                except BlockingIOError:
                    time.sleep(0.0004)      # 非阻塞 socket：没数据就等一会，别退出
                    continue
                except Exception:
                    break
                if len(raw) < 16:
                    continue
                cid, dlc, d = FRAME.unpack(raw)
                if dlc < 8:
                    continue
                for mid in ids:
                    if (cid & 0x7FF) == cfg[mid]["mst"]:
                        pmax, vmax, tmax = cfg[mid]["pmax"], cfg[mid]["vmax"], cfg[mid]["tmax"]
                        st = d[0]
                        ident = st & 0x0F
                        err = st >> 4
                        pos = u2f((d[1] << 8) | d[2], -pmax, pmax, 16)
                        vel = u2f((d[3] << 4) | (d[4] >> 4), -vmax, vmax, 12)
                        tor = u2f(((d[4] & 0xF) << 8) | d[5], -tmax, tmax, 12)
                        last[mid] = dict(id=ident, err=err, pos=pos, vel=vel,
                                         tor=tor, tmos=d[6], trot=d[7],
                                         hexd=bytes(d).hex(" "))
                        cfg[mid]["p"] = pos
                        nframe += 1
                        break

            dt = time.time() - t0
            t_loop = time.time()
            loop_dt = (t_loop - t_loop_prev) if t_loop_prev else period
            t_loop_prev = t_loop
            lines = []
            if tty:
                now = time.strftime("%H:%M:%S")
                lines.append("─" * tw)
                lines.append(" 达妙反馈帧监控  %s  %.1f Hz  %s  帧数=%d"
                             % (args.channel, 1.0 / max(loop_dt, 1e-6), now, nframe))
                lines.append("─" * tw)
                lines.append(col("关节", 6) + col("ID", 4, True) + col("状态", 9)
                             + col("位置(rad)", 11, True) + col("位置(°)", 9, True)
                             + col("速度", 9, True) + col("力矩", 9, True)
                             + col("MOS℃", 6, True) + col("线圈℃", 7, True))
                for j, mid in zip(joints, ids):
                    e = last[mid]
                    if not e:
                        lines.append(col(j, 6) + col(mid, 4, True) + "  无应答")
                        continue
                    bad = e["err"] not in (0x0, 0x1)
                    lines.append(col(j, 6) + col(mid, 4, True)
                                 + col(ERR.get(e["err"], "0x%X" % e["err"]), 9)
                                 + col("%+.4f" % e["pos"], 11, True)
                                 + col("%+.2f" % math.degrees(e["pos"]), 9, True)
                                 + col("%+.3f" % e["vel"], 9, True)
                                 + col("%+.3f" % e["tor"], 9, True)
                                 + col(e["tmos"], 6, True) + col(e["trot"], 7, True)
                                 + ("  ⚠" if bad else ""))
                if args.raw:
                    for j, mid in zip(joints, ids):
                        e = last[mid]
                        if e:
                            lines.append("  %s %s" % (j, e["hexd"]))
            else:
                ts = time.strftime("%H:%M:%S")
                for j, mid in zip(joints, ids):
                    e = last[mid]
                    lines.append("%s %s id=%d %s pos=%+.4f vel=%+.3f tor=%+.3f MOS=%d 线圈=%d%s"
                                 % (ts, j, mid, ERR.get(e["err"], "?") if e else "无应答",
                                    e["pos"] if e else float("nan"),
                                    e["vel"] if e else float("nan"),
                                    e["tor"] if e else float("nan"),
                                    e["tmos"] if e else -1, e["trot"] if e else -1,
                                    " raw=" + e["hexd"] if (e and args.raw) else ""))

            out = "\n".join(lines)
            if tty:
                if lines_prev:
                    sys.stdout.write("\033[%dA" % lines_prev)
                sys.stdout.write("\033[K" + "\n\033[K".join(lines) + "\n")
                lines_prev = len(lines)
            else:
                sys.stdout.write(out + "\n")
            sys.stdout.flush()

            if csv_f:
                for mid in ids:
                    e = last[mid]
                    if e:
                        csv_f.write("%.3f,%d,%.5f,%.4f,%.4f,%d,%d,%d\n"
                                    % (time.time() - t_start, mid, e["pos"], e["vel"],
                                       e["tor"], e["tmos"], e["trot"], e["err"]))
                csv_f.flush()

            if args.count:
                args.count -= 1
                if args.count <= 0:
                    break
            rest = period - (time.time() - t0)
            if rest > 0:
                time.sleep(rest)
    except KeyboardInterrupt:
        print()
    finally:
        bus.s.close()
        if csv_f:
            csv_f.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
