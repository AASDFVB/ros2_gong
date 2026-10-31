#!/usr/bin/env python3
"""实时读取 6 台达妙电机角度。

    q = dir * q_encoder + offset           (来自 robotarm/config/joint_offsets.yaml)

只做寄存器读（0x7FF 广播 + RID 0x51），不使能、不出力、不改任何电机状态。

用法:
    python3 live_angles.py                  # 10 Hz 原地刷新
    python3 live_angles.py --rate 20        # 20 Hz
    python3 live_angles.py --raw            # 同时显示原始编码器角
    python3 live_angles.py --csv log.csv    # 记录到 CSV
    python3 live_angles.py --count 5        # 打印 5 帧后退出（管道/非交互用）
"""

from __future__ import annotations

import argparse
import errno
import math
import os
import pathlib
import shutil
import socket
import struct
import sys
import time
import unicodedata

AF_CAN = getattr(socket, "AF_CAN", 29)
CAN_RAW = getattr(socket, "CAN_RAW", 1)
FRAME = struct.Struct("=IB3x8s")
RID_XOUT = 0x51

DEFAULT_JOINTS = ["J0", "J1", "J2", "J3", "J4", "J5"]
DEFAULT_IDS = [1, 2, 3, 4, 5, 6]
DEFAULT_YAML = pathlib.Path(__file__).resolve().parent.parent / \
    "src/robotarm/config/joint_offsets.yaml"


def load_calibration(path):
    offsets = {j: 0.0 for j in DEFAULT_JOINTS}
    dirs = {j: 1 for j in DEFAULT_JOINTS}
    if not path.exists():
        print("⚠️ 找不到 %s，按 dir=1 offset=0 处理" % path, file=sys.stderr)
        return offsets, dirs
    in_off = in_dir = False
    for line in path.read_text(encoding="utf-8").splitlines():
        s = line.strip()
        if s.startswith("offsets:"):
            in_off, in_dir = True, False
            continue
        if s.startswith("directions:"):
            in_off, in_dir = False, True
            continue
        if s.startswith("can_ids:"):
            in_off = in_dir = False
            continue
        if s.startswith("#") or not s or ":" not in s:
            continue
        k, v = s.split(":", 1)
        k = k.strip()
        try:
            if k not in DEFAULT_JOINTS:
                continue
            if in_off:
                offsets[k] = float(v.strip())
            elif in_dir:
                dirs[k] = int(float(v.strip()))
        except ValueError:
            pass
    return offsets, dirs


def can_is_up(ifname):
    try:
        return open("/sys/class/net/%s/operstate" % ifname).read().strip() == "up"
    except OSError:
        return False


def open_bus(channel):
    s = socket.socket(AF_CAN, socket.SOCK_RAW, CAN_RAW)
    s.bind((channel,))
    s.setsockopt(101, 4, 0)          # CAN_RAW_RECV_OWN_MSGS = 0
    s.settimeout(0.0)
    return s


def read_all(sock, ids, repeats=2, listen=0.018):
    for _ in range(repeats):
        for mid in ids:
            try:
                sock.send(FRAME.pack(0x7FF, 8,
                                     bytes([mid & 0xFF, 0, 0x33, RID_XOUT, 0, 0, 0, 0])))
            except OSError as exc:
                if getattr(exc, "errno", None) == errno.ENOBUFS:
                    time.sleep(0.001)
        time.sleep(0.002)
    got = {}
    end = time.time() + listen
    while time.time() < end and len(got) < len(ids):
        try:
            raw = sock.recv(16)
        except (socket.timeout, BlockingIOError, OSError):
            break
        if len(raw) < 16:
            continue
        cid, _, d = FRAME.unpack(raw)
        if (cid & 0x7FF) == 0x7FF or d[2] != 0x33 or d[3] != RID_XOUT:
            continue
        got[d[0] & 0x0F] = struct.unpack("<f", d[4:8])[0]
    return got


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--ids", default=",".join(str(i) for i in DEFAULT_IDS))
    ap.add_argument("--rate", type=float, default=10.0, help="刷新率 Hz")
    ap.add_argument("--count", type=int, default=0, help="打印 N 帧后退出（0=一直跑）")
    ap.add_argument("--raw", action="store_true", help="额外显示原始编码器角")
    ap.add_argument("--csv", default=None, help="记录到 CSV")
    ap.add_argument("--yaml", default=str(DEFAULT_YAML))
    args = ap.parse_args()

    if not can_is_up(args.channel):
        print("❌ %s 不是 UP 状态。" % args.channel, file=sys.stderr)
        print("   先执行：", file=sys.stderr)
        print("     sudo ip link set %s down" % args.channel, file=sys.stderr)
        print("     sudo ip link set %s type can bitrate 1000000" % args.channel, file=sys.stderr)
        print("     sudo ip link set %s up" % args.channel, file=sys.stderr)
        return 2

    ids = [int(x) for x in args.ids.split(",")]
    joints = DEFAULT_JOINTS[:len(ids)]
    offsets, dirs = load_calibration(pathlib.Path(args.yaml))

    sock = open_bus(args.channel)
    tty = sys.stdout.isatty()
    period = 1.0 / max(args.rate, 0.5)
    q_prev, t_prev = {}, None
    dq = {j: 0.0 for j in joints}
    nrows = len(joints) + (4 if tty else 0)
    frame_i = 0
    csv_f = None
    if args.csv:
        csv_f = open(args.csv, "a", encoding="utf-8")
        if csv_f.tell() == 0:
            csv_f.write("t," + ",".join(joints) + "\n")
    t_start = time.time()
    term_w = shutil.get_terminal_size((80, 24)).columns

    def _w(s):
        """字符串在终端里占几列（中文/全角算 2 列）。"""
        return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in str(s))

    def col(s, width, right=False):
        s = str(s)
        pad = max(width - _w(s), 0)
        return (" " * pad + s) if right else (s + " " * pad)

    try:
        while True:
            t0 = time.time()
            got = read_all(sock, ids)
            missing = [m for m in ids if m not in got]

            q = {}
            for mid, j in zip(ids, joints):
                if mid in got:
                    q[j] = dirs[j] * got[mid] + offsets[j]
                    if t_prev and len(q_prev) == len(joints):
                        raw_dq = (q[j] - q_prev[j]) / (t0 - t_prev)
                        dq[j] += 0.3 * (raw_dq - dq[j])
                else:
                    q[j] = q_prev.get(j, float("nan"))
            q_prev, t_prev = q, t0
            frame_i += 1

            lines = []
            if tty:
                lines.append("─" * term_w)
                lines.append(" 达妙电机实时角度   %s   %.1f Hz   %s"
                             % (args.channel, args.rate, time.strftime("%H:%M:%S")))
                lines.append("─" * term_w)
                lines.append(col("关节", 5) + col("ID", 4, True)
                             + col("编码器(rad)", 14, True) + col("q(rad)", 12, True)
                             + col("q(deg)", 10, True) + col("dq(rad/s)", 11, True))
                for mid, j in zip(ids, joints):
                    enc = got.get(mid, float("nan"))
                    lines.append(col(j, 5) + col(mid, 4, True)
                                 + col("%+.4f" % enc, 14, True)
                                 + col("%+.4f" % q[j], 12, True)
                                 + col("%+.2f" % math.degrees(q[j]), 10, True)
                                 + col("%+.3f" % dq[j], 11, True)
                                 + ("  ⚠无应答" if mid in missing else ""))
                if missing:
                    lines.append("⚠️ 无应答电机 ID: %s" % missing)
            else:
                ts = time.strftime("%H:%M:%S")
                lines.append("%s %s" % (ts, "  ".join(
                    "%s=%+7.3f" % (j, q[j]) for j in joints)))
                if args.raw:
                    lines.append("         raw " + "  ".join(
                        "%s=%+7.3f" % (j, got.get(mid, float("nan")))
                        for mid, j in zip(ids, joints)))

            out = "\n".join(lines)
            if tty:
                if frame_i > 1:
                    sys.stdout.write("\033[%dA" % len(lines))
                sys.stdout.write("\033[K" + "\n\033[K".join(lines) + "\n")
            else:
                sys.stdout.write(out + "\n")
            sys.stdout.flush()

            if csv_f:
                csv_f.write("%.3f," % (t0 - t_start)
                            + ",".join("%.6f" % q[j] for j in joints) + "\n")
                csv_f.flush()

            if args.count and frame_i >= args.count:
                break
            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        print()
    finally:
        sock.close()
        if csv_f:
            csv_f.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
