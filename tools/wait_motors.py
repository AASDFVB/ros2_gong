#!/usr/bin/env python3
"""等 can0 上 6 台电机全部应答，再让 ros2_control 启动。

用途：开机自启时，PCAN 刚 up、电机刚上电，ros2_control 一上来就读会导致
"motor N 无应答" → configure 失败 → 整个进程 abort。本脚本在 ExecStartPre
里先探一遍，探到六台都在才放行。

用法
----
    python3 tools/wait_motors.py --timeout 120

退出码
------
    0  六台都应答
    1  超时仍未就绪
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

AF_CAN = 29
CAN_RAW = 1
FRAME = struct.Struct("=IB3x8s")
PROBE_ID = 0x7FF          # 广播 ID，data[0] 里放目标电机 ID
REG_PMAX = 0x15           # 读量程寄存器，应答即说明电机在线


def probe_once(sock: socket.socket, motor_id: int) -> bool:
    """发一帧读寄存器，0.06 s 内收到同 ID 的 0x33 应答就算在线。"""
    while True:
        try:
            sock.recv(16)
        except OSError:
            break
    for _ in range(3):
        try:
            sock.send(FRAME.pack(PROBE_ID, 8, bytes([motor_id, 0, 0x33, REG_PMAX, 0, 0, 0, 0])))
        except OSError:
            return False
        time.sleep(0.003)
    deadline = time.time() + 0.06
    while time.time() < deadline:
        try:
            _, _, data = FRAME.unpack(sock.recv(16))
        except OSError:
            continue
        if (data[0] & 0x0F) == motor_id and data[2] == 0x33:
            return True
    return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--motors", type=int, default=6, help="电机数量（ID 1..N）")
    ap.add_argument("--timeout", type=float, default=120.0, help="最多等多少秒")
    args = ap.parse_args()

    try:
        sock = socket.socket(AF_CAN, socket.SOCK_RAW, CAN_RAW)
        sock.bind((args.interface,))
        sock.settimeout(0.0)
    except OSError as e:
        print(f"打不开 {args.interface}: {e}", file=sys.stderr)
        return 1

    t0 = time.time()
    last_report = 0.0
    while time.time() - t0 < args.timeout:
        online = [i for i in range(1, args.motors + 1) if probe_once(sock, i)]
        if len(online) == args.motors:
            print(f"{args.interface}: {args.motors} 台电机全部应答（等了 {time.time() - t0:.1f}s）")
            sock.close()
            return 0
        now = time.time()
        if now - last_report > 5.0:
            last_report = now
            missing = [i for i in range(1, args.motors + 1) if i not in online]
            print(f"等待中… 在线 {online}，缺 {missing}", flush=True)
        time.sleep(0.5)

    print(f"超时：{args.timeout:.0f}s 内电机未全部应答", file=sys.stderr)
    sock.close()
    return 1


if __name__ == "__main__":
    sys.exit(main())
