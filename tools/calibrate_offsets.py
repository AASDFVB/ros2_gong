#!/usr/bin/env python3
"""6 轴 robotarm 零点标定（编码器零位 -> URDF 关节零位）。

约定
----
    q_joint = dir * q_encoder + offset          (单位 rad)

  * q_encoder: 电机输出轴编码器角（MIT 反馈同一来源，RID 0x51）
  * dir      : +1 / -1，编码器正方向与 URDF 关节正方向的对应关系
  * offset   : 常量，捕获"编码器零位相对 URDF 关节零位"的位置差

安全性
------
本脚本**只做寄存器读**（0x7FF 广播 + RID 0x51），不使能电机、不下发力矩、
不写电机 flash 零位。机械臂可以在悬垂/断电保持状态下安全运行。

用法
----
    # 1) 只读，确认 6 台电机都在线
    python3 calibrate_offsets.py --measure

    # 2) 把实物摆到与 SolidWorks 装配一致的姿态（= URDF 全零位），计算 offset
    python3 calibrate_offsets.py --target 0,0,0,0,0,0 --write

    # 3) 如果某个关节方向反了，改 dir 重算（例如 J3 反）
    python3 calibrate_offsets.py --dirs 1,1,1,-1,1,1 --target 0,0,0,0,0,0 --write

    # 4) 校验：读编码器 + 套用已存的 offset，打印 URDF 关节角
    python3 calibrate_offsets.py --apply
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

AF_CAN = getattr(socket, "AF_CAN", 29)
CAN_RAW = getattr(socket, "CAN_RAW", 1)
FRAME = struct.Struct("=IB3x8s")
RID_XOUT = 0x51          # 输出轴位置 (rad)，与 MIT 反馈一致
RID_P_M = 0x50           # 转子位置 (rad)

DEFAULT_JOINTS = ["J0", "J1", "J2", "J3", "J4", "J5"]
DEFAULT_OUT = pathlib.Path(__file__).resolve().parent.parent / \
    "src/robotarm/config/joint_offsets.yaml"


# ---------------------------------------------------------------- CAN
def open_bus(channel: str) -> socket.socket:
    s = socket.socket(AF_CAN, socket.SOCK_RAW, CAN_RAW)
    s.bind((channel,))
    s.setsockopt(101, 4, 0)          # CAN_RAW_RECV_OWN_MSGS = 0
    s.settimeout(0.0)
    return s


def read_reg(sock, motor_id: int, rid: int, listen: float = 0.035):
    """只读一个 float 寄存器；返回 None 表示无应答。"""
    query = bytes([motor_id & 0xFF, 0x00, 0x33, rid, 0, 0, 0, 0])
    for _ in range(4):
        try:
            sock.send(FRAME.pack(0x7FF, 8, query))
        except OSError as exc:
            if getattr(exc, "errno", None) == errno.ENOBUFS:
                time.sleep(0.002)
                continue
            raise
        time.sleep(0.004)
    end = time.time() + listen
    while time.time() < end:
        try:
            raw = sock.recv(16)
        except (socket.timeout, BlockingIOError, OSError):
            break
        if len(raw) < 16:
            continue
        cid, _, d = FRAME.unpack(raw)
        cid &= 0x7FF
        if cid == 0x7FF:                       # 自己的查询帧
            continue
        if d[2] != 0x33 or d[3] != rid:
            continue
        if (d[0] & 0x0F) != (motor_id & 0x0F):
            continue
        return cid, struct.unpack("<f", d[4:8])[0]
    return None, None


def measure(channel, ids, verbose=False):
    sock = open_bus(channel)
    enc, rotor = {}, {}
    try:
        for mid in ids:
            err_cid, x = read_reg(sock, mid, RID_XOUT)
            _, pm = read_reg(sock, mid, RID_P_M)
            if x is None:
                print("  motor %-3d (0x%02X): 无应答" % (mid, mid))
                continue
            enc[mid] = x
            rotor[mid] = pm
            if verbose:
                print("  motor %-3d 回复 CID=0x%03X  xout=%+.4f rad (%+8.2f deg)  p_m=%s"
                      % (mid, err_cid, x, math.degrees(x),
                         "?" if pm is None else "%+.4f" % pm))
    finally:
        sock.close()
    return enc


def check_stability(channel, ids, duration=2.0, tol=0.005):
    """连续采样，判断机械臂是否静止。

    返回 (是否稳定, 各电机峰峰值 dict, 最后一次采样)
    """
    t0 = time.time()
    samples = []
    while time.time() - t0 < duration:
        samples.append(measure(channel, ids))
        time.sleep(0.2)
    last = samples[-1] if samples else {}
    p2p = {}
    for mid in ids:
        vals = [s[mid] for s in samples if mid in s]
        p2p[mid] = (max(vals) - min(vals)) if len(vals) > 1 else 0.0
    ok = bool(last) and all(v <= tol for v in p2p.values())
    return ok, p2p, last


# ---------------------------------------------------------------- YAML
def load_yaml_map(path: pathlib.Path, key: str) -> dict:
    """极简解析，只认 `key:` 下的 `  name: number` 两列。"""
    out = {}
    if not path.exists():
        return out
    in_block = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip().startswith(key + ":"):
            in_block = True
            continue
        if in_block:
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            if not line.startswith((" ", "\t")):
                break
            if ":" not in line:
                continue
            k, v = line.split(":", 1)
            try:
                out[k.strip()] = float(v.split("#")[0].strip())
            except ValueError:
                pass
    return out


def dump_yaml(joints, ids, dirs, offsets) -> str:
    lines = [
        "# 6 轴 robotarm 零点标定结果",
        "#   q_joint = dir * q_encoder + offset      (rad)",
        "#   由 tools/calibrate_offsets.py 生成，重新 launch 生效（--symlink-install 不需要 rebuild）",
        "",
        "offsets:",
    ]
    lines += ["  %-4s: %+.6f" % (j, offsets[j]) for j in joints]
    lines += ["", "directions:"]
    lines += ["  %-4s: %+d" % (j, d) for j, d in zip(joints, dirs)]
    lines += ["", "can_ids:"]
    lines += ["  %-4s: %d" % (j, i) for j, i in zip(joints, ids)]
    return "\n".join(lines) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--ids", default="1,2,3,4,5,6", help="6 台电机的 CAN ID")
    ap.add_argument("--joints", default=",".join(DEFAULT_JOINTS))
    ap.add_argument("--dirs", default="1,1,1,1,1,1", help="各关节 dir，+1/-1")
    ap.add_argument("--target", default="0,0,0,0,0,0",
                    help="当前实物姿态对应的 URDF 关节角（rad）。"
                         "全 0 = SolidWorks 装配姿态")
    ap.add_argument("--measure", action="store_true", help="只读打印编码器角")
    ap.add_argument("--stable", type=float, default=0.0, metavar="SEC",
                    help="先采样 SEC 秒确认机械臂静止；不满足则不写入")
    ap.add_argument("--tol", type=float, default=0.005,
                    help="稳定性阈值（峰峰值, rad），默认 0.005")
    ap.add_argument("--apply", action="store_true",
                    help="读编码器并套用已存 offset，打印 URDF 关节角")
    ap.add_argument("--write", action="store_true", help="写入 joint_offsets.yaml")
    ap.add_argument("--flip-joints", default=None,
                    help="只翻转指定关节的方向，如 J0,J3（配合 --flip 使用）")
    ap.add_argument("--flip", action="store_true",
                    help="不改动机械臂：把已存的 dir 全部取反、offset 取反"
                         "（整体方向反了时用，零点保持不变）")
    ap.add_argument("--out", default=None, help="输出 yaml 路径")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    joints = [s.strip() for s in args.joints.split(",")]
    ids = [int(s) for s in args.ids.split(",")]
    dirs = [int(float(s)) for s in args.dirs.split(",")]
    target = [float(s) for s in args.target.split(",")]
    n = len(joints)
    if not (len(ids) == len(dirs) == len(target) == n):
        print("error: --ids/--dirs/--target 长度必须等于关节数 %d" % n, file=sys.stderr)
        return 1

    out_path = pathlib.Path(args.out) if args.out else DEFAULT_OUT

    if args.flip or args.flip_joints:
        offs = load_yaml_map(out_path, "offsets")
        dirs_saved = load_yaml_map(out_path, "directions")
        ids_saved = load_yaml_map(out_path, "can_ids")
        if not offs:
            print("❌ %s 里没有 offsets，无法翻转" % out_path, file=sys.stderr)
            return 1
        # q_new = dir_new*enc + off_new 应等于 -q_old = -(dir_old*enc + off_old)
        # 取 dir_new = -dir_old 时, off_new = -off_old, 零点位置完全不变
        which = set(joints) if not args.flip_joints else \
            {x.strip() for x in args.flip_joints.split(",")}
        new_dirs = [(-1 if j in which else 1) * int(dirs_saved.get(j, 1)) for j in joints]
        new_offs = [(-1 if j in which else 1) * offs.get(j, 0.0) for j in joints]
        print("\n%-6s %10s %10s %10s %10s" %
              ("joint", "旧dir", "新dir", "旧offset", "新offset"))
        for j, do, dn, oo, on in zip(joints, dirs_saved, new_dirs, offs, new_offs):
            print("%-6s %+10d %+10d %+10.6f %+10.6f"
                  % (j, int(dirs_saved.get(j, 1)), dn, offs.get(j, 0.0), on))
        body = dump_yaml(joints, [int(ids_saved.get(j, i)) for j, i in zip(joints, ids)],
                         new_dirs, dict(zip(joints, new_offs)))
        if args.write or args.out:
            out_path.write_text(body, encoding="utf-8")
            print("\n✅ 已写入 %s（零点不变，只是方向取反）" % out_path)
        else:
            print("\n--- 预览（加 --write 才会写入）---\n" + body)
        return 0

    if args.apply:
        offs = load_yaml_map(out_path, "offsets")
        dirs_saved = load_yaml_map(out_path, "directions")
        print("读取编码器角…")
        enc = measure(args.channel, ids, args.verbose)
        if not enc:
            print("❌ 没有电机应答：检查 can0 / 24V 供电 / 接线 / 波特率", file=sys.stderr)
            return 2
        print("\n%-6s %10s %10s %10s %10s" %
              ("joint", "编码器(rad)", "dir", "offset", "URDF角(deg)"))
        for j, mid, d in zip(joints, ids, dirs):
            if mid not in enc:
                print("%-6s %10s" % (j, "无应答"))
                continue
            d = int(dirs_saved.get(j, d))
            q = d * enc[mid] + offs.get(j, 0.0)
            print("%-6s %+10.4f %+10d %+10.4f %+10.2f"
                  % (j, enc[mid], d, offs.get(j, 0.0), math.degrees(q)))
        return 0

    if args.stable > 0:
        print("检查机械臂是否静止（采样 %.1f s，阈值 %.3f rad）…" % (args.stable, args.tol))
        ok, p2p, enc = check_stability(args.channel, ids, args.stable, args.tol)
        for j, mid in zip(joints, ids):
            v = p2p.get(mid, float("nan"))
            print("  %-4s 峰峰值 %.4f rad (%5.2f deg)%s"
                  % (j, v, math.degrees(v), "  ← 在动" if v > args.tol else ""))
        if not ok:
            print("❌ 机械臂仍在移动，本次不写入。请垫稳/夹紧后重试", file=sys.stderr)
            return 3
        print("  ✅ 静止稳定")
    else:
        print("读取编码器角（只读寄存器，不使能、不出力）…")
        enc = measure(args.channel, ids, args.verbose)
    if not enc:
        print("❌ 没有电机应答：检查 can0 / 24V 供电 / 接线 / 波特率", file=sys.stderr)
        return 2
    missing = [j for j, mid in zip(joints, ids) if mid not in enc]
    if missing:
        print("⚠️ 只读到 %d/%d 台电机，缺: %s" % (len(enc), n, ", ".join(missing)))

    if args.measure:
        return 0

    print("\n%-6s %10s %10s %12s %8s" %
          ("joint", "编码器(rad)", "目标(rad)", "offset(rad)", "dir"))
    offsets = {}
    for j, mid, d, t in zip(joints, ids, dirs, target):
        if mid not in enc:
            offsets[j] = 0.0
            continue
        off = t - d * enc[mid]
        offsets[j] = off
        print("%-6s %+10.4f %+10.4f %+12.6f %+8d" % (j, enc[mid], t, off, d))

    body = dump_yaml(joints, ids, dirs, offsets)
    print("\n--- joint_offsets.yaml ---\n" + body)

    if args.write:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(body, encoding="utf-8")
        print("✅ 已写入 %s" % out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
