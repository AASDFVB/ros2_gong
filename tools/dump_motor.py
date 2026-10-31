#!/usr/bin/env python3
"""达妙电机全量数据抓取：寄存器 + MIT 反馈帧。

两种数据源：
  1) 寄存器读（默认）：[ID, 00, 33, RID, 0,0,0,0] 发到 0x7FF
     —— 只读，不使能、不出力。含配置、限幅、版本、当前角度
  2) MIT 反馈帧（--mit）：发 p=当前角度, v=0, kp=0, kd=0, t=0 的零力矩帧，
     电机回 8 字节反馈：位置/速度/力矩/MOS温度/线圈温度/错误码

用法:
    python3 dump_motor.py                 # 全部寄存器
    python3 dump_motor.py --mit           # 额外抓 MIT 反馈帧（零力矩）
    python3 dump_motor.py --raw           # 显示原始字节
    python3 dump_motor.py --json          # JSON 输出
    python3 dump_motor.py --rid 21        # 只读某个寄存器
"""

from __future__ import annotations

import argparse
import errno
import json
import math
import socket
import struct
import sys
import time

AF_CAN = getattr(socket, "AF_CAN", 29)
CAN_RAW = getattr(socket, "CAN_RAW", 1)
FRAME = struct.Struct("=IB3x8s")
RAD2DEG = 180.0 / math.pi

RID_TABLE = {
    0: ("UV_Value", "欠压保护", "float"), 1: ("KT_Value", "扭矩系数", "float"),
    2: ("OT_Value", "过温保护", "float"), 3: ("OC_Value", "过流保护", "float"),
    4: ("ACC", "加速度", "float"), 5: ("DEC", "减速度", "float"),
    6: ("MAX_SPD", "最大速度", "float"), 7: ("MST_ID", "反馈ID", "uint"),
    8: ("ESC_ID", "接收ID", "uint"), 9: ("TIMEOUT", "通信超时×50us", "uint"),
    10: ("CTRL_MODE", "控制模式", "uint"), 11: ("Damp", "阻尼系数", "float"),
    12: ("Inertia", "转动惯量", "float"), 13: ("hw_ver", "硬件版本", "uint"),
    14: ("sw_ver", "软件版本", "uint"), 15: ("SN", "序列号", "uint"),
    16: ("NPP", "极对数", "uint"), 17: ("Rs", "相电阻", "float"),
    18: ("Ls", "相电感", "float"), 19: ("Flux", "磁链", "float"),
    20: ("Gr", "齿轮比", "float"), 21: ("PMAX", "位置上限", "float"),
    22: ("VMAX", "速度上限", "float"), 23: ("TMAX", "力矩上限", "float"),
    35: ("can_br", "CAN波特率档位", "uint"),
    # 以下 RID 24~34 已用达妙上位机「参数设置」页逐项对照确认（电机 3 实测值与界面完全一致）
    24: ("I_BW", "电流环带宽", "float"),
    25: ("SPD_KP", "速度环KP", "float"),
    26: ("SPD_KI", "速度环KI", "float"),
    27: ("POS_KP", "位置环KP", "float"),
    28: ("POS_KI", "位置环KI", "float"),
    29: ("OV_Value", "过压保护", "float"),
    30: ("GearCoef", "齿轮系数", "float"),
    31: ("DampFactor", "阻尼因子", "float"),
    32: ("SPD_BW", "速度环滤波带宽", "float"),
    33: ("I_ENH", "电流环增强系数", "float"),
    34: ("SPD_ENH", "速度环增强系数", "float"),
    80: ("p_m", "电机转子位置(RO)", "float"), 81: ("xout", "输出轴位置(RO)", "float"),
}
UNIT = {21: "rad", 22: "rad/s", 23: "Nm"}

ERR_STATE = {
    0x0: "禁用", 0x1: "使能", 0x8: "过压", 0x9: "欠压",
    0xA: "过流", 0xB: "MOS过温", 0xC: "线圈过温",
    0xD: "通信丢失", 0xE: "过载",
}


def iface_up(ch):
    try:
        return open("/sys/class/net/%s/operstate" % ch).read().strip() == "up"
    except OSError:
        return False


def f2u(x, xmin, xmax, bits):
    span = xmax - xmin
    x = min(max(x, xmin), xmax)
    return int((x - xmin) * ((1 << bits) - 1) / span)


def u2f(v, xmin, xmax, bits):
    span = xmax - xmin
    return v * span / ((1 << bits) - 1) + xmin


class Bus:
    def __init__(self, ch):
        self.s = socket.socket(AF_CAN, socket.SOCK_RAW, CAN_RAW)
        self.s.bind((ch,))
        self.s.setsockopt(101, 4, 0)
        self.s.settimeout(0.0)

    def send(self, cid, data):
        f = FRAME.pack(cid & 0x7FF, len(data), bytes(data))
        for _ in range(50):
            try:
                self.s.send(f)
                return True
            except OSError as exc:
                if getattr(exc, "errno", None) == errno.ENOBUFS:
                    time.sleep(0.002)
                    continue
                raise
        return False

    def drain(self):
        while True:
            try:
                self.s.recv(16)
            except Exception:
                return

    def read_rids(self, mid, rids, repeats=2, listen=0.030, chunk=6):
        """分批发查询+收回复。一次喷 40+ 帧会把 PCAN 的 qlen 打爆，导致丢帧。"""
        got = {}
        todo = list(rids)
        for _attempt in range(2):
            if not todo:
                break
            missing = []
            for i in range(0, len(todo), chunk):
                part = todo[i:i + chunk]
                want = set(part)
                self.drain()
                for _rep in range(repeats):
                    for rid in part:
                        self.send(0x7FF, [mid, 0x00, 0x33, rid, 0, 0, 0, 0])
                        time.sleep(0.001)
                end = time.time() + listen
                while time.time() < end:
                    try:
                        raw = self.s.recv(16)
                    except (socket.timeout, BlockingIOError, OSError):
                        break
                    if len(raw) < 16:
                        continue
                    cid, dlc, d = FRAME.unpack(raw)
                    if (cid & 0x7FF) == 0x7FF or dlc < 8:
                        continue
                    if d[2] != 0x33 or d[3] not in want:
                        continue
                    if (d[0] & 0x0F) != (mid & 0x0F):
                        continue
                    got[d[3]] = (cid & 0x7FF, bytes(d))
                missing.extend(r for r in part if r not in got)
            todo = missing
        return got

    def close(self):
        self.s.close()


def decode(rid, d):
    raw = struct.unpack("<I", d[4:8])[0]
    if rid in RID_TABLE:
        kind = RID_TABLE[rid][2]
    else:
        # 未知 RID：达妙绝大多数是 float，用启发式判断
        f = struct.unpack("<f", d[4:8])[0]
        kind = "float" if (math.isfinite(f) and abs(f) < 1e12
                           and (f == 0 or abs(f) > 1e-20)) else "uint"
    if kind == "float":
        return raw, struct.unpack("<f", d[4:8])[0]
    return raw, float(raw)


def read_mit_feedback(bus, mid, p_des, pmax, vmax, tmax, mst_id=None, tries=25):
    """发零力矩 MIT 帧（kp=kd=0, t=0），抓一帧反馈。

    两个实测要点（这台 DM4310 的固件）:
      * 反馈帧走 MST_ID（默认 0x10|mid），不是电机 ID
      * 单次往返要 10~20 ms，监听窗口 4 ms 太短会全部丢
    """
    expect = mst_id if mst_id else (0x10 | (mid & 0x0F))
    frame = bytearray(8)
    p = f2u(p_des, -pmax, pmax, 16)
    v = f2u(0.0, -vmax, vmax, 12)
    kp = f2u(0.0, 0.0, 500.0, 12)
    kd = f2u(0.0, 0.0, 5.0, 12)
    t = f2u(0.0, -tmax, tmax, 12)
    frame[0] = p >> 8
    frame[1] = p & 0xFF
    frame[2] = v >> 4
    frame[3] = ((v & 0xF) << 4) | (kp >> 8)
    frame[4] = kp & 0xFF
    frame[5] = kd >> 4
    frame[6] = ((kd & 0xF) << 4) | (t >> 8)
    frame[7] = t & 0xFF
    bus.drain()
    for _ in range(tries):
        bus.send(mid, frame)
        t_end = time.time() + 0.016
        while time.time() < t_end:
            try:
                raw = bus.s.recv(16)
            except (socket.timeout, BlockingIOError, OSError):
                break
            if len(raw) < 16:
                continue
            cid, dlc, d = FRAME.unpack(raw)
            if (cid & 0x7FF) == 0x7FF or dlc < 8:
                continue
            if (cid & 0x7FF) == expect:
                return bytes(d)
    return None


def decode_mit(d, pmax, vmax, tmax):
    # 实测这台固件: data[0] = (state << 4) | id，和常见 SDK 的 id<<4 相反
    ident = d[0] & 0x0F
    state = d[0] >> 4
    p_i = (d[1] << 8) | d[2]
    v_i = (d[3] << 4) | (d[4] >> 4)
    t_i = ((d[4] & 0xF) << 8) | d[5]
    return {
        "id": ident,
        "state": state,
        "state_cn": ERR_STATE.get(state, "未知(0x%X)" % state),
        "pos": u2f(p_i, -pmax, pmax, 16),
        "vel": u2f(v_i, -vmax, vmax, 12),
        "torque": u2f(t_i, -tmax, tmax, 12),
        "t_mos": d[6],
        "t_rotor": d[7],
        "raw": d.hex(" "),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--ids", default="1,2,3,4,5,6")
    ap.add_argument("--rid", type=int, default=None)
    ap.add_argument("--raw", action="store_true")
    ap.add_argument("--mit", action="store_true", help="额外抓 MIT 反馈帧（零力矩）")
    ap.add_argument("--summary", action="store_true", help="6 台电机关键参数横向对比")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    if not iface_up(args.channel):
        print("❌ %s 不是 UP。先执行：" % args.channel)
        print("   sudo ip link set %s down" % args.channel)
        print("   sudo ip link set %s type can bitrate 1000000" % args.channel)
        print("   sudo ip link set %s up" % args.channel)
        return 2

    ids = [int(x) for x in args.ids.split(",")]
    rids = [args.rid] if args.rid is not None else sorted(
        [r for r in list(range(0, 41)) + [80, 81] if r != 35 or True])
    if args.rid is None:
        rids = list(range(0, 41)) + [80, 81]

    bus = Bus(args.channel)
    out = {}
    try:
        for mid in ids:
            got = bus.read_rids(mid, rids)
            entry = {"regs": {}, "mit": None}
            for rid, (cid, d) in sorted(got.items()):
                raw, val = decode(rid, d)
                entry["regs"][rid] = {
                    "name": RID_TABLE.get(rid, ("RID%d" % rid, "未知", "uint"))[0],
                    "cn": RID_TABLE.get(rid, ("", "未知", "uint"))[1],
                    "raw": raw,
                    "value": val,
                    "bytes": d.hex(" "),
                }
            pmax = entry["regs"].get(21, {}).get("value", 12.5) or 12.5
            vmax = entry["regs"].get(22, {}).get("value", 30.0) or 30.0
            tmax = entry["regs"].get(23, {}).get("value", 10.0) or 10.0
            entry["limits"] = {"PMAX": pmax, "VMAX": vmax, "TMAX": tmax}
            if args.mit and entry["regs"]:
                p_now = entry["regs"].get(81, {}).get("value", 0.0)
                mst = entry["regs"].get(7, {}).get("value")
                fb = read_mit_feedback(bus, mid, p_now or 0.0, pmax, vmax, tmax,
                                       mst_id=int(mst) if mst else None)
                if fb:
                    entry["mit"] = decode_mit(fb, pmax, vmax, tmax)
            out[mid] = entry
    finally:
        bus.close()

    if args.json:
        print(json.dumps({str(k): v for k, v in out.items()}, ensure_ascii=False, indent=2))
        return 0

    if args.summary:
        keys = [("ESC_ID", 8), ("MST_ID", 7), ("CTRL", 10), ("PMAX", 21),
                ("VMAX", 22), ("TMAX", 23), ("Gr", 20), ("NPP", 16),
                ("TIMEOUT", 9), ("xout", 81)]
        print("  %-4s" % "ID" + "".join("%9s" % k for k, _ in keys))
        for mid in ids:
            e = out[mid]
            if not e["regs"]:
                print("  %-4d  无应答" % mid)
                continue
            row = []
            for _, rid in keys:
                v = e["regs"].get(rid, {}).get("value")
                row.append("%9s" % ("-" if v is None else ("%g" % v)))
            print("  %-4d" % mid + "".join(row))
        print()
        return 0

    for mid in ids:
        e = out[mid]
        print("\n══ 电机 %d (CAN 0x%03X) ══════════════════════════════════════" % (mid, mid))
        if not e["regs"]:
            print("   无应答（未上电 / CAN 断 / ID 不对）")
            continue
        print("   %-4s %-14s %-14s %14s  %s" % ("RID", "名称", "中文", "值", "原始字节"))
        for rid, r in sorted(e["regs"].items()):
            extra = ""
            if rid in (80, 81):
                extra = " (%.2f°)" % (r["value"] * RAD2DEG)
            elif rid in UNIT:
                extra = " " + UNIT[rid]
            b = r["bytes"] if args.raw else ""
            print("   %-4d %-14s %-14s %14.4f%-8s %s"
                  % (rid, r["name"], r["cn"], r["value"], extra, b))
        lim = e["limits"]
        print("   限幅: PMAX=%.3f rad  VMAX=%.2f rad/s  TMAX=%.2f Nm" %
              (lim["PMAX"], lim["VMAX"], lim["TMAX"]))
        if args.mit:
            m = e["mit"]
            if not m:
                print("   MIT 反馈: 无应答")
            else:
                print("   MIT 反馈: 状态=%s  pos=%+.4f rad (%.2f°)  vel=%+.3f rad/s  "
                      "torque=%+.3f Nm" % (m["state_cn"], m["pos"], m["pos"] * RAD2DEG,
                                           m["vel"], m["torque"]))
                print("             MOS=%d℃  线圈=%d℃   原始=%s"
                      % (m["t_mos"], m["t_rotor"], m["raw"]))
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
