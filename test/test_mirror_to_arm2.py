#!/usr/bin/env python3
"""mirror_to_arm2 的主机侧测试：只测纯逻辑，不碰 ROS、不碰串口。

两个测点（seam）：
  1. format_cmd()      —— 帧格式：顺序、单位、精度、换行
  2. MirrorCore        —— 该不该发：没数据 / 过期 / 缺关节 / 乱序 / 越界

跑法：python3 test/test_mirror_to_arm2.py
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "tools"))

import struct  # noqa: E402

from mirror_to_arm2 import (BIN_FRAME_LEN, BIN_HEADER, JOINTS,  # noqa: E402
                            MirrorCore, format_bin, format_cmd)

FAILED = []


def check(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        print(f"  ✓ {name}")
    else:
        print(f"  ✗ {name}  {detail}")
        FAILED.append(name)


def eq(name: str, got, want) -> None:
    check(name, got == want, f"\n      got  = {got!r}\n      want = {want!r}")


# ---------------------------------------------------------------- 1. 帧格式
def test_format() -> None:
    print("format_cmd")
    eq("六个 0 → 'P 0.0000 ×6'",
       format_cmd([0, 0, 0, 0, 0, 0]),
       b"P 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000\n")
    eq("正负号与四位小数",
       format_cmd([1.23456, -0.5, 3.0, -0.0001, 2.71828, -7.0]),
       b"P 1.2346 -0.5000 3.0000 -0.0001 2.7183 -7.0000\n")
    eq("单行以 \\n 结尾（固件按 \\n 切帧）",
       format_cmd([0, 0, 0, 0, 0, 0])[-1:], b"\n")
    check("纯 ASCII（无线串口透传不做编码转换）",
          all(b < 0x80 for b in format_cmd([1, 2, 3, 4, 5, 6])))


# ---------------------------------------------------------------- 1b. 二进制帧
def test_bin() -> None:
    print("format_bin")

    f = format_bin([0, 0, 0, 0, 0, 0])
    eq("长度 = 14 字节（与固件 APP_BIN_FRAME_LEN 一致）", len(f), BIN_FRAME_LEN)
    eq("首字节是同步头 0xAA", f[0], BIN_HEADER)
    eq("全 0 的角度 → 数据区全 0", bytes(f[1:13]), b"\x00" * 12)
    eq("校验和 = 前 13 字节累加和低 8 位", f[13], sum(f[:13]) & 0xFF)

    f = format_bin([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
    eq("小端 int16，单位 mrad",
       struct.unpack("<6h", f[1:13]), (100, -200, 300, -400, 500, -600))
    eq("校验和随内容变化", f[13], sum(f[:13]) & 0xFF)
    check("校验和确实校验了数据（不是常数）", f[13] != 0xAA)

    f = format_bin([100.0, -100.0, 0, 0, 0, 0])
    eq("超出 int16 被夹紧到 ±32767",
       struct.unpack("<6h", f[1:13])[:2], (32767, -32768))

    eq("帧长恒为 14（不同角度值也不变）",
       len(format_bin([1.234, 5.678, -3.21, 0.001, -0.001, 2.0])), 14)


# ---------------------------------------------------------------- 2. 该不该发
def test_core() -> None:
    print("MirrorCore")
    # 下面测的是排序/夹紧/超时逻辑，与帧格式无关，固定用文本格式方便断言

    core = MirrorCore(stale_ms=200.0, formatter=format_cmd)
    eq("没收到过 /joint_states → 不发", core.build_frame(now=10.0), None)
    check("年龄未知时 age_ms 为 None", core.age_ms(10.0) is None)

    # 完整的六轴，乱序给定，验证按 J0..J5 重排
    t = 100.0
    core.update(["J3", "J0", "J5", "J1", "J4", "J2"],
                [0.3, 0.1, 0.6, -0.2, 0.5, 0.4], now=t)
    eq("乱序输入 → 按 J0..J5 顺序输出",
       core.build_frame(now=t),
       b"P 0.1000 -0.2000 0.4000 0.3000 0.5000 0.6000\n")

    check("新鲜数据立刻可发（t+199ms）", core.build_frame(now=t + 0.199) is not None)
    eq("超过 stale_ms 就不发（t+201ms）", core.build_frame(now=t + 0.201), None)

    # 缺一个关节：不该发
    partial = MirrorCore(stale_ms=200.0, formatter=format_cmd)
    partial.update(["J0", "J1", "J2", "J3", "J4"], [0, 0, 0, 0, 0], now=t)
    eq("缺 J5 → 不发（宁可不发也不发半套角度）", partial.build_frame(now=t), None)

    # 分两条消息凑齐六轴：应能发
    split = MirrorCore(stale_ms=200.0, formatter=format_cmd)
    split.update(["J0", "J1", "J2"], [0.1, 0.2, 0.3], now=t)
    eq("只到一半 → 不发", split.build_frame(now=t), None)
    split.update(["J3", "J4", "J5"], [0.4, 0.5, 0.6], now=t)
    eq("凑齐六轴 → 发", split.build_frame(now=t),
       b"P 0.1000 0.2000 0.3000 0.4000 0.5000 0.6000\n")

    # 越界夹紧（固件对整帧做 ±APP_TELEOP_RANGE 校验，越界会导致整帧被拒）
    clamp = MirrorCore(stale_ms=200.0, limit_rad=7.0, formatter=format_cmd)
    clamp.update(list(JOINTS), [99.0, -99.0, 0, 0, 0, 0], now=t)
    eq("越界值被夹到 ±7 rad",
       clamp.build_frame(now=t),
       b"P 7.0000 -7.0000 0.0000 0.0000 0.0000 0.0000\n")
    eq("夹紧次数被记下来", clamp.clamped, 2)

    # 默认 formatter 是二进制：帧长 14、首字节 0xAA
    dflt = MirrorCore(stale_ms=200.0)
    dflt.update(list(JOINTS), [0, 0, 0, 0, 0, 0], now=t)
    bin_frame = dflt.build_frame(now=t)
    eq("MirrorCore 默认输出二进制帧", len(bin_frame), BIN_FRAME_LEN)
    eq("MirrorCore 默认帧首字节 0xAA", bin_frame[0], BIN_HEADER)

    txt = MirrorCore(stale_ms=200.0, formatter=format_cmd)
    txt.update(list(JOINTS), [0, 0, 0, 0, 0, 0], now=t)
    eq("formatter=format_cmd 时输出文本帧", txt.build_frame(now=t),
       b"P 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000\n")

    # 未知关节名（比如别的机器人挂上来的）应被忽略，不影响六轴
    noise = MirrorCore(stale_ms=200.0, formatter=format_cmd)
    noise.update(list(JOINTS) + ["gripper"], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 9.9], now=t)
    eq("多余关节被忽略", noise.build_frame(now=t),
       b"P 0.1000 0.2000 0.3000 0.4000 0.5000 0.6000\n")


if __name__ == "__main__":
    test_format()
    test_bin()
    test_core()
    print()
    if FAILED:
        print(f"❌ {len(FAILED)} 项失败: {FAILED}")
        sys.exit(1)
    print("✅ 全部通过")
