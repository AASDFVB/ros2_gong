#!/usr/bin/env python3
"""读取 HID 按键模块（0483:5750）的按键事件，只读，不发送任何东西。

用途
----
1. 数清楚板子实际有几路输入
2. 看清每路映射成哪个键

用法
----
    sudo chmod 666 /dev/input/event*        # 或者把用户加进 input 组
    python3 tools/keywatch.py

然后依次把每个输入脚短接到 GND，屏幕会打印：

    [按下] KEY_1 (2)
    [松开] KEY_1 (2)

把 10 路都试一遍，就得到「输入脚 → 键」的对照表。
"""

from __future__ import annotations

import glob
import os
import select
import struct
import sys

# Linux input_event: struct timeval(16) + type(2) + code(2) + value(4)
EVENT_FMT = "llHHi"
EVENT_SIZE = struct.calcsize(EVENT_FMT)
EV_KEY = 1

KEY_NAMES = {
    1: "ESC", 2: "1", 3: "2", 4: "3", 5: "4", 6: "5", 7: "6", 8: "7", 9: "8", 10: "9", 11: "0",
    12: "MINUS", 13: "EQUAL", 14: "BACKSPACE", 15: "TAB",
    16: "Q", 17: "W", 18: "E", 19: "R", 20: "T", 21: "Y", 22: "U", 23: "I", 24: "O", 25: "P",
    26: "LEFTBRACE", 27: "RIGHTBRACE", 28: "ENTER", 29: "LEFTCTRL",
    30: "A", 31: "S", 32: "D", 33: "F", 34: "G", 35: "H", 36: "J", 37: "K", 38: "L",
    39: "SEMICOLON", 40: "APOSTROPHE", 41: "GRAVE", 42: "LEFTSHIFT", 43: "BACKSLASH",
    44: "Z", 45: "X", 46: "C", 47: "V", 48: "B", 49: "N", 50: "M",
    51: "COMMA", 52: "DOT", 53: "SLASH", 54: "RIGHTSHIFT", 55: "KPASTERISK",
    56: "LEFTALT", 57: "SPACE", 58: "CAPSLOCK",
    59: "F1", 60: "F2", 61: "F3", 62: "F4", 63: "F5", 64: "F6",
    65: "F7", 66: "F8", 67: "F9", 68: "F10", 87: "F11", 88: "F12",
    102: "HOME", 103: "UP", 104: "PAGEUP", 105: "LEFT", 106: "RIGHT",
    107: "END", 108: "DOWN", 109: "PAGEDOWN", 110: "INSERT", 111: "DELETE",
    272: "BTN_LEFT", 273: "BTN_RIGHT", 274: "BTN_MIDDLE",
}


def find_keyboards() -> list[str]:
    """找出名字里带 0483:5750 的输入设备。"""
    out = []
    for path in sorted(glob.glob("/dev/input/event*")):
        ev = os.path.basename(path)
        name_path = f"/sys/class/input/{ev}/device/name"
        try:
            name = open(name_path, encoding="utf-8").read().strip()
        except OSError:
            continue
        if "0483:5750" in name:
            out.append(path)
    return out


def key_name(code: int) -> str:
    return KEY_NAMES.get(code, f"KEY_{code}")


def main() -> int:
    devs = find_keyboards()
    if not devs:
        print("找不到 0483:5750 输入设备；模块插好了吗？", file=sys.stderr)
        return 1

    fds = {}
    print("监听:", ", ".join(devs))
    for d in devs:
        try:
            fds[os.open(d, os.O_RDONLY | os.O_NONBLOCK)] = d
        except PermissionError:
            print(f"没有权限读 {d}：先跑 sudo chmod 666 {d}", file=sys.stderr)
            return 1

    print("请依次短接每个输入脚到 GND；Ctrl-C 退出\n")
    seen = set()
    try:
        while True:
            ready, _, _ = select.select(list(fds), [], [])
            for fd in ready:
                data = os.read(fd, EVENT_SIZE * 64)
                for i in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
                    _, _, etype, code, value = struct.unpack(EVENT_FMT, data[i:i + EVENT_SIZE])
                    if etype != EV_KEY:
                        continue
                    if value == 1:
                        seen.add(code)
                        print(f"[按下] {key_name(code)} ({code})    已识别 {len(seen)} 种键")
                    elif value == 0:
                        print(f"[松开] {key_name(code)} ({code})")
                    # value == 2 是自动重复，忽略
    except KeyboardInterrupt:
        print(f"\n共识别 {len(seen)} 种键: {sorted(seen)}")
    finally:
        for fd in fds:
            os.close(fd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
