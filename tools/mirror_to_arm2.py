#!/usr/bin/env python3
"""机械臂 1 → 机械臂 2 镜像遥操作（经 UART1 + 无线串口透传）。

链路
----
    /joint_states（J0..J5，URDF 关节角 rad，200 Hz，由 joint_state_broadcaster 发）
        ↓  本脚本按 --rate 组装一行
    `P q0 q1 q2 q3 q4 q5\\n`  →  /dev/ttyACM0（无线模块的 USB 端）
        ↓  空口透传
    C 板 UART1（PA9=TX / PB7=RX）→ 固件 app_uart_read() → app_serial_input()
        ↓  复用固件现成的 `P` 命令：限速 2.5 rad/s 追目标，`M 1` 打开 kp/kd
    机械臂 2 跟随

为什么直传 URDF 关节角，而不是电机编码器角
------------------------------------------
    两台手臂各自标定过零点：
      arm1 → ros2_gong/src/robotarm/config/joint_offsets.yaml
      arm2 → igh6_jxb/host/joint_offsets.yaml
    两边 dir 表完全相同（+1 -1 +1 +1 +1 +1），且 igh6_jxb 的重力模型就是从
    ros2_gong 的同一个 robotarm.urdf 生成的 —— 所以 `q_joint` 在两边指同一个物理
    姿态。传关节角，两端各自用本机的 dir/offset 换算到电机侧，不需要任何偏置补偿。

安全
----
    * /joint_states 超过 --stale-ms 没更新 → 不发（绝不拿旧角度去驱动另一台）
    * 断链 300 ms → 固件侧看门狗自动退回纯重力补偿（等价 M 0）
    * Ctrl-C / 退出 → 先发 M 0 再关端口
    * 单轴超出固件的 ±APP_TELEOP_RANGE → 客户端先夹紧并告警（否则整帧被固件拒绝）

用法
----
    python3 tools/mirror_to_arm2.py                 # 真机镜像（自动找无线模块）
    python3 tools/mirror_to_arm2.py --dry-run       # 只打印帧，不碰串口
    python3 tools/mirror_to_arm2.py --log           # 顺带回显机械臂2 的日志
    python3 tools/mirror_to_arm2.py --port /dev/ttyACM0 --rate 100 --kp 3 --kd 1
"""

from __future__ import annotations

import argparse
import glob
import os
import pathlib
import struct
import sys
import time

JOINTS = ("J0", "J1", "J2", "J3", "J4", "J5")

# 与固件 app/app_config.h 的 APP_TELEOP_RANGE 保持一致
LIMIT_RAD = 7.0
# 与固件 APP_TELEOP_TIMEOUT_MS 对应：这边先停发，那边 300 ms 后自动放开保持
DEFAULT_STALE_MS = 200.0
DEFAULT_RATE = 100.0
BAUD = 115200

# 无线模块（逐飞 SEEKFREE）的 USB 端；C 板自带 CDC 是 0483:5740
WIRELESS_VID_PID = ("2e3c", "5740")
BOARD_VID_PID = ("0483", "5740")


# ---------------------------------------------------------------- 纯逻辑（可测）
#: 二进制帧：0xAA | q0..q5（各 int16 小端，mrad）| 前 13 字节累加和的低 8 位
BIN_HEADER = 0xAA
BIN_FRAME_LEN = 14


def format_bin(qs) -> bytes:
    """六个关节角 → 14 字节二进制帧。单位转 mrad，比文本省 3 倍带宽。"""
    body = bytearray([BIN_HEADER])
    for q in qs:
        v = int(round(q * 1000.0))          # rad → mrad
        v = max(-32768, min(32767, v))      # int16 收口（±32.7 rad，远超关节行程）
        body += struct.pack("<h", v)
    body.append(sum(body) & 0xFF)
    assert len(body) == BIN_FRAME_LEN
    return bytes(body)


def format_cmd(qs) -> bytes:
    """六个关节角 → 一行 `P` 命令（文本，调试用）。顺序固定 J0..J5，单位 rad。"""
    return ("P " + " ".join("%.4f" % q for q in qs) + "\n").encode("ascii")


class MirrorCore:
    """把 /joint_states 的最新值攒成待发的帧；只管逻辑，不碰 ROS 和串口。

    单独拆出来是为了能在主机侧直接测：超时、缺关节、乱序、越界。
    """

    def __init__(self, stale_ms: float = DEFAULT_STALE_MS, limit_rad: float = LIMIT_RAD,
                 formatter=format_bin):
        self.stale_ms = stale_ms
        self.limit_rad = limit_rad
        self.formatter = formatter
        self._q: dict[str, float] = {}
        self._stamp: float | None = None
        self.clamped = 0

    def update(self, names, positions, now: float) -> None:
        """喂一条 JointState（名字和位置一一对应，可以是乱序的）。"""
        incoming = {n: float(p) for n, p in zip(names, positions) if n in JOINTS}
        self._q.update(incoming)
        if all(j in self._q for j in JOINTS):
            self._stamp = now

    def age_ms(self, now: float) -> float | None:
        if self._stamp is None:
            return None
        return (now - self._stamp) * 1000.0

    def build_frame(self, now: float) -> bytes | None:
        """返回该发的帧；不该发（没数据 / 过期 / 缺关节）时返回 None。"""
        if self._stamp is None:
            return None
        if (now - self._stamp) * 1000.0 > self.stale_ms:
            return None
        qs = []
        for j in JOINTS:
            v = self._q[j]
            if v > self.limit_rad:
                v = self.limit_rad
                self.clamped += 1
            elif v < -self.limit_rad:
                v = -self.limit_rad
                self.clamped += 1
            qs.append(v)
        return self.formatter(qs)


# ---------------------------------------------------------------- 串口
def _usb_ids(port: str) -> tuple[str, str] | None:
    name = os.path.basename(port)
    try:
        usb = pathlib.Path(f"/sys/class/tty/{name}/device").resolve().parent
        return (
            (usb / "idVendor").read_text().strip().lower(),
            (usb / "idProduct").read_text().strip().lower(),
        )
    except OSError:
        return None


def find_port(prefer=("wireless", "board")) -> str | None:
    """默认优先无线模块（2e3c:5740），其次 C 板 CDC，最后第一个 ttyACM*。"""
    ports = sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        return None
    order = [WIRELESS_VID_PID, BOARD_VID_PID]
    for want in order:
        for p in ports:
            if _usb_ids(p) == want:
                return p
    return ports[0]


class SerialSink:
    def __init__(self, port: str | None, dry_run: bool = False, echo: bool = False):
        self.port = port
        self.dry_run = dry_run
        self.echo = echo
        self.ser = None
        self.sent = 0
        self.failed = 0

    def open(self) -> None:
        if self.dry_run:
            return
        import serial  # 延迟导入：--dry-run 时不需要 pyserial

        port = self.port or find_port()
        if not port:
            raise SystemExit("找不到串口设备（/dev/ttyACM*）；插上无线模块或指定 --port")
        self.port = port
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0, write_timeout=0.5)
        except PermissionError:
            raise SystemExit(
                f"没有权限打开 {port}。二选一：\n"
                f"  1) 当前终端先跑:  newgrp dialout\n"
                f"  2) 一次性永久修复（需要 sudo 密码）:\n"
                f"     echo 'SUBSYSTEM==\"tty\", ATTRS{{idVendor}}==\"2e3c\", ATTRS{{idProduct}}==\"5740\", "
                f"MODE=\"0666\", GROUP=\"dialout\"' | sudo tee /etc/udev/rules.d/99-seekfree-wireless.rules\n"
                f"     sudo udevadm control --reload && sudo udevadm trigger"
            ) from None
        _discard_pending(self.ser)
        print(f"串口: {port} @ {BAUD}")

    def send(self, frame: bytes) -> None:
        if self.dry_run:
            if frame[:1] == bytes([BIN_HEADER]):
                vals = struct.unpack("<6h", frame[1:13])
                print("[bin %dB] %s mrad  chk=0x%02X"
                      % (len(frame), " ".join("%+d" % v for v in vals), frame[13]))
            else:
                sys.stdout.write(frame.decode())
            self.sent += 1
            return
        try:
            self.ser.write(frame)
            self.sent += 1
        except Exception as exc:  # 串口断开/写超时：计数，不中断循环
            self.failed += 1
            if self.failed in (1, 50, 500):
                print(f"⚠️  写串口失败（第 {self.failed} 次）: {exc}")

        if self.echo:
            self._drain_rx()

    def _drain_rx(self) -> None:
        """把机械臂2 打回来的日志读掉并打印，避免 CDC 缓冲区积压。"""
        try:
            n = self.ser.in_waiting
            if n:
                sys.stdout.write(self.ser.read(n).decode("utf-8", "replace"))
        except Exception:
            pass

    def close(self) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None


def _discard_pending(ser) -> None:
    """打开端口时清掉内核里积压的历史字节，避免第一帧是几秒前的旧数据。"""
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass


# ---------------------------------------------------------------- ROS 接线
def main() -> int:
    ap = argparse.ArgumentParser(description="机械臂1 → 机械臂2 镜像遥操作")
    ap.add_argument("--port", default=None, help="串口设备（默认自动找无线模块）")
    ap.add_argument("--topic", default="/joint_states", help="关节状态话题")
    ap.add_argument("--fmt", choices=("binary", "text"), default="binary",
                    help="帧格式：binary=14 字节（默认，高速）；text=P 行（调试用）")
    ap.add_argument("--rate", type=float, default=None,
                    help="发送频率 Hz（默认 binary=50 / text=20，按空口实测吞吐定）")
    ap.add_argument("--stale-ms", type=float, default=DEFAULT_STALE_MS,
                    help="超过这么久没更新就不发")
    ap.add_argument("--kp", type=float, default=None, help="位置刚度（固件默认 3.0）")
    ap.add_argument("--kd", type=float, default=None, help="阻尼（固件默认 1.0）")
    ap.add_argument("--dry-run", action="store_true", help="只打印帧，不开串口")
    ap.add_argument("--log", action="store_true", help="回显机械臂2 的日志")
    args = ap.parse_args()

    if args.rate is None:
        args.rate = 50.0 if args.fmt == "binary" else 20.0

    if args.rate > 150:
        print(f"⚠️  --rate {args.rate} 超过 150 Hz：115200 下文本帧会吃掉大半带宽，"
              f"建议先跑 100 Hz", file=sys.stderr)

    import threading

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    core = MirrorCore(stale_ms=args.stale_ms,
                      formatter=(format_bin if args.fmt == "binary" else format_cmd))
    sink = SerialSink(args.port, dry_run=args.dry_run, echo=args.log)

    rclpy.init()
    node = Node("mirror_to_arm2")

    def on_joint_state(msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            return
        core.update(msg.name, msg.position, time.monotonic())

    node.create_subscription(JointState, args.topic, on_joint_state, 10)

    # 关键：让 ROS 消息投递独立于串口主循环。之前用 spin_once(timeout=0) 在 100 Hz
    # 循环里抽一下，executor 不等待就返回，订阅回调经常被饿到 → 数据明明在 200 Hz
    # 发布，这里却隔三差五判「过期」。改成后台线程 spin，回调只更新 core，主循环
    # 只负责按自己的节拍取最新值发串口。
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def _spin() -> None:
        try:
            executor.spin()
        except Exception:
            pass   # 上下文 shutdown 时 spin() 抛 ExternalShutdownException，属正常退出

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    sink.open()
    if not args.dry_run:
        if args.kp is not None or args.kd is not None:
            kp = args.kp if args.kp is not None else 3.0
            kd = args.kd if args.kd is not None else 1.0
            sink.send(f"K {kp:.2f} {kd:.2f}\n".encode())
        sink.send(b"M 1\n")   # 打开位置保持；第一帧 P 到来前它不会动
        print(f"已发送 M 1，开始以 {args.rate:.0f} Hz 镜像 {args.topic}（{args.fmt}）")

    period = 1.0 / args.rate
    next_t = time.monotonic()
    warned_no_data = False
    last_stale_warn = 0.0
    try:
        while rclpy.ok():
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(next_t - now, 0.002))
                continue
            next_t += period

            frame = core.build_frame(now)
            if frame is None:
                age = core.age_ms(now)
                if age is None:
                    if not warned_no_data:
                        print(f"⏳ 等 {args.topic} 的第一帧…", file=sys.stderr)
                        warned_no_data = True
                elif now - last_stale_warn > 2.0:   # 限流，别刷屏
                    print(f"⚠️  {args.topic} 已 {age:.0f} ms 没更新，暂停下发"
                          "（机械臂2 将在 300 ms 后自动退回纯重力补偿）", file=sys.stderr)
                    last_stale_warn = now
                continue
            sink.send(frame)
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C")
    finally:
        if not args.dry_run:
            try:
                sink.send(b"M 0\n")   # 先放开保持，再关端口
            except Exception:
                pass
        sink.close()
        executor.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()   # 信号处理可能已经 shutdown 过一次，这里必须幂等
        print(f"已发送 {sink.sent} 帧，失败 {sink.failed} 次"
              + (f"，夹紧 {core.clamped} 次" if core.clamped else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
