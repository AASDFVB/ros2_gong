#!/usr/bin/env python3
"""对拍：电机实测保持力矩 vs Pinocchio 模型算的重力矩。

用法
----
    1. 重启 ros2_control（加载导出了 effort 的新插件）
    2. 跑本脚本
    3. 用手把某个关节**停住**在几个角度上，每个角度停 2 秒

输出
----
    J1 = +12.3°   实测 -1.052 N·m   模型 -1.021 N·m   比值 1.03

判断
----
    比值 ≈ 1.0            → 模型准，gain 合适
    比值稳定 < 0.9         → 模型偏小（或 gain 不够）→ 加大 gain
    比值稳定 > 1.1         → 补偿过头 → 减小 gain
    正负号相反             → dir 或符号错了
    某个角度区间偏差特别大  → URDF 质心与实机不符 / 机械有额外阻力

注意
----
* 必须在**静止**时读（速度接近 0），否则读到的是加速/摩擦项
* effort 是电机反馈的力矩，已乘 dir，和关节侧同向
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HoldTorque(Node):
    def __init__(self, urdf: str, joint: str, all_joints: bool):
        super().__init__("hold_torque")
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.names = [n for n in self.model.names if n != "universe"]
        self.joint = joint
        self.all_joints = all_joints
        self.q = np.zeros(self.model.nq)
        self.eff = np.zeros(len(self.names))
        self.vel = np.zeros(len(self.names))
        self.have = False
        self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.timer = self.create_timer(0.5, self.report)
        print(f"模型关节: {self.names}")
        print("把关节停住，读数会在下面刷新（Ctrl-C 退出）\n")

    def cb(self, msg: JointState):
        try:
            idx = [msg.name.index(n) for n in self.names]
        except ValueError:
            return
        self.q = np.array([msg.position[i] for i in idx])
        self.vel = np.array([msg.velocity[i] for i in idx]) if msg.velocity else np.zeros(len(idx))
        self.eff = np.array([msg.effort[i] for i in idx]) if msg.effort else np.zeros(len(idx))
        self.have = True

    def report(self):
        if not self.have:
            return
        g = pin.computeGeneralizedGravity(self.model, self.data, self.q)
        targets = range(len(self.names)) if self.all_joints else [
            self.names.index(self.joint)] if self.joint in self.names else []
        for i in targets:
            name = self.names[i]
            if abs(self.vel[i]) > 0.15:
                print(f"{name}  = {math.degrees(self.q[i]):+7.1f}°   转动中，等它停稳…")
                continue
            measured = self.eff[i]
            predicted = g[i]
            ratio = measured / predicted if abs(predicted) > 0.15 else float("nan")
            r = f"{ratio:5.2f}" if not math.isnan(ratio) else "  —  "
            print(f"{name}  = {math.degrees(self.q[i]):+7.1f}°   "
                  f"实测 {measured:+7.3f} N·m   模型 {predicted:+7.3f} N·m   比值 {r}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default="src/robotarm/urdf/robotarm.urdf")
    ap.add_argument("--joint", default="J1")
    ap.add_argument("--all", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = HoldTorque(args.urdf, args.joint, args.all)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
