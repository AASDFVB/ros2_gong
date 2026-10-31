#!/usr/bin/env python3
"""6 轴机械臂 viser 可视化 + 关节控制 + 末端 IK 规划。

浏览器打开 http://localhost:8080 即可看到：
  * 机械臂实时姿态（Pinocchio 正解驱动）
  * 6 个关节滑条（度）
  * 末端目标拖拽手柄（拖动后实时 IK）
  * 规划并播放（从当前姿态平滑到目标）
  * 可选：把结果发到真机（/joint_cmd）

用法:
    python3 arm_viser.py                 # 默认 8080 端口
    python3 arm_viser.py --port 8082
    python3 arm_viser.py --no-ik         # 只做关节可视化
"""

from __future__ import annotations

import argparse
import os
import pathlib
import sys
import threading
import time

HERE = pathlib.Path(__file__).resolve().parent
sys.path.append(str(HERE / "viser-libs"))   # append：让系统 numpy 1.26 优先，否则 pinocchio 段错误

import numpy as np                      # noqa: E402
import pinocchio as pin                 # noqa: E402
import trimesh                          # noqa: E402
import viser                            # noqa: E402
import yourdfpy                         # noqa: E402

ROOT = HERE.parent
DEF_URDF = ROOT / "src/robotarm/urdf/robotarm.urdf"
DEF_MESH = ROOT / "src/robotarm/meshes"
JOINTS = ["J0", "J1", "J2", "J3", "J4", "J5"]
PALETTE = [(0.80, 0.73, 0.58), (0.52, 0.60, 0.72), (0.72, 0.62, 0.58),
           (0.56, 0.70, 0.62), (0.70, 0.58, 0.74), (0.85, 0.65, 0.45),
           (0.62, 0.64, 0.68)]


def mat_to_pos_wxyz(T):
    """SE3/4x4 -> (position, (w,x,y,z))。

    注意：trimesh.quaternion_from_matrix 返回的是 (w,x,y,z)，
    不要再重排；这里直接用 pinocchio 的 Quaternion 更不容易出错。
    """
    T = np.asarray(T, dtype=np.float64)
    pos = T[:3, 3]
    R = T[:3, :3]
    qd = pin.Quaternion(R)
    return pos, np.array([qd.w, qd.x, qd.y, qd.z])


def decimate_mesh(mesh, target_faces):
    """顶点聚类减面（纯 numpy），用于浏览器端显示。不改源文件。"""
    if len(mesh.faces) <= target_faces:
        return mesh
    v = np.asarray(mesh.vertices, dtype=np.float64)
    f = np.asarray(mesh.faces, dtype=np.int64)

    def cluster(n):
        lo, hi = v.min(0), v.max(0)
        cell = max((hi - lo).max() / max(n, 1), 1e-9)
        idx = np.floor((v - lo) / cell).astype(np.int64)
        key = (idx[:, 0] * (n + 2) + idx[:, 1]) * (n + 2) + idx[:, 2]
        uniq, inv = np.unique(key, return_inverse=True)
        cnt = np.bincount(inv).astype(np.float64)
        nv = np.zeros((len(uniq), 3))
        for k in range(3):
            nv[:, k] = np.bincount(inv, weights=v[:, k]) / cnt
        nf = inv[f]
        keep = (nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2]) & (nf[:, 0] != nf[:, 2])
        nf = np.unique(np.sort(nf[keep], axis=1), axis=0)
        return nv, nf

    lo_n, hi_n, best = 10, 900, None
    for _ in range(14):
        mid = (lo_n + hi_n) // 2
        nv, nf = cluster(mid)
        if len(nf) > target_faces:
            hi_n = mid - 1
        else:
            lo_n = mid + 1
            best = (nv, nf)
        if lo_n > hi_n:
            break
    if best is None:
        best = cluster(10)
    out = trimesh.Trimesh(vertices=best[0], faces=best[1], process=False)
    return out


def rpy_to_matrix(rpy):
    return trimesh.transformations.euler_matrix(rpy[0], rpy[1], rpy[2], "sxyz")


class ArmModel:
    def __init__(self, urdf_path, mesh_dir, max_faces=8000):
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.urdf = yourdfpy.URDF.load(str(urdf_path), load_meshes=False, build_scene_graph=False)
        self.tool_fid = self.model.getFrameId("tool0")
        self.mesh_dir = pathlib.Path(mesh_dir)
        self.links = []          # (link_name, frame_id, trimesh, T_link_mesh)

        names = [n for n in self.model.names if n != "universe"]
        assert names == JOINTS, f"关节顺序不符: {names}"

        for name, link in self.urdf.link_map.items():
            if name in ("world", "tool0"):
                continue
            try:
                fid = self.model.getFrameId(name)
            except Exception:
                continue
            for vis in link.visuals:
                g = vis.geometry
                if g.mesh is None:
                    continue
                fn = os.path.basename(g.mesh.filename)
                path = self.mesh_dir / fn
                if not path.exists():
                    continue
                mesh = trimesh.load(str(path), force="mesh")
                raw_faces = len(mesh.faces)
                mesh = decimate_mesh(mesh, max_faces)
                if vis.origin is not None:
                    mesh.apply_transform(vis.origin)
                self.links.append((name, fid, mesh, vis.origin))

    def fk(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data

    def tool_pose(self, q):
        d = self.fk(q)
        return d.oMf[self.tool_fid].copy()

    def ik(self, q0, target_pos, target_R=None, iters=300, tol=1e-4):
        q = np.array(q0, dtype=np.float64)
        lo = self.model.lowerPositionLimit
        hi = self.model.upperPositionLimit
        lam = 1e-3
        for it in range(iters):
            d = self.fk(q)
            oMf = d.oMf[self.tool_fid]
            e_pos = target_pos - oMf.translation
            if target_R is None:
                err = np.concatenate([e_pos, np.zeros(3)])
                if np.linalg.norm(e_pos) < tol:
                    return q, float(np.linalg.norm(e_pos))
            else:
                e_rot = 0.5 * (np.cross(oMf.rotation[:, 0], target_R[:, 0]) +
                               np.cross(oMf.rotation[:, 1], target_R[:, 1]) +
                               np.cross(oMf.rotation[:, 2], target_R[:, 2]))
                err = np.concatenate([e_pos, e_rot])
                if np.linalg.norm(e_pos) < tol and np.linalg.norm(e_rot) < tol:
                    return q, float(np.linalg.norm(e_pos))
            J = pin.computeFrameJacobian(self.model, self.data, q, self.tool_fid, pin.LOCAL)
            dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), err)
            q = pin.integrate(self.model, q, np.clip(dq, -0.2, 0.2))
            q = np.clip(q, lo + 1e-4, hi - 1e-4)
        return q, float(np.linalg.norm(target_pos - self.fk(q).oMf[self.tool_fid].translation))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=str(DEF_URDF))
    ap.add_argument("--mesh-dir", default=str(DEF_MESH))
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--no-ik", action="store_true", help="关掉末端 IK 面板")
    ap.add_argument("--max-faces", type=int, default=8000,
                    help="每块网格减到多少面（只影响显示，不动源文件）")
    args = ap.parse_args()

    arm = ArmModel(args.urdf, args.mesh_dir, max_faces=args.max_faces)
    print(f"加载 {len(arm.links)} 块网格, {len(arm.links)} 个 visual")
    q = np.zeros(arm.model.nq)
    lo, hi = arm.model.lowerPositionLimit, arm.model.upperPositionLimit

    server = viser.ViserServer(host=args.host, port=args.port)
    server.scene.add_grid("/ground", width=1.2, height=1.2, cell_size=0.05)

    handles = []
    for i, (link, fid, mesh, _) in enumerate(arm.links):
        m = mesh.copy()
        rgb = PALETTE[i % len(PALETTE)]
        m.visual.face_colors = np.array(
            [int(rgb[0] * 255), int(rgb[1] * 255), int(rgb[2] * 255), 255], dtype=np.uint8)
        h = server.scene.add_mesh_trimesh(f"/robot/{link}/{i}", m)
        handles.append(h)

    tgt_h = server.scene.add_icosphere("/target", radius=0.012,
                                       color=(230, 60, 60), position=(0.15, 0.0, 0.25))
    real_h = server.scene.add_icosphere("/real_tcp", radius=0.009,
                                        color=(40, 200, 120),
                                        position=(0.0, 0.0, 0.4), visible=False)
    gizmo = server.scene.add_transform_controls(
        "/target_ctrl", scale=0.09,
        position=(0.15, 0.0, 0.25), wxyz=(1.0, 0.0, 0.0, 0.0),
        disable_rotations=args.no_ik)

    state = {"q": q.copy(), "live_ik": False, "busy": False}

    def redraw():
        d = arm.fk(state["q"])
        for (link, fid, mesh, _), h in zip(arm.links, handles):
            pos, wxyz = mat_to_pos_wxyz(d.oMf[fid].homogeneous)
            h.position = tuple(pos)
            h.wxyz = tuple(wxyz)
        p = d.oMf[arm.tool_fid]
        tgt_h.position = tuple(p.translation)

    redraw()
    gizmo.position = tuple(arm.tool_pose(state["q"]).translation)

    # ---------------- GUI
    with server.gui.add_folder("关节控制 (°)"):
        sliders = []
        for i, jn in enumerate(JOINTS):
            s = server.gui.add_slider(
                jn, min=float(np.degrees(lo[i])), max=float(np.degrees(hi[i])),
                step=0.5, initial_value=0.0)

            def make_cb(idx, slider):
                def cb(_):
                    state["q"][idx] = np.radians(slider.value)
                    redraw()
                return cb
            s.on_update(make_cb(i, s))
            sliders.append(s)

    ui = {}
    if not args.no_ik:
        with server.gui.add_folder("末端目标 / IK"):
            ui["live"] = server.gui.add_checkbox("拖动目标实时 IK", False)
            ui["pos"] = server.gui.add_vector3("目标位置 (m)", initial_value=(0.15, 0.0, 0.25), step=0.005)
            ui["snap"] = server.gui.add_button("📍 目标对齐当前末端")
            ui["solve"] = server.gui.add_button("🎯 求解 IK")
            ui["plan"] = server.gui.add_button("▶ 规划并播放到目标")
            ui["dur"] = server.gui.add_number("规划时长 (s)", initial_value=2.0, min=0.5, max=10.0, step=0.5)
            ui["status"] = server.gui.add_text("状态", "就绪")

    with server.gui.add_folder("其他"):
        ui["home"] = server.gui.add_button("⟲ 回零位")
        ui["sync"] = server.gui.add_checkbox("同步到真机 (/joint_cmd)", False)
        ui["qtext"] = server.gui.add_text("当前关节 (°)", "0 0 0 0 0 0")
        ui["realtext"] = server.gui.add_text("真机关节 (°)", "等待 /joint_states")
        ui["errtext"] = server.gui.add_text("跟随误差 (°)", "—")

    if args.no_ik:
        ui["live"] = None

    # ---------------- 事件
    def do_ik():
        pos = np.array(ui["pos"].value)
        q0 = state["q"].copy()
        qs, err = arm.ik(q0, pos)
        state["q"] = qs
        redraw()
        for i, s in enumerate(sliders):
            s.value = float(np.degrees(qs[i]))
        ui["status"].value = ("✅ 收敛，残差 %.1f mm" % (err * 1000)) if err < 1e-3 \
            else ("⚠️ 未收敛，残差 %.1f mm（可能超出可达范围）" % (err * 1000))

    def do_ik_async():
        """IK 放到后台线程，避免阻塞 viser 的事件循环（会掐断 websocket）"""
        def work():
            try:
                do_ik()
            except Exception as exc:      # noqa: BLE001
                ui["status"].value = "❌ IK 异常: %s" % exc
        threading.Thread(target=work, daemon=True).start()

    if not args.no_ik:
        @ui["snap"].on_click
        def _(_):
            p_now = arm.tool_pose(state["q"]).translation
            gizmo.position = tuple(p_now)
            ui["pos"].value = tuple(p_now)
            ui["status"].value = "目标已对齐到当前末端"

        ui["solve"].on_click(lambda _: do_ik_async())

        @ui["live"].on_update
        def _(_):
            state["live_ik"] = ui["live"].value

        @gizmo.on_update
        def _(_):
            ui["pos"].value = tuple(gizmo.position)
            if state["live_ik"] and not state["busy"]:
                state["busy"] = True

                def _work():
                    try:
                        do_ik()
                    finally:
                        state["busy"] = False
                threading.Thread(target=_work, daemon=True).start()

        play_state = {"busy": False}

        def _plan_worker():
            """后台线程：先解 IK，再平滑插值播放"""
            try:
                do_ik()
                q_end = state["q"].copy()
                q_start = np.array([np.radians(sl.value) for sl in sliders])
                dur = float(ui["dur"].value)
                steps = max(int(dur * 50), 10)
                for k in range(steps + 1):
                    t = k / steps
                    sm = t * t * (3 - 2 * t)      # 平滑插值
                    state["q"] = q_start + (q_end - q_start) * sm
                    redraw()
                    for i, sl in enumerate(sliders):
                        sl.value = float(np.degrees(state["q"][i]))
                    time.sleep(dur / steps)
                ui["status"].value = "✅ 播放完成"
            except Exception as exc:          # noqa: BLE001
                ui["status"].value = "❌ 播放异常: %s" % exc
            finally:
                play_state["busy"] = False

        def plan_and_play(_=None):
            if play_state["busy"]:
                return
            play_state["busy"] = True
            threading.Thread(target=_plan_worker, daemon=True).start()

        ui["plan"].on_click(plan_and_play)

    @ui["home"].on_click
    def _(_):
        state["q"] = np.zeros(arm.model.nq)
        redraw()
        for s in sliders:
            s.value = 0.0

    # ---------------- 同步真机
    ros = {"node": None, "pub": None, "real": None}
    try:
        import rclpy
        from sensor_msgs.msg import JointState
        rclpy.init()
        node = rclpy.create_node("arm_viser")
        ros["node"] = node
        ros["pub"] = node.create_publisher(JointState, "/joint_cmd", 10)

        def _js_cb(msg):
            qq = np.zeros(len(JOINTS))
            for n, val in zip(msg.name, msg.position):
                if n in JOINTS:
                    qq[JOINTS.index(n)] = float(val)
            ros["real"] = qq

        node.create_subscription(JointState, "/joint_states", _js_cb, 10)

        def _spin_safe():
            """Ctrl-C 时 rclpy 上下文会先被销毁，spin 会抛异常；这里吞掉，避免吓人的 traceback"""
            try:
                rclpy.spin(node)
            except Exception:      # noqa: BLE001
                pass
        threading.Thread(target=_spin_safe, daemon=True).start()
        print("已连接 ROS，可同步到 /joint_cmd")
    except Exception as e:
        print("ROS 未启用:", e)

    print(f"\n  👉 浏览器打开  http://localhost:{args.port}\n")
    try:
        while True:
            if ros["pub"] is not None and ui["sync"].value:
                m = JointState()
                m.header.stamp = ros["node"].get_clock().now().to_msg()
                m.name = JOINTS
                m.position = [float(x) for x in state["q"]]
                ros["pub"].publish(m)
            ui["qtext"].value = " ".join("%.1f" % np.degrees(x) for x in state["q"])
            qr = ros["real"]
            if qr is not None:
                real_h.visible = True
                real_h.position = tuple(arm.tool_pose(qr).translation)
                ui["realtext"].value = " ".join("%.1f" % np.degrees(x) for x in qr)
                err = np.degrees(np.abs(qr - state["q"]))
                ui["errtext"].value = "最大 %.1f°，各轴 %s" % (
                    err.max(), " ".join("%.1f" % e for e in err))
                if ui["sync"].value:
                    # 目标=当前模型姿态；真机在跟就正常
                    pass
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if ros["node"] is not None:
                ros["node"].destroy_node()
            import rclpy as _r
            if _r.ok():
                _r.shutdown()
        except Exception:      # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
