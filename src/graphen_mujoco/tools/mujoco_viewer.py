"""顯示 graphen_mujoco/MujocoSystem 的模擬畫面（只顯示，不做物理）。

MujocoSystem（ros2_control_node 內）以 UDP 把 [time, qpos(nq), actuator_force(nu)] 送到
127.0.0.1:<port>，這裡載入同一份 MJCF，收到後設定 qpos、mj_forward，再更新視窗；
各致動器的力矩以文字疊加在畫面右上角（力矩 / 上限 與百分比，達 warn_ratio 以上標 "!"），
並在關節上畫箭頭：沿轉軸、依右手定則指向力矩方向，長度與顏色（綠 → 紅）依 力矩 / 上限。
箭頭的起點在關節中心、會被機身遮住，所以顯示箭頭時機身改為半透明（T 鍵可以再切換）。
按 End 鍵送出 "reset" 到 <port>+1，模擬回到初始姿態；Home 鍵切換文字，PageUp 鍵切換箭頭。

需要 mujoco 與 glfw（主機的 ~/learnWalk_env），通常由 mujoco_viewer.sh 啟動：
  bash mujoco_viewer.sh                         # 預設 fb70 的 t170_free.xml、port 9870
  bash mujoco_viewer.sh --model ../../robot_description/fb70/mjcf/t170_fixed.xml
"""
import argparse
import socket
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

HERE = Path(__file__).resolve().parent
# 預設模型：fb70_description 套件（src/robot_description/fb70）的浮動基座 MJCF
FB70_MJCF = HERE.parents[1] / "robot_description" / "fb70" / "mjcf"
KEY_END = 269
KEY_HOME = 268
KEY_PAGE_UP = 266
ARROW_MIN_LENGTH = 1e-3   # 比這個短的箭頭不畫（m）


def actuator_labels(m):
    """致動器顯示名稱：驅動 joint 的致動器用 joint 名稱，其他用致動器名稱。"""
    labels = []
    for i in range(m.nu):
        if m.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT:
            labels.append(m.joint(m.actuator_trnid[i, 0]).name)
        else:
            labels.append(m.actuator(i).name)
    return labels


def actuator_limits(m):
    """各致動器的力矩上限；沒有上限時為 NaN（只顯示數值，不算百分比）。

    forcerange 優先；motor 致動器（gain 固定、沒有 bias）沒有 forcerange 時，由 ctrlrange × gear × gain 推得。
    """
    limits = np.full(m.nu, np.nan)
    for i in range(m.nu):
        if m.actuator_forcelimited[i]:
            limits[i] = np.abs(m.actuator_forcerange[i]).max()
        elif (m.actuator_ctrllimited[i]
              and m.actuator_gaintype[i] == mujoco.mjtGain.mjGAIN_FIXED
              and m.actuator_biastype[i] == mujoco.mjtBias.mjBIAS_NONE):
            limits[i] = (np.abs(m.actuator_ctrlrange[i]).max()
                         * abs(m.actuator_gear[i, 0] * m.actuator_gainprm[i, 0]))
    return limits


def actuator_hinges(m):
    """各致動器驅動的 hinge joint id；不是驅動 hinge 的致動器為 -1（不畫箭頭）。"""
    hinges = np.full(m.nu, -1)
    for i in range(m.nu):
        if m.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT:
            joint = m.actuator_trnid[i, 0]
            if m.jnt_type[joint] == mujoco.mjtJoint.mjJNT_HINGE:
                hinges[i] = joint
    return hinges


def draw_torque_arrows(scn, d, hinges, limits, forces, args):
    """在 user_scn 畫各關節的力矩箭頭（呼叫前 d 已經 mj_forward）。

    有上限的軸長度為 arrow_length × |力矩| / 上限；沒有上限的軸以 arrow_scale（m / N·m）換算、灰色顯示。
    """
    scn.ngeom = 0
    for joint, limit, force in zip(hinges, limits, forces):
        if joint < 0 or scn.ngeom >= scn.maxgeom:
            continue
        if np.isfinite(limit) and limit > 0:
            ratio = min(abs(force) / limit, 1.0)
            length = args.arrow_length * ratio
            rgba = np.array([ratio, 1.0 - ratio, 0.0, 0.9])
        else:
            length = args.arrow_scale * abs(force)
            rgba = np.array([0.6, 0.6, 0.6, 0.9])
        if length < ARROW_MIN_LENGTH:
            continue
        start = d.xanchor[joint]
        end = start + np.sign(force) * length * d.xaxis[joint]
        geom = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3),
                            np.zeros(9), rgba.astype(np.float32))
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, args.arrow_width, start, end)
        scn.ngeom += 1


def torque_text(labels, limits, forces, warn_ratio):
    names, values = [], []
    for label, limit, force in zip(labels, limits, forces):
        if np.isfinite(limit) and limit > 0:
            ratio = abs(force) / limit
            mark = " !" if ratio >= warn_ratio else ""
            values.append(f"{force:7.1f} / {limit:5.0f} ({ratio * 100:3.0f}%){mark}")
        else:
            values.append(f"{force:7.1f}")
        names.append(label)
    return "\n".join(names), "\n".join(values)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=str(FB70_MJCF / "t170_free.xml"))
    ap.add_argument("--port", type=int, default=9870)
    ap.add_argument("--no-torque", action="store_true", help="hide the joint torque overlay")
    ap.add_argument("--warn-ratio", type=float, default=0.9,
                    help="mark joints whose |torque| / limit is at least this ratio")
    ap.add_argument("--no-arrows", action="store_true", help="hide the joint torque arrows")
    ap.add_argument("--arrow-length", type=float, default=0.3,
                    help="arrow length (m) at the torque limit")
    ap.add_argument("--arrow-scale", type=float, default=0.005,
                    help="arrow length per N*m (m) for actuators without a torque limit")
    ap.add_argument("--arrow-width", type=float, default=0.02, help="arrow shaft width (m)")
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(args.model)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    sock.setblocking(False)
    frame_bytes = 8 * (1 + m.nq + m.nu)
    labels, limits = actuator_labels(m), actuator_limits(m)
    hinges = actuator_hinges(m)
    show_torque = [not args.no_torque]
    show_arrows = [not args.no_arrows]

    def on_key(key):
        if key == KEY_END:
            sock.sendto(b"reset", ("127.0.0.1", args.port + 1))
            print("reset sent")
        elif key == KEY_HOME:
            show_torque[0] = not show_torque[0]
        elif key == KEY_PAGE_UP:
            show_arrows[0] = not show_arrows[0]

    print(f"model {args.model}: nq={m.nq}, nu={m.nu}; listening on 127.0.0.1:{args.port} "
          "(End = reset, Home = torque text, PageUp = torque arrows)")
    base = m.body("BASE_S").id
    last_rx = 0.0
    with mujoco.viewer.launch_passive(m, d, key_callback=on_key) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = base
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 3.0, -15, 135
        arrows_shown = None   # 主迴圈已套用的箭頭狀態；key callback 在 viewer thread，只改旗標
        while v.is_running():
            if arrows_shown != show_arrows[0]:
                arrows_shown = show_arrows[0]
                with v.lock():
                    v.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = int(arrows_shown)
                    if not arrows_shown:
                        v.user_scn.ngeom = 0
                v.sync()
            latest = None
            try:
                while True:  # 只取最新一筆
                    latest = sock.recv(65536)
            except BlockingIOError:
                pass
            if latest is not None:
                if len(latest) != frame_bytes:
                    print(f"frame size {len(latest)} != {frame_bytes}: model mismatch?")
                else:
                    frame = np.frombuffer(latest, dtype=np.float64)
                    with v.lock():
                        d.time = frame[0]
                        d.qpos[:] = frame[1:1 + m.nq]
                        mujoco.mj_forward(m, d)
                        if arrows_shown:
                            draw_torque_arrows(v.user_scn, d, hinges, limits, frame[1 + m.nq:], args)
                    if show_torque[0]:
                        names, values = torque_text(labels, limits, frame[1 + m.nq:], args.warn_ratio)
                        v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100,
                                     mujoco.mjtGridPos.mjGRID_TOPRIGHT, names, values))
                    else:
                        v.set_texts([])
                    v.sync()
                    last_rx = time.monotonic()
            elif time.monotonic() - last_rx > 2.0 and last_rx > 0:
                print("no data from MujocoSystem for 2 s")
                last_rx = 0.0
            time.sleep(0.005)


if __name__ == "__main__":
    main()
