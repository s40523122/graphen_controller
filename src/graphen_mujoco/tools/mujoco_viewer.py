"""顯示 graphen_mujoco/MujocoSystem 的模擬畫面（只顯示，不做物理）。

MujocoSystem（ros2_control_node 內）以 UDP 把 [time, qpos...] 送到 127.0.0.1:<port>，
這裡載入同一份 MJCF，收到後設定 qpos、mj_forward，再更新視窗。
按 End 鍵送出 "reset" 到 <port>+1，模擬回到初始姿態。

需要 mujoco 與 glfw（主機的 ~/learnWalk_env），通常由 mujoco_viewer.sh 啟動：
  bash mujoco_viewer.sh                         # 預設 models/t170_free.xml、port 9870
  bash mujoco_viewer.sh --model ../models/t170_fixed.xml
"""
import argparse
import socket
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

HERE = Path(__file__).resolve().parent
KEY_END = 269


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=str(HERE.parent / "models" / "t170_free.xml"))
    ap.add_argument("--port", type=int, default=9870)
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(args.model)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    sock.setblocking(False)
    frame_bytes = 8 * (1 + m.nq)

    def on_key(key):
        if key == KEY_END:
            sock.sendto(b"reset", ("127.0.0.1", args.port + 1))
            print("reset sent")

    print(f"model {args.model}: nq={m.nq}; listening on 127.0.0.1:{args.port} (End = reset)")
    base = m.body("BASE_S").id
    last_rx = 0.0
    with mujoco.viewer.launch_passive(m, d, key_callback=on_key) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = base
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 3.0, -15, 135
        while v.is_running():
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
                        d.qpos[:] = frame[1:]
                        mujoco.mj_forward(m, d)
                    v.sync()
                    last_rx = time.monotonic()
            elif time.monotonic() - last_rx > 2.0 and last_rx > 0:
                print("no data from MujocoSystem for 2 s")
                last_rx = 0.0
            time.sleep(0.005)


if __name__ == "__main__":
    main()
