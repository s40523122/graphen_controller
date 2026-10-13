"""把 FullBodyT70 的訓練場景匯出成 MJCF，給 graphen_mujoco/MujocoSystem 載入。

場景直接沿用 play_mujoco.py 的 build_spec()（freejoint、PD 位置致動器、碰撞設定、200 Hz），
只做 ros2_control 需要的修改：
  - NECK_Y1 改名為 NECK_Y（graphen 的 joint 名稱）
  - imu site 移到 BASE_S 的 (0, 0, 0.1)，對應 URDF 的 imu_link；加上 framequat 與 accelerometer
  - 關閉 fusestatic（見 make_spec 的註解）
  - meshdir 改成絕對路徑（容器內掛載路徑相同），不複製 79 MB 的 STL

輸出：
  models/t170_free.xml   浮動基座（自由站立）
  models/t170_fixed.xml  固定基座，懸在 BASE_HEIGHT（相當於吊掛）

需要 mujoco（主機的 ~/learnWalk_env）：
  ~/learnWalk_env/bin/python tools/export_mjcf.py
"""
import argparse
import runpy
import sys
import types
from pathlib import Path

import mujoco

HERE = Path(__file__).resolve().parent
DEFAULT_PLAY = Path("/devtest/FullBodyT70/cpp_interface/play_mujoco.py")
IMU_POS = (0.0, 0.0, 0.1)  # 與 ti5_170a_graphen.urdf.xacro 的 imu_xyz 相同
RENAME = {"NECK_Y1": "NECK_Y"}


def add_sensor(spec, type_, name, site):
    s = spec.add_sensor()
    s.type, s.name = type_, name
    s.objtype, s.objname = mujoco.mjtObj.mjOBJ_SITE, site


def make_spec(play, fixed_base):
    spec = play["build_spec"]()
    spec.meshdir = str(play["ASSET"] / "assets")
    # URDF 的 fixed joint（ZJ1_S、chest_camera_s、NECK_CAMERA_S）在 compile 時會合併到父 body，
    # 但 to_xml() 寫出的檔案會遺失合併進來的質量與 geom 位置（總質量少約 1 kg）。
    # 不合併、保留成沒有 joint 的子 body，動力學相同
    spec.compiler.fusestatic = False
    spec.modelname = "t170_fixed" if fixed_base else "t170_free"

    for old, new in RENAME.items():
        spec.joint(old).name = new
        act = spec.actuator(old)
        act.name, act.target = new, new

    spec.site("imu").pos = IMU_POS
    add_sensor(spec, mujoco.mjtSensor.mjSENS_FRAMEQUAT, "imu_quat", "imu")
    add_sensor(spec, mujoco.mjtSensor.mjSENS_ACCELEROMETER, "imu_acc", "imu")

    base = spec.body("BASE_S")
    if fixed_base:
        spec.delete(spec.joint("root"))
        base.pos = (0.0, 0.0, play["BASE_HEIGHT"])
    return spec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--play", type=Path, default=DEFAULT_PLAY, help="play_mujoco.py 的路徑")
    ap.add_argument("--out", type=Path, default=HERE.parent / "models")
    args = ap.parse_args()

    # play_mujoco 在模組層 import onnxruntime，匯出用不到：沒有安裝時放一個空模組
    try:
        import onnxruntime  # noqa: F401
    except ImportError:
        sys.modules["onnxruntime"] = types.ModuleType("onnxruntime")
    play = runpy.run_path(str(args.play), run_name="play_mujoco")
    args.out.mkdir(parents=True, exist_ok=True)
    for fixed in (False, True):
        spec = make_spec(play, fixed)
        model = spec.compile()
        path = args.out / f"{spec.modelname}.xml"
        path.write_text(spec.to_xml(), encoding="utf-8")
        # 重新載入確認檔案可用
        reloaded = mujoco.MjModel.from_xml_path(str(path))
        assert (reloaded.nq, reloaded.nu) == (model.nq, model.nu)
        assert abs(reloaded.body_mass.sum() - model.body_mass.sum()) < 1e-4
        print(f"{path}: nq={model.nq} nv={model.nv} nu={model.nu} nsensor={model.nsensor}")


if __name__ == "__main__":
    main()
