#!/usr/bin/env python3
"""由 CAD 匯出的 xml/T170-V2.1-A0_t_pose.xml 產生 graphen_mujoco/MujocoSystem 用的 mjcf/ti5_t170.xml。

訓練端（robolab Ti5T170-Flat）用的是同一份 T-pose 模型（rl_flat 的 SHOULDER_R = ∓1.3 在 Q-pose 的
範圍外），這裡只補上模擬與 ros2_control 需要、原檔沒有的設定，質量與幾何不動：
  - timestep 0.005、implicitfast（訓練的 sim dt）
  - 各關節 armature 取自 param.csv（與訓練的 actuators.armature 相同）
  - motor 的 ctrlrange 取自原檔 joint 的 actuatorfrcrange
  - 碰撞只算機器人與地面（訓練為 self_collision: false）
  - BASE_S 原點的 imu site 與 framequat / gyro / accelerometer（MujocoSystem 依類型與名稱尋找）
  - keyframe home：訓練的 init_state.joint_pos，基座高度為腳底貼地的高度

  - 腳掌（RECENTERED_GEOMS）的 mesh 移回 link 原點：原檔帶著 URDF 給 Gazebo box 用的碰撞偏移
    （z -0.085），腳掌會和腳踝分開

例外：FLIPPED_JOINTS 的旋轉方向改成與 ti5_170a 相同（與 URDF 一致），這幾個關節的正負號和原檔、
也就是訓練端相反。

原檔更新後重新執行（只用標準函式庫）：
  python3 scripts/make_mjcf.py --base-height 0.985   # 腳底貼地為 0.98 m，留 5 mm 落地
"""
import argparse
import csv
import xml.etree.ElementTree as ET
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
SOURCE = PKG / 'xml' / 'T170-V2.1-A0_t_pose.xml'
OUTPUT = PKG / 'mjcf' / 'ti5_t170.xml'
PARAMS = PKG / 'param.csv'

# 訓練的 init_state.joint_pos，沒有列出的為 0
HOME_JOINT_POS = {
    'L_HIP_P': -0.2, 'L_KNEE_P': 0.4, 'L_ANKLE_P': 0.2,
    'R_HIP_P': 0.2, 'R_KNEE_P': -0.4, 'R_ANKLE_P': -0.2,
    'L_SHOULDER_R': -1.3, 'R_SHOULDER_R': 1.3,
    'L_ELBOW_Y': -0.3, 'R_ELBOW_Y': 0.3,
}
# 旋轉方向與 ti5_170a 相反的關節：axis 反向、range 對調變號（物理範圍不變），與 URDF 保持一致。
# 訓練端的模型沒有反向：ti5_t170 訓練的 policy 在這幾個關節的 obs / action 正負號要另外處理
FLIPPED_JOINTS = ('L_WRIST_Y', 'L_WRIST_R', 'R_SHOULDER_Y', 'R_WRIST_Y', 'R_WRIST_R', 'NECK_P')
# mesh 要放回 link 原點的 geom（以 mesh 名稱指定）。原檔由 URDF 的碰撞轉出，腳掌帶著只給 Gazebo box 用的
# 下移量；mesh 本身已經在 link 座標系（URDF 的 visual origin 為 0）。
# TODO(verify-with-training): 訓練端由 URDF 載入，碰撞可能仍帶著這個偏移（腳底低 85 mm）
RECENTERED_GEOMS = ('L_ANKLE_R_S', 'R_ANKLE_R_S')
# param.csv 與 graphen 的 joint 名稱不同的項目
PARAM_RENAME = {'NECK_Y1': 'NECK_Y'}


def load_armature():
    with PARAMS.open(encoding='utf-8') as f:
        rows = list(csv.reader(f))
    return {PARAM_RENAME.get(r[0], r[0]): r[1] for r in rows[1:] if r}


def negate(values):
    """把空白分隔的數值字串逐一變號（0 維持 0，不輸出 -0）。"""
    return [f'{-float(v):g}' if float(v) != 0 else '0' for v in values.split()]


def flip_joint(joint):
    name = joint.get('name')
    if name in HOME_JOINT_POS:
        raise SystemExit(f"flipped joint '{name}' has a home position, negate it as well")
    joint.set('axis', ' '.join(negate(joint.get('axis'))))
    joint.set('range', ' '.join(reversed(negate(joint.get('range')))))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--base-height', type=float, required=True,
                        help='BASE_S height of the home keyframe (m)')
    args = parser.parse_args()

    tree = ET.parse(SOURCE)
    root = tree.getroot()
    root.set('model', 'ti5_t170')

    compiler = root.find('compiler')
    compiler.set('meshdir', '../meshes')
    compiler.set('autolimits', 'true')
    option = ET.Element('option', timestep='0.005', integrator='implicitfast')
    root.insert(list(root).index(compiler) + 1, option)

    # 機器人的 geom 只和地面碰撞：robot contype=1 / conaffinity=0，地面 contype=0 / conaffinity=1
    default = ET.Element('default')
    ET.SubElement(default, 'geom', contype='1', conaffinity='0', condim='3',
                  friction='1.0 0.005 0.0001')
    root.insert(list(root).index(option) + 1, default)

    armature = load_armature()
    joints = []
    ranges = {}
    for joint in root.iter('joint'):
        if joint.get('type') == 'free':
            continue
        name = joint.get('name')
        if name not in armature:
            raise SystemExit(f"joint '{name}' is missing in {PARAMS.name}")
        joint.set('armature', armature[name])
        ranges[name] = joint.attrib.pop('actuatorfrcrange')
        joints.append(name)
    missing = set(FLIPPED_JOINTS) - set(joints)
    if missing:
        raise SystemExit(f'flipped joints not found in {SOURCE.name}: {sorted(missing)}')
    for joint in root.iter('joint'):
        if joint.get('name') in FLIPPED_JOINTS:
            flip_joint(joint)

    recentered = [g for g in root.iter('geom') if g.get('mesh') in RECENTERED_GEOMS]
    if len(recentered) != len(RECENTERED_GEOMS):
        raise SystemExit(f'expected one geom for each of {RECENTERED_GEOMS} in {SOURCE.name}')
    for geom in recentered:
        geom.attrib.pop('pos', None)
        geom.attrib.pop('quat', None)

    base = next(b for b in root.iter('body') if b.get('name') == 'BASE_S')
    base.set('pos', f'0 0 {args.base_height:g}')
    free = base.find('joint')
    base.insert(list(base).index(free) + 1, ET.Element('site', name='imu', size='0.01'))

    for motor in root.find('actuator'):
        motor.set('ctrlrange', ranges[motor.get('joint')])

    floor = next(g for g in root.iter('geom') if g.get('name') == 'floor')
    floor.set('contype', '0')
    floor.set('conaffinity', '1')
    floor.set('friction', '1.0 0.005 0.0001')

    sensor = ET.SubElement(root, 'sensor')
    ET.SubElement(sensor, 'framequat', name='imu_quat', objtype='site', objname='imu')
    ET.SubElement(sensor, 'gyro', name='gyro', site='imu')
    ET.SubElement(sensor, 'accelerometer', name='imu_acc', site='imu')

    # qpos：free joint（7）+ 依 body 樹順序的關節
    qpos = [0.0, 0.0, args.base_height, 1.0, 0.0, 0.0, 0.0]
    qpos += [HOME_JOINT_POS.get(name, 0.0) for name in joints]
    keyframe = ET.SubElement(root, 'keyframe')
    ET.SubElement(keyframe, 'key', name='home', qpos=' '.join(f'{v:g}' for v in qpos))

    ET.indent(tree, space='  ')
    OUTPUT.parent.mkdir(exist_ok=True)
    header = (f'<!-- 由 scripts/make_mjcf.py 從 {SOURCE.relative_to(PKG)} 產生，'
              '不要手動修改 -->\n')
    OUTPUT.write_text(header + ET.tostring(root, encoding='unicode') + '\n', encoding='utf-8')
    print(f'wrote {OUTPUT} ({len(joints)} joints)')


if __name__ == '__main__':
    main()
