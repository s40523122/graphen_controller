#!/usr/bin/env python3
"""由本套件的 URDF（urdf/robot.urdf.xacro）產生 graphen_mujoco/MujocoSystem 用的 mjcf/ti5_170a.xml。

流程：xacro 展開 → 調整 URDF → MuJoCo 的 compile（URDF → MJCF）→ 補上模擬與 ros2_control 需要的設定。
質量與幾何沿用 URDF，不修改：
  - 靈巧手的關節改成固定（模擬中沒有致動器，不固定會自由晃動）
  - 加上 floating 基座（MJCF 的 free joint，名稱 floating_base_joint）
  - timestep 0.005、implicitfast（與 ti5_t170 相同）
  - 各關節 armature：本模型沒有參數表，借用 ti5_t170_description/param.csv（同型馬達；推測值）
  - motor 致動器，ctrlrange 取自 URDF 的 effort limit
  - 碰撞只算機器人與地面；只保留碰撞 geom（visual 與 collision 是同一份 mesh）
  - base_link 上 (0, 0, 0.1) 的 imu site（與 ti5_170a_graphen.urdf.xacro 的 imu_link 預設相同）
    與 framequat / gyro / accelerometer（MujocoSystem 依類型與名稱尋找）
  - keyframe home：config/initial_positions.yaml 的姿態；基座高度由腳底 mesh 的最低點自動計算（留 5 mm 落地）

注意：URDF 的質量分布可疑（WAIST_R_S 49 kg、本體 93.7 kg；ti5_t170 為 59.5 kg），
模擬結果不代表實機，以 ti5_t170 為準。

URDF 或 mesh 更新後重新執行（需要 source ROS 環境以展開 xacro；MuJoCo 依 MUJOCO_DIR，
或 ~/.local、/opt 下的 mujoco-3.11.0，與 graphen_mujoco 相同）：
  python3 scripts/make_mjcf.py
執行後把印出的基座高度同步到 graphen_bringup/launch/bringup.launch.py 的 robots['ti5_170a']。
"""
import math
import os
import struct
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parent.parent
SOURCE = PKG / 'urdf' / 'robot.urdf.xacro'
OUTPUT = PKG / 'mjcf' / 'ti5_170a.xml'
INITIAL_POSITIONS = PKG / 'config' / 'initial_positions.yaml'
# 同型馬達的 armature（推測值，見 docstring）
ARMATURE_TABLE = PKG.parent / 'ti5_t170_description' / 'param.csv'
ARMATURE_RENAME = {'NECK_Y1': 'NECK_Y'}
MUJOCO_VERSION = '3.11.0'
# 與 ti5_170a_graphen.urdf.xacro 的 imu_xyz 預設相同
IMU_POS = '0 0 0.1'
# 腳底最低點與地面的間隙（m），讓模型從略高處落地，避免初始穿透
GROUND_CLEARANCE = 0.005


def find_mujoco():
    candidates = [os.environ.get('MUJOCO_DIR')] + [
        str(Path(base) / f'mujoco-{MUJOCO_VERSION}') for base in (Path.home() / '.local', '/opt')]
    for candidate in candidates:
        if candidate and (Path(candidate) / 'bin' / 'compile').exists():
            return Path(candidate)
    raise SystemExit(f'MuJoCo {MUJOCO_VERSION} not found (set MUJOCO_DIR)')


def load_armature():
    rows = ARMATURE_TABLE.read_text(encoding='utf-8').splitlines()[1:]
    table = {}
    for row in rows:
        if row.strip():
            name, armature = row.split(',')[:2]
            table[ARMATURE_RENAME.get(name, name)] = armature
    return table


def prepare_urdf(urdf_xml):
    """手的關節改固定、加上 floating 基座、mesh 改成絕對路徑，回傳 (URDF 字串, 本體關節名稱)。"""
    root = ET.fromstring(urdf_xml)
    body_joints = []
    for joint in root.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        if joint.get('name').startswith(('left_', 'right_')):
            joint.set('type', 'fixed')
        else:
            body_joints.append(joint.get('name'))
    prefix = 'package://ti5_170a_graphen/'
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename')
        if not filename.startswith(prefix):
            raise SystemExit(f"mesh outside this package is not supported: {filename}")
        mesh.set('filename', str(PKG / filename[len(prefix):]))
    # MuJoCo 的 URDF 解析器把 floating joint 轉成 free joint；world link 對應 worldbody
    ET.SubElement(root, 'link', name='world')
    floating = ET.SubElement(root, 'joint', name='floating_base_joint', type='floating')
    ET.SubElement(floating, 'parent', link='world')
    ET.SubElement(floating, 'child', link='base_link')
    mujoco = ET.SubElement(root, 'mujoco')
    ET.SubElement(mujoco, 'compiler', discardvisual='false', fusestatic='false', strippath='false')
    return ET.tostring(root, encoding='unicode'), body_joints


def compile_urdf(urdf, mujoco_dir):
    with tempfile.TemporaryDirectory() as tmp:
        src, dst = Path(tmp) / 'robot.urdf', Path(tmp) / 'robot.xml'
        src.write_text(urdf, encoding='utf-8')
        env = dict(os.environ, LD_LIBRARY_PATH=str(mujoco_dir / 'lib'))
        subprocess.run([str(mujoco_dir / 'bin' / 'compile'), str(src), str(dst)],
                       check=True, env=env, stdout=subprocess.DEVNULL)
        return ET.parse(dst)


# ---- 基座高度：零位以外的姿態也適用，以 body 的 pos / quat 與關節角度做 FK ----

def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def quat_rotate(q, v):
    w, x, y, z = q
    p = quat_mul(quat_mul(q, (0.0, *v)), (w, -x, -y, -z))
    return p[1:]


def axis_angle(axis, angle):
    norm = math.sqrt(sum(a * a for a in axis))
    s = math.sin(angle / 2) / norm
    return (math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def stl_vertices(path):
    data = path.read_bytes()
    if data[:5].lower() == b'solid' and b'facet' in data[:1024]:
        for line in data.decode(errors='ignore').splitlines():
            parts = line.split()
            if parts and parts[0] == 'vertex':
                yield tuple(float(v) for v in parts[1:4])
        return
    count = struct.unpack_from('<I', data, 80)[0]
    for i in range(count):
        values = struct.unpack_from('<12f', data, 84 + 50 * i)
        for k in range(3, 12, 3):
            yield values[k:k + 3]


def lowest_point(worldbody, mesh_files, joint_positions):
    """base_link 放在原點時，所有碰撞 mesh 頂點的最低 z。"""
    lowest = math.inf

    def visit(body, pos, quat):
        nonlocal lowest
        local_pos = tuple(float(v) for v in body.get('pos', '0 0 0').split())
        local_quat = tuple(float(v) for v in body.get('quat', '1 0 0 0').split())
        pos = tuple(p + r for p, r in zip(pos, quat_rotate(quat, local_pos)))
        quat = quat_mul(quat, local_quat)
        for joint in body.findall('joint'):
            if joint.get('type', 'hinge') == 'hinge':
                axis = tuple(float(v) for v in joint.get('axis', '0 0 1').split())
                quat = quat_mul(quat, axis_angle(axis, joint_positions.get(joint.get('name'), 0.0)))
        for geom in body.findall('geom'):
            if geom.get('type') == 'mesh':
                for v in stl_vertices(mesh_files[geom.get('mesh')]):
                    lowest = min(lowest, pos[2] + quat_rotate(quat, v)[2])
        for child in body.findall('body'):
            visit(child, pos, quat)

    base = worldbody.find('body')
    base_frame = dict(base.attrib)
    base.attrib.pop('pos', None)
    base.attrib.pop('quat', None)
    visit(base, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    base.attrib.update(base_frame)
    return lowest


def main():
    mujoco_dir = find_mujoco()
    urdf_xml = subprocess.run(['xacro', str(SOURCE)], check=True, capture_output=True,
                              text=True).stdout
    urdf, body_joints = prepare_urdf(urdf_xml)
    tree = compile_urdf(urdf, mujoco_dir)
    root = tree.getroot()
    root.set('model', 'ti5_170a')

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

    # mesh 改成相對 meshdir 的路徑，安裝後與原始碼樹都能載入
    mesh_files = {}
    for mesh in root.find('asset').findall('mesh'):
        path = Path(mesh.get('file'))
        mesh_files[mesh.get('name')] = path
        mesh.set('file', os.path.relpath(path, PKG / 'meshes'))

    # 只保留碰撞 geom（visual 為 contype=0 / conaffinity=0 / group=1 的同一份 mesh）
    for body in root.iter('body'):
        for geom in body.findall('geom'):
            if geom.get('group') == '1' and geom.get('contype') == '0':
                body.remove(geom)

    armature = load_armature()
    ranges = {}
    for joint in root.iter('joint'):
        name = joint.get('name')
        if joint.get('type') == 'free':
            continue
        if name not in armature:
            raise SystemExit(f"joint '{name}' is missing in {ARMATURE_TABLE}")
        joint.set('armature', armature[name])
        ranges[name] = joint.attrib.pop('actuatorfrcrange')
    if sorted(ranges) != sorted(body_joints):
        raise SystemExit('compiled joints do not match the URDF body joints')

    worldbody = root.find('worldbody')
    base = worldbody.find('body')
    if base.get('name') != 'base_link' or base.find('joint').get('type') != 'free':
        raise SystemExit('expected base_link with a free joint as the root body')
    free = base.find('joint')
    base.insert(list(base).index(free) + 1, ET.Element('site', name='imu', pos=IMU_POS,
                                                         size='0.01'))

    initial = yaml.safe_load(INITIAL_POSITIONS.read_text())['initial_positions']
    base_height = round(GROUND_CLEARANCE - lowest_point(worldbody, mesh_files, initial), 4)
    base.set('pos', f'0 0 {base_height:g}')

    worldbody.insert(0, ET.Element('light', name='sun', pos='0 0 5', dir='0 0 -1',
                                   directional='true'))
    worldbody.insert(1, ET.Element('geom', name='floor', type='plane', size='0 0 0.05',
                                   rgba='0.8 0.8 0.8 1', contype='0', conaffinity='1',
                                   friction='1.0 0.005 0.0001'))

    actuator = ET.SubElement(root, 'actuator')
    joints_in_order = [j.get('name') for j in root.iter('joint') if j.get('type') != 'free']
    for name in joints_in_order:
        ET.SubElement(actuator, 'motor', name=name, joint=name, ctrlrange=ranges[name])

    sensor = ET.SubElement(root, 'sensor')
    ET.SubElement(sensor, 'framequat', name='imu_quat', objtype='site', objname='imu')
    ET.SubElement(sensor, 'gyro', name='gyro', site='imu')
    ET.SubElement(sensor, 'accelerometer', name='imu_acc', site='imu')

    # qpos：free joint（7）+ 依 body 樹順序的關節
    qpos = [0.0, 0.0, base_height, 1.0, 0.0, 0.0, 0.0]
    qpos += [float(initial.get(name, 0.0)) for name in joints_in_order]
    keyframe = ET.SubElement(root, 'keyframe')
    ET.SubElement(keyframe, 'key', name='home', qpos=' '.join(f'{v:g}' for v in qpos))

    ET.indent(tree, space='  ')
    OUTPUT.parent.mkdir(exist_ok=True)
    header = (f'<!-- 由 scripts/make_mjcf.py 從 {SOURCE.relative_to(PKG)} 產生，'
              '不要手動修改 -->\n')
    OUTPUT.write_text(header + ET.tostring(root, encoding='unicode') + '\n', encoding='utf-8')
    print(f'wrote {OUTPUT} ({len(joints_in_order)} joints, base height {base_height:g} m)')


if __name__ == '__main__':
    sys.exit(main())
