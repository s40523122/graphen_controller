#!/usr/bin/env python3
"""由本套件的 urdf/graphen_aiterone_ny.urdf.xacro 產生 graphen_mujoco/MujocoSystem 用的 mjcf/graphen_aiterone_ny.xml。

改寫自 graphen_ws 的 ti5_170a_graphen/scripts/make_mjcf.py，流程相同：
xacro 展開 → 調整 URDF → MuJoCo 的 compile（URDF → MJCF）→ 補上模擬與 ros2_control 需要的設定。
  - 靈巧手的關節改成固定（模擬中沒有致動器，不固定會自由晃動）
  - 加上 floating 基座（MJCF 的 free joint，名稱 floating_base_joint）
  - timestep 0.005、implicitfast（與 ti5_t170 相同）
  - 各關節 armature 取自 config/ti5_t170_param.csv（複製自 ti5_t170_description/param.csv，同型馬達；推測值）
  - 位置伺服致動器（general、affine bias，與 fb70 相同），forcerange 取自 URDF 的 effort limit：
    hardware:=mujoco 直接當位置伺服（近似實機位置模式的驅動器三環，偏硬）；
    hardware:=mujoco_pd 時 MujocoSystem 會把它改成力矩馬達（gain 1、無 bias），forcerange 仍然有效
  - 伺服增益以頻寬決定：kp = I·ω²、kv = 2ζ·I·ω，I 為 home 姿態下質量矩陣的對角項（含 armature），
    ω = 2π·--servo-bandwidth；kv 由 implicitfast 隱式積分，kp 為顯式，ω·dt 要遠小於 1
    （預設 10 Hz → 0.31）。只是模擬的近似，最好以實機位置模式的階躍響應校正
  - 碰撞只算機器人與地面
  - 顯示與碰撞分成兩組 geom（MuJoCo 對應 URDF visual / collision 的慣用寫法）：
    class visual（group 2，contype / conaffinity 0、density 0，只顯示）為 URDF visual 的原始網格，
    class collision（group 3，viewer 預設隱藏，按 3 顯示）為 URDF collision 的凸包。
    靈巧手的 visual 沿用碰撞網格（原始網格在別的套件、約 70 MB；要原始外觀用 --hand-mesh-type original）；
    --visual-mesh hull 時本體也沿用碰撞網格（不載入原始網格，啟動較快）
  - base_link 上 (0, 0, 0.1) 的 imu site 與 framequat / gyro / accelerometer（MujocoSystem 依類型與名稱尋找）
  - keyframe home：initial_positions.yaml 的姿態；基座高度由腳底碰撞 mesh 的最低點自動計算（留 5 mm 落地）
與原腳本的差異：
  - 靈巧手的網格在 dexterous_hand_description；MuJoCo 不認得 package://，只能用相對 meshdir
    或絕對路徑引用檔案。預設把手部碰撞凸包產生在本套件的 meshes/hulls/dexterous_hand/，
    MJCF 只依賴本套件；可用 --hand-mesh-dir / --hand-mesh-type 指定其他位置或改用原始網格
  - URDF 的 visual 由本腳本自己加入 MJCF（MuJoCo 以檔名命名 mesh asset，visual 原檔與同名的碰撞凸包
    會被合併成同一個 asset），asset 名稱為 <碰撞 mesh>_visual

本體碰撞已是 meshes/hulls/ 的凸包；網格更新後先執行 scripts/make_hulls.py 再執行本腳本。
需要 source ROS 環境（展開 xacro、尋找 dexterous_hand_description），以及 MuJoCo 3.11.0
（MUJOCO_DIR，或 ~/.local、/opt 下的 mujoco-3.11.0，與 graphen_mujoco 相同）：
  python3 scripts/make_mjcf.py
  python3 scripts/make_mjcf.py --hand-mesh-type original            # 直接引用 dexterous_hand_description 的原始網格
  python3 scripts/make_mjcf.py --hand-mesh-dir /path/to/hand_hulls   # 凸包產生到指定位置（絕對路徑）
  python3 scripts/make_mjcf.py --visual-mesh hull                     # 外觀也用凸包（較快、較粗糙）
執行後把印出的基座高度填到 graphen_bringup/launch/bringup.launch.py 的 robots['aiterone_ny'].base_height。
"""
import argparse
import ctypes
import math
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_hulls import convex_hull, load_vertices, write_binary_stl  # noqa: E402

PKG = Path(__file__).resolve().parent.parent
PKG_NAME = 'graphen_aiterone_ny_description'
SOURCE = PKG / 'urdf' / 'graphen_aiterone_ny.urdf.xacro'
OUTPUT = PKG / 'mjcf' / 'graphen_aiterone_ny.xml'
MESH_DIR = PKG / 'meshes'
HAND_PKG = 'dexterous_hand_description'
HAND_HULL_DIR = MESH_DIR / 'hulls' / 'dexterous_hand'
DEFAULT_INITIAL_POSITIONS = PKG / 'config' / 'initial_positions.yaml'
ARMATURE_TABLE = PKG / 'config' / 'ti5_t170_param.csv'
ARMATURE_RENAME = {'NECK_Y1': 'NECK_Y'}
MUJOCO_VERSION = '3.11.0'
# 與 ti5_170a_graphen.urdf.xacro 的 imu_xyz 預設相同
IMU_POS = '0 0 0.1'
# 腳底最低點與地面的間隙（m），讓模型從略高處落地，避免初始穿透
GROUND_CLEARANCE = 0.005
# 位置伺服的預設頻寬（Hz）與阻尼比；ω·dt（timestep 0.005）超過約 0.5 時顯式的 kp 項容易不穩
DEFAULT_SERVO_BANDWIDTH = 10.0
DEFAULT_SERVO_DAMPING_RATIO = 1.0
TIMESTEP = 0.005
# 沒有 material 顏色的 visual 與碰撞 geom（group 3，顯示時半透明以便和外觀疊在一起看）的顏色
DEFAULT_VISUAL_RGBA = '0.7 0.7 0.7 1'
COLLISION_RGBA = '0.9 0.45 0.2 0.5'


def find_mujoco():
    candidates = [os.environ.get('MUJOCO_DIR')] + [
        str(Path(base) / f'mujoco-{MUJOCO_VERSION}') for base in (Path.home() / '.local', '/opt')]
    for candidate in candidates:
        if candidate and (Path(candidate) / 'bin' / 'compile').exists():
            return Path(candidate)
    raise SystemExit(f'MuJoCo {MUJOCO_VERSION} not found (set MUJOCO_DIR)')


def joint_inertia(mjcf_xml, output_dir, mujoco_dir, nv):
    """編譯 mjcf_xml，回傳 home keyframe 姿態下質量矩陣的對角項（長度 nv，含 armature）。

    沒有 MuJoCo 的 Python 套件時也能用：直接以 ctypes 呼叫 libmujoco，並以 mj_mulM 乘上單位向量
    取得對角項，不依賴 mjModel / mjData 的結構佈局。暫存檔放在輸出目錄，讓相對的 meshdir 有效。
    """
    lib = ctypes.CDLL(str(mujoco_dir / 'lib' / 'libmujoco.so'))
    lib.mj_loadXML.restype = ctypes.c_void_p
    lib.mj_loadXML.argtypes = [ctypes.c_char_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    lib.mj_makeData.restype = ctypes.c_void_p
    lib.mj_makeData.argtypes = [ctypes.c_void_p]
    lib.mj_resetDataKeyframe.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
    lib.mj_forward.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    vec_t = ctypes.c_double * nv
    lib.mj_mulM.argtypes = [ctypes.c_void_p, ctypes.c_void_p, vec_t, vec_t]
    lib.mj_deleteData.argtypes = [ctypes.c_void_p]
    lib.mj_deleteModel.argtypes = [ctypes.c_void_p]

    output_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile('w', suffix='.xml', dir=output_dir, delete=False,
                                     encoding='utf-8') as tmp:
        tmp.write(mjcf_xml)
    try:
        error = ctypes.create_string_buffer(1000)
        model = lib.mj_loadXML(tmp.name.encode(), None, error, len(error))
        if not model:
            raise SystemExit(f'failed to load the generated MJCF: {error.value.decode()}')
        data = lib.mj_makeData(model)
        lib.mj_resetDataKeyframe(model, data, 0)
        lib.mj_forward(model, data)
        diagonal = []
        for i in range(nv):
            unit, res = vec_t(), vec_t()
            unit[i] = 1.0
            lib.mj_mulM(model, data, res, unit)
            diagonal.append(res[i])
        lib.mj_deleteData(data)
        lib.mj_deleteModel(model)
        return diagonal
    finally:
        os.unlink(tmp.name)


def load_armature():
    rows = ARMATURE_TABLE.read_text(encoding='utf-8').splitlines()[1:]
    table = {}
    for row in rows:
        if row.strip():
            name, armature = row.split(',')[:2]
            table[ARMATURE_RENAME.get(name, name)] = armature
    return table


def hand_source_dir():
    """dexterous_hand_description 的 meshes/（手部原始網格）。"""
    return Path(get_package_share_directory(HAND_PKG)) / 'meshes'


def hand_mesh(relative, hand_dir, hand_type):
    """回傳 MJCF 要引用的手部碰撞網格路徑。

    relative：相對 dexterous_hand_description/meshes/ 的路徑（例如 right/right_hand_base.STL）。
    hand_type 為 hull 時，在 hand_dir 產生凸包（來源較新時重新產生）；
    為 original 時直接引用 hand_dir 下的同名檔案，不產生任何檔案。
    """
    target = hand_dir / relative
    if hand_type == 'original':
        if not target.exists():
            raise SystemExit(f'hand mesh not found: {target}')
        return target
    source = hand_source_dir() / relative
    if not target.exists() or target.stat().st_mtime < source.stat().st_mtime:
        target.parent.mkdir(parents=True, exist_ok=True)
        write_binary_stl(target, *convex_hull(load_vertices(source)))
    return target


def resolve_mesh(filename, hand_dir, hand_type):
    """碰撞 mesh 的 package:// 路徑改成實際檔案路徑。"""
    own = f'package://{PKG_NAME}/'
    hand = f'package://{HAND_PKG}/meshes/'
    if filename.startswith(own):
        return str(PKG / filename[len(own):])
    if filename.startswith(hand):
        return str(hand_mesh(filename[len(hand):], hand_dir, hand_type))
    raise SystemExit(f'unsupported mesh path: {filename}')


def mjcf_mesh_path(path):
    """MJCF 中的 mesh 路徑：本套件 meshes/ 內用相對 meshdir 的路徑（原始碼與安裝後都能載入），
    其他位置用絕對路徑（跨套件的相對路徑在原始碼樹與 install 下不同）。"""
    path = path.resolve()
    if MESH_DIR.resolve() in path.parents:
        return os.path.relpath(path, MESH_DIR.resolve())
    return str(path)


def visual_meshes(root):
    """URDF 各 link 的 visual：link 名稱 → (網格的 package:// 路徑, scale, rgba)。

    只支援每個 link 一個 mesh visual。scale 為 None 表示沒有縮放（部分 STL 以 mm 為單位，靠 scale 0.001）。
    """
    colors = {m.get('name'): m.find('color').get('rgba') for m in root.findall('material')
              if m.find('color') is not None}
    visuals = {}
    for link in root.findall('link'):
        found = link.findall('visual')
        if not found:
            continue
        if len(found) > 1 or found[0].find('geometry/mesh') is None:
            raise SystemExit(f"link '{link.get('name')}': expected one mesh visual")
        visual = found[0]
        collision = link.find('collision')
        # visual 直接沿用碰撞 geom 的位置與姿態，兩者的 origin 必須相同
        origin = [(o.get('xyz'), o.get('rpy')) if o is not None else None
                  for o in (visual.find('origin'), collision.find('origin') if collision is not None
                            else None)]
        if collision is None or origin[0] != origin[1]:
            raise SystemExit(f"link '{link.get('name')}': visual origin differs from collision")
        material = visual.find('material')
        rgba = DEFAULT_VISUAL_RGBA
        if material is not None:
            color = material.find('color')
            rgba = color.get('rgba') if color is not None else colors.get(material.get('name'), rgba)
        mesh = visual.find('geometry/mesh')
        visuals[link.get('name')] = (mesh.get('filename'), mesh.get('scale'), rgba)
    return visuals


def prepare_urdf(urdf_xml, hand_dir, hand_type):
    """手的關節改固定、加上 floating 基座、mesh 改成絕對路徑。

    回傳 (URDF 字串, 本體關節名稱, visual_meshes() 的結果)。
    """
    root = ET.fromstring(urdf_xml)
    visuals = visual_meshes(root)
    body_joints = []
    for joint in root.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        if joint.get('name').startswith(('left_', 'right_')):
            joint.set('type', 'fixed')
            for tag in ('mimic', 'limit', 'axis'):
                for child in joint.findall(tag):
                    joint.remove(child)
        else:
            body_joints.append(joint.get('name'))
    for link in root.findall('link'):
        # MJCF 只保留碰撞；先移除 visual，避免 MuJoCo 以檔名命名 mesh 時，
        # 同名的 visual 原檔與碰撞凸包（hulls/ 下）被合併成同一個 asset
        for visual in link.findall('visual'):
            link.remove(visual)
        for mesh in link.findall('collision/geometry/mesh'):
            mesh.set('filename', resolve_mesh(mesh.get('filename'), hand_dir, hand_type))
    # MuJoCo 的 URDF 解析器把 floating joint 轉成 free joint；world link 對應 worldbody
    ET.SubElement(root, 'link', name='world')
    floating = ET.SubElement(root, 'joint', name='floating_base_joint', type='floating')
    ET.SubElement(floating, 'parent', link='world')
    ET.SubElement(floating, 'child', link='base_link')
    mujoco = ET.SubElement(root, 'mujoco')
    ET.SubElement(mujoco, 'compiler', discardvisual='false', fusestatic='false', strippath='false')
    return ET.tostring(root, encoding='unicode'), body_joints, visuals


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


def lowest_point(worldbody, meshes, joint_positions):
    """base_link 放在原點時，所有碰撞 mesh 頂點的最低 z。meshes：name → (路徑, scale)。"""
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
            if geom.get('type') != 'mesh':
                continue
            path, scale = meshes[geom.get('mesh')]
            gpos = tuple(float(v) for v in geom.get('pos', '0 0 0').split())
            gquat = tuple(float(v) for v in geom.get('quat', '1 0 0 0').split())
            for v in load_vertices(path):
                local = quat_rotate(gquat, tuple(c * s for c, s in zip(v, scale)))
                world = quat_rotate(quat, tuple(a + b for a, b in zip(gpos, local)))
                lowest = min(lowest, pos[2] + world[2])
        for child in body.findall('body'):
            visit(child, pos, quat)

    base = worldbody.find('body')
    base_frame = dict(base.attrib)
    base.attrib.pop('pos', None)
    base.attrib.pop('quat', None)
    visit(base, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    base.attrib.update(base_frame)
    return lowest


def add_visual_geoms(root, asset, visuals, visual_mesh):
    """每個碰撞 mesh geom 加上 class visual 的顯示 geom（同一個 body、同樣的位置與姿態），
    碰撞 geom 改成 class collision。本體的 visual 依 visual_mesh 用原始網格或沿用碰撞網格，靈巧手一律沿用。"""
    own = f'package://{PKG_NAME}/'
    mesh_files = {mesh.get('name'): mesh for mesh in asset.findall('mesh')}
    for body in root.iter('body'):
        for geom in body.findall('geom'):
            if geom.get('type') != 'mesh':
                continue
            if body.get('name') not in visuals:
                raise SystemExit(f"body '{body.get('name')}' has a collision mesh but no visual")
            filename, scale, rgba = visuals[body.get('name')]
            mesh_name = geom.get('mesh')
            if visual_mesh == 'original' and filename.startswith(own):
                visual_name = f'{mesh_name}_visual'
                if visual_name in mesh_files:
                    raise SystemExit(f"mesh asset name '{visual_name}' already exists")
                source = PKG / filename[len(own):]
                if not source.exists():
                    raise SystemExit(f'visual mesh not found: {source}')
                attrs = {'name': visual_name, 'content_type': 'model/stl',
                         'file': mjcf_mesh_path(source)}
                if scale is not None:
                    attrs['scale'] = scale
                mesh_files[visual_name] = ET.SubElement(asset, 'mesh', attrs)
            else:
                visual_name = mesh_name
            attrs = {'class': 'visual', 'type': 'mesh', 'mesh': visual_name, 'rgba': rgba}
            for key in ('pos', 'quat'):
                if geom.get(key) is not None:
                    attrs[key] = geom.get(key)
            body.insert(list(body).index(geom), ET.Element('geom', attrs))
            geom.set('class', 'collision')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--initial-positions', type=Path, default=DEFAULT_INITIAL_POSITIONS,
                        help='YAML with initial_positions used for the home keyframe')
    parser.add_argument('--hand-mesh-type', choices=('hull', 'original'), default='hull',
                        help='hull: generate convex hulls of the hand meshes into --hand-mesh-dir; '
                             'original: reference the hand meshes in --hand-mesh-dir as is')
    parser.add_argument('--hand-mesh-dir', type=Path, default=None,
                        help='directory of the hand collision meshes (left/, right/) referenced by '
                             'the MJCF. Default: meshes/hulls/dexterous_hand for hull, '
                             f'{HAND_PKG}/meshes for original. Paths outside this package\'s '
                             'meshes/ are written as absolute paths')
    parser.add_argument('--visual-mesh', choices=('original', 'hull'), default='original',
                        help='original: body visual geoms use the original meshes of this package; '
                             'hull: visual geoms reuse the collision hulls (faster to load). '
                             'Hand visual geoms always reuse the hand collision meshes')
    parser.add_argument('--servo-bandwidth', type=float, default=DEFAULT_SERVO_BANDWIDTH,
                        help='position servo bandwidth in Hz (kp = I*w^2, w = 2*pi*bandwidth)')
    parser.add_argument('--servo-damping-ratio', type=float, default=DEFAULT_SERVO_DAMPING_RATIO,
                        help='position servo damping ratio (kv = 2*zeta*I*w)')
    parser.add_argument('--output', type=Path, default=OUTPUT,
                        help='output MJCF; inside this package meshdir is relative, otherwise absolute')
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    omega = 2.0 * math.pi * args.servo_bandwidth
    if args.servo_bandwidth <= 0.0 or args.servo_damping_ratio <= 0.0:
        raise SystemExit('--servo-bandwidth and --servo-damping-ratio must be positive')
    if omega * TIMESTEP > 0.5:
        print(f'warning: servo bandwidth {args.servo_bandwidth:g} Hz gives w*dt = '
              f'{omega * TIMESTEP:.2f} at timestep {TIMESTEP:g}; the explicit kp term may be unstable')

    if args.hand_mesh_dir is None:
        hand_dir = HAND_HULL_DIR if args.hand_mesh_type == 'hull' else hand_source_dir()
    else:
        hand_dir = args.hand_mesh_dir.expanduser().resolve()
    if MESH_DIR.resolve() not in hand_dir.resolve().parents:
        print(f'note: hand meshes in {hand_dir} are outside {MESH_DIR}; '
              'the MJCF will use absolute paths for them (not portable)')

    mujoco_dir = find_mujoco()
    urdf_xml = subprocess.run(['xacro', str(SOURCE)], check=True, capture_output=True,
                              text=True).stdout
    urdf, body_joints, visuals = prepare_urdf(urdf_xml, hand_dir, args.hand_mesh_type)
    tree = compile_urdf(urdf, mujoco_dir)
    root = tree.getroot()
    root.set('model', 'graphen_aiterone_ny')

    compiler = root.find('compiler')
    # 輸出在本套件內時 meshdir 用相對路徑（原始碼與安裝後都能載入），否則用絕對路徑
    if PKG.resolve() in output.parents:
        compiler.set('meshdir', os.path.relpath(MESH_DIR.resolve(), output.parent))
    else:
        compiler.set('meshdir', str(MESH_DIR.resolve()))
    compiler.set('autolimits', 'true')
    option = ET.Element('option', timestep=f'{TIMESTEP:g}', integrator='implicitfast')
    root.insert(list(root).index(compiler) + 1, option)
    # 機器人的 geom 只和地面碰撞：robot contype=1 / conaffinity=0，地面 contype=0 / conaffinity=1
    default = ET.Element('default')
    ET.SubElement(default, 'geom', contype='1', conaffinity='0', condim='3',
                  friction='1.0 0.005 0.0001')
    # 顯示用：不碰撞、不貢獻質量（質量與慣量來自 URDF 的 inertial）；碰撞用：放在 viewer 預設隱藏的 group 3
    visual_class = ET.SubElement(default, 'default', {'class': 'visual'})
    ET.SubElement(visual_class, 'geom', contype='0', conaffinity='0', group='2', density='0')
    collision_class = ET.SubElement(default, 'default', {'class': 'collision'})
    ET.SubElement(collision_class, 'geom', group='3', rgba=COLLISION_RGBA)
    root.insert(list(root).index(option) + 1, default)

    # 只保留碰撞 geom（visual 為 contype=0 / conaffinity=0 / group=1）
    for body in root.iter('body'):
        for geom in body.findall('geom'):
            if geom.get('group') == '1' and geom.get('contype') == '0':
                body.remove(geom)

    # 移除只被 visual 使用的 mesh；路徑規則見 mjcf_mesh_path()
    used = {g.get('mesh') for g in root.iter('geom') if g.get('type') == 'mesh'}
    asset = root.find('asset')
    meshes = {}
    for mesh in list(asset.findall('mesh')):
        if mesh.get('name') not in used:
            asset.remove(mesh)
            continue
        path = Path(mesh.get('file'))
        scale = tuple(float(v) for v in mesh.get('scale', '1 1 1').split())
        meshes[mesh.get('name')] = (path, scale)
        mesh.set('file', mjcf_mesh_path(path))

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

    initial = yaml.safe_load(args.initial_positions.read_text())['initial_positions']
    base_height = round(GROUND_CLEARANCE - lowest_point(worldbody, meshes, initial), 4)
    base.set('pos', f'0 0 {base_height:g}')

    add_visual_geoms(root, asset, visuals, args.visual_mesh)

    worldbody.insert(0, ET.Element('light', name='sun', pos='0 0 5', dir='0 0 -1',
                                   directional='true'))
    worldbody.insert(1, ET.Element('geom', name='floor', type='plane', size='0 0 0.05',
                                   rgba='0.8 0.8 0.8 1', contype='0', conaffinity='1',
                                   friction='1.0 0.005 0.0001'))

    actuator = ET.SubElement(root, 'actuator')
    joints_in_order = [j.get('name') for j in root.iter('joint') if j.get('type') != 'free']

    sensor = ET.SubElement(root, 'sensor')
    ET.SubElement(sensor, 'framequat', name='imu_quat', objtype='site', objname='imu')
    ET.SubElement(sensor, 'gyro', name='gyro', site='imu')
    ET.SubElement(sensor, 'accelerometer', name='imu_acc', site='imu')

    # qpos：free joint（7）+ 依 body 樹順序的關節；dof 同理（free joint 6 + 每個 hinge 1）
    qpos = [0.0, 0.0, base_height, 1.0, 0.0, 0.0, 0.0]
    qpos += [float(initial.get(name, 0.0)) for name in joints_in_order]
    keyframe = ET.SubElement(root, 'keyframe')
    ET.SubElement(keyframe, 'key', name='home', qpos=' '.join(f'{v:g}' for v in qpos))

    # 先以沒有致動器的模型算出 home 姿態的關節慣量，再依頻寬填入伺服增益
    inertia = joint_inertia(ET.tostring(root, encoding='unicode'), output.parent, mujoco_dir,
                            6 + len(joints_in_order))[6:]
    for name, inertia_ii in zip(joints_in_order, inertia):
        kp = inertia_ii * omega ** 2
        kv = 2.0 * args.servo_damping_ratio * inertia_ii * omega
        ET.SubElement(actuator, 'general', name=name, joint=name, forcerange=ranges[name],
                      biastype='affine', gainprm=f'{kp:.6g}', biasprm=f'0 {-kp:.6g} {-kv:.6g}')

    ET.indent(tree, space='  ')
    output.parent.mkdir(parents=True, exist_ok=True)
    header = (f'<!-- 由 scripts/make_mjcf.py 從 {SOURCE.relative_to(PKG)} 產生，不要手動修改；'
              f'伺服頻寬 {args.servo_bandwidth:g} Hz、阻尼比 {args.servo_damping_ratio:g} -->\n')
    output.write_text(header + ET.tostring(root, encoding='unicode') + '\n', encoding='utf-8')
    print(f'wrote {output} ({len(joints_in_order)} joints, base height {base_height:g} m, '
          f'servo {args.servo_bandwidth:g} Hz / zeta {args.servo_damping_ratio:g})')
    for name, inertia_ii in zip(joints_in_order, inertia):
        print(f'  {name:16s} I {inertia_ii:8.4f}  kp {inertia_ii * omega ** 2:9.2f}  '
              f'kv {2.0 * args.servo_damping_ratio * inertia_ii * omega:7.2f}')


if __name__ == '__main__':
    sys.exit(main())
