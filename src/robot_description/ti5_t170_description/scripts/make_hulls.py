#!/usr/bin/env python3
"""把 meshes/*.STL 各自取凸包，輸出到 meshes/hulls/ 的同名檔案（binary STL），作為 URDF 的碰撞模型。

凸包與原檔使用相同的座標與單位，所以 URDF 中原本搭配該 mesh 的 origin / scale 可以直接沿用。
原始 mesh 更新後重新執行（需要 numpy、scipy）：
  python3 scripts/make_hulls.py
"""
import argparse
import struct
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

PKG = Path(__file__).resolve().parent.parent
MESH_DIR = PKG / 'meshes'
HULL_DIR = MESH_DIR / 'hulls'

STL_HEADER_SIZE = 80
# binary STL 的每個三角形：法向量 + 三個頂點（12 個 float32）+ 2 bytes 屬性
STL_TRIANGLE = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])


def load_vertices(path):
    """讀取 STL（binary 或 ASCII）的所有頂點，回傳 (N, 3) 陣列。"""
    data = path.read_bytes()
    if len(data) >= STL_HEADER_SIZE + 4:
        count = struct.unpack_from('<I', data, STL_HEADER_SIZE)[0]
        # 有些 binary STL 的 header 也以 "solid" 開頭，所以用檔案大小判斷而不是看開頭
        if len(data) == STL_HEADER_SIZE + 4 + count * STL_TRIANGLE.itemsize:
            triangles = np.frombuffer(data, dtype=STL_TRIANGLE, offset=STL_HEADER_SIZE + 4)
            return triangles['vertices'].reshape(-1, 3).astype(np.float64)
    vertices = [line.split()[1:4] for line in data.decode('ascii', errors='ignore').splitlines()
                if line.lstrip().startswith('vertex')]
    if not vertices:
        raise SystemExit(f'{path.name}: not a valid STL file')
    return np.array(vertices, dtype=np.float64)


def convex_hull(vertices):
    """回傳凸包的三角形 (M, 3, 3) 與朝外的法向量 (M, 3)。"""
    points = np.unique(vertices, axis=0)
    hull = ConvexHull(points)
    triangles = points[hull.simplices]
    normals = hull.equations[:, :3]
    # scipy 的 simplices 頂點順序不固定：依朝外的法向量把繞向統一成逆時針
    winding = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    flipped = np.einsum('ij,ij->i', winding, normals) < 0
    triangles[flipped] = triangles[flipped][:, ::-1]
    return triangles, normals


def write_binary_stl(path, triangles, normals):
    out = np.zeros(len(triangles), dtype=STL_TRIANGLE)
    out['normal'] = normals
    out['vertices'] = triangles
    header = f'convex hull of {path.name}'.encode('ascii').ljust(STL_HEADER_SIZE, b' ')
    with path.open('wb') as f:
        f.write(header)
        f.write(struct.pack('<I', len(out)))
        f.write(out.tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--mesh-dir', type=Path, default=MESH_DIR, help='directory of source STL')
    parser.add_argument('--output-dir', type=Path, default=HULL_DIR, help='directory of hulls')
    args = parser.parse_args()

    sources = sorted(p for p in args.mesh_dir.iterdir() if p.suffix.lower() == '.stl')
    if not sources:
        raise SystemExit(f'no STL files in {args.mesh_dir}')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for source in sources:
        vertices = load_vertices(source)
        triangles, normals = convex_hull(vertices)
        write_binary_stl(args.output_dir / source.name, triangles, normals)
        print(f'{source.name}: {len(vertices) // 3} -> {len(triangles)} triangles')
    print(f'wrote {len(sources)} hulls to {args.output_dir}')


if __name__ == '__main__':
    main()
