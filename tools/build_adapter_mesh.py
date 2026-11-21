#!/usr/bin/env python3
"""按目标底盘厚度与总高生成 adapter 米制网格，原始 CAD 保留。"""
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'assets/robots/adapter/meshes/adapter.stl'


def main():
    mesh = trimesh.load_mesh(ROOT / 'assets/robots/adapter/source/连接件.STL')
    # STEP uses the bottom face center; STL has a positive-octant offset.
    # Explicit source-specific transform, not automatic bounding-box centering.
    if not np.allclose(mesh.bounds, [[0, 0, 0], [63, 63, 25]], atol=1e-4):
        raise ValueError('Adapter source bounds changed; recheck the STEP/STL transform')
    # 保持 XY 与孔径不变，沿 Z 分别拉伸底盘和凸台：10→20 mm、15→27 mm。
    z = mesh.vertices[:, 2].copy()
    mesh.vertices[:, 2] = np.where(z <= 10.0, z * 2.0, 20.0 + (z - 10.0) * 1.8)
    mesh.apply_translation([-31.5, -31.5, 0])
    mesh.apply_scale(.001)
    if not np.allclose(mesh.bounds, [[-.0315, -.0315, 0], [.0315, .0315, .047]], atol=1e-7):
        raise ValueError(f'Unexpected adapter target bounds: {mesh.bounds.tolist()}')
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(OUTPUT)
    print(f'{OUTPUT}: {len(mesh.faces)} triangles, bounds (m): {mesh.bounds.tolist()}')


if __name__ == '__main__':
    main()
