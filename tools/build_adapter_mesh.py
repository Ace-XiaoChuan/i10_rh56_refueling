#!/usr/bin/env python3
"""Convert the supplied millimeter STL to the STEP-centered adapter frame."""
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
    mesh.apply_translation([-31.5, -31.5, 0])
    mesh.apply_scale(.001)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(OUTPUT)
    print(f'{OUTPUT}: {len(mesh.faces)} triangles, bounds (m): {mesh.bounds.tolist()}')


if __name__ == '__main__':
    main()
