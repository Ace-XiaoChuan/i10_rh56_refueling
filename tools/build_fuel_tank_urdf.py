#!/usr/bin/env python3
"""Generate a static tank URDF and convex STL pieces, using only stdlib."""
import argparse
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'assets/objects/fuel_tank'
SEGMENTS = 64
# User-confirmed 2026-09-30 entrance widening; see assembly_contract.md.
RECESS_ENTRANCE_RADIUS_M = .11


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def wedge(index, lower, upper, inner_low, inner_high, outer):
    vertices = []
    for z, inner in ((lower, inner_low), (upper, inner_high)):
        for radius, angle in ((inner, index), (outer, index),
                              (outer, index+1), (inner, index+1)):
            theta = 2 * math.pi * angle / SEGMENTS
            c, s = math.cos(theta), math.sin(theta)
            r = .4 / max(abs(c), abs(s)) if radius is None else radius
            vertices.append((r*c, r*s, z))
    center = tuple(sum(v[k] for v in vertices)/8 for k in range(3))
    triangles = []
    for face in ((0,1,2,3), (4,5,6,7), (0,1,5,4),
                 (1,2,6,5), (2,3,7,6), (3,0,4,7)):
        for ids in ((face[0],face[1],face[2]), (face[0],face[2],face[3])):
            a, b, c = (vertices[i] for i in ids)
            normal = cross(sub(b,a), sub(c,a))
            if sum(x*y for x,y in zip(normal, sub(a,center))) < 0:
                b, c = c, b
            triangles.append((a,b,c))
    return triangles


def write_stl(path, triangles):
    with path.open('wb') as stream:
        stream.write(b'Provisional fuel tank geometry'.ljust(80, b'\0'))
        stream.write(struct.pack('<I', len(triangles)))
        for a,b,c in triangles:
            n = cross(sub(b,a), sub(c,a))
            length = math.sqrt(sum(x*x for x in n))
            n = tuple(x/length for x in n)
            stream.write(struct.pack('<12fH', *n, *a, *b, *c, 0))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recess-depth', type=float, default=.04,
                        help='Provisional recess depth in meters (default: 0.04)')
    args = parser.parse_args()
    if not .003 < args.recess_depth < .4:
        parser.error('recess depth must be between 0.003 and 0.4 m')
    floor = .4 - args.recess_depth
    meshdir = DEST / 'meshes'
    meshdir.mkdir(parents=True, exist_ok=True)
    (DEST / 'urdf').mkdir(exist_ok=True)
    robot = ET.Element('robot', name='fuel_tank')
    robot.append(ET.Comment(
        ' Units: meters. User dimensions: 0.8 x 0.8 x 0.4; recess diameter '
        f'{2 * RECESS_ENTRANCE_RADIUS_M:g} to 0.08; neck OD/ID 0.06/0.05, height 0.003. '
        f'PROVISIONAL recess depth: {args.recess_depth:g}. '
        'Bore continues through bottom as an internal-cavity simplification. '
        'Static fixture; no calibrated mass, inertia or contact parameters. '))
    ET.SubElement(robot, 'link', name='world')
    body = ET.SubElement(robot, 'link', name='fuel_tank_body')
    joint = ET.SubElement(robot, 'joint', name='fuel_tank_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link='world')
    ET.SubElement(joint, 'child', link='fuel_tank_body')
    # Stand the original slab on its edge: opening faces world -Y, bottom Z=0.
    ET.SubElement(joint, 'origin', xyz='0 0.2 0.4', rpy=f'{math.pi / 2:.16g} 0 0')
    groups = (
        ('body', 0, floor, .025, .025, None),
        ('recess', floor, .4, .04, RECESS_ENTRANCE_RADIUS_M, None),
        ('neck', floor, floor+.003, .025, .025, .03),
    )
    visuals = {'tank': [], 'neck': []}
    for name, low, high, rlow, rhigh, outer in groups:
        for i in range(SEGMENTS):
            triangles = wedge(i, low, high, rlow, rhigh, outer)
            filename = f'{name}_{i:02d}.stl'
            write_stl(meshdir / filename, triangles)
            visuals['neck' if name == 'neck' else 'tank'].extend(triangles)
            collision = ET.SubElement(body, 'collision', name=f'{name}_{i:02d}')
            geometry = ET.SubElement(collision, 'geometry')
            ET.SubElement(geometry, 'mesh', filename=f'../meshes/{filename}')
    for name, triangles in visuals.items():
        write_stl(meshdir / f'{name}_visual.stl', triangles)
        visual = ET.SubElement(body, 'visual', name=name)
        geometry = ET.SubElement(visual, 'geometry')
        ET.SubElement(geometry, 'mesh', filename=f'../meshes/{name}_visual.stl')
        material = ET.SubElement(visual, 'material', name=f'{name}_gray')
        ET.SubElement(material, 'color', rgba='0.65 0.68 0.72 1' if name == 'tank' else '0.3 0.32 0.35 1')
    # Outer face center, before the recess: local +Z points out of the tank.
    for name, z in (('tank_opening_center', .4),
                    ('tank_recess_floor', floor), ('tank_filler_top', floor+.003)):
        ET.SubElement(robot, 'link', name=name)
        joint = ET.SubElement(robot, 'joint', name=f'{name}_fixed', type='fixed')
        ET.SubElement(joint, 'parent', link='fuel_tank_body')
        ET.SubElement(joint, 'child', link=name)
        ET.SubElement(joint, 'origin', xyz=f'0 0 {z:.9g}', rpy='0 0 0')
    ET.indent(robot)
    path = DEST / 'urdf/fuel_tank.urdf'
    ET.ElementTree(robot).write(path, encoding='utf-8', xml_declaration=True)
    print(path)


if __name__ == '__main__':
    main()
