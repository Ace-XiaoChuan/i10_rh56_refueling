#!/usr/bin/env python3
"""Export supplied sensor STL in meters, with fixed mounting frames."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'assets/robots/sensor'


def main():
    mesh = trimesh.load_mesh(DEST / 'source/传感器.STL')
    if not np.allclose(mesh.bounds, [[0, 0, 0], [75, 75, 33.5]], atol=1e-4):
        raise ValueError('Sensor source bounds changed; recheck STEP/STL coordinates')
    mesh.apply_translation([-37.5, -37.5, 0])
    mesh.apply_scale(.001)
    (DEST / 'meshes').mkdir(parents=True, exist_ok=True)
    (DEST / 'urdf').mkdir(exist_ok=True)
    mesh.export(DEST / 'meshes/sensor.stl')
    robot = ET.Element('robot', name='sensor')
    robot.append(ET.Comment(' Visual inspection only. Meters; bottom center origin, '
                            '+Z toward adapter. Provisional mounting direction. '
                            'No calibrated mass, inertia, collision or sensing model. '))
    extension = ET.SubElement(robot, 'mujoco')
    ET.SubElement(extension, 'compiler', strippath='false', discardvisual='false',
                  fusestatic='false')
    body = ET.SubElement(robot, 'link', name='sensor_link')
    visual = ET.SubElement(body, 'visual', name='sensor_visual')
    geometry = ET.SubElement(visual, 'geometry')
    ET.SubElement(geometry, 'mesh', filename='../meshes/sensor.stl')
    material = ET.SubElement(visual, 'material', name='sensor_silver')
    ET.SubElement(material, 'color', rgba='.65 .70 .76 1')
    ET.SubElement(robot, 'link', name='sensor_output_frame')
    joint = ET.SubElement(robot, 'joint', name='sensor_output_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link='sensor_link')
    ET.SubElement(joint, 'child', link='sensor_output_frame')
    ET.SubElement(joint, 'origin', xyz='0 0 .0335', rpy='0 0 0')
    ET.indent(robot)
    path = DEST / 'urdf/sensor.urdf'
    ET.ElementTree(robot).write(path, encoding='utf-8', xml_declaration=True)
    print(f'{path}: {len(mesh.faces)} triangles; bounds (m): {mesh.bounds.tolist()}')


if __name__ == '__main__':
    main()
