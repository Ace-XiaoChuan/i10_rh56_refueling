#!/usr/bin/env python3
"""Regenerate inspection MJCF from the workspace URDFs; requires .venv."""
import copy
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import trimesh
from i10_collision_rules import add_adjacent_exclusions

ROOT = Path(__file__).resolve().parents[1]
ARM = ROOT / 'assets/robots/aubo_i10'
CAP = ROOT / 'assets/objects/fuel_cap'
SCENE = ROOT / 'assets/scenes/i10_fuel_cap.xml'
FRAMES = ('cap_grip_center', 'cap_grip_top_center', 'cap_bottom_center')


def write_xml(tree, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree)
    ET.ElementTree(tree).write(path, encoding='utf-8', xml_declaration=True)


def convert(source, output):
    robot = ET.parse(source).getroot()
    for link in robot.findall('link'):
        for index, collision in enumerate(link.findall('collision')):
            collision.set('name', f"{link.get('name')}_collision_{index}")
    extension = ET.SubElement(robot, 'mujoco')
    ET.SubElement(extension, 'compiler', discardvisual='false', fusestatic='false',
                  strippath='false', inertiafromgeom='auto', inertiagrouprange='0 0')
    for mesh in robot.iter('mesh'):
        name = mesh.get('filename')
        prefix = 'package://aubo_description/'
        if not name.startswith(prefix):
            raise ValueError(name)
        path = ARM / name[len(prefix):]
        if path.suffix.lower() == '.dae':
            dest = ARM / 'mjcf/meshes' / f'visual_{path.stem}.obj'
            dest.parent.mkdir(parents=True, exist_ok=True)
            # Bake Collada scene-node transforms into vertices. URDF supplies color.
            geometry = trimesh.load(path, force='scene').to_mesh()
            geometry.export(dest, include_texture=False)
            path = dest
        mesh.set('filename', str(path))
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        urdf = Path(tmp) / 'model.urdf'
        write_xml(robot, urdf)
        model = mujoco.MjModel.from_xml_path(str(urdf))
        canonical = Path(tmp) / 'model.xml'
        mujoco.mj_saveLastXML(str(canonical), model)
        result = ET.parse(canonical).getroot()
    for mesh in result.findall('./asset/mesh'):
        mesh.set('file', os.path.relpath(mesh.get('file'), output.parent))
    # Collision geoms stay in group 0; visual geoms in group 1.
    # The cap has no measured inertia: importer infers it from collision geometry
    # at MuJoCo's default density (1000 kg/m^3), solely for compilation/inspection.
    for frame in FRAMES:
        body = result.find(f".//body[@name='{frame}']")
        if body is not None:
            ET.SubElement(body, 'site', name=frame, size='0.001', rgba='1 0.5 0 1')
    if source == ARM / 'urdf/aubo_i10_JY.urdf':
        add_adjacent_exclusions(result)
    write_xml(result, output)
    return result


def main():
    arm_path = ARM / 'mjcf/aubo_i10.xml'
    cap_path = CAP / 'mjcf/fuel_cap.xml'
    arm = convert(ARM / 'urdf/aubo_i10_JY.urdf', arm_path)
    cap = convert(CAP / 'urdf/fuel_cap.urdf', cap_path)
    scene = ET.Element('mujoco', model='i10_fuel_cap_inspection')
    ET.SubElement(scene, 'compiler', angle='radian')
    asset = ET.SubElement(scene, 'asset')
    world = ET.SubElement(scene, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -1 3', dir='0 0 -1')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='2 2 .01',
                  rgba='.25 .28 .32 1', contype='0', conaffinity='0', group='1')
    for model, path in [(arm, arm_path), (cap, cap_path)]:
        for element in model.findall('./asset/*'):
            element = copy.deepcopy(element)
            if 'file' in element.attrib:
                file = (path.parent / element.get('file')).resolve()
                element.set('file', os.path.relpath(file, SCENE.parent))
            asset.append(element)
        destination = world
        if path == cap_path:
            destination = ET.SubElement(world, 'body', name='inspection_cap_mount',
                                        pos='.65 -.35 .55')
        destination.extend(copy.deepcopy(list(model.find('worldbody'))))
    add_adjacent_exclusions(scene)
    write_xml(scene, SCENE)
    for path in (arm_path, cap_path, SCENE):
        model = mujoco.MjModel.from_xml_path(str(path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        print(f'{path.relative_to(ROOT)}: {model.nq} joints, {model.ngeom} geoms, {model.nsite} sites')


if __name__ == '__main__':
    main()
