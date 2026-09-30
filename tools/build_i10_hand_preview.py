#!/usr/bin/env python3
"""Build the provisional i10 + RH56 right hand + cap inspection scene."""
import copy
import hashlib
import math
import os
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
from fuel_door import add_door
from i10_collision_rules import add_adjacent_exclusions

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'assets/scenes/i10_rh56_fuel_cap_preview.xml'
# Horizontal base-center to opening-face distance, in meters (preview placeholder).
BASE_TO_TANK_FACE = 1.0


def read_tank():
    path = ROOT / 'assets/objects/fuel_tank/urdf/fuel_tank.urdf'
    robot = ET.parse(path).getroot()
    for mesh in robot.iter('mesh'):
        mesh.set('filename', str((path.parent / mesh.get('filename')).resolve()))
    extension = ET.SubElement(robot, 'mujoco')
    ET.SubElement(extension, 'compiler', strippath='false',
                  discardvisual='false', fusestatic='false')
    model = mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding='unicode'))
    with tempfile.TemporaryDirectory() as tmp:
        canonical = Path(tmp) / 'tank.xml'
        mujoco.mj_saveLastXML(str(canonical), model)
        tank = ET.parse(canonical).getroot()
    for mesh in tank.findall('./asset/mesh'):
        mesh.set('file', os.path.relpath(mesh.get('file'), OUTPUT.parent))
    frame = tank.find(".//body[@name='tank_opening_center']")
    ET.SubElement(frame, 'site', name='tank_opening_center', size='.002', rgba='1 .5 0 1')
    return tank


def read_model(relative):
    path = ROOT / relative
    model = ET.parse(path).getroot()
    compiler = model.find('compiler')
    meshdir = compiler.get('meshdir', '') if compiler is not None else ''
    for mesh in model.findall('./asset/mesh'):
        resolved = (path.parent / meshdir / mesh.get('file')).resolve()
        if not resolved.is_file():
            raise FileNotFoundError(resolved)
        mesh.set('file', os.path.relpath(resolved, OUTPUT.parent))
    return model


def main():
    arm = read_model('assets/robots/aubo_i10/mjcf/aubo_i10.xml')
    cap = read_model('assets/objects/fuel_cap/mjcf/fuel_cap.xml')
    tank = read_tank()
    hand = read_model('assets/reference/aviation/third_party/correlllab_rh56_controller_h12/mujoco_inspire/inspire_right.xml')
    scene = ET.Element('mujoco', model='i10_rh56_fuel_cap_provisional_preview')
    scene.append(ET.Comment(' PREVIEW ONLY: sensor diameter 75 mm, height 33.5 mm; '
                            'supplied adapter mesh, diameter 63 mm, height 25 mm; '
                            'adapter spigot inserted 15 mm into hand; hand base at adapter Z=10 mm. '
                            'Hand mounted along wrist3_Link +Z with no rotation. '
                            'Installation remains uncalibrated; use mj_forward only. '))
    ET.SubElement(scene, 'compiler', angle='radian')
    scene.append(copy.deepcopy(hand.find('default')))
    visual = ET.SubElement(scene, 'visual')
    ET.SubElement(visual, 'headlight', ambient='.5 .5 .5', diffuse='.7 .7 .7')
    asset = ET.SubElement(scene, 'asset')
    for source in (arm, cap, hand, tank):
        asset.extend(copy.deepcopy(source.findall('./asset/*')))
    ET.SubElement(asset, 'mesh', name='adapter_mesh',
                  file='../robots/adapter/meshes/adapter.stl')
    ET.SubElement(asset, 'mesh', name='sensor_mesh',
                  file='../robots/sensor/meshes/sensor.stl')
    world = ET.SubElement(scene, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -1 3', dir='0 0 -1')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='2 2 .01',
                  rgba='.25 .28 .32 1', contype='0', conaffinity='0', group='1')
    arm_mount = ET.SubElement(world, 'body', name='inspection_arm_mount', quat='0 0 0 1')
    arm_mount.extend(copy.deepcopy(arm.findall('./worldbody/body')))
    wrist = world.find(".//body[@name='wrist3_Link']")
    sensor = ET.SubElement(wrist, 'body', name='sensor_link')
    ET.SubElement(sensor, 'geom', name='sensor_visual', type='mesh',
                  mesh='sensor_mesh', rgba='.65 .70 .76 1',
                  group='1', contype='0', conaffinity='0', density='0')
    sensor_urdf = ET.parse(ROOT / 'assets/robots/sensor/urdf/sensor.urdf')
    output = sensor_urdf.find("joint[@name='sensor_output_fixed']/origin")
    sensor_output = ET.SubElement(sensor, 'body', name='sensor_output_frame',
                                 pos=output.get('xyz'), euler=output.get('rpy'))
    adapter = ET.SubElement(sensor_output, 'body', name='adapter')
    ET.SubElement(adapter, 'geom', name='adapter_visual', type='mesh',
                  mesh='adapter_mesh', rgba='.85 .55 .15 1',
                  group='1', contype='0', conaffinity='0', density='0')
    # Fully seat the hand on the 10 mm shoulder: 25 - 10 = 15 mm insertion.
    hand_mount = ET.SubElement(adapter, 'body', name='rh56_provisional_mount', pos='0 0 .010')
    hand_mount.extend(copy.deepcopy(hand.findall('./worldbody/body')))
    # The upstream world-level end-effector marker follows the hand mount.
    hand_mount.extend(copy.deepcopy(hand.findall('./worldbody/site')))
    # Display placement only. Preserve the tank's upright, ground-level mount.
    # The upright tank opening face is 0.2 m in front of its bottom center.
    tank_mount = ET.SubElement(world, 'body', name='inspection_tank_mount',
                               pos=f'0 {BASE_TO_TANK_FACE + .2:g} 0')
    tank_mount.extend(copy.deepcopy(list(tank.find('worldbody'))))
    # User-confirmed initial opening; see doc/assembly_contract.md.
    add_door(tank_mount.find(".//body[@name='fuel_tank_body']"),
             initial_angle=math.radians(135))
    filler_top = tank_mount.find(".//body[@name='tank_filler_top']")
    # Seat the gray cap shoulder on the ring face. The cap source places
    # cap_body 30 mm above its reference root; compensate that offset here.
    # Its 40 mm white plug then extends 20 mm into the 50 mm bore.
    cap_body_origin = cap.find(".//body[@name='cap_body']").get('pos').split()
    cap_offset = ' '.join(str(-float(value)) for value in cap_body_origin)
    cap_mount = ET.SubElement(filler_top, 'body', name='inspection_cap_mount', pos=cap_offset)
    cap_mount.extend(copy.deepcopy(list(cap.find('worldbody'))))
    for section in ('contact', 'equality', 'actuator'):
        scene.append(copy.deepcopy(hand.find(section)))
    add_adjacent_exclusions(scene)
    # Standalone hand keyframes are task-specific and are not combined here.
    tank_path = ROOT / 'assets/objects/fuel_tank/urdf/fuel_tank.urdf'
    tank_digest = hashlib.sha256(tank_path.read_bytes())
    referenced = sorted({(tank_path.parent / mesh.get('filename')).resolve()
                         for mesh in ET.parse(tank_path).getroot().iter('mesh')})
    for path in referenced:
        tank_digest.update(path.name.encode())
        tank_digest.update(path.read_bytes())
    scene.append(ET.Comment(f' Tank source and mesh SHA256: {tank_digest.hexdigest()} '))
    ET.indent(scene)
    ET.ElementTree(scene).write(OUTPUT, encoding='utf-8', xml_declaration=True)
    model = mujoco.MjModel.from_xml_path(str(OUTPUT))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.nq == 20 and model.nu == 6
    assert model.body('base').parentid[0] == model.body('rh56_provisional_mount').id
    print(f'{OUTPUT}: {model.nq} joints, {model.nu} hand actuators; provisional mount')


if __name__ == '__main__':
    main()
