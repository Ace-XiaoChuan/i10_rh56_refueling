#!/usr/bin/env python3
"""Kinematic MJCF inspection: joint sliders, no time integration or hardware."""
import argparse
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import mujoco.viewer

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--cap-only', action='store_true')
    selection.add_argument('--tank-only', action='store_true', help='Inspect the fuel tank URDF')
    selection.add_argument('--sensor-only', action='store_true', help='Inspect the supplied sensor URDF')
    selection.add_argument('--with-hand', action='store_true', help='Preview provisional i10 + RH56 + cap assembly')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    path = ROOT / ('assets/objects/fuel_cap/mjcf/fuel_cap.xml' if args.cap_only
                   else 'assets/scenes/i10_fuel_cap.xml')
    if args.with_hand:
        path = ROOT / 'assets/scenes/i10_rh56_fuel_cap_preview.xml'
    if args.sensor_only:
        path = ROOT / 'assets/robots/sensor/urdf/sensor.urdf'
        model = mujoco.MjModel.from_xml_path(str(path))
    elif args.tank_only:
        path = ROOT / 'assets/objects/fuel_tank/urdf/fuel_tank.urdf'
        robot = ET.parse(path).getroot()
        for mesh in robot.iter('mesh'):
            mesh.set('filename', str((path.parent / mesh.get('filename')).resolve()))
        extension = ET.SubElement(robot, 'mujoco')
        ET.SubElement(extension, 'compiler', strippath='false', discardvisual='false')
        model = mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding='unicode'))
        # URDF reference links are fused on import; expose the opening frame as
        # a MuJoCo site using the URDF fixed transforms as the source of truth.
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            canonical = Path(tmp) / 'tank.xml'
            mujoco.mj_saveLastXML(str(canonical), model)
            scene = ET.parse(canonical).getroot()
        import numpy as np
        mount = robot.find("joint[@name='fuel_tank_fixed']/origin")
        frame = robot.find("joint[@name='tank_opening_center_fixed']/origin")
        angles = np.fromstring(mount.get('rpy'), sep=' ')
        quat = np.zeros(4)
        mujoco.mju_euler2Quat(quat, angles, 'xyz')
        position = np.zeros(3)
        mujoco.mju_rotVecQuat(position, np.fromstring(frame.get('xyz'), sep=' '), quat)
        position += np.fromstring(mount.get('xyz'), sep=' ')
        ET.SubElement(scene.find('worldbody'), 'site', name='tank_opening_center',
                      pos=' '.join(map(str, position)), quat=' '.join(map(str, quat)),
                      size='.002', rgba='1 .5 0 1')
        visual = scene.find('visual')
        if visual is None:
            visual = ET.SubElement(scene, 'visual')
        ET.SubElement(visual, 'scale', framelength='.08', framewidth='.003')
        model = mujoco.MjModel.from_xml_string(ET.tostring(scene, encoding='unicode'))
    else:
        model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    if not args.cap_only and not args.tank_only and not args.sensor_only:
        data.joint('upperArm_joint').qpos[0] = -.5
        data.joint('foreArm_joint').qpos[0] = 1.2
    if args.with_hand:
        # Display a near-open hand within its limits and upstream coupling rules.
        data.joint('thumb_proximal_pitch_joint').qpos[0] = .1
        for finger, offset in (('index', -.05), ('middle', -.15),
                               ('ring', -.15), ('pinky', -.15)):
            data.joint(f'{finger}_proximal_joint').qpos[0] = .15
            data.joint(f'{finger}_intermediate_joint').qpos[0] = offset + 1.1169*.15
        data.joint('thumb_intermediate_joint').qpos[0] = .15 + 1.33*.1
        data.joint('thumb_distal_joint').qpos[0] = .15 + .66*.1
    mujoco.mj_forward(model, data)
    if args.check:
        print(f'MJCF OK: {model.nq} joints, {model.nsite} reference sites')
        return
    with mujoco.viewer.launch_passive(model, data) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = [0, 0, .03] if args.cap_only else [.2, 0, .55]
            viewer.cam.distance = .24 if args.cap_only else 2.6
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
            if args.sensor_only:
                viewer.cam.lookat[:] = [0, 0, .01675]
                viewer.cam.distance = .25
            if args.with_hand:
                viewer.cam.lookat[:] = [0, .5, .55]
                viewer.cam.distance = 3.2
                viewer.cam.azimuth = -110
            if args.tank_only:
                viewer.cam.lookat[:] = [0, -.16, .4]
                viewer.cam.distance = 1.5
                viewer.cam.azimuth = -90
                viewer.cam.elevation = -15
            viewer.opt.geomgroup[0] = 0  # Hide collision geometry.
            if args.with_hand:
                viewer.opt.geomgroup[3] = 0  # RH56 collision geometry.
            viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        while viewer.is_running():
            with viewer.lock():
                mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1 / 60)


if __name__ == '__main__':
    main()
