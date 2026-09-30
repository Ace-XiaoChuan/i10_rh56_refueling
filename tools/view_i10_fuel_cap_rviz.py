#!/usr/bin/env python3
"""Inspect the supplied URDFs in RViz; no hardware or dynamics control."""
import argparse
import os
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ARM_ASSETS = ROOT / 'assets/robots/aubo_i10'
ARM_URDF = ARM_ASSETS / 'urdf/aubo_i10_JY.urdf'
CAP_URDF = ROOT / 'assets/objects/fuel_cap/urdf/fuel_cap.urdf'


def description(cap_only=False):
    robot = ET.parse(CAP_URDF if cap_only else ARM_URDF).getroot()
    if not cap_only:
        for mesh in robot.iter('mesh'):
            name = mesh.attrib['filename']
            prefix = 'package://aubo_description/'
            if not name.startswith(prefix):
                raise ValueError(f'Unexpected mesh URL: {name}')
            path = ARM_ASSETS / name[len(prefix):]
            if not path.is_file():
                raise FileNotFoundError(path)
            mesh.set('filename', path.as_uri())
        robot.extend(ET.parse(CAP_URDF).getroot())
        # Display placement only, not a measured installation transform.
        joint = ET.SubElement(robot, 'joint', name='inspection_cap_mount', type='fixed')
        ET.SubElement(joint, 'parent', link='base_link')
        ET.SubElement(joint, 'child', link='filler_neck')
        ET.SubElement(joint, 'origin', xyz='0.65 -0.35 0.55', rpy='0 0 0')
    return ET.tostring(robot, encoding='unicode')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cap-only', action='store_true', help='Close-up of the fuel cap')
    parser.add_argument('--check', action='store_true', help='Check inputs without opening windows')
    args = parser.parse_args()
    xml = description(args.cap_only)
    if args.check:
        robot = ET.fromstring(xml)
        print(f"URDF OK: {len(robot.findall('link'))} links, {len(robot.findall('joint'))} joints; all mesh files exist")
        return

    os.environ.setdefault('ROS_LOG_DIR', str(ROOT / 'log/rviz_inspection'))
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    os.environ.setdefault('ROS_DOMAIN_ID', '56')
    os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')
    from launch import LaunchDescription, LaunchService
    from launch.actions import RegisterEventHandler, EmitEvent
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch_ros.actions import Node

    config = 'fuel_cap_inspection.rviz' if args.cap_only else 'i10_fuel_cap_inspection.rviz'
    rviz = Node(package='rviz2', executable='rviz2', arguments=['-d', str(ROOT / 'config' / config)], output='screen')
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': xml}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[{'robot_description': xml, 'zeros.upperArm_joint': -0.5,
                          'zeros.foreArm_joint': 1.2}], output='screen'),
        rviz,
        RegisterEventHandler(OnProcessExit(target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))])),
    ]))
    return service.run()


if __name__ == '__main__':
    raise SystemExit(main())
