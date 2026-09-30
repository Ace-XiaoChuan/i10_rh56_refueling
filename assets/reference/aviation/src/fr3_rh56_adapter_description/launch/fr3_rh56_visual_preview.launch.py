import math
import os
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PREVIEW_NAMESPACE = "adapter_preview"


# Matches the official fr3_arm "ready" group state in
# franka_description/robots/common/group_definition.xacro.
FR3_READY_JOINT_POSITIONS = {
    "fr3_joint1": 0.0,
    "fr3_joint2": -math.pi / 4.0,
    "fr3_joint3": 0.0,
    "fr3_joint4": -3.0 * math.pi / 4.0,
    "fr3_joint5": 0.0,
    "fr3_joint6": math.pi / 2.0,
    "fr3_joint7": math.pi / 4.0,
}


def _format_vector(value, path):
    """Turn one finite three-vector from the mount contract into xacro text."""
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f'{path} must contain exactly three numbers')
    try:
        numbers = tuple(float(component) for component in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{path} must contain exactly three numbers') from error
    if not all(math.isfinite(component) for component in numbers):
        raise ValueError(f'{path} must contain finite numbers')
    return ' '.join(str(component) for component in numbers)


def _load_mount_mappings(config_path):
    """Read the one provisional mount contract used by this visual preview."""
    path = Path(config_path)
    try:
        with path.open('r', encoding='utf-8') as config_file:
            contract = yaml.safe_load(config_file)
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(f'cannot read mount contract {path}: {error}') from error
    if not isinstance(contract, dict) or contract.get('schema_version') != 1:
        raise RuntimeError('mount contract must have schema_version: 1')
    if contract.get('status') not in {
        'provisional_r1_visual',
        'provisional_r1_model_collision',
    }:
        raise RuntimeError(
            'mount contract status must be provisional_r1_visual or '
            'provisional_r1_model_collision'
        )
    segments = contract.get('segments')
    preview = contract.get('preview')
    if not isinstance(segments, dict) or not isinstance(preview, dict):
        raise RuntimeError('mount contract requires segments and preview mappings')

    def segment(name):
        value = segments.get(name)
        if not isinstance(value, dict):
            raise RuntimeError(f'mount contract is missing segments.{name}')
        return value

    adapter_1 = segment('adapter_1')
    adapter_2 = segment('adapter_2')
    rh56_palm = segment('rh56_palm')
    grasp_tcp = segment('grasp_tcp')
    show_adapter_2 = preview.get('show_adapter_2')
    show_rh56 = preview.get('show_rh56')
    if not isinstance(show_adapter_2, bool) or not isinstance(show_rh56, bool):
        raise RuntimeError(
            'mount contract preview.show_adapter_2 and preview.show_rh56 '
            'must be bools'
        )
    if show_rh56 and not show_adapter_2:
        raise RuntimeError('preview.show_rh56 requires preview.show_adapter_2')
    return {
        'adapter_1_xyz': _format_vector(
            adapter_1.get('xyz_m'), 'segments.adapter_1.xyz_m'
        ),
        'adapter_1_rpy': _format_vector(
            adapter_1.get('rpy_rad'), 'segments.adapter_1.rpy_rad'
        ),
        'adapter_2_xyz': _format_vector(
            adapter_2.get('xyz_m'), 'segments.adapter_2.xyz_m'
        ),
        'adapter_2_rpy': _format_vector(
            adapter_2.get('rpy_rad'), 'segments.adapter_2.rpy_rad'
        ),
        'rh56_xyz': _format_vector(
            rh56_palm.get('xyz_m'), 'segments.rh56_palm.xyz_m'
        ),
        'rh56_rpy': _format_vector(
            rh56_palm.get('rpy_rad'), 'segments.rh56_palm.rpy_rad'
        ),
        'rh56_grasp_tcp_xyz': _format_vector(
            grasp_tcp.get('xyz_m'), 'segments.grasp_tcp.xyz_m'
        ),
        'rh56_grasp_tcp_rpy': _format_vector(
            grasp_tcp.get('rpy_rad'), 'segments.grasp_tcp.rpy_rad'
        ),
        'show_adapter_2': str(show_adapter_2).lower(),
        'show_rh56': str(show_rh56).lower(),
    }


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory("fr3_rh56_adapter_description")
    xacro_file = os.path.join(package_share, "urdf", "fr3_rh56_visual_preview.urdf.xacro")
    rviz_config = os.path.join(package_share, "rviz", "fr3_rh56_visual_preview.rviz")
    mount_mappings = _load_mount_mappings(
        LaunchConfiguration('mount_config').perform(context)
    )
    argument_names = (
        "adapter_1_xyz",
        "adapter_1_rpy",
        "show_adapter_2",
        "show_rh56",
        "adapter_2_xyz",
        "adapter_2_rpy",
        "rh56_xyz",
        "rh56_rpy",
        "rh56_grasp_tcp_xyz",
        "rh56_grasp_tcp_rpy",
    )
    mappings = {}
    for name in argument_names:
        override = LaunchConfiguration(name).perform(context)
        mappings[name] = override if override else mount_mappings[name]
    robot_description = {
        "robot_description": xacro.process_file(xacro_file, mappings=mappings).toxml()
    }
    return [
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="fr3_ready_joint_state_publisher",
            namespace=PREVIEW_NAMESPACE,
            parameters=[
                robot_description,
                {"rate": 30.0, "zeros": FR3_READY_JOINT_POSITIONS},
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="fr3_rh56_robot_state_publisher",
            namespace=PREVIEW_NAMESPACE,
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="fr3_rh56_visual_preview_rviz",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            output="screen",
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("fr3_rh56_adapter_description")
    mount_config = os.path.join(
        package_share, 'config', 'fr3_rh56_mount_provisional.yaml'
    )
    arguments = [
        ("adapter_1_xyz", ""),
        ("adapter_1_rpy", ""),
        ("show_adapter_2", ""),
        ("show_rh56", ""),
        ("adapter_2_xyz", ""),
        ("adapter_2_rpy", ""),
        ("rh56_xyz", ""),
        ("rh56_rpy", ""),
        ("rh56_grasp_tcp_xyz", ""),
        ("rh56_grasp_tcp_rpy", ""),
    ]
    launch_arguments = [
        DeclareLaunchArgument(name, default_value=value) for name, value in arguments
    ]
    launch_arguments.append(
        DeclareLaunchArgument("mount_config", default_value=mount_config)
    )
    launch_arguments.append(
        DeclareLaunchArgument("use_rviz", default_value="true")
    )
    launch_arguments.append(OpaqueFunction(function=launch_setup))
    return LaunchDescription(launch_arguments)
