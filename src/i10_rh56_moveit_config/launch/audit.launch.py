"""Load and audit real MoveIt models headlessly; does not plan or execute."""
import hashlib
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('i10_rh56_moveit_config')) / 'config'
    kinematics = json.loads((config / 'kinematics.json').read_text())
    for relative, expected in json.loads((config / 'generated_hashes.json').read_text()).items():
        path = config.parent / relative
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f'Stale or damaged installed planning model: {path}; regenerate/rebuild')
    audit = Node(package='i10_rh56_moveit_config', executable='audit_model', output='screen', parameters=[{
        'config_dir': str(config),
        'output_path': LaunchConfiguration('output_path'),
        'coverage': LaunchConfiguration('coverage'),
        'robot_description_kinematics': kinematics,
    }])

    def check_exit(event, context):
        if event.returncode != 0:
            raise RuntimeError(f'Model audit failed with exit code {event.returncode}')
        return []

    return LaunchDescription([
        DeclareLaunchArgument('coverage', default_value='planning'),
        DeclareLaunchArgument('output_path', default_value='/tmp/i10_rh56_moveit_audit.yaml'),
        RegisterEventHandler(OnProcessExit(target_action=audit, on_exit=check_exit)),
        audit,
    ])
