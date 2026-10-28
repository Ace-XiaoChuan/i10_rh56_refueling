"""用 MoveIt 读取六轴新状态；手指状态仍待 RH56 实测反馈接入。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory('i10_rh56_moveit_config')) / 'config'
    reader = Node(package='i10_rh56_moveit_config', executable='snapshot_arm_state',
                  output='screen', parameters=[{
                      'config_dir': str(config), 'use_sim_time': False,
                      'joint_states_topic': LaunchConfiguration('joint_states_topic'),
                      'output_path': LaunchConfiguration('output_path'),
                      'state_timeout_s': ParameterValue(LaunchConfiguration('state_timeout_s'), value_type=float),
                      'state_max_age_s': ParameterValue(LaunchConfiguration('state_max_age_s'), value_type=float),
                  }])

    def check_exit(event, context):
        if event.returncode != 0:
            raise RuntimeError(f'MoveIt 关节状态读取失败，返回码 {event.returncode}')
        return []

    return LaunchDescription([
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('output_path', default_value=''),
        DeclareLaunchArgument('state_timeout_s', default_value='3.0'),
        DeclareLaunchArgument('state_max_age_s', default_value='0.5'),
        RegisterEventHandler(OnProcessExit(target_action=reader, on_exit=check_exit)),
        reader,
    ])
