"""显式提供地址后启动只读 RTDE 状态桥，默认发布六轴实测关节状态。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    parameters = {
        'host': LaunchConfiguration('host'),
        'rate_hz': ParameterValue(LaunchConfiguration('rate_hz'), value_type=float),
        'stale_timeout_s': ParameterValue(LaunchConfiguration('stale_timeout_s'), value_type=float),
        'joint_states_topic': LaunchConfiguration('joint_states_topic'),
        'use_sim_time': False,
    }
    bridge = Node(package='aubo_joint_state_bridge', executable='joint_state_bridge',
                  output='screen', parameters=[parameters])

    def check_exit(event, context):
        if event.returncode != 0:
            raise RuntimeError(f'关节状态桥已退出，返回码 {event.returncode}')
        return []

    return LaunchDescription([
        DeclareLaunchArgument('host'),
        DeclareLaunchArgument('rate_hz', default_value='50.0'),
        DeclareLaunchArgument('stale_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=check_exit)),
        bridge,
    ])
