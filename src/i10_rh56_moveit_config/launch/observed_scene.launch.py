"""核对与发布 MTC 使用的实测场景，可在 RViz 中查看；不启动控制器。"""
import hashlib
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """审计与 RViz 读取同一安装模型，审计器发布其实际加载的完整 PlanningScene。"""
    package = Path(get_package_share_directory('i10_rh56_moveit_config'))
    config = package/'config'
    for relative, expected in json.loads((config/'generated_hashes.json').read_text()).items():
        if hashlib.sha256((package/relative).read_bytes()).hexdigest() != expected:
            raise RuntimeError('Installed model changed; regenerate and rebuild')
    def setup(context):
        """模型选择来自已绑定的请求；RViz 与审计器读取同一套双臂或单臂描述。"""
        request_path=Path(LaunchConfiguration('request_path').perform(context)).resolve()
        request=json.loads(request_path.read_text())
        profile=request.get('robot_model_profile','single')
        if profile not in ('single','dual_arm'):
            raise RuntimeError('Unknown robot model profile')
        model_config=package/'dual_arm' if profile=='dual_arm' else config
        if profile=='dual_arm':
            for name,expected in request['observed_scene']['robot_model_binding']['files_sha256'].items():
                if hashlib.sha256((model_config/name).read_bytes()).hexdigest()!=expected:
                    raise RuntimeError('Installed dual-arm model changed: '+name)
        kinematics=json.loads((model_config/'kinematics.json').read_text())
        audit = Node(package='i10_rh56_moveit_config', executable='audit_observed_scene', output='screen', parameters=[{
            'config_dir': str(config), 'request_path': str(request_path),
            'output_path': LaunchConfiguration('output_path'),
            'publish_seconds': ParameterValue(LaunchConfiguration('publish_seconds'), value_type=float),
            'trajectory_path': LaunchConfiguration('trajectory_path'),
            'robot_description_kinematics': kinematics,
        }])
        viewer = Node(package='rviz2', executable='rviz2', output='screen',
                      arguments=['-d', str(config/('observed_dual_arm.rviz' if profile=='dual_arm' else 'observed_scene.rviz'))],
                      condition=IfCondition(LaunchConfiguration('rviz')),
                      parameters=[{'robot_description': (model_config/'robot.urdf').read_text(),
                                   'robot_description_semantic': (model_config/'robot.srdf').read_text(),
                                   'robot_description_kinematics': kinematics}])
        def check_exit(event, context):
            """审计失败时让 launch 返回失败，不把不完整检查当作成功。"""
            if event.returncode != 0:
                raise RuntimeError(f'Observed scene audit failed: {event.returncode}')
            return []
        return [RegisterEventHandler(OnProcessExit(target_action=audit,on_exit=check_exit)),audit,viewer]

    return LaunchDescription([
        DeclareLaunchArgument('request_path'), DeclareLaunchArgument('output_path'),
        DeclareLaunchArgument('publish_seconds', default_value='0'),
        DeclareLaunchArgument('trajectory_path', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=setup)])
