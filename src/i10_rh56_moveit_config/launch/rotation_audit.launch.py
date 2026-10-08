"""完整规划覆盖的旋转前缀审计；核对来源，不启动控制器。"""
import hashlib
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('i10_rh56_moveit_config'))/'config'
    def setup(context):
        path = Path(LaunchConfiguration('diagnostic_path').perform(context)).resolve()
        output = Path(LaunchConfiguration('output_path').perform(context)).resolve()
        if output == path:
            raise RuntimeError('诊断输入与审计输出必须不同')
        diagnostic = json.loads(path.read_text())
        if diagnostic.get('mode') not in ('fixed_depth_preform_rotation_diagnostic_not_execution', 'segmented_fixed_depth_rotation_not_execution') or not diagnostic.get('analysis_finished'):
            raise RuntimeError('诊断输入不完整')
        # 轨迹路径通过祖先中的工作区定位，禁止从不相关的同名模型加载。
        workspace = next((p for p in path.parents if (p/'src/fuel_cap_task').is_dir()), None)
        if workspace is None:
            raise RuntimeError('无法定位工作区')
        trajectory = (workspace/diagnostic['trajectory_path']).resolve()
        if not trajectory.is_relative_to(workspace) or hashlib.sha256(trajectory.read_bytes()).hexdigest() != diagnostic['trajectory_sha256']:
            raise RuntimeError('原接近轨迹来源已变更')
        request = json.loads(trajectory.read_text())['request']
        for relative, expected in request['source_hashes'].items():
            source = (workspace/relative).resolve()
            if not source.is_relative_to(workspace) or hashlib.sha256(source.read_bytes()).hexdigest() != expected:
                raise RuntimeError('工作区规划来源已变更')
        for name in ('provenance.json', 'generated_hashes.json'):
            expected = request['source_hashes'][f'src/i10_rh56_moveit_config/config/{name}']
            if hashlib.sha256((config/name).read_bytes()).hexdigest() != expected:
                raise RuntimeError('安装模型与诊断来源不一致')
        for relative, expected in json.loads((config/'generated_hashes.json').read_text()).items():
            if hashlib.sha256((config.parent/relative).read_bytes()).hexdigest() != expected:
                raise RuntimeError('安装产物已变更')
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps({'mode': 'rotation_audit_not_completed'})+'\n')
        audit = Node(package='i10_rh56_moveit_config', executable='audit_rotation', output='screen', parameters=[{
            'config_dir': str(config), 'diagnostic_path': str(path), 'output_path': str(output),
            'diagnostic_sha256': hashlib.sha256(path.read_bytes()).hexdigest()}])
        def check_exit(event, context):
            if event.returncode != 0:
                raise RuntimeError('旋转审计程序失败')
            return []
        return [RegisterEventHandler(OnProcessExit(target_action=audit, on_exit=check_exit)), audit]
    return LaunchDescription([DeclareLaunchArgument('diagnostic_path'), DeclareLaunchArgument('output_path'),
                              OpaqueFunction(function=setup)])
