"""Headless MTC planning; never executes a solution."""
import hashlib
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('i10_rh56_moveit_config')) / 'config'
    for relative, expected in json.loads((config / 'generated_hashes.json').read_text()).items():
        path = config.parent / relative
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f'Stale installed model: {path}')
    ompl = {'planning_plugin': 'ompl_interface/OMPLPlanner', 'request_adapters': '',
            'planner_configs': {'RRTConnectkConfigDefault': {'type': 'geometric::RRTConnect', 'range': 0.0}},
            'arm': {'planner_configs': ['RRTConnectkConfigDefault'], 'longest_valid_segment_fraction': 0.005}}
    def setup(context):
        request_path = Path(LaunchConfiguration('request_path').perform(context)).resolve()
        output_path = Path(LaunchConfiguration('output_path').perform(context)).resolve()
        if output_path == request_path:
            raise RuntimeError('Request and output must differ')
        request = json.loads(request_path.read_text())
        for name in ('generated_hashes.json', 'provenance.json'):
            key = f'src/i10_rh56_moveit_config/config/{name}'
            expected = request['source_hashes'][key]
            if hashlib.sha256((config/name).read_bytes()).hexdigest() != expected:
                raise RuntimeError('Request and installed model differ; regenerate/rebuild')
        # Invalidate any old success before launching a new planning attempt.
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps({'mode': 'planning_not_completed_no_replay'})+'\n')
        planner = Node(package='i10_rh56_moveit_config', executable='plan_approach', output='screen', parameters=[{
            'config_dir': str(config), 'request_path': str(request_path), 'output_path': str(output_path),
            'ompl': ompl, 'robot_description_kinematics': json.loads((config/'kinematics.json').read_text()),
        }])
        def check_exit(event, context):
            if event.returncode != 0:
                raise RuntimeError(f'MTC approach failed with exit code {event.returncode}; no replay')
            return []
        return [RegisterEventHandler(OnProcessExit(target_action=planner, on_exit=check_exit)), planner]
    return LaunchDescription([
        DeclareLaunchArgument('request_path'), DeclareLaunchArgument('output_path'), OpaqueFunction(function=setup)])
