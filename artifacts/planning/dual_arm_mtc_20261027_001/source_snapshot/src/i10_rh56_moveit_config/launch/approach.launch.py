"""无界面 MTC 规划；不执行生成的解。"""
import hashlib
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        model_config = config
        if request.get('robot_model_profile','single') == 'dual_arm':
            model_config=config.parent/'dual_arm'
            binding=request['observed_scene']['robot_model_binding']
            for name, expected in binding['files_sha256'].items():
                if hashlib.sha256((model_config/name).read_bytes()).hexdigest()!=expected:
                    raise RuntimeError('Installed dual-arm model changed: '+name)
        if 'observed_scene' in request:
            binding = request['observed_scene']
            manifest_path = Path(binding['manifest_path'])
            if hashlib.sha256(manifest_path.read_bytes()).hexdigest() != binding['manifest_sha256']:
                raise RuntimeError('Observed scene manifest changed')
            manifest = json.loads(manifest_path.read_text())
            for field, digest in [('map_path','map_sha256'), ('cap_object_path','cap_object_sha256')]:
                if hashlib.sha256((manifest_path.parent/manifest[field]).read_bytes()).hexdigest() != manifest[digest]:
                    raise RuntimeError('Observed scene geometry changed')
            if 'planning_scene' in request or 'scene_variation' in request:
                raise RuntimeError('Observed scene cannot mix legacy environment snapshots')
            ompl['arm']['longest_valid_segment_fraction'] = .0002
        if ('scene_variation' in request) != ('planning_scene' in request):
            raise RuntimeError('Sample and planning scene must be supplied together')
        if 'planning_scene' in request:
            import copy
            import math
            baseline = json.loads((config/'scene.json').read_text())
            snapshot = copy.deepcopy(request['planning_scene'])
            if len(snapshot['objects']) != len(baseline['objects']):
                raise RuntimeError('Sampled scene object count changed')
            for actual, original in zip(snapshot['objects'], baseline['objects']):
                values = actual['position'] + [v for row in actual['rotation_matrix'] for v in row]
                if not all(math.isfinite(v) for v in values):
                    raise RuntimeError('Nonfinite sampled scene pose')
                actual['position'] = original['position']
                actual['rotation_matrix'] = original['rotation_matrix']
            if snapshot != baseline:
                raise RuntimeError('Sampled scene changed shapes or collision rules')
        for name in ('generated_hashes.json', 'provenance.json'):
            key = f'src/i10_rh56_moveit_config/config/{name}'
            expected = request['source_hashes'][key]
            if hashlib.sha256((config/name).read_bytes()).hexdigest() != expected:
                raise RuntimeError('Request and installed model differ; regenerate/rebuild')
        # 启动新规划前使旧成功结果失效，避免失败后误用旧轨迹。
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps({'mode': 'planning_not_completed_no_replay'})+'\n')
        planner = Node(package='i10_rh56_moveit_config', executable='plan_approach', output='screen', parameters=[{
            'config_dir': str(config), 'request_path': str(request_path), 'output_path': str(output_path),
            'start_source': LaunchConfiguration('start_source'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'state_timeout_s': ParameterValue(LaunchConfiguration('state_timeout_s'), value_type=float),
            'state_max_age_s': ParameterValue(LaunchConfiguration('state_max_age_s'), value_type=float),
            'ompl': ompl, 'robot_description_kinematics': json.loads((model_config/'kinematics.json').read_text()),
        }])
        def check_exit(event, context):
            if event.returncode != 0:
                raise RuntimeError(f'MTC approach failed with exit code {event.returncode}; no replay')
            return []
        return [RegisterEventHandler(OnProcessExit(target_action=planner, on_exit=check_exit)), planner]
    return LaunchDescription([
        DeclareLaunchArgument('request_path'), DeclareLaunchArgument('output_path'),
        DeclareLaunchArgument('start_source', default_value='request', choices=['request', 'joint_states']),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('state_timeout_s', default_value='3.0'),
        DeclareLaunchArgument('state_max_age_s', default_value='0.5'),
        OpaqueFunction(function=setup)])
