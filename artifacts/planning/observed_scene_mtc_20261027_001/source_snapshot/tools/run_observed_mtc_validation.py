#!/usr/bin/env python3
"""离线运行实测场景检查和两方向 MTC 接近；不连接设备或执行轨迹。"""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))


def run_logged(command, path, env, timeout):
    """保存原始标准输出和退出码，失败后保留诊断，禁止把失败结果当作轨迹。"""
    with path.open('w') as stream:
        try:
            process = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, env=env, timeout=timeout)
            return process.returncode
        except subprocess.TimeoutExpired:
            stream.write('\nOffline validation process timed out\n')
            return 124


def main():
    """每次写入新目录；相交/分离对照通过后才调用 MTC，结果如实记录成功或失败。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--binary-dir', type=Path, default=ROOT/'install/i10_rh56_moveit_config/lib/i10_rh56_moveit_config')
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists():
        parser.error('输出目录已存在，请指定新目录')
    output.mkdir(parents=True)
    config_dir = ROOT/'src/i10_rh56_moveit_config/config'
    config = json.loads((ROOT/'config/active_point_cloud_scene.json').read_text())
    manifest = ROOT/config['collision_scene']['mtc_manifest']
    placement = json.loads((ROOT/config['octomap']['placement_config']).read_text())
    env = os.environ.copy()
    env['ROS_LOCALHOST_ONLY'] = '1'
    # 本机全局 DDS 配置绑定现场网卡；离线测试使用中间件自己的 localhost 配置。
    env.pop('CYCLONEDDS_URI', None)
    env['ROS_DOMAIN_ID'] = '47'
    env['ROS_LOG_DIR'] = str(output/'ros_logs')
    env['AMENT_PREFIX_PATH'] = str(ROOT/'install/i10_rh56_moveit_config')+':'+env.get('AMENT_PREFIX_PATH','')
    env['PYTHONPATH'] = str(ROOT/'src')+':'+env.get('PYTHONPATH','')
    summary = {'mode': 'offline_observed_mtc_validation', 'robot_connections': 0,
               'trajectory_execution': False, 'candidates': {}}
    for direction in ['direction_000', 'direction_180']:
        folder = output/direction
        folder.mkdir()
        request = folder/'request.json'
        command = [sys.executable, str(ROOT/'tools/prepare_cap_approach.py'), '--start',
                   *map(str, placement['preview_arm_joints_rad']), '--distance', str(config['collision_scene']['approach_distance_m']),
                   '--observed-scene', str(manifest), '--direction', direction, '--output', str(request)]
        print('Preparing '+direction, flush=True)
        code = run_logged(command, folder/'prepare.log', env, 120)
        if code:
            raise RuntimeError(f'目标准备失败，退出码 {code}：{folder}/prepare.log')
        parameters = {'config_dir': str(config_dir), 'request_path': str(request),
                      'output_path': str(folder/'scene_audit.json'),
                      'robot_description_kinematics': json.loads((config_dir/'kinematics.json').read_text())}
        param_file = folder/'audit_parameters.yaml'
        param_file.write_text(json.dumps({'/**': {'ros__parameters': parameters}}, indent=2)+'\n')
        print('Checking loaded scene and FCL: '+direction, flush=True)
        code = run_logged([str(args.binary_dir/'audit_observed_scene'), '--ros-args', '--params-file', str(param_file)],
                          folder/'scene_audit.log', env, 120)
        row = {'audit_exit_code': code}
        summary['candidates'][direction] = row
        if code:
            (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
            raise RuntimeError(f'场景对照失败，退出码 {code}：{folder}/scene_audit.log')
        parameters['output_path'] = str(folder/'trajectory.json')
        parameters['start_source'] = 'request'
        parameters['ompl'] = {'planning_plugin': 'ompl_interface/OMPLPlanner', 'request_adapters': '',
                              'planner_configs': {'RRTConnectkConfigDefault': {'type': 'geometric::RRTConnect', 'range': 0.0}},
                              'arm': {'planner_configs': ['RRTConnectkConfigDefault'], 'longest_valid_segment_fraction': .0002}}
        param_file = folder/'planning_parameters.yaml'
        param_file.write_text(json.dumps({'/**': {'ros__parameters': parameters}}, indent=2)+'\n')
        print('Planning '+direction, flush=True)
        code = run_logged([str(args.binary_dir/'plan_approach'), '--ros-args', '--params-file', str(param_file)],
                          folder/'planning.log', env, 180)
        row['planning_exit_code'] = code
        if (folder/'trajectory.json').is_file():
            report = json.loads((folder/'trajectory.json').read_text())
            row.update({key: report[key] for key in ['mode','error','endpoint_position_error_m',
                        'endpoint_orientation_error_rad','dense_fcl_check_passed','endpoint_check_passed'] if key in report})
            row['validated_waypoints'] = sum(len(stage['points']) for stage in report.get('stages', []))
        print(direction+': '+json.dumps(row, ensure_ascii=False), flush=True)
        (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    summary['all_audits_passed'] = all(row['audit_exit_code'] == 0 for row in summary['candidates'].values())
    summary['all_candidates_planned'] = all(row['planning_exit_code'] == 0 for row in summary['candidates'].values())
    (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(output/'summary.json', flush=True)


if __name__ == '__main__':
    main()
