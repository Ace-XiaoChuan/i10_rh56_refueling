"""分段旋转与 MTC 重新接近的几何实验；手保持预型，不执行物理松手。"""
import asyncio
import copy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from .arm_ik import body_transform, rotation_error, transform
from .grasp_search import pose_arrays, quaternion
from .planning_scene_snapshot import scene_snapshot, check_snapshot
from .rotation_diagnostic import RotationDiagnostic, hand_target, solve_continuous
from .trajectory_replay import ROOT


# 当前合同确认的本轮位姿验收值；报告必须保存它，避免影响历史解释。
CURRENT_POSE_TOLERANCES = {'position_m': .0002, 'orientation_rad': .005}


def pose_tolerances(record):
    """历史记录缺少字段时沿用当时的严格容差。"""
    values = record.get('endpoint_tolerances', {'position_m': .0001, 'orientation_rad': .002})
    if (set(values) != {'position_m', 'orientation_rad'}
            or not all(np.isfinite(v) and v > 0 for v in values.values())):
        raise ValueError('位姿容差必须为有限正值')
    return values


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    """每阶段原子保存检查点，未完成结果也保留。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    temporary.replace(path)


def pose_error(actual, target):
    return (float(np.linalg.norm(actual[:3, 3]-target[:3, 3])),
            float(np.linalg.norm(rotation_error(target[:3, :3], actual[:3, :3]))))


def rotate(source, q0, begin, end, output, tolerances=None):
    """沿本次到达构型推进一段；累计盖角不随重新抓握归零。"""
    request = source.replay.report['request']
    report = {'schema_version': 1, 'mode': 'segmented_fixed_depth_rotation_not_execution',
              'trajectory_path': str(source.path.resolve().relative_to(ROOT)),
              'trajectory_sha256': digest(source.path), 'arm_joints': request['arm_joints'],
              'initial_joint_positions': request['initial_joint_positions'],
              'planning_scene': source.world, 'begin_angle_deg': begin, 'target_angle_deg': end,
              'samples': [], 'failure': None, 'analysis_finished': False,
              'hand_state': 'approach_preform_not_closed', 'axial_motion_m': 0.,
              'physical_grasp_verified': False, 'thread_motion_verified': False}
    if tolerances is not None:
        report['endpoint_tolerances'] = dict(tolerances)
    limits = pose_tolerances(report)
    previous, previous_angle = np.asarray(q0).copy(), begin
    for angle in np.linspace(begin, end, int(np.ceil(end-begin))+1):
        source.set_state(previous, angle)
        target = hand_target(source.cap0, source.relative, np.deg2rad(angle))
        if angle == begin:
            q, found = previous.copy(), True
        else:
            q, error, found = solve_continuous(source.preview, target, previous)
        failure = None
        if not found:
            # 后续失败保留求解残差与候选关节，不能用它们补造旧试验的状态。
            failure = {'kind': 'continuous_ik_not_converged', 'angle_deg': float(angle),
                       'arm_qpos_rad': q.tolist(),
                       'position_error_m': float(np.linalg.norm(error[:3])),
                       'orientation_error_rad': float(np.linalg.norm(error[3:]))}
        elif np.max(np.abs(q-previous)) > .1:
            failure = {'kind': 'joint_branch_jump', 'angle_deg': float(angle)}
        else:
            # 臂路点加密与盖角同步；检查误差、限位及原覆盖碰撞。
            steps = max(1, int(np.ceil(np.max(np.abs(q-previous))/.02)))
            for fraction in np.linspace(1/steps, 1., steps):
                dense = previous+fraction*(q-previous)
                a = previous_angle+fraction*(angle-previous_angle)
                row = source.row(dense, a)
                expected = hand_target(source.cap0, source.relative, np.deg2rad(a))
                p, r = pose_error(body_transform(source.preview.data, 'base'), expected)
                contacts = [c for c in source.preview.contacts() if c['penetration_m'] > .0001]
                if contacts or p > limits['position_m'] or r > limits['orientation_rad'] or row['minimum_joint_margin_rad'] < 0:
                    failure = {'kind': 'mujoco_robot_interference' if contacts else 'pose_or_bounds',
                               'angle_deg': float(a), 'arm_qpos_rad': dense.tolist(),
                               'contacts': contacts, 'position_error_m': p, 'orientation_error_rad': r}
                    break
                row.update(position_error_m=p, orientation_error_rad=r, phase='rotation')
                report['samples'].append(row)
        if failure:
            report['failure'] = failure
            break
        previous, previous_angle = q.copy(), float(angle)
        save(output, report)
    report['analysis_finished'] = True
    report['mujoco_prefix_angle_deg'] = previous_angle
    save(output, report)
    return report


def regrasp_request(source, q, angle, tolerances=None):
    """读取旋转后的新观测，再构造退开、调整与接近请求。"""
    source.set_state(q, angle)
    observed = asyncio.run(source.preview.scene.estimator.estimate())
    if not observed.valid or observed.pose.frame_id != 'world':
        raise ValueError('重新观测无效')
    cap = transform(*pose_arrays(asdict(observed.pose)))
    expected = hand_target(source.cap0, np.eye(4), np.deg2rad(angle))
    if not np.allclose(cap, expected, atol=1e-10, rtol=0):
        raise ValueError('新观测与本轮盖体状态不一致')
    # 保留当前手盖候选；新盖姿态给出新世界目标，不假定臂会自动复位。
    target = cap @ source.relative
    request = copy.deepcopy(source.replay.report['request'])
    request['initial_joint_positions'].update(zip(request['arm_joints'], map(float, q)))
    request.update(cap_estimate=asdict(observed), hand_position=target[:3, 3].tolist(),
                   hand_orientation_xyzw=list(quaternion(target[:3, :3])),
                   pregrasp_position=(target[:3, 3]+.1*cap[:3, 2]).tolist(),
                   outward_axis_world=cap[:3, 2].tolist(), approach_distance_m=.1,
                   planning_scene=scene_snapshot(source.world, source.preview.model, source.preview.data),
                   regrasp={'cumulative_angle_deg': angle, 'retreat_distance_m': .1,
                            'release_executed': False, 'grasp_verified': False,
                            'observation_stage': 'post_rotation_before_retreat_geometric_snapshot',
                            'candidate_policy': 'unchanged_cap_relative_candidate'})
    if tolerances is not None:
        request['endpoint_tolerances'] = dict(tolerances)
    # 静态几何实验允许在退开前读同一冻结盖姿态；实际流程必须退开后识别。
    return json.loads(json.dumps(request))


def validate_regrasp(source, request, report):
    """独立复核 MTC 三段路点；插值仅检查真实规划结果。"""
    if (report.get('mode') != 'mtc_geometric_approach_not_execution'
            or report.get('dense_fcl_check_passed') is not True or report['request'] != request
            or [s['name'] for s in report['stages']] != ['axial retreat', 'move to pregrasp', 'axial approach']):
        raise ValueError('重新接近结果不完整')
    limits = pose_tolerances(request)
    p_limit, r_limit = limits['position_m'], limits['orientation_rad']
    points = np.array([q for stage in report['stages'] for q in stage['points']])
    q0 = np.array([request['initial_joint_positions'][n] for n in request['arm_joints']])
    if (points.ndim != 2 or points.shape[1] != 6 or not np.isfinite(points).all()
            or not np.allclose(points[0], q0, atol=1e-9, rtol=0)
            or np.max(np.abs(np.diff(points, axis=0))) > .02000001):
        raise ValueError('重新接近起点或连续性无效')
    angle = request['regrasp']['cumulative_angle_deg']
    source.set_state(q0, angle)
    check_snapshot(request['planning_scene'], scene_snapshot(source.world, source.preview.model, source.preview.data))
    axis = np.array(request['outward_axis_world'])
    origin = body_transform(source.preview.data, 'base')[:3, 3].copy()
    start_orientation = body_transform(source.preview.data, 'base')[:3, :3].copy()
    target = transform(*pose_arrays({'position_m': request['hand_position'],
                                     'orientation_xyzw': request['hand_orientation_xyzw']}))
    rows = []
    for stage in report['stages']:
        projected = []
        for values in stage['points']:
            q = np.array(values)
            row = source.row(q, angle)
            actual = body_transform(source.preview.data, 'base')
            if row['minimum_joint_margin_rad'] < -1e-8 or any(c['penetration_m'] > .0001 for c in source.preview.contacts()):
                raise ValueError('重新接近 MuJoCo 复核检出碰撞或越界')
            if stage['name'] != 'move to pregrasp':
                base = origin if stage['name'] == 'axial retreat' else np.array(request['pregrasp_position'])
                delta = actual[:3, 3]-base
                along = float(delta @ axis)
                orientation = start_orientation if stage['name'] == 'axial retreat' else target[:3, :3]
                if (np.linalg.norm(delta-along*axis) > p_limit
                        or np.linalg.norm(rotation_error(orientation, actual[:3, :3])) > r_limit):
                    raise ValueError('轴向段偏离轴线或朝向')
                projected.append(along)
            row['phase'] = stage['name']
            rows.append(row)
        if projected:
            expected_end = .1 if stage['name'] == 'axial retreat' else -.1
            sign = 1 if expected_end > 0 else -1
            if (abs(projected[0]) > p_limit or abs(projected[-1]-expected_end) > p_limit
                    or np.any(sign*np.diff(projected) < -p_limit)
                    or np.any(sign*np.array(projected) < -p_limit)
                    or np.any(sign*np.array(projected) > .1+p_limit)):
                raise ValueError('轴向距离或方向不符')
    p, r = pose_error(body_transform(source.preview.data, 'base'), target)
    if p > p_limit or r > r_limit:
        raise ValueError('重新接近末态误差越界')
    # 退开后再次调用估计接口，保留实际流程所需的观测回执。
    source.set_state(np.array(report['stages'][0]['points'][-1]), angle)
    post_retreat = asyncio.run(source.preview.scene.estimator.estimate())
    if not post_retreat.valid or not np.allclose(transform(*pose_arrays(asdict(post_retreat.pose))),
                                                transform(*pose_arrays(request['cap_estimate']['pose'])), atol=1e-10, rtol=0):
        raise ValueError('退开后观测不一致')
    return rows, {'post_retreat_observation': asdict(post_retreat),
                  'endpoint_position_error_m': p, 'endpoint_orientation_error_rad': r,
                  'physical_release_verified': False, 'physical_regrasp_verified': False}


def launch(name, arguments, log):
    """一次调用 ROS 规划/审计；失败不重试，日志直接留档。"""
    command = ['bash', '-c', 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec "$@"',
               'segmented-geometry', 'ros2', 'launch', 'i10_rh56_moveit_config', name]
    command += [f'{key}:={Path(value).resolve()}' for key, value in arguments.items()]
    with Path(log).open('w') as stream:
        result = subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError(f'{name} 退出码 {result.returncode}，日志：{log}')


def run(trajectory, folder, *, resume_validation=False):
    """顺序运行三个分段和两次重新接近；任何首个失败立即结束。"""
    folder = Path(folder)
    existing = None
    if (folder/'summary.json').exists():
        existing = json.loads((folder/'summary.json').read_text())
        if (not resume_validation or existing.get('failure', {}).get('phase') != 'regrasp_1'
                or existing['failure'].get('detail') != '重新接近结果不完整'):
            raise ValueError('已有实验结果，不自动覆盖或续跑；请使用独立目录')
        # 仅允许修复格式复核后继续原轨迹，不能借此重试路径或物理失败。
        for relative, expected in existing['artifacts_sha256'].items():
            if digest(ROOT/relative) != expected:
                raise ValueError('原阶段产物已变更，禁止恢复复核')
        save(folder/'validation_failure_before_fix.json', existing)
    folder.mkdir(parents=True, exist_ok=True)
    source = RotationDiagnostic(trajectory)
    q, begin = source.q0.copy(), 0.
    summary = {'mode': 'segmented_fixed_depth_preform_geometry_not_execution',
               'segments_deg': [90., 90., 45.], 'target_angle_deg': 225.,
               'completed_rotation_segments': 0, 'completed_regrasp_paths': 0,
               'cumulative_angle_deg': 0., 'failure': None, 'analysis_finished': False,
               'physical_release_verified': False, 'physical_grasp_verified': False,
               'thread_motion_verified': False, 'cap_removed': False,
               'trajectory_path': str(source.path.resolve().relative_to(ROOT)),
               'trajectory_sha256': digest(source.path), 'artifacts_sha256': {}, 'samples': []}
    tolerances = pose_tolerances(existing) if existing else dict(CURRENT_POSE_TOLERANCES)
    if not existing:
        summary['endpoint_tolerances'] = tolerances
    def checkpoint():
        save(folder/'summary.json', summary)
    def bind(path):
        resolved = path.resolve()
        key = str(resolved.relative_to(ROOT)) if resolved.is_relative_to(ROOT) else str(resolved)
        summary['artifacts_sha256'][key] = digest(path)
    if existing:
        summary['validation_correction'] = '首次格式比较误报；使用原保存 MTC 轨迹复核，没有重新规划'
    checkpoint()
    phase = 'initial'
    try:
        for i, delta in enumerate(summary['segments_deg'], 1):
            phase = f'rotation_{i}'
            path = folder/f'{phase}.json'
            diagnostic = json.loads(path.read_text()) if existing and i == 1 else rotate(source, q, begin, begin+delta, path, tolerances)
            bind(path)
            audit = folder/f'{phase}_fcl.json'
            if not (existing and i == 1):
                launch('rotation_audit.launch.py', {'diagnostic_path': path, 'output_path': audit}, folder/f'{phase}_fcl.log')
            fcl = json.loads(audit.read_text())
            bind(audit)
            if fcl.get('diagnostic_sha256') != digest(path) or not fcl.get('completed_input_check'):
                summary['failure'] = {'phase': phase, 'kind': 'fcl_failure', 'detail': fcl.get('failure')}
                break
            summary['samples'].extend(diagnostic['samples'])
            if not diagnostic['samples']:
                summary['failure'] = {'phase': phase, 'kind': 'empty_checked_prefix', 'detail': diagnostic['failure']}
                break
            summary['cumulative_angle_deg'] = diagnostic['samples'][-1]['angle_deg']
            if diagnostic['failure']:
                summary['failure'] = {'phase': phase, **diagnostic['failure']}
                break
            q = np.array(diagnostic['samples'][-1]['arm_qpos_rad'])
            begin += delta
            summary['completed_rotation_segments'] += 1
            checkpoint()
            if i == len(summary['segments_deg']):
                break
            phase = f'regrasp_{i}'
            request = regrasp_request(source, q, begin, None if existing else tolerances)
            reqpath, outpath = folder/f'{phase}_request.json', folder/f'{phase}_mtc.json'
            if existing and i == 1:
                if json.loads(reqpath.read_text()) != request:
                    raise ValueError('恢复复核时请求已变更')
            else:
                save(reqpath, request)
            bind(reqpath)
            if not (existing and i == 1):
                launch('approach.launch.py', {'request_path': reqpath, 'output_path': outpath}, folder/f'{phase}_mtc.log')
            report = json.loads(outpath.read_text())
            bind(outpath)
            rows, validation = validate_regrasp(source, request, report)
            validation_path = folder/f'{phase}_validation.json'
            save(validation_path, validation)
            bind(validation_path)
            summary['samples'].extend(rows)
            summary['completed_regrasp_paths'] += 1
            q = np.array(rows[-1]['arm_qpos_rad'])
            checkpoint()
    except (ValueError, RuntimeError) as error:
        # 失败输出与日志也绑定，不能只记录成功资产。
        for artifact in folder.glob(f'{phase}*'):
            if artifact.is_file() and artifact.suffix in ('.json', '.log'):
                bind(artifact)
        summary['failure'] = {'phase': phase, 'kind': 'stage_failed', 'detail': str(error)}
    summary['analysis_finished'] = True
    summary['full_geometric_sequence_passed'] = summary['failure'] is None and summary['completed_rotation_segments'] == 3
    checkpoint()
    return summary
