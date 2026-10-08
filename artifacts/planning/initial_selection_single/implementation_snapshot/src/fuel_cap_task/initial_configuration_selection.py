"""初次接近的有限构型筛选：检查候选、真实路径及实际末态旋转空间。"""
import copy
from dataclasses import asdict
import json
import os
from pathlib import Path

import mujoco
import numpy as np

from .arm_ik import body_transform, rotation_error
from .configuration_selection import (
    CandidateContext, JOINT_TOLERANCE_RAD, SCAN_COUNT, SEED_COUNT,
    check_joint_endpoint, diagnostic, ordered_candidates, unique_raw, winding_variants,
)
from .grasp_search import pose_arrays
from .interfaces import TaskParameters
from .regrasp_margin import checked_endpoint
from .rotation_diagnostic import RotationDiagnostic, RotationScanSettings, hand_target, joint_margins, solve_continuous
from .segmented_rotation import digest, launch, pose_error, save
from .trajectory_replay import ApproachReplay, ROOT


class InitialContext:
    """以原接近轨迹恢复同一场景；0° 是观测初态，不能用绝对盖关节角代替。"""
    def __init__(self, trajectory):
        self.trajectory = Path(trajectory).resolve()
        if not self.trajectory.is_relative_to(ROOT):
            raise ValueError('输入轨迹必须位于工作区')
        source = RotationDiagnostic(self.trajectory)
        self.baseline_trajectory = self.trajectory
        self.before = source.q0.copy()
        self.after = self.before.copy()
        self.angle = 0.
        self.request = copy.deepcopy(source.replay.report['request'])
        if 'regrasp' in self.request or 'configuration_selection' in self.request:
            raise ValueError('需要原始两段接近轨迹，不能将重新抓握或已筛选结果作为新基线')
        self.names = self.request['arm_joints']
        self.initial_arm = np.asarray([self.request['initial_joint_positions'][name] for name in self.names])
        self.input_hashes = {**self.request['source_hashes'],
                             str(self.trajectory.relative_to(ROOT)): digest(self.trajectory)}

    def origin(self, side):
        return {'kind': 'initial_approach_baseline', 'side': side, 'cumulative_angle_deg': self.angle,
                'baseline_trajectory_path': str(self.baseline_trajectory.relative_to(ROOT)),
                'input_hashes': self.input_hashes}


class InitialCandidateContext(CandidateContext):
    """复用既有扫描，同时明确初始候选与实际新路径的来源。"""
    def __init__(self, base, q, *, candidate_id, bindings, role, trajectory=None):
        super().__init__(base, q, candidate_id=candidate_id, bindings=bindings, role=role)
        if trajectory is not None:
            path = Path(trajectory).resolve()
            if not path.is_relative_to(ROOT) or self.input_hashes.get(str(path.relative_to(ROOT))) != digest(path):
                raise ValueError('实际路径没有有效来源绑定')
            self.trajectory = path

    def origin(self, side):
        if side != 'after':
            raise ValueError('初始构型诊断只使用指定到达状态')
        return {'kind': 'selected_initial_configuration', 'side': side,
                'candidate_id': self.candidate_id, 'role': self.role, 'cumulative_angle_deg': self.angle,
                'baseline_trajectory_path': str(self.baseline_trajectory.relative_to(ROOT)),
                'diagnostic_trajectory_path': str(self.trajectory.relative_to(ROOT)),
                'input_hashes': self.input_hashes}


def source_bindings():
    """新增入口独立记录版本，保持旧扫描和重新抓握记录的来源不变。"""
    paths = ['src/fuel_cap_task/initial_configuration_selection.py', 'tools/select_initial_configuration.py',
             'src/fuel_cap_task/configuration_selection.py', 'src/fuel_cap_task/trajectory_replay.py',
             'src/i10_rh56_moveit_config/src/plan_selected_approach.cpp',
             'src/i10_rh56_moveit_config/launch/selected_approach.launch.py',
             'install/i10_rh56_moveit_config/lib/i10_rh56_moveit_config/plan_selected_approach',
             'install/i10_rh56_moveit_config/share/i10_rh56_moveit_config/launch/selected_approach.launch.py']
    return {name: digest(ROOT/name) for name in paths}


def prefilter(rows):
    """静态关节距离仅用于预算分配；最终选择依据真实旋转前缀和接近路径。"""
    baseline = [row for row in rows if row['labels'] == ['baseline']]
    if len(baseline) != 1:
        raise ValueError('必须保留唯一的原接近终点对照')
    others = sorted([row for row in rows if not row['labels'] and row['initial_geometry_passed']],
                    key=lambda row: (-row['minimum_joint_margin_rad'], row['distance_from_before_rad'], row['id']))
    return {row['id'] for row in baseline+others[:SCAN_COUNT-1]}


def generate_pool(context):
    """一次有限求解与单关节绕数扩展，保留初始几何拒绝和未收敛初值。"""
    source = RotationDiagnostic(context.trajectory)
    target = hand_target(source.cap0, source.relative, 0.)
    rows, attempts = [], []

    def add(q, origin, label=None):
        q = np.asarray(q, dtype=float)
        if not unique_raw(q, rows):
            return
        source.set_state(q, 0.)
        p, r = pose_error(body_transform(source.preview.data, 'base'), target)
        margin = joint_margins(q, source.preview.limits)['minimum_joint_margin_rad']
        contacts = source.preview.contacts()
        passed = margin >= 0 and p <= .0001 and r <= .002 and not any(c['penetration_m'] > .0001 for c in contacts)
        rows.append({'id': f'candidate_{len(rows):03d}', 'arm_qpos_rad': q.tolist(), 'origin': origin,
                     'labels': [label] if label else [], 'initial_geometry_passed': passed,
                     'position_error_m': p, 'orientation_error_rad': r, 'contacts': contacts,
                     'minimum_joint_margin_rad': margin,
                     'distance_from_before_rad': float(np.linalg.norm(q-context.before))})

    add(context.before, {'kind': 'original_approach_endpoint'}, 'baseline')
    seeds = [context.before, context.initial_arm, np.zeros(6)]
    rng = np.random.default_rng(0)
    seeds += list(rng.uniform(source.preview.limits[:, 0], source.preview.limits[:, 1], size=(SEED_COUNT-3, 6)))
    for index, seed in enumerate(seeds):
        source.set_state(context.before, 0.)
        q, error, ok = solve_continuous(source.preview, target, seed)
        attempts.append({'index': index, 'seed_q_rad': np.asarray(seed).tolist(), 'converged': bool(ok),
                         'candidate_q_rad': q.tolist(), 'position_error_m': float(np.linalg.norm(error[:3])),
                         'orientation_error_rad': float(np.linalg.norm(error[3:]))})
        if ok:
            add(q, {'kind': 'ik_seed', 'seed_index': index})
    for row in list(rows):
        for q, winding in winding_variants(row['arm_qpos_rad'], source.preview.limits):
            add(q, {'kind': 'single_joint_winding', 'parent_id': row['id'], **winding})
    selected = prefilter(rows)
    for row in rows:
        row['scan_selected'] = row['id'] in selected
        row['prefilter_reason'] = 'selected' if row['id'] in selected else (
            'initial_geometry_rejected' if not row['initial_geometry_passed'] else 'finite_scan_budget')
    return {'mode': 'finite_initial_configuration_pool_not_execution', 'origin': context.origin('before'),
            'seed_count': SEED_COUNT, 'random_seed': 0, 'scan_count_budget': SCAN_COUNT,
            'joint_limits_rad': source.preview.limits.tolist(), 'ik_attempts': attempts, 'candidates': rows}


def selected_request(context, candidate, bindings):
    """原仿真起点保持不变，候选外退 IK 只构造预抓取目标，不作为路径。"""
    source = RotationDiagnostic(context.trajectory)
    request = copy.deepcopy(context.request)
    target = hand_target(source.cap0, source.relative, 0.)
    q = np.asarray(candidate['arm_qpos_rad'], dtype=float)
    distance = request['approach_distance_m']
    if not np.isfinite(distance) or distance <= 0:
        raise ValueError('原接近距离必须有限且为正')
    rows = []
    for offset in np.linspace(0., distance, int(np.ceil(distance/.002))+1):
        wanted = target.copy()
        wanted[:3, 3] += offset*np.asarray(request['outward_axis_world'])
        source.set_state(q, 0.)
        values, error, ok = solve_continuous(source.preview, wanted, q)
        source.set_state(values, 0.)
        contacts = source.preview.contacts()
        rows.append({'distance_m': float(offset), 'arm_qpos_rad': values.tolist(), 'converged': bool(ok),
                     'position_error_m': float(np.linalg.norm(error[:3])),
                     'orientation_error_rad': float(np.linalg.norm(error[3:])), 'contacts': contacts})
        if (not ok or np.max(np.abs(values-q)) > .1
                or joint_margins(values, source.preview.limits)['minimum_joint_margin_rad'] < 0
                or any(c['penetration_m'] > .0001 for c in contacts)):
            return None, {'mode': 'initial_candidate_axial_ik_not_path', 'passed': False, 'samples': rows}
        q = values
    request['configuration_selection'] = {'role': 'initial_approach', 'candidate_id': candidate['id'],
        'grasp_q_rad': candidate['arm_qpos_rad'], 'pregrasp_q_rad': q.tolist(),
        'endpoint_joint_tolerance_rad': JOINT_TOLERANCE_RAD, 'input_hashes': bindings}
    return request, {'mode': 'initial_candidate_axial_ik_not_path', 'passed': True, 'samples': rows}


def validate_path(path, context, request, candidate):
    """复核完整两段真实轨迹、固定起点、原始关节构型以及完整轴向距离。"""
    replay = ApproachReplay(path)
    report = replay.report
    if (report['request'] != request or report.get('selected_configuration_check_passed') is not True
            or report.get('selected_pregrasp_configuration_check_passed') is not True
            or request['initial_joint_positions'] != context.request['initial_joint_positions']):
        raise ValueError('初始接近来源、构型检查或固定起点不一致')
    validation = replay.validate()
    selection = request['configuration_selection']
    actual = np.asarray(report['stages'][-1]['points'][-1])
    joint_error = check_joint_endpoint(actual, candidate['arm_qpos_rad'])
    pre_error = check_joint_endpoint(report['stages'][0]['points'][-1], selection['pregrasp_q_rad'])
    # 旧回放已检查直线侧偏；此处再核对完整距离、单调方向和沿途朝向。
    _, wanted_rotation = pose_arrays(
        {'position_m': request['hand_position'], 'orientation_xyzw': request['hand_orientation_xyzw']})
    axis, pre = np.asarray(request['outward_axis_world']), np.asarray(request['pregrasp_position'])
    positions = []
    for q in report['stages'][-1]['points']:
        replay.data.qpos[:] = replay.initial
        replay.data.qpos[replay.addresses] = q
        mujoco.mj_forward(replay.model, replay.data)
        pose = body_transform(replay.data, 'base')
        positions.append(float((pose[:3, 3]-pre) @ -axis))
        if np.linalg.norm(rotation_error(wanted_rotation, pose[:3, :3])) > .002:
            raise ValueError('接近阶段沿途朝向偏离原目标')
    distance = request['approach_distance_m']
    if (abs(positions[0]) > .0001 or abs(positions[-1]-distance) > .0001
            or np.any(np.diff(positions) < -.0001)
            or np.any(np.asarray(positions) < -.0001) or np.any(np.asarray(positions) > distance+.0001)):
        raise ValueError('接近未完成指定轴向距离或方向错误')
    if (not np.isclose(report['endpoint_joint_error_rad'], joint_error, atol=1e-12, rtol=0)
            or not np.isclose(report['pregrasp_joint_error_rad'], pre_error, atol=1e-12, rtol=0)):
        raise ValueError('原生规划器与独立构型误差不一致')
    validation.update(candidate_id=candidate['id'], endpoint_joint_error_rad=joint_error,
                      pregrasp_joint_error_rad=pre_error, physical_grasp_verified=False)
    return actual, validation


def actual_bindings(bindings, endpoint):
    """实际扫描绑定新请求、真实路点和独立验证，不复用候选诊断冒充结果。"""
    return {**bindings, **{str((endpoint/name).relative_to(ROOT)): digest(endpoint/name)
                           for name in ('request.json', 'mtc.json', 'validation.json')}}


def result_status(summary):
    if summary['selected']:
        return 'selected_initial_path_and_margin_verified'
    if summary['baseline_angle_deg'] is None:
        return 'baseline_invalid_no_comparison'
    return 'baseline_retained_no_verified_improvement'


def run(trajectory, output_dir):
    """有限离线搜索，每个候选一次规划，记录全部拒绝和未尝试项。"""
    folder = Path(output_dir).resolve()
    if folder.exists() or not folder.is_relative_to(ROOT):
        raise ValueError('输出必须是工作区内不存在的新目录，不覆盖或自动续跑')
    context = InitialContext(trajectory)
    settings = RotationScanSettings(TaskParameters().unlock_angle_deg)
    bindings = source_bindings()
    pool = generate_pool(context)
    save(folder/'candidate_pool.json', pool)
    bindings[str((folder/'candidate_pool.json').relative_to(ROOT))] = digest(folder/'candidate_pool.json')
    summary = {'mode': 'initial_configuration_selection_not_execution', 'analysis_finished': False,
               'origin': context.origin('before'), 'input_hashes': bindings, 'scan_settings': asdict(settings),
               'candidate_results': [], 'path_attempts': [], 'selected': None,
               'physical_grasp_verified': False, 'thread_motion_verified': False,
               'physical_release_verified': False, 'execution_stop_angle_deg': None,
               'execution_stopping_reserve_deg': None}
    save(folder/'checkpoint.json', summary)
    old_logs = os.environ.get('ROS_LOG_DIR')
    try:
        for candidate in pool['candidates']:
            if not candidate['scan_selected']:
                continue
            node = InitialCandidateContext(context, candidate['arm_qpos_rad'], candidate_id=candidate['id'],
                                           bindings=bindings, role='candidate')
            result = {**candidate, **diagnostic(node, folder/candidate['id']/'candidate', settings)}
            summary['candidate_results'].append(result)
            save(folder/'checkpoint.json', summary)
        baseline = next(row for row in summary['candidate_results'] if row['labels'] == ['baseline'])
        summary['baseline_angle_deg'] = baseline['checked_additional_angle_deg']
        eligible = ordered_candidates(summary['candidate_results'], summary['baseline_angle_deg'])
        for candidate in eligible:
            endpoint = folder/candidate['id']/'path'
            endpoint.mkdir()
            attempt = {'candidate_id': candidate['id'], 'accepted': False}
            summary['path_attempts'].append(attempt)
            request, axial = selected_request(context, candidate, bindings)
            save(endpoint/'axial_target_diagnostic.json', axial)
            if request is None:
                attempt['rejection'] = 'candidate_axial_ik_rejected'
                save(folder/'checkpoint.json', summary)
                continue
            save(endpoint/'request.json', request)
            os.environ['ROS_LOG_DIR'] = str(endpoint/'ros_logs')
            try:
                launch('selected_approach.launch.py', {'request_path': endpoint/'request.json',
                       'output_path': endpoint/'mtc.json'}, endpoint/'mtc.log')
            except RuntimeError:
                report = json.loads((endpoint/'mtc.json').read_text()) if (endpoint/'mtc.json').exists() else {}
                if report.get('mode') not in ('mtc_endpoint_rejected_no_replay', 'mtc_selected_approach_planning_rejected_no_replay'):
                    raise
                attempt['rejection'] = report['mode']
                save(folder/'checkpoint.json', summary)
                continue
            try:
                actual, validation = validate_path(endpoint/'mtc.json', context, request, candidate)
            except ValueError as error:
                attempt['rejection'] = str(error)
                save(folder/'checkpoint.json', summary)
                continue
            save(endpoint/'validation.json', validation)
            node = InitialCandidateContext(context, actual, candidate_id=candidate['id'],
                bindings=actual_bindings(bindings, endpoint), role='actual_mtc_endpoint', trajectory=endpoint/'mtc.json')
            result = diagnostic(node, folder/candidate['id']/'actual_endpoint', settings)
            attempt['actual_endpoint'] = result
            angle = result['checked_additional_angle_deg']
            if angle is None or angle <= summary['baseline_angle_deg']:
                attempt['rejection'] = 'actual_endpoint_did_not_increase_checked_range'
                save(folder/'checkpoint.json', summary)
                continue
            attempt['accepted'] = True
            summary['selected'] = {'candidate_id': candidate['id'], 'trajectory_path': str((endpoint/'mtc.json').relative_to(ROOT)),
                'actual_arm_qpos_rad': actual.tolist(), 'endpoint_joint_error_rad': validation['endpoint_joint_error_rad'],
                'checked_angle_deg': angle, 'checked_range_change_deg': angle-summary['baseline_angle_deg']}
            break
        summary['analysis_finished'] = True
        summary['status'] = result_status(summary)
        tried = {row['candidate_id'] for row in summary['path_attempts']}
        summary['unattempted_improving_candidate_ids'] = [row['id'] for row in eligible if row['id'] not in tried]
        summary['artifacts_sha256'] = {str(p.relative_to(ROOT)): digest(p) for p in folder.rglob('*')
                                     if p.is_file() and p.name != 'checkpoint.json'}
        save(folder/'summary.json', summary)
        save(folder/'checkpoint.json', summary)
        write_report(folder, summary)
        return summary
    finally:
        if old_logs is None:
            os.environ.pop('ROS_LOG_DIR', None)
        else:
            os.environ['ROS_LOG_DIR'] = old_logs


def write_report(folder, summary):
    """展示原终点、候选拒绝及实际路径末态的共同通过转角。"""
    lines = ['# 初始到达构型筛选', '', '同一观测初态、起点和预型；固定深度，累计盖角从 0° 开始。', '',
             '| 候选 | 对照 | 共同通过盖角 ° | FCL 通过状态数 | 首个拒绝 |', '| --- | --- | ---: | ---: | --- |']
    for row in summary['candidate_results']:
        reason = row['failure']['kind'] if row.get('failure') else '扫描预算通过，最大范围未知'
        lines.append(f"| {row['id']} | {','.join(row['labels'])} | {row['checked_additional_angle_deg']} | {row['passed_samples']} | {reason} |")
    lines += ['', f"最终状态：{summary['status']}。"]
    if summary['selected']:
        selected = summary['selected']
        lines += [f"所选 {selected['candidate_id']} 的真实 MTC 路径到达后，共同通过 {selected['checked_angle_deg']}°，"
                  f"比原接近终点增加 {selected['checked_range_change_deg']}°。",
                  f"真实路径：`{selected['trajectory_path']}`。"]
    else:
        lines += ['有限搜索没有取得可用的改进结果；原路径保留，不替换为未通过候选。']
    lines += ['', '## 路径尝试', '']
    for row in summary['path_attempts']:
        lines.append(f"- {row['candidate_id']}：{'通过' if row['accepted'] else row['rejection']}")
    lines += ['', '候选绕数是待规划目标，不能直接改写实际状态；有限搜索不证明全局最优。',
              '几何通过不代表抓稳、允许松手、轴向柔顺、脱离或动力学执行；停止裕量未知。', '']
    (folder/'report.md').write_text('\n'.join(lines))


def validate_pool(context, pool):
    """只复核已保存初值、候选来源与预筛，不再运行数值求解。"""
    source = RotationDiagnostic(context.trajectory)
    if (pool.get('mode') != 'finite_initial_configuration_pool_not_execution'
            or pool['origin'] != context.origin('before') or pool['seed_count'] != SEED_COUNT
            or pool['scan_count_budget'] != SCAN_COUNT or pool['random_seed'] != 0
            or not np.array_equal(pool['joint_limits_rad'], source.preview.limits)
            or len(pool['ik_attempts']) != SEED_COUNT):
        raise ValueError('初始候选池来源或预算不一致')
    rng = np.random.default_rng(0)
    seeds = [context.before, context.initial_arm, np.zeros(6)]
    seeds += list(rng.uniform(source.preview.limits[:, 0], source.preview.limits[:, 1], size=(SEED_COUNT-3, 6)))
    for index, (attempt, seed) in enumerate(zip(pool['ik_attempts'], seeds)):
        q = np.asarray(attempt['candidate_q_rad'])
        if (attempt['index'] != index or not np.array_equal(attempt['seed_q_rad'], seed)
                or type(attempt['converged']) is not bool or q.shape != (6,) or not np.isfinite(q).all()):
            raise ValueError('IK 初值或求解记录无效')
    target = hand_target(source.cap0, source.relative, 0.)
    by_id, prior = {}, []
    for index, row in enumerate(pool['candidates']):
        q = np.asarray(row['arm_qpos_rad'])
        if row['id'] != f'candidate_{index:03d}' or not unique_raw(q, prior):
            raise ValueError('候选编号或原始角去重不一致')
        origin = row['origin']
        if origin['kind'] == 'original_approach_endpoint':
            if index != 0 or row['labels'] != ['baseline'] or not np.array_equal(q, context.before):
                raise ValueError('基线不是原接近终点')
        elif origin['kind'] == 'ik_seed':
            attempt = pool['ik_attempts'][origin['seed_index']]
            if row['labels'] or not attempt['converged'] or not np.array_equal(q, attempt['candidate_q_rad']):
                raise ValueError('候选与收敛初值记录不一致')
        elif origin['kind'] == 'single_joint_winding':
            parent = by_id[origin['parent_id']]
            wanted = np.asarray(parent['arm_qpos_rad']).copy()
            if row['labels'] or origin['joint_index'] not in range(6) or origin['turns'] not in (-1, 1):
                raise ValueError('单关节绕数来源无效')
            wanted[origin['joint_index']] += origin['turns']*2*np.pi
            if not np.array_equal(q, wanted):
                raise ValueError('绕数候选不等于所记录的单关节整圈变化')
        else:
            raise ValueError('未知候选来源')
        source.set_state(q, 0.)
        p, r = pose_error(body_transform(source.preview.data, 'base'), target)
        margin = joint_margins(q, source.preview.limits)['minimum_joint_margin_rad']
        contacts = source.preview.contacts()
        passed = margin >= 0 and p <= .0001 and r <= .002 and not any(c['penetration_m'] > .0001 for c in contacts)
        for key, actual in {'position_error_m': p, 'orientation_error_rad': r, 'minimum_joint_margin_rad': margin,
                            'distance_from_before_rad': float(np.linalg.norm(q-context.before))}.items():
            if not np.isclose(row[key], actual, atol=1e-12, rtol=0):
                raise ValueError('候选位姿或静态关节距离不一致')
        if row['initial_geometry_passed'] is not passed or row['contacts'] != contacts:
            raise ValueError('候选初始几何复核不一致')
        by_id[row['id']] = row
        prior.append(row)
    ids = prefilter(pool['candidates'])
    for row in pool['candidates']:
        reason = 'selected' if row['id'] in ids else (
            'initial_geometry_rejected' if not row['initial_geometry_passed'] else 'finite_scan_budget')
        if row['scan_selected'] is not (row['id'] in ids) or row['prefilter_reason'] != reason:
            raise ValueError('预筛结果不符合有限预算')
    return by_id


def checked_candidate(endpoint, context, settings):
    """复核共同前缀，空前缀只表达起点拒绝，不能作为可靠 0° 余量。"""
    report = json.loads((endpoint/'diagnostic.json').read_text())
    if report['samples']:
        result, _ = checked_endpoint(endpoint, context, 'after')
        result['fcl_audit_performed'] = True
    else:
        if (report.get('mode') != 'fixed_depth_preform_rotation_diagnostic_not_execution'
                or report.get('analysis_finished') is not True or report['origin'] != context.origin('after')
                or (report.get('failure') or {}).get('angle_deg') != 0.
                or report['trajectory_sha256'] != digest(context.trajectory)
                or report['scan_settings'] != asdict(settings) or (endpoint/'fcl_validation.json').exists()):
            raise ValueError('没有有效的起点拒绝记录')
        result = {'checked_additional_angle_deg': None, 'passed_samples': 0,
                  'failure': report['failure'], 'fcl_audit_performed': False}
    if report['scan_settings'] != asdict(settings):
        raise ValueError('候选与基线扫描预算不同')
    return result


def verify(output_dir):
    """只重建模型、复核保存来源和路点，不重新扫描、求解或规划。"""
    folder = Path(output_dir).resolve()
    summary = json.loads((folder/'summary.json').read_text())
    if summary.get('mode') != 'initial_configuration_selection_not_execution' or summary.get('analysis_finished') is not True:
        raise ValueError('初始构型筛选未完成')
    context = InitialContext(ROOT/summary['origin']['baseline_trajectory_path'])
    settings = RotationScanSettings(TaskParameters().unlock_angle_deg)
    pool_path = folder/'candidate_pool.json'
    bindings = {**source_bindings(), str(pool_path.relative_to(ROOT)): digest(pool_path)}
    if (summary['origin'] != context.origin('before') or summary['input_hashes'] != bindings
            or summary['scan_settings'] != asdict(settings)):
        raise ValueError('筛选输入、算法、规划器或预算已变更')
    for relative, expected in summary['artifacts_sha256'].items():
        path = (ROOT/relative).resolve()
        if not path.is_relative_to(folder) or digest(path) != expected:
            raise ValueError(f'初始筛选产物已变更：{relative}')
    pool = json.loads(pool_path.read_text())
    by_id = validate_pool(context, pool)
    expected_ids = [row['id'] for row in pool['candidates'] if row['scan_selected']]
    if [row['id'] for row in summary['candidate_results']] != expected_ids:
        raise ValueError('候选结果遗漏或排序变更')
    for item in summary['candidate_results']:
        candidate = by_id[item['id']]
        if any(item[key] != value for key, value in candidate.items()):
            raise ValueError('结果的候选原始关节角或来源不一致')
        node = InitialCandidateContext(context, candidate['arm_qpos_rad'], candidate_id=candidate['id'],
                                       bindings=bindings, role='candidate')
        checked = checked_candidate(folder/candidate['id']/'candidate', node, settings)
        if any(item[key] != value for key, value in checked.items()):
            raise ValueError('候选共同通过范围与独立复核不一致')
    baseline = next(row for row in summary['candidate_results'] if row['labels'] == ['baseline'])
    if baseline['checked_additional_angle_deg'] != summary['baseline_angle_deg']:
        raise ValueError('原接近终点的基线范围不一致')
    eligible = ordered_candidates(summary['candidate_results'], summary['baseline_angle_deg'])
    attempts = summary['path_attempts']
    if [row['candidate_id'] for row in attempts] != [row['id'] for row in eligible[:len(attempts)]]:
        raise ValueError('路径尝试遗漏、重复或不符合候选排序')
    accepted = [row for row in attempts if row['accepted']]
    if (bool(accepted) != bool(summary['selected']) or len(accepted) > 1
            or (accepted and attempts[-1] != accepted[0]) or (not accepted and len(attempts) != len(eligible))):
        raise ValueError('接受记录或有限路径尝试未完成')
    for attempt in attempts:
        candidate = by_id[attempt['candidate_id']]
        endpoint = folder/candidate['id']/'path'
        axial = json.loads((endpoint/'axial_target_diagnostic.json').read_text())
        if not axial['passed']:
            if attempt.get('rejection') != 'candidate_axial_ik_rejected' or (endpoint/'mtc.json').exists():
                raise ValueError('外退 IK 拒绝后仍产生路径或拒绝标记不一致')
            continue
        request = json.loads((endpoint/'request.json').read_text())
        expected_request = copy.deepcopy(context.request)
        expected_request['configuration_selection'] = {'role': 'initial_approach', 'candidate_id': candidate['id'],
            'grasp_q_rad': candidate['arm_qpos_rad'], 'pregrasp_q_rad': axial['samples'][-1]['arm_qpos_rad'],
            'endpoint_joint_tolerance_rad': JOINT_TOLERANCE_RAD, 'input_hashes': bindings}
        if request != expected_request:
            raise ValueError('真实接近请求改变了起点、场景、目标或构型')
        mtc = json.loads((endpoint/'mtc.json').read_text())
        if mtc['mode'] in ('mtc_endpoint_rejected_no_replay', 'mtc_selected_approach_planning_rejected_no_replay'):
            if attempt.get('rejection') != mtc['mode'] or attempt['accepted']:
                raise ValueError('明确规划拒绝被误记为接受')
            continue
        try:
            actual, validation = validate_path(endpoint/'mtc.json', context, request, candidate)
        except ValueError as error:
            if attempt.get('rejection') != str(error) or attempt['accepted']:
                raise ValueError('路径复核拒绝与保存结果不一致')
            continue
        saved = json.loads((endpoint/'validation.json').read_text())
        for key, value in validation.items():
            # MuJoCo FK 允许舍入误差；其余字段严格一致。
            if isinstance(value, float):
                if not np.isclose(saved[key], value, atol=1e-12, rtol=0):
                    raise ValueError('路径独立验证记录不一致')
            elif saved[key] != value:
                raise ValueError('路径独立验证记录不一致')
        node = InitialCandidateContext(context, actual, candidate_id=candidate['id'],
            bindings=actual_bindings(bindings, endpoint), role='actual_mtc_endpoint', trajectory=endpoint/'mtc.json')
        checked = checked_candidate(folder/candidate['id']/'actual_endpoint', node, settings)
        if attempt['actual_endpoint'] != checked:
            raise ValueError('实际到达后的诊断记录不一致')
        angle = checked['checked_additional_angle_deg']
        improved = angle is not None and angle > summary['baseline_angle_deg']
        if attempt['accepted'] is not improved:
            raise ValueError('实际路径末态没有增加范围却被接受，或遗漏可接受结果')
        if improved:
            expected = {'candidate_id': candidate['id'], 'trajectory_path': str((endpoint/'mtc.json').relative_to(ROOT)),
                'actual_arm_qpos_rad': actual.tolist(), 'endpoint_joint_error_rad': validation['endpoint_joint_error_rad'],
                'checked_angle_deg': angle, 'checked_range_change_deg': angle-summary['baseline_angle_deg']}
            if summary['selected'] != expected:
                raise ValueError('最终所选构型或实际末态范围不一致')
        elif attempt.get('rejection') != 'actual_endpoint_did_not_increase_checked_range':
            raise ValueError('实际末态拒绝原因不一致')
    tried = {row['candidate_id'] for row in attempts}
    if (summary['unattempted_improving_candidate_ids'] != [row['id'] for row in eligible if row['id'] not in tried]
            or summary['status'] != result_status(summary)):
        raise ValueError('未尝试项或最终状态不一致')
    if any(summary[key] is not False for key in ('physical_grasp_verified', 'thread_motion_verified', 'physical_release_verified')):
        raise ValueError('几何筛选不能记录为物理验收')
    return summary
