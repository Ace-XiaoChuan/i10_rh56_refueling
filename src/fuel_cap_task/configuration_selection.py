"""有限离线筛选重新抓握构型，并复核真实 MTC 到达后的旋转空间。"""
import copy
from dataclasses import asdict
import json
import os
from pathlib import Path

import numpy as np

from .arm_ik import body_transform
from .interfaces import TaskParameters
from .regrasp_margin import RegraspContext, EndpointRotationDiagnostic, checked_endpoint
from .rotation_diagnostic import RotationDiagnostic, RotationScanSettings, hand_target, joint_margins, solve_continuous
from .segmented_rotation import digest, launch, pose_error, pose_tolerances, regrasp_request, save, validate_regrasp
from .trajectory_replay import ROOT

SEED_COUNT = 24
SCAN_COUNT = 12
JOINT_TOLERANCE_RAD = .01


def winding_variants(q, limits):
    """生成单关节整圈候选；这是待规划目标，不能直接替换实际状态。"""
    q, limits = np.asarray(q, dtype=float), np.asarray(limits, dtype=float)
    joint_margins(q, limits)
    for index in range(6):
        for turn in (-1, 1):
            candidate = q.copy()
            candidate[index] += turn*2*np.pi
            if np.all(candidate >= limits[:, 0]) and np.all(candidate <= limits[:, 1]):
                yield candidate, {'joint_index': index, 'turns': turn}


def unique_raw(q, rows):
    """按原始角去重；q 与 q+2π 不是同一个可执行构型。"""
    return not any(np.linalg.norm(q-np.asarray(row['arm_qpos_rad'])) < .01 for row in rows)


def ordered_candidates(rows, baseline_angle):
    """只让完整覆盖下确实增加追加范围的候选进入路径尝试。"""
    if baseline_angle is None:
        return []
    eligible = [r for r in rows if r['checked_additional_angle_deg'] is not None
                and r['checked_additional_angle_deg'] > baseline_angle]
    return sorted(eligible, key=lambda r: (-r['checked_additional_angle_deg'],
                                         -r['minimum_joint_margin_rad'], r['distance_from_before_rad'], r['id']))


def check_joint_endpoint(actual, selected, tolerance=JOINT_TOLERANCE_RAD):
    """逐关节检查原始角，禁止用取模掩盖绕数变化。"""
    actual, selected = np.asarray(actual, dtype=float), np.asarray(selected, dtype=float)
    if (actual.shape != (6,) or selected.shape != (6,) or not np.isfinite(actual).all()
            or not np.isfinite(selected).all() or not np.isfinite(tolerance) or tolerance <= 0):
        raise ValueError('构型一致性检查需要有限六关节角与正阈值')
    error = float(np.max(np.abs(actual-selected)))
    if error > tolerance:
        raise ValueError(f'MTC 实际末态改变了所选构型或绕数：最大差 {error} rad')
    return error


class CandidateContext:
    """保留原成功节点来源，给独立候选或真实路径末态设置诊断起点。"""
    def __init__(self, base, q, *, candidate_id, bindings, role):
        self.__dict__.update(base.__dict__)
        self.after = np.asarray(q, dtype=float).copy()
        if self.after.shape != (6,) or not np.isfinite(self.after).all():
            raise ValueError('候选关节无效')
        self.input_hashes = {**base.input_hashes, **bindings}
        for name, expected in self.input_hashes.items():
            path = (ROOT/name).resolve()
            if not path.is_relative_to(ROOT) or digest(path) != expected:
                raise ValueError(f'候选来源变更：{name}')
        self.candidate_id, self.role = candidate_id, role

    def origin(self, side):
        if side != 'after':
            raise ValueError('候选诊断只使用指定构型')
        return {'kind': 'selected_regrasp_configuration', 'side': side,
                'segment_summary_path': str(self.summary_path.relative_to(ROOT)),
                'candidate_id': self.candidate_id, 'role': self.role,
                'cumulative_angle_deg': self.angle, 'input_hashes': self.input_hashes}


def source_bindings():
    """记录新增算法和真实规划器版本；不修改旧诊断算法以迁就新结果。"""
    paths = ['src/fuel_cap_task/configuration_selection.py',
             'src/i10_rh56_moveit_config/src/plan_selected_regrasp.cpp',
             'src/i10_rh56_moveit_config/launch/selected_regrasp.launch.py',
             'install/i10_rh56_moveit_config/lib/i10_rh56_moveit_config/plan_selected_regrasp',
             'install/i10_rh56_moveit_config/share/i10_rh56_moveit_config/launch/selected_regrasp.launch.py']
    return {name: digest(ROOT/name) for name in paths}


def generate_pool(context):
    """每个初值求解一次，保留失败、有限预筛和未入选候选。"""
    source = RotationDiagnostic(context.trajectory)
    source.set_state(context.before, context.angle)
    target = hand_target(source.cap0, source.relative, np.deg2rad(context.angle))
    rows, attempts = [], []

    def add(q, origin, label=None):
        q = np.asarray(q, dtype=float)
        if not unique_raw(q, rows):
            if label:
                next(r for r in rows if np.linalg.norm(q-np.asarray(r['arm_qpos_rad'])) < .01)['labels'].append(label)
            return
        source.set_state(q, context.angle)
        p, r = pose_error(body_transform(source.preview.data, 'base'), target)
        margin = joint_margins(q, source.preview.limits)['minimum_joint_margin_rad']
        contacts = source.preview.contacts()
        valid = margin >= 0 and p <= .0001 and r <= .002 and not any(c['penetration_m'] > .0001 for c in contacts)
        rows.append({'id': f'candidate_{len(rows):03d}', 'arm_qpos_rad': q.tolist(), 'origin': origin,
                     'labels': [label] if label else [], 'initial_geometry_passed': valid,
                     'position_error_m': p, 'orientation_error_rad': r, 'contacts': contacts,
                     'minimum_joint_margin_rad': margin,
                     'distance_from_before_rad': float(np.linalg.norm(q-context.before))})

    # 原前后节点强制纳入对照；失败也保留，不能筛掉后伪称基线通过。
    add(context.before, {'kind': 'known_before'}, 'before')
    add(context.after, {'kind': 'known_after'}, 'old_after')
    rng = np.random.default_rng(0)
    seeds = [context.before, context.after, np.zeros(6)]
    seeds += list(rng.uniform(source.preview.limits[:, 0], source.preview.limits[:, 1], size=(SEED_COUNT-3, 6)))
    for index, seed in enumerate(seeds):
        source.set_state(context.before, context.angle)
        q, error, converged = solve_continuous(source.preview, target, seed)
        attempts.append({'index': index, 'seed_q_rad': np.asarray(seed).tolist(), 'converged': bool(converged),
                         'candidate_q_rad': q.tolist(), 'position_error_m': float(np.linalg.norm(error[:3])),
                         'orientation_error_rad': float(np.linalg.norm(error[3:]))})
        if converged:
            add(q, {'kind': 'ik_seed', 'seed_index': index})
    # 不递归扩展整圈，不枚举所有整圈组合；变换后的 FK 和限位仍独立检查。
    for row in list(rows):
        for q, winding in winding_variants(row['arm_qpos_rad'], source.preview.limits):
            add(q, {'kind': 'single_joint_winding', 'parent_id': row['id'], **winding})
    controls = [row for row in rows if row['labels']]
    remaining = sorted([row for row in rows if not row['labels'] and row['initial_geometry_passed']],
                       key=lambda row: (-row['minimum_joint_margin_rad'], row['distance_from_before_rad'], row['id']))
    chosen = controls+remaining[:SCAN_COUNT-len(controls)]
    ids = {row['id'] for row in chosen}
    for row in rows:
        row['scan_selected'] = row['id'] in ids
        row['prefilter_reason'] = 'selected' if row['id'] in ids else (
            'initial_geometry_rejected' if not row['initial_geometry_passed'] else 'finite_scan_budget')
    return {'mode': 'finite_regrasp_candidate_pool_not_execution', 'origin': context.origin('before'),
            'seed_count': SEED_COUNT, 'random_seed': 0, 'scan_count_budget': SCAN_COUNT,
            'joint_limits_rad': source.preview.limits.tolist(), 'ik_attempts': attempts, 'candidates': rows}


def diagnostic(context, endpoint, settings):
    """一次连续扫描和一次真实 FCL 审计；启动故障直接上抛。"""
    endpoint.mkdir(parents=True)
    report = EndpointRotationDiagnostic(context, 'after').run(endpoint/'diagnostic.json', settings=settings)
    if not report['samples']:
        return {'checked_additional_angle_deg': None, 'passed_samples': 0,
                'failure': report['failure'], 'fcl_audit_performed': False}
    os.environ['ROS_LOG_DIR'] = str(endpoint/'ros_logs')
    launch('rotation_audit.launch.py', {'diagnostic_path': endpoint/'diagnostic.json',
           'output_path': endpoint/'fcl_validation.json'}, endpoint/'fcl_audit.log')
    result, _ = checked_endpoint(endpoint, context, 'after')
    result['fcl_audit_performed'] = True
    return result


def selected_request(context, candidate):
    """从候选沿轴外退求预抓取目标；不是用插值代替真实规划。"""
    source = RotationDiagnostic(context.trajectory)
    tolerances = pose_tolerances(json.loads(context.summary_path.read_text()))
    request = regrasp_request(source, context.before, context.angle, tolerances)
    target = hand_target(source.cap0, source.relative, np.deg2rad(context.angle))
    q = np.asarray(candidate['arm_qpos_rad'], dtype=float)
    steps = int(np.ceil(request['approach_distance_m']/.002))
    rows = []
    for distance in np.linspace(0., request['approach_distance_m'], steps+1):
        wanted = target.copy()
        wanted[:3, 3] += distance*np.asarray(request['outward_axis_world'])
        source.set_state(q, context.angle)
        values, error, ok = solve_continuous(source.preview, wanted, q)
        row = {'distance_m': float(distance), 'arm_qpos_rad': values.tolist(),
               'position_error_m': float(np.linalg.norm(error[:3])),
               'orientation_error_rad': float(np.linalg.norm(error[3:])), 'converged': bool(ok)}
        source.set_state(values, context.angle)
        row['contacts'] = source.preview.contacts()
        rows.append(row)
        if (not ok or np.max(np.abs(values-q)) > .1
                or joint_margins(values, source.preview.limits)['minimum_joint_margin_rad'] < 0
                or any(c['penetration_m'] > .0001 for c in row['contacts'])):
            return None, {'mode': 'candidate_axial_ik_diagnostic_not_path', 'passed': False, 'samples': rows}
        q = values
    request['configuration_selection'] = {'candidate_id': candidate['id'],
        'grasp_q_rad': candidate['arm_qpos_rad'], 'pregrasp_q_rad': q.tolist(),
        'endpoint_joint_tolerance_rad': JOINT_TOLERANCE_RAD}
    return request, {'mode': 'candidate_axial_ik_diagnostic_not_path', 'passed': True, 'samples': rows}


def run(segment_dir, output_dir):
    """有限候选全部留档，每个候选最多一次路径尝试，成功后停止离线搜索。"""
    folder = Path(output_dir).resolve()
    if folder.exists() or not folder.is_relative_to(ROOT):
        raise ValueError('输出必须是工作区内不存在的新目录，不覆盖或自动续跑')
    context = RegraspContext(segment_dir)
    settings = RotationScanSettings(TaskParameters().unlock_angle_deg-context.angle)
    bindings = source_bindings()
    pool = generate_pool(context)
    save(folder/'candidate_pool.json', pool)
    bindings[str((folder/'candidate_pool.json').relative_to(ROOT))] = digest(folder/'candidate_pool.json')
    summary = {'mode': 'regrasp_configuration_selection_not_execution', 'analysis_finished': False,
        'origin': context.origin('before'), 'input_hashes': bindings, 'scan_settings': asdict(settings),
        'candidate_results': [], 'path_attempts': [], 'selected': None,
        'physical_release_verified': False, 'physical_regrasp_verified': False,
        'thread_motion_verified': False, 'execution_stop_angle_deg': None, 'execution_stopping_reserve_deg': None}
    save(folder/'checkpoint.json', summary)
    old_log_dir = os.environ.get('ROS_LOG_DIR')
    try:
        for candidate in pool['candidates']:
            if not candidate['scan_selected']:
                continue
            item = copy.deepcopy(candidate)
            node = CandidateContext(context, candidate['arm_qpos_rad'], candidate_id=candidate['id'],
                                    bindings=bindings, role='candidate')
            item.update(diagnostic(node, folder/candidate['id']/'candidate', settings))
            summary['candidate_results'].append(item)
            save(folder/'checkpoint.json', summary)
        baseline = next(row for row in summary['candidate_results'] if 'before' in row['labels'])
        summary['baseline_additional_angle_deg'] = baseline['checked_additional_angle_deg']
        for candidate in ordered_candidates(summary['candidate_results'], baseline['checked_additional_angle_deg']):
            endpoint = folder/candidate['id']/'path'
            endpoint.mkdir()
            attempt = {'candidate_id': candidate['id'], 'accepted': False}
            summary['path_attempts'].append(attempt)
            request, axial = selected_request(context, candidate)
            save(endpoint/'axial_target_diagnostic.json', axial)
            if request is None:
                attempt['rejection'] = 'candidate_axial_ik_rejected'
                save(folder/'checkpoint.json', summary)
                continue
            request['configuration_selection']['input_hashes'] = bindings
            save(endpoint/'request.json', request)
            os.environ['ROS_LOG_DIR'] = str(endpoint/'ros_logs')
            try:
                launch('selected_regrasp.launch.py', {'request_path': endpoint/'request.json',
                       'output_path': endpoint/'mtc.json'}, endpoint/'mtc.log')
            except RuntimeError:
                # 只将原生规划器的明确拒绝计为候选失败，ROS 启动故障不冒充规划失败。
                saved = json.loads((endpoint/'mtc.json').read_text()) if (endpoint/'mtc.json').exists() else {}
                if saved.get('mode') not in ('mtc_endpoint_rejected_no_replay', 'mtc_selected_regrasp_planning_rejected_no_replay'):
                    raise
                attempt['rejection'] = saved['mode']
                save(folder/'checkpoint.json', summary)
                continue
            report = json.loads((endpoint/'mtc.json').read_text())
            actual = np.asarray(report['stages'][-1]['points'][-1])
            try:
                joint_error = check_joint_endpoint(actual, candidate['arm_qpos_rad'])
                rows, validation = validate_regrasp(RotationDiagnostic(context.trajectory), request, report)
            except ValueError as error:
                attempt['rejection'] = str(error)
                save(folder/'checkpoint.json', summary)
                continue
            validation.update(endpoint_joint_error_rad=joint_error, checked_states=len(rows))
            save(endpoint/'validation.json', validation)
            actual_bindings = {**bindings, **{str((endpoint/name).relative_to(ROOT)): digest(endpoint/name)
                               for name in ('request.json', 'mtc.json', 'validation.json')}}
            node = CandidateContext(context, actual, candidate_id=candidate['id'], bindings=actual_bindings,
                                    role='actual_mtc_endpoint')
            result = diagnostic(node, folder/candidate['id']/'actual_endpoint', settings)
            attempt['actual_endpoint'] = result
            passed = result['checked_additional_angle_deg']
            if passed is None or passed <= baseline['checked_additional_angle_deg']:
                attempt['rejection'] = 'actual_endpoint_did_not_increase_checked_range'
                save(folder/'checkpoint.json', summary)
                continue
            attempt['accepted'] = True
            summary['selected'] = {'candidate_id': candidate['id'], 'actual_arm_qpos_rad': actual.tolist(),
                'endpoint_joint_error_rad': joint_error, 'checked_additional_angle_deg': passed,
                'checked_cumulative_angle_deg': context.angle+passed,
                'checked_range_change_deg': passed-baseline['checked_additional_angle_deg']}
            break
        summary['analysis_finished'] = True
        summary['status'] = 'selected_path_and_margin_verified' if summary['selected'] else 'no_acceptable_improving_candidate'
        # 全部诊断、失败和日志都绑定；未尝试者的状态单独列出。
        tried = {row['candidate_id'] for row in summary['path_attempts']}
        summary['unattempted_improving_candidate_ids'] = [row['id'] for row in ordered_candidates(
            summary['candidate_results'], baseline['checked_additional_angle_deg']) if row['id'] not in tried]
        summary['artifacts_sha256'] = {str(path.relative_to(ROOT)): digest(path) for path in folder.rglob('*')
            if path.is_file() and path.name not in ('checkpoint.json', 'summary.json', 'report.md')}
        save(folder/'summary.json', summary)
        write_report(folder, summary)
        return summary
    finally:
        if old_log_dir is None:
            os.environ.pop('ROS_LOG_DIR', None)
        else:
            os.environ['ROS_LOG_DIR'] = old_log_dir


def write_report(folder, summary):
    """读者入口同时展示对照、拒绝和实际到达后的结果。"""
    lines = ['# 重新抓握构型筛选', '', '固定盖姿态、预型、轴向深度和手盖关系；仅作几何验证。', '',
             '| 候选 | 对照 | 共同通过追加角 ° | 通过状态数 | 首个拒绝 |', '| --- | --- | ---: | ---: | --- |']
    for row in summary['candidate_results']:
        failure = row.get('failure')
        reason = failure['kind'] if failure else '到达扫描预算，最大范围未知'
        lines.append(f"| {row['id']} | {','.join(row['labels'])} | {row['checked_additional_angle_deg']} | {row['passed_samples']} | {reason} |")
    lines += ['', f"最终状态：{summary['status']}。"]
    if summary['selected']:
        result = summary['selected']
        lines += [f"所选 {result['candidate_id']} 经真实 MTC 路径后，追加共同通过 {result['checked_additional_angle_deg']}°，"
                  f"累计 {result['checked_cumulative_angle_deg']}°，比节点前增加 {result['checked_range_change_deg']}°。",
                  f"实际末态原始关节差最大 {result['endpoint_joint_error_rad']} rad。"]
    lines += ['', '## 路径尝试', '']
    for attempt in summary['path_attempts']:
        lines.append(f"- {attempt['candidate_id']}：{'通过' if attempt['accepted'] else attempt['rejection']}")
    lines += ['', '有限搜索不能证明全局最优；预算通过不等于最大转角或物理脱离角。',
              '没有验证释放、抓稳、轴向柔顺、螺纹、动力学或真机；实际停转阈值与停止余量未知。', '']
    (folder/'report.md').write_text('\n'.join(lines))


def verify(output_dir):
    """只重建几何和校验保存来源，不再求解 IK、搜索路径或覆盖产物。"""
    folder = Path(output_dir).resolve()
    summary = json.loads((folder/'summary.json').read_text())
    if (summary.get('mode') != 'regrasp_configuration_selection_not_execution'
            or summary.get('analysis_finished') is not True):
        raise ValueError('筛选记录未完成')
    bindings = source_bindings()
    pool_path = folder/'candidate_pool.json'
    bindings[str(pool_path.relative_to(ROOT))] = digest(pool_path)
    if summary['input_hashes'] != bindings:
        raise ValueError('构型筛选算法、规划器或候选清单已变更')
    for relative, expected in summary['artifacts_sha256'].items():
        path = (ROOT/relative).resolve()
        if not path.is_relative_to(folder) or digest(path) != expected:
            raise ValueError(f'筛选产物已变更：{relative}')
    context = RegraspContext((ROOT/summary['origin']['segment_summary_path']).parent)
    pool = json.loads(pool_path.read_text())
    settings = RotationScanSettings(TaskParameters().unlock_angle_deg-context.angle)
    if (summary['origin'] != context.origin('before') or pool['origin'] != context.origin('before')
            or summary['scan_settings'] != asdict(settings)):
        raise ValueError('筛选节点或扫描预算不一致')
    expected_ids = [row['id'] for row in pool['candidates'] if row['scan_selected']]
    if [row['id'] for row in summary['candidate_results']] != expected_ids:
        raise ValueError('候选结果遗漏或顺序变更')
    pool_by_id = {row['id']: row for row in pool['candidates']}
    for item in summary['candidate_results']:
        candidate = pool_by_id[item['id']]
        if any(item[key] != value for key, value in candidate.items()):
            raise ValueError('候选关节或预筛字段不一致')
        node = CandidateContext(context, candidate['arm_qpos_rad'], candidate_id=candidate['id'],
                                bindings=bindings, role='candidate')
        endpoint = folder/candidate['id']/'candidate'
        report = json.loads((endpoint/'diagnostic.json').read_text())
        if report['samples']:
            checked, _ = checked_endpoint(endpoint, node, 'after')
            checked['fcl_audit_performed'] = True
        else:
            if (report.get('analysis_finished') is not True or report['origin'] != node.origin('after')
                    or report['failure']['angle_deg'] != 0. or (endpoint/'fcl_validation.json').exists()):
                raise ValueError('候选空前缀没有有效拒绝记录')
            checked = {'checked_additional_angle_deg': None, 'passed_samples': 0,
                       'failure': report['failure'], 'fcl_audit_performed': False}
        if any(item[key] != value for key, value in checked.items()):
            raise ValueError('候选通过范围与独立复核不一致')
    baseline = next(row for row in summary['candidate_results'] if 'before' in row['labels'])
    if summary['baseline_additional_angle_deg'] != baseline['checked_additional_angle_deg']:
        raise ValueError('对照范围不一致')
    eligible = ordered_candidates(summary['candidate_results'], baseline['checked_additional_angle_deg'])
    attempts = summary['path_attempts']
    if [row['candidate_id'] for row in attempts] != [row['id'] for row in eligible[:len(attempts)]]:
        raise ValueError('路径尝试不符合有限候选排序')
    accepted = [row for row in attempts if row['accepted']]
    if bool(accepted) != bool(summary['selected']) or len(accepted) > 1 or (accepted and attempts[-1] != accepted[0]):
        raise ValueError('所选构型与接受记录不一致')
    if accepted:
        chosen = summary['selected']
        candidate = pool_by_id[chosen['candidate_id']]
        if accepted[0]['candidate_id'] != candidate['id']:
            raise ValueError('选用的候选标识不一致')
        endpoint = folder/candidate['id']/'path'
        request, mtc = (json.loads((endpoint/name).read_text()) for name in ('request.json', 'mtc.json'))
        actual = mtc['stages'][-1]['points'][-1]
        joint_error = check_joint_endpoint(actual, candidate['arm_qpos_rad'])
        selection = request['configuration_selection']
        if (selection['input_hashes'] != bindings or selection['candidate_id'] != candidate['id']
                or selection['grasp_q_rad'] != candidate['arm_qpos_rad']
                or selection['endpoint_joint_tolerance_rad'] != JOINT_TOLERANCE_RAD
                or mtc.get('selected_configuration_check_passed') is not True):
            raise ValueError('MTC 请求未绑定所选构型')
        _, validation = validate_regrasp(RotationDiagnostic(context.trajectory), request, mtc)
        saved = json.loads((endpoint/'validation.json').read_text())
        for key in ('endpoint_position_error_m', 'endpoint_orientation_error_rad'):
            if not np.isclose(saved[key], validation[key], atol=1e-12, rtol=0):
                raise ValueError('路径末态残差不一致')
        actual_bindings = {**bindings, **{str((endpoint/name).relative_to(ROOT)): digest(endpoint/name)
                           for name in ('request.json', 'mtc.json', 'validation.json')}}
        node = CandidateContext(context, actual, candidate_id=candidate['id'], bindings=actual_bindings,
                                role='actual_mtc_endpoint')
        checked, _ = checked_endpoint(folder/candidate['id']/'actual_endpoint', node, 'after')
        passed = checked['checked_additional_angle_deg']
        expected = {'candidate_id': candidate['id'], 'actual_arm_qpos_rad': actual,
                    'endpoint_joint_error_rad': joint_error, 'checked_additional_angle_deg': passed,
                    'checked_cumulative_angle_deg': context.angle+passed,
                    'checked_range_change_deg': passed-baseline['checked_additional_angle_deg']}
        if chosen != expected or passed <= baseline['checked_additional_angle_deg']:
            raise ValueError('实际规划末态未增加旋转范围')
        if accepted[0]['actual_endpoint'] != {**checked, 'fcl_audit_performed': True}:
            raise ValueError('实际末态诊断与接受记录不一致')
    if summary['status'] != ('selected_path_and_margin_verified' if accepted else 'no_acceptable_improving_candidate'):
        raise ValueError('筛选状态不一致')
    return summary
