"""同一盖姿态下评价一次已成功重新接近的前后旋转空间；仅作几何诊断。"""
import asyncio
import copy
import csv
from dataclasses import asdict
import json
import os
from pathlib import Path

import numpy as np

from .arm_ik import body_transform, transform
from .grasp_search import pose_arrays
from .interfaces import TaskParameters
from .planning_scene_snapshot import check_snapshot, scene_snapshot
from .rotation_diagnostic import RotationDiagnostic, RotationScanSettings, hand_target, joint_margins
from .rotation_margin import FAILURE_LABELS, joint_table
from .segmented_rotation import digest, launch, pose_error, pose_tolerances, save, validate_regrasp
from .trajectory_replay import ROOT


class RegraspContext:
    """从来源绑定的成功节点恢复前后构型，不使用后续失败状态。"""
    def __init__(self, folder):
        self.folder = Path(folder).resolve()
        self.summary_path = self.folder/'summary.json'
        summary = json.loads(self.summary_path.read_text())
        if (summary.get('mode') != 'segmented_fixed_depth_preform_geometry_not_execution'
                or summary.get('analysis_finished') is not True
                or summary.get('completed_regrasp_paths', 0) < 1):
            raise ValueError('输入没有第一次成功重新接近节点')
        self.input_hashes = dict(summary['artifacts_sha256'])
        self.input_hashes[str(self.summary_path.relative_to(ROOT))] = digest(self.summary_path)
        for relative, expected in self.input_hashes.items():
            path = (ROOT/relative).resolve()
            if not path.is_relative_to(ROOT) or not path.is_file() or digest(path) != expected:
                raise ValueError(f'分段输入来源已变更：{relative}')
        trajectory = (ROOT/summary['trajectory_path']).resolve()
        if not trajectory.is_relative_to(ROOT) or digest(trajectory) != summary['trajectory_sha256']:
            raise ValueError('原接近轨迹来源不一致')
        self.trajectory = trajectory
        self.input_hashes[str(trajectory.relative_to(ROOT))] = digest(trajectory)
        source = RotationDiagnostic(trajectory)

        def load(name):
            path = self.folder/name
            if str(path.relative_to(ROOT)) not in summary['artifacts_sha256']:
                raise ValueError(f'成功节点缺少来源绑定：{name}')
            return json.loads(path.read_text())

        rotation = load('rotation_1.json')
        fcl = load('rotation_1_fcl.json')
        request, mtc = load('regrasp_1_request.json'), load('regrasp_1_mtc.json')
        validation = load('regrasp_1_validation.json')
        rows = rotation['samples']
        if (rotation.get('analysis_finished') is not True or rotation.get('failure') is not None or not rows
                or rotation.get('mode') != 'segmented_fixed_depth_rotation_not_execution'
                or rotation['trajectory_sha256'] != summary['trajectory_sha256']
                or fcl.get('mode') != 'full_fcl_rotation_prefix_audit_not_execution'
                or fcl.get('diagnostic_sha256') != digest(self.folder/'rotation_1.json')
                or fcl.get('completed_input_check') is not True or fcl.get('failure') is not None
                or fcl.get('passed_samples') != len(rows) or fcl.get('checked_samples') != len(rows)
                or rotation['begin_angle_deg'] != 0. or rows[0]['angle_deg'] != 0.
                or not np.allclose(rows[0]['arm_qpos_rad'], source.q0, atol=1e-12, rtol=0)
                or rows[-1]['angle_deg'] != rotation['target_angle_deg']
                or fcl['clear_prefix_angle_deg'] != rows[-1]['angle_deg']):
            raise ValueError('重新接近前的旋转段缺少完整通过证据')
        self.angle = float(request['regrasp']['cumulative_angle_deg'])
        if not np.isfinite(self.angle) or self.angle <= 0 or self.angle != rows[-1]['angle_deg']:
            raise ValueError('重新接近节点累计角与前段不一致')
        self.before = np.asarray(rows[-1]['arm_qpos_rad'], dtype=float)
        original = source.replay.report['request']
        expected_initial = copy.deepcopy(original['initial_joint_positions'])
        expected_initial.update(zip(original['arm_joints'], self.before.tolist()))
        if (request['arm_joints'] != original['arm_joints']
                or request['source_hashes'] != original['source_hashes']
                or request['pregrasp_channels'] != original['pregrasp_channels']
                or request['initial_joint_positions'] != expected_initial
                or pose_tolerances(request) != pose_tolerances(summary)
                or pose_tolerances(rotation) != pose_tolerances(summary)):
            raise ValueError('重新接近的模型、预型、起点或容差与原记录不一致')
        # 先独立复核前段路点，确认比较输入来自同一条已通过的运动链。
        previous, previous_angle = source.q0, -1.
        for row in rows:
            q, angle = np.asarray(row['arm_qpos_rad']), float(row['angle_deg'])
            if (q.shape != (6,) or not np.isfinite(q).all() or not np.isfinite(angle)
                    or angle <= previous_angle or angle > self.angle
                    or np.max(np.abs(q-previous)) > .02000001
                    or joint_margins(q, source.preview.limits)['minimum_joint_margin_rad'] < 0):
                raise ValueError('前段关节或累计角不连续')
            actual_row = source.row(q, angle)
            check_snapshot({'objects': row['cap_objects']}, {'objects': actual_row['cap_objects']})
            target = hand_target(source.cap0, source.relative, np.deg2rad(angle))
            p, r = pose_error(body_transform(source.preview.data, 'base'), target)
            limits = pose_tolerances(rotation)
            if (p > limits['position_m'] or r > limits['orientation_rad']
                    or any(c['penetration_m'] > .0001 for c in source.preview.contacts())):
                raise ValueError('前段路点几何复核失败')
            previous, previous_angle = q, angle
        source.set_state(self.before, self.angle)
        cap = hand_target(source.cap0, np.eye(4), np.deg2rad(self.angle))
        target = cap @ source.relative
        observed = request['cap_estimate']
        request_target = transform(*pose_arrays({'position_m': request['hand_position'],
                                                  'orientation_xyzw': request['hand_orientation_xyzw']}))
        if (observed.get('valid') is not True or observed['pose']['frame_id'] != 'world'
                or observed['source'] != source.preview.estimate.source
                or not np.allclose(transform(*pose_arrays(observed['pose'])), cap, atol=1e-10, rtol=0)
                or not np.allclose(request_target, target, atol=1e-10, rtol=0)
                or not np.allclose(request['outward_axis_world'], cap[:3, 2], atol=1e-10, rtol=0)):
            raise ValueError('重新接近观测或目标与同一盖姿态不一致')
        _, actual_validation = validate_regrasp(source, request, mtc)
        actual_validation = json.loads(json.dumps(actual_validation, allow_nan=False))
        # FK 再计算允许浮点舍入差；不放宽阶段位姿验收或改变历史记录。
        checked_validation = copy.deepcopy(validation)
        for key in ('endpoint_position_error_m', 'endpoint_orientation_error_rad'):
            if not np.isclose(validation[key], actual_validation[key], atol=1e-12, rtol=0):
                raise ValueError('重新接近末态残差与保存记录不一致')
            checked_validation[key] = actual_validation[key]
        if actual_validation != checked_validation:
            raise ValueError('重新接近复核与保存的验证记录不一致')
        self.after = np.asarray(mtc['stages'][-1]['points'][-1], dtype=float)
        self.names = original['arm_joints']

    def origin(self, side):
        if side not in ('before', 'after'):
            raise ValueError('仅支持重新接近前或后构型')
        return {'kind': 'first_successful_regrasp_endpoint', 'side': side,
                'segment_summary_path': str(self.summary_path.relative_to(ROOT)),
                'cumulative_angle_deg': self.angle, 'input_hashes': self.input_hashes}


class EndpointRotationDiagnostic(RotationDiagnostic):
    """把原扫描的 0°移到同一累计盖角，保留原始关节绕数和手盖关系。"""
    def __init__(self, context, side):
        super().__init__(context.trajectory)
        self.origin = context.origin(side)
        q = context.before if side == 'before' else context.after
        self.set_state(q, context.angle)
        observed = asyncio.run(self.preview.scene.estimator.estimate())
        if not observed.valid or observed.pose.frame_id != 'world':
            raise ValueError('节点盖体观测无效')
        # start 已包含累计盖角；后续追加角只在这个初态上增加，不能归零盖体。
        self.start = self.preview.data.qpos.copy()
        self.q0 = q.copy()
        self.cap0 = transform(*pose_arrays(asdict(observed.pose)))
        self.world = scene_snapshot(self.world, self.preview.model, self.preview.data)
        self.cap_objects = [o for o in self.world['objects'] if o['source_body'] in ('cap_body', 'cap_grip_center')]
        self.replay.report = copy.deepcopy(self.replay.report)
        self.replay.report['request']['initial_joint_positions'].update(zip(context.names, q.tolist()))

    def row(self, q, angle_deg, *, detailed=False):
        row = super().row(q, angle_deg, detailed=detailed)
        row['cumulative_angle_deg'] = self.origin['cumulative_angle_deg']+float(angle_deg)
        return row

    def run(self, checkpoint, *, settings):
        # 输入清单先落盘；中断时的原始检查点缺少最终来源头，不能用于汇总或审计结论。
        save(Path(checkpoint).with_name('input.json'), {'origin': self.origin, 'scan_settings': asdict(settings)})
        report = super().run(checkpoint, settings=settings)
        report['origin'] = self.origin
        for module in ('regrasp_margin.py', 'segmented_rotation.py', 'rotation_margin.py'):
            path = ROOT/'src/fuel_cap_task'/module
            report['analysis_source_hashes'][str(path.relative_to(ROOT))] = digest(path)
        if report['failure']:
            report['failure']['cumulative_angle_deg'] = self.origin['cumulative_angle_deg']+report['failure']['angle_deg']
        save(checkpoint, report)
        return report


def prepare(segment_dir, output_dir, *, max_additional_angle_deg=None, angle_step_deg=1.):
    """每侧独立扫描一次；数值拒绝是诊断结果，不改分支或重试。"""
    folder = Path(output_dir).resolve()
    if folder.exists() or not folder.is_relative_to(ROOT):
        raise ValueError('请使用工作区内不存在的独立输出目录')
    context = RegraspContext(segment_dir)
    maximum = TaskParameters().unlock_angle_deg-context.angle if max_additional_angle_deg is None else max_additional_angle_deg
    settings = RotationScanSettings(maximum, angle_step_deg)
    save(folder/'inputs.json', {'origin': context.origin('before'), 'scan_settings': asdict(settings)})
    for side in ('before', 'after'):
        EndpointRotationDiagnostic(context, side).run(folder/side/'diagnostic.json', settings=settings)


def audit(output_dir):
    """只审计两份已有诊断；不重新计算，也不覆盖既有审计。"""
    folder = Path(output_dir).resolve()
    if any((folder/side/'fcl_validation.json').exists() for side in ('before', 'after')):
        raise ValueError('已有审计记录，不覆盖或自动续跑')
    previous_log_dir = os.environ.get('ROS_LOG_DIR')
    try:
        for side in ('before', 'after'):
            endpoint = folder/side
            os.environ['ROS_LOG_DIR'] = str(endpoint/'ros_logs')
            launch('rotation_audit.launch.py', {'diagnostic_path': endpoint/'diagnostic.json',
                   'output_path': endpoint/'fcl_validation.json'}, endpoint/'fcl_audit.log')
    finally:
        if previous_log_dir is None:
            os.environ.pop('ROS_LOG_DIR', None)
        else:
            os.environ['ROS_LOG_DIR'] = previous_log_dir


def checked_endpoint(folder, context, side):
    """按原输入重建节点并复核已审计前缀；不再调用 IK。"""
    path, fcl_path = folder/'diagnostic.json', folder/'fcl_validation.json'
    report, fcl = json.loads(path.read_text()), json.loads(fcl_path.read_text())
    if (report.get('mode') != 'fixed_depth_preform_rotation_diagnostic_not_execution'
            or report.get('analysis_finished') is not True or report.get('origin') != context.origin(side)
            or report['trajectory_sha256'] != digest(context.trajectory)
            or report['trajectory_path'] != str(context.trajectory.relative_to(ROOT))):
        raise ValueError('节点诊断未完成或来源/累计角不一致')
    for relative, expected in report['analysis_source_hashes'].items():
        source = (ROOT/relative).resolve()
        if not source.is_relative_to(ROOT) or not source.is_file() or digest(source) != expected:
            raise ValueError('余量诊断算法来源已变更')
    settings = RotationScanSettings(**report['scan_settings'])
    source = EndpointRotationDiagnostic(context, side)
    if (report['target_angle_deg'] != settings.max_angle_deg or report['arm_joints'] != context.names
            or report['initial_joint_positions'] != source.replay.report['request']['initial_joint_positions']
            or not np.array_equal(report['joint_limits_rad'], source.preview.limits)):
        raise ValueError('节点的扫描预算、关节或限位不一致')
    check_snapshot(report['planning_scene'], source.world)
    rows, count, checked = report['samples'], fcl.get('passed_samples'), fcl.get('checked_samples')
    failure = fcl.get('failure')
    if (fcl.get('mode') != 'full_fcl_rotation_prefix_audit_not_execution'
            or fcl.get('diagnostic_sha256') != digest(path)
            or type(count) is not int or not 0 <= count <= len(rows)
            or type(checked) is not int or checked != count+(failure is not None)
            or (failure is None and count != len(rows))
            or fcl.get('completed_input_check') is not (failure is None)
            or (failure is not None and (count >= len(rows) or failure.get('angle_deg') != rows[count]['angle_deg']))):
        raise ValueError('FCL 节点审计的来源或检查前缀无效')
    prefix = rows[:count]
    if not rows and (report.get('failure') or {}).get('angle_deg') != 0.:
        raise ValueError('空数值前缀必须对应起点被拒绝')
    previous, previous_angle = source.q0, -1.
    for row in prefix:
        q, angle = np.asarray(row['arm_qpos_rad']), row['angle_deg']
        if (q.shape != (6,) or not np.isfinite(q).all() or not np.isfinite(angle)
                or angle <= previous_angle or not 0 <= angle <= settings.max_angle_deg
                or np.max(np.abs(q-previous)) > .02000001
                or row['cumulative_angle_deg'] != context.angle+angle
                or joint_margins(q, source.preview.limits)['minimum_joint_margin_rad'] < 0):
            raise ValueError('节点前缀的累计角、关节或连续性无效')
        actual_row = source.row(q, angle, detailed=True)
        check_snapshot({'objects': row['cap_objects']}, {'objects': actual_row['cap_objects']})
        for key in ('joint_margin_rad', 'joint_lower_margin_rad', 'joint_upper_margin_rad', 'minimum_joint_margin_rad'):
            if not np.allclose(row[key], actual_row[key], atol=1e-12, rtol=0):
                raise ValueError('节点关节余量与状态不一致')
        p, r = pose_error(body_transform(source.preview.data, 'base'), hand_target(source.cap0, source.relative, np.deg2rad(angle)))
        if p > .0001 or r > .002 or any(c['penetration_m'] > .0001 for c in source.preview.contacts()):
            raise ValueError('节点前缀几何复核失败')
        previous, previous_angle = q, angle
    if rows and (rows[0]['angle_deg'] != 0. or not np.allclose(rows[0]['arm_qpos_rad'], source.q0, atol=1e-12, rtol=0)):
        raise ValueError('节点前缀起点与来源不一致')
    angle = prefix[-1]['angle_deg'] if prefix else None
    if count and fcl['clear_prefix_angle_deg'] != angle:
        raise ValueError('FCL 前缀角与记录不一致')
    failure = copy.deepcopy(failure or report['failure'])
    if failure:
        failure['cumulative_angle_deg'] = context.angle+failure['angle_deg']
    summary = {'side': side, 'scan_settings': report['scan_settings'], 'passed_samples': count,
               'checked_additional_angle_deg': angle,
               'checked_cumulative_angle_deg': None if angle is None else context.angle+angle,
               'scan_horizon_reached': failure is None and angle == settings.max_angle_deg,
               'failure': failure, 'mujoco_failure': report['failure'], 'fcl_failure': fcl['failure'],
               'initial_joint_table_deg': joint_table(context.names, source.q0, source.preview.limits),
               'diagnostic_sha256': digest(path), 'fcl_sha256': digest(fcl_path)}
    return summary, prefix


def write_outputs(output_dir):
    """比较共同检查前缀；扫描上限截断时不推断绝对最大角度。"""
    folder = Path(output_dir).resolve()
    targets = [folder/name for name in ('summary.json', 'report.md', 'joint_margins.csv')]
    if any(path.exists() for path in targets):
        raise ValueError('已有比较结果，不覆盖')
    inputs = json.loads((folder/'inputs.json').read_text())
    context = RegraspContext((ROOT/inputs['origin']['segment_summary_path']).parent)
    if inputs['origin'] != context.origin('before'):
        raise ValueError('比较输入来源或累计角不一致')
    results, samples = {}, {}
    for side in ('before', 'after'):
        results[side], samples[side] = checked_endpoint(folder/side, context, side)
        if results[side]['scan_settings'] != inputs['scan_settings']:
            raise ValueError('前后扫描预算不一致，不能比较')
    before, after = (results[side]['checked_additional_angle_deg'] for side in ('before', 'after'))
    delta = None if before is None or after is None else after-before
    summary = {'mode': 'regrasp_rotation_margin_comparison_not_execution', 'analysis_finished': True,
               'origin': inputs['origin'], 'scan_settings': inputs['scan_settings'], 'endpoints': results,
               'checked_range_change_deg': delta, 'checked_range_increased': None if delta is None else delta > 0,
               'execution_stop_angle_deg': None, 'execution_stopping_reserve_deg': None,
               'physical_release_verified': False, 'physical_regrasp_verified': False, 'thread_motion_verified': False}
    save(targets[0], summary)
    lines = ['# 重新抓握前后的旋转余量', '',
             f'同一盖体累计角：{context.angle}°；手保持预型，固定轴向深度。', '',
             '| 构型 | 共同通过追加角 ° | 共同通过累计角 ° | 停止原因 |', '| --- | ---: | ---: | --- |']
    for side, label in (('before', '重新接近前'), ('after', '重新接近后')):
        result, failure = results[side], results[side]['failure']
        reason = FAILURE_LABELS.get(failure['kind'], failure['kind']) if failure else '到达扫描预算，最大范围未知'
        lines.append(f"| {label} | {result['checked_additional_angle_deg']} | {result['checked_cumulative_angle_deg']} | {reason} |")
    conclusion = '存在无可用前缀的构型，不能判断余量是否增加。' if delta is None else (
        f'共同检查通过的追加范围变化：{delta:+.3f}°。本次比较'+('增加了通过范围。' if delta > 0 else '未增加通过范围。'))
    lines += ['', conclusion, '有限采样和数值失败不证明绝对最大可转角；预算耗尽时不排除继续旋转。', '',
              '| 关节 | 前：当前角 ° | 后：当前角 ° | 前：最近余量 ° | 后：最近余量 ° |', '| --- | ---: | ---: | ---: | ---: |']
    for a, b in zip(results['before']['initial_joint_table_deg'], results['after']['initial_joint_table_deg']):
        lines.append(f"| {a['joint']} | {a['q_deg']:.3f} | {b['q_deg']:.3f} | {a['nearest_margin_deg']:.3f} | {b['nearest_margin_deg']:.3f} |")
    lines += ['', '关节角保留原始绕数。最近关节余量与追加盖角含义不同。',
              '详细状态见两侧 diagnostic.json / fcl_validation.json 和 joint_margins.csv。',
              '本轮没有重新搜索 MTC、实际松手、抓稳、轴向柔顺或取盖验收；实际停转阈值和停止余量未确定。', '']
    targets[1].write_text('\n'.join(lines))
    with targets[2].open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['side', 'additional_angle_deg', 'cumulative_angle_deg', 'joint', 'q_deg', 'to_lower_deg', 'to_upper_deg', 'nearest_margin_deg'])
        for side in ('before', 'after'):
            for sample in samples[side]:
                for i, name in enumerate(context.names):
                    writer.writerow([side, sample['angle_deg'], sample['cumulative_angle_deg'], name,
                                     *[float(np.rad2deg(sample[key][i])) for key in ('arm_qpos_rad', 'joint_lower_margin_rad', 'joint_upper_margin_rad', 'joint_margin_rad')]])
    return summary
