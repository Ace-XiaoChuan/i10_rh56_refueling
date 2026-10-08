"""汇总单构型的关节余量和共同检查前缀，不生成执行停止指令。"""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from .rotation_diagnostic import RotationDiagnostic, RotationReplay, RotationScanSettings, joint_margins
from .planning_scene_snapshot import check_snapshot
from .trajectory_replay import ROOT


FAILURE_LABELS = {
    'continuous_ik_not_converged': '连续逆运动学未收敛（不证明限位或全局不可达）',
    'joint_branch_jump': '相邻解变化超过连续性诊断尺度',
    'joint_bounds': '关节越界',
    'invalid_solver_state': '求解器返回无效关节状态',
    'mujoco_robot_interference': 'MuJoCo 原覆盖检出机器人干涉',
    'dense_pose_deviation': '加密状态的手根位姿偏离目标',
    'fcl_robot_collision': 'FCL 完整规划覆盖检出机器人碰撞',
}


def joint_table(names, q, limits, previous=None):
    """用于阅读的角度表；近期变化只描述已走过的运动，不外推剩余盖角。"""
    margins = joint_margins(q, limits)
    result = []
    for i, name in enumerate(names):
        result.append({
            'joint': name, 'q_deg': float(np.rad2deg(q[i])),
            'lower_limit_deg': float(np.rad2deg(limits[i][0])),
            'upper_limit_deg': float(np.rad2deg(limits[i][1])),
            'to_lower_deg': float(np.rad2deg(margins['joint_lower_margin_rad'][i])),
            'to_upper_deg': float(np.rad2deg(margins['joint_upper_margin_rad'][i])),
            'nearest_margin_deg': float(np.rad2deg(margins['joint_margin_rad'][i])),
            'last_increment_deg': None if previous is None else float(np.rad2deg(q[i]-previous[i])),
        })
    return result


def summarize(diagnostic_path, fcl_path):
    """先复核来源和有效前缀，再输出两种不同含义的余量。"""
    path, fcl_path = Path(diagnostic_path), Path(fcl_path)
    diagnostic, fcl = json.loads(path.read_text()), json.loads(fcl_path.read_text())
    rows = diagnostic['samples']
    count, checked = fcl.get('passed_samples'), fcl.get('checked_samples')
    if (diagnostic.get('analysis_finished') is not True
            or fcl.get('mode') != 'full_fcl_rotation_prefix_audit_not_execution'
            or fcl.get('diagnostic_sha256') != hashlib.sha256(path.read_bytes()).hexdigest()
            or type(count) is not int or not 0 <= count <= len(rows)
            or type(checked) is not int or checked != count+(1 if fcl.get('failure') else 0)
            or (not fcl.get('failure') and count != len(rows))
            or fcl.get('completed_input_check') is not (fcl.get('failure') is None)):
        raise ValueError('FCL 审计未完成、来源不一致或检查计数无效')
    if count:
        replay = RotationReplay(path, fcl_path)
        source = replay.source
    else:
        # 起点被完整覆盖拒绝时没有可用前缀；仍输出初态供排查，不能当作通过状态。
        initial_fcl_rejection = bool(rows) and fcl.get('failure', {}).get('angle_deg') == 0.
        initial_numerical_rejection = not rows and (diagnostic.get('failure') or {}).get('angle_deg') == 0.
        if not (initial_fcl_rejection or initial_numerical_rejection):
            raise ValueError('空前缀必须对应起点被 FCL 拒绝')
        source = RotationDiagnostic(ROOT/diagnostic['trajectory_path'])
        if (diagnostic['trajectory_sha256'] != hashlib.sha256(source.path.read_bytes()).hexdigest()
                or (rows and not np.allclose(rows[0]['arm_qpos_rad'], source.q0, atol=1e-12, rtol=0))):
            raise ValueError('起点或原轨迹不一致')
        check_snapshot(diagnostic['planning_scene'], source.world)
        settings = RotationScanSettings(**diagnostic['scan_settings'])
        if (diagnostic['target_angle_deg'] != settings.max_angle_deg
                or diagnostic['arm_joints'] != source.replay.report['request']['arm_joints']
                or diagnostic['initial_joint_positions'] != source.replay.report['request']['initial_joint_positions']):
            raise ValueError('扫描范围或初始状态不一致')
        for relative, expected in diagnostic.get('analysis_source_hashes', {}).items():
            bound_source = (ROOT/relative).resolve()
            if (not bound_source.is_relative_to(ROOT) or not bound_source.is_file()
                    or hashlib.sha256(bound_source.read_bytes()).hexdigest() != expected):
                raise ValueError('旋转诊断算法来源已变更')
    limits = source.preview.limits
    if not np.array_equal(diagnostic['joint_limits_rad'], limits):
        raise ValueError('报告关节限位与模型不一致')
    prefix = rows[:count]
    final = prefix[-1] if prefix else None
    previous = prefix[-2]['arm_qpos_rad'] if len(prefix) > 1 else None
    failure = fcl['failure'] or diagnostic['failure']
    angle = final['angle_deg'] if final else None
    if count and fcl['clear_prefix_angle_deg'] != angle:
        raise ValueError('FCL 前缀角度与有效状态不一致')
    tables = {'initial': joint_table(diagnostic['arm_joints'], source.q0, limits),
              'last_checked': None if final is None else joint_table(
                  diagnostic['arm_joints'], final['arm_qpos_rad'], limits, previous)}
    summary = {
        'mode': 'single_configuration_rotation_margin_not_execution',
        'analysis_finished': True,
        'diagnostic_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'fcl_sha256': hashlib.sha256(fcl_path.read_bytes()).hexdigest(),
        'scan_settings': diagnostic['scan_settings'],
        'checked_prefix_angle_deg': angle, 'passed_samples': count,
        'scan_horizon_reached': failure is None and angle == diagnostic['target_angle_deg'],
        'failure': failure, 'mujoco_failure': diagnostic['failure'], 'fcl_failure': fcl['failure'],
        'joint_tables_deg': tables,
        'execution_stop_angle_deg': None, 'execution_stopping_reserve_deg': None,
        'physical_grasp_verified': False, 'thread_motion_verified': False,
        'coverage_note': 'MuJoCo 原覆盖与 FCL 完整规划覆盖共同通过的有限采样；未验证连续安全或物理执行',
    }
    if prefix:
        margins = np.array([row['joint_margin_rad'] for row in prefix])
        sample_index, joint_index = np.unravel_index(np.argmin(margins), margins.shape)
        summary['closest_joint_to_limit_in_prefix'] = {
            'joint': diagnostic['arm_joints'][joint_index],
            'angle_deg': prefix[sample_index]['angle_deg'],
            'margin_deg': float(np.rad2deg(margins[sample_index, joint_index])),
        }
    else:
        summary['closest_joint_to_limit_in_prefix'] = None
    return summary, prefix


def write_outputs(folder):
    """只汇总已有计算与审计，不重新运行 IK、规划或接触实验。"""
    folder = Path(folder)
    targets = [folder/name for name in ('summary.json', 'joint_margins.csv', 'report.md')]
    if any(path.exists() for path in targets):
        raise ValueError('已有余量汇总，不覆盖；请保留原结果')
    summary, prefix = summarize(folder/'diagnostic.json', folder/'fcl_validation.json')
    summary['summary_source_hashes'] = {
        str(Path(__file__).resolve().relative_to(ROOT)):
        hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    targets[0].write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    names = [row['joint'] for row in summary['joint_tables_deg']['initial']]
    with targets[1].open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['cap_angle_deg', 'joint', 'q_deg', 'to_lower_deg', 'to_upper_deg', 'nearest_margin_deg'])
        for sample in prefix:
            for i, name in enumerate(names):
                writer.writerow([sample['angle_deg'], name, *[float(np.rad2deg(sample[field][i]))
                    for field in ('arm_qpos_rad', 'joint_lower_margin_rad', 'joint_upper_margin_rad', 'joint_margin_rad')]])
    failure = summary['failure']
    reason = FAILURE_LABELS.get(failure['kind'], failure['kind']) if failure else '到达本次扫描上限，未确定最大可转角'
    lines = ['# 单构型旋转余量诊断', '',
             '固定轴向深度，手保持接近预型，固定手盖关系；仅运动学检查。', '',
             f"共同通过前缀终点：{summary['checked_prefix_angle_deg']}°；状态数：{len(prefix)}。",
             f'停止原因：{reason}。',
             *([f"首个拒绝角度：{failure['angle_deg']}°。"] if failure else []), '',
             '最近关节限位距离不能直接换算为盖体可转角。实际停转阈值及停止余量未确定。', '']
    for key, title in (('initial', '接近终点'), ('last_checked', '共同通过的最后状态')):
        table = summary['joint_tables_deg'][key]
        if table is None:
            continue
        lines += [f'## {title}', '', '| 关节 | 当前角度 ° | 到下限 ° | 到上限 ° | 最近余量 ° |',
                  '| --- | ---: | ---: | ---: | ---: |']
        for row in table:
            lines.append(f"| {row['joint']} | {row['q_deg']:.3f} | {row['to_lower_deg']:.3f} | {row['to_upper_deg']:.3f} | {row['nearest_margin_deg']:.3f} |")
        lines.append('')
    lines += ['数据：`summary.json`、`joint_margins.csv`；拒绝状态详情：`diagnostic.json` / `fcl_validation.json`。',
              '未验证闭合抓稳、轴向柔顺、螺纹脱离、支撑检查、持续持盖或真机。', '']
    targets[2].write_text('\n'.join(lines))
    return summary
