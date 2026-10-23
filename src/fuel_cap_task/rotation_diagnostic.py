"""固定轴向深度的单构型旋转初筛；不闭合手指、不模拟螺纹或动力学。"""
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from .arm_ik import IKSettings, body_transform, rotation_error, transform
from .fixed_grasp import DEFAULT_FIXED_GRASP
from .grasp_search import pose_arrays, rotation
from .interfaces import TaskParameters
from .planning_scene_snapshot import scene_snapshot
from .trajectory_replay import ApproachReplay, ROOT


def hand_target(cap_pose, cap_from_hand, angle_rad):
    """绕盖的局部外向 Z 轴转动，再以固定手盖关系求手根目标。"""
    return cap_pose @ transform(np.zeros(3), rotation(2, angle_rad)) @ cap_from_hand


@dataclass(frozen=True)
class RotationScanSettings:
    """扫描范围是几何分析预算；不作为必须拧满的角度或执行停止阈值。"""
    max_angle_deg: float
    angle_step_deg: float = 1.

    def __post_init__(self):
        for value in (self.max_angle_deg, self.angle_step_deg):
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError('扫描范围和角度步长必须为有限正值')
        if self.angle_step_deg > 1.:
            raise ValueError('角度步长不得超过合同的 1° 诊断尺度')

    def angles(self):
        """按指定微元推进，最后一步可较短；包含起点和精确扫描终点。"""
        count = int(np.ceil(self.max_angle_deg/self.angle_step_deg))
        return np.r_[np.arange(count)*self.angle_step_deg, self.max_angle_deg]


def joint_margins(q, limits):
    """保留上下限距离；最近边界距离不表示沿拧盖方向的剩余角度。"""
    q, limits = np.asarray(q, dtype=float), np.asarray(limits, dtype=float)
    if (q.shape != (6,) or limits.shape != (6, 2)
            or not np.isfinite(q).all() or not np.isfinite(limits).all()
            or np.any(limits[:, 0] >= limits[:, 1])):
        raise ValueError('需要六个有限关节角及有效的上下限')
    lower, upper = q-limits[:, 0], limits[:, 1]-q
    nearest = np.minimum(lower, upper)
    return {'joint_lower_margin_rad': lower.tolist(), 'joint_upper_margin_rad': upper.tolist(),
            'joint_margin_rad': nearest.tolist(), 'minimum_joint_margin_rad': float(nearest.min())}


def solve_continuous(preview, target, seed):
    """仅使用上一步的臂关节状态，避免重新随机求解导致构型跳变。"""
    model, data = preview.model, preview.data
    cfg = IKSettings()
    q = np.array(seed, dtype=float)
    weight = np.array([1., 1., 1., cfg.rotation_weight_m, cfg.rotation_weight_m, cfg.rotation_weight_m])
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    def residual(values):
        data.qpos[preview.addresses] = values
        mujoco.mj_forward(model, data)
        actual = body_transform(data, 'base')
        return np.r_[target[:3, 3] - actual[:3, 3],
                     rotation_error(target[:3, :3], actual[:3, :3])]
    for _ in range(cfg.iterations):
        error = residual(q)
        if np.linalg.norm(error[:3]) <= .0001 and np.linalg.norm(error[3:]) <= .002:
            return q, error, True
        # Jacobian 将六个臂关节的小变化映射到手根位姿误差；阻尼用于稳定数值求解。
        mujoco.mj_jacBody(model, data, jp, jr, model.body('base').id)
        jac = np.vstack((jp[:, preview.dofs], jr[:, preview.dofs])) * weight[:, None]
        delta = jac.T @ np.linalg.solve(jac @ jac.T + cfg.damping**2*np.eye(6), error*weight)
        delta *= min(1., cfg.max_step_rad / max(np.linalg.norm(delta), 1e-12))
        # 限位内线搜索；不收敛就记录失败，不能据此证明全局没有逆解。
        score = np.linalg.norm(error*weight)
        for scale in (1., .5, .25, .125, .0625):
            trial = np.clip(q + scale*delta, preview.limits[:, 0], preview.limits[:, 1])
            if np.linalg.norm(residual(trial)*weight) < score:
                q = trial
                break
        else:
            break
    error = residual(q)
    return q, error, False


class RotationDiagnostic:
    def __init__(self, trajectory):
        self.path = Path(trajectory)
        self.replay = ApproachReplay(self.path)
        self.replay.validate()
        self.replay.set_progress(1)
        from .arm_ik import ArmIKPreview
        request = self.replay.report['request']
        candidate = request.get('candidate_path', str(DEFAULT_FIXED_GRASP.relative_to(ROOT)))
        self.preview = ArmIKPreview(ROOT/candidate, scene=self.replay.scene)
        self.start = self.replay.data.qpos.copy()
        self.q0 = self.start[self.preview.addresses].copy()
        self.cap0 = transform(*pose_arrays(asdict(self.preview.estimate.pose)))
        self.relative = transform(*pose_arrays(self.preview.candidate['cap_from_hand']))
        self.cap_joint = self.preview.model.joint('cap_twist')
        self.cap_address = int(self.cap_joint.qposadr[0])
        axis = self.preview.data.xaxis[self.cap_joint.id]
        if not np.allclose(axis, self.cap0[:3, 2], atol=1e-10, rtol=0):
            raise ValueError('模型盖关节轴与估计外向轴不一致，停止诊断')
        self.world = json.loads((ROOT/'src/i10_rh56_moveit_config/config/scene.json').read_text())
        self.world = scene_snapshot(self.world, self.preview.model, self.preview.data)
        self.cap_objects = [o for o in self.world['objects'] if o['source_body'] in ('cap_body', 'cap_grip_center')]
        if not self.cap_objects:
            raise ValueError('规划场景缺少盖体几何')

    def set_state(self, q, angle_deg):
        self.preview.data.qpos[:] = self.start
        self.preview.data.qpos[self.preview.addresses] = q
        self.preview.data.qpos[self.cap_address] += np.deg2rad(angle_deg)
        self.preview.data.qvel[:] = 0
        mujoco.mj_forward(self.preview.model, self.preview.data)

    def row(self, q, angle_deg, *, detailed=False):
        self.set_state(q, angle_deg)
        margins = joint_margins(q, self.preview.limits)
        # 既有分段摘要按原字段严格复核；新增上下限字段只用于新扫描报告。
        if not detailed:
            margins = {key: margins[key] for key in ('minimum_joint_margin_rad', 'joint_margin_rad')}
        return {'angle_deg': float(angle_deg), 'arm_qpos_rad': q.tolist(),
                **margins,
                'cap_objects': scene_snapshot({'frame_id': 'world', 'objects': self.cap_objects},
                                              self.preview.model, self.preview.data)['objects']}

    def run(self, checkpoint, *, settings=None):
        checkpoint = Path(checkpoint)
        if checkpoint.exists():
            raise ValueError('已有旋转诊断，不覆盖或续跑；请使用独立输出文件')
        settings = settings or RotationScanSettings(TaskParameters().unlock_angle_deg)
        report = {'schema_version': 1, 'mode': 'fixed_depth_preform_rotation_diagnostic_not_execution',
                  'trajectory_path': str(self.path.resolve().relative_to(ROOT)),
                  'trajectory_sha256': hashlib.sha256(self.path.read_bytes()).hexdigest(),
                  'target_angle_deg': settings.max_angle_deg, 'scan_settings': asdict(settings),
                  'joint_limits_rad': self.preview.limits.tolist(),
                  'analysis_source_hashes': {
                      str(Path(__file__).resolve().relative_to(ROOT)):
                      hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
                  'hand_state': 'approach_preform_not_closed', 'axial_motion_m': 0.,
                  'assumption': '固定手盖相对候选；不证明接触保持或螺纹运动',
                  'initial_joint_positions': self.replay.report['request']['initial_joint_positions'],
                  'arm_joints': self.replay.report['request']['arm_joints'],
                  'planning_scene': self.world, 'samples': [], 'failure': None,
                  'mujoco_prefix_angle_deg': 0., 'analysis_finished': False,
                  'physical_grasp_verified': False, 'thread_motion_verified': False}
        def save():
            checkpoint.parent.mkdir(parents=True, exist_ok=True)
            tmp = checkpoint.with_suffix('.tmp')
            tmp.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
            tmp.replace(checkpoint)
        save()
        previous, previous_angle = self.q0.copy(), 0.
        for angle in settings.angles():
            self.set_state(previous, angle)
            target = hand_target(self.cap0, self.relative, np.deg2rad(angle))
            if angle == 0:
                q = previous.copy()
                actual = body_transform(self.preview.data, 'base')
                error = np.r_[target[:3, 3]-actual[:3, 3], rotation_error(target[:3, :3], actual[:3, :3])]
                found = np.linalg.norm(error[:3]) <= .0001 and np.linalg.norm(error[3:]) <= .002
            else:
                q, error, found = solve_continuous(self.preview, target, previous)
            failure = None
            if np.asarray(q).shape != (6,) or not np.isfinite(q).all():
                failure = {'kind': 'invalid_solver_state', 'angle_deg': float(angle)}
            elif not found:
                failure = {'kind': 'continuous_ik_not_converged', 'angle_deg': float(angle),
                           'position_error_m': float(np.linalg.norm(error[:3])),
                           'orientation_error_rad': float(np.linalg.norm(error[3:]))}
            elif joint_margins(q, self.preview.limits)['minimum_joint_margin_rad'] < 0:
                failure = {'kind': 'joint_bounds', 'angle_deg': float(angle)}
            elif np.max(np.abs(q-previous)) > .1:
                failure = {'kind': 'joint_branch_jump', 'angle_deg': float(angle)}
            else:
                # 在两个连续解之间加密，盖体角度同步变化；只检查已求出的局部段。
                steps = max(1, int(np.ceil(np.max(np.abs(q-previous))/.02)))
                segment = []
                for fraction in np.linspace(1/steps, 1., steps):
                    dense_q = previous + fraction*(q-previous)
                    dense_angle = previous_angle + fraction*(angle-previous_angle)
                    self.set_state(dense_q, dense_angle)
                    contacts = self.preview.contacts()
                    forbidden = [c for c in contacts if c['penetration_m'] > .0001]
                    actual = body_transform(self.preview.data, 'base')
                    expected = hand_target(self.cap0, self.relative, np.deg2rad(dense_angle))
                    if forbidden:
                        failure = {'kind': 'mujoco_robot_interference', 'angle_deg': float(dense_angle), 'contacts': forbidden}
                    elif (np.linalg.norm(actual[:3, 3]-expected[:3, 3]) > .0001
                          or np.linalg.norm(rotation_error(expected[:3, :3], actual[:3, :3])) > .002):
                        failure = {'kind': 'dense_pose_deviation', 'angle_deg': float(dense_angle)}
                    if failure:
                        # 保存真正发生失败时的状态，供人工复核；不把它加入通过前缀。
                        failure['arm_qpos_rad'] = dense_q.tolist()
                        failure['cap_qpos_rad'] = float(self.preview.data.qpos[self.cap_address])
                        failure.update(joint_margins(dense_q, self.preview.limits))
                        failure['position_error_m'] = float(np.linalg.norm(actual[:3, 3]-expected[:3, 3]))
                        failure['orientation_error_rad'] = float(np.linalg.norm(rotation_error(expected[:3, :3], actual[:3, :3])))
                        break
                    row = self.row(dense_q, dense_angle, detailed=True)
                    row['position_error_m'] = float(np.linalg.norm(actual[:3, 3]-expected[:3, 3]))
                    row['orientation_error_rad'] = float(np.linalg.norm(rotation_error(expected[:3, :3], actual[:3, :3])))
                    segment.append(row)
                report['samples'].extend(segment)
            if failure:
                if failure['kind'] != 'invalid_solver_state' and 'arm_qpos_rad' not in failure:
                    failure.update(arm_qpos_rad=q.tolist(), **joint_margins(q, self.preview.limits))
                    failure['position_error_m'] = float(np.linalg.norm(error[:3]))
                    failure['orientation_error_rad'] = float(np.linalg.norm(error[3:]))
                report['failure'] = failure
                break
            previous, previous_angle = q.copy(), float(angle)
            report['mujoco_prefix_angle_deg'] = previous_angle
            save()
        report['analysis_finished'] = True
        # 失败可能发生在微元内部；保留实际检查通过的最后一个加密状态。
        report['mujoco_prefix_angle_deg'] = report['samples'][-1]['angle_deg'] if report['samples'] else 0.
        save()
        return report


class RotationReplay:
    """只显示 MuJoCo 与完整 FCL 都检查通过的前缀；失败角度单独标记。"""
    def __init__(self, diagnostic_path, fcl_path):
        path = Path(diagnostic_path)
        self.diagnostic = json.loads(path.read_text())
        self.fcl = json.loads(Path(fcl_path).read_text())
        if (self.diagnostic.get('mode') != 'fixed_depth_preform_rotation_diagnostic_not_execution'
                or not self.diagnostic.get('analysis_finished')
                or self.fcl.get('mode') != 'full_fcl_rotation_prefix_audit_not_execution'
                or self.fcl.get('diagnostic_sha256') != hashlib.sha256(path.read_bytes()).hexdigest()):
            raise ValueError('旋转报告未完成或来源不一致')
        self.source = RotationDiagnostic(ROOT/self.diagnostic['trajectory_path'])
        if self.diagnostic['trajectory_sha256'] != hashlib.sha256(self.source.path.read_bytes()).hexdigest():
            raise ValueError('接近轨迹已变更')
        from .planning_scene_snapshot import check_snapshot
        check_snapshot(self.diagnostic['planning_scene'], self.source.world)
        if self.diagnostic['initial_joint_positions'] != self.source.replay.report['request']['initial_joint_positions']:
            raise ValueError('预型或初始状态不一致')
        if 'scan_settings' in self.diagnostic:
            settings = RotationScanSettings(**self.diagnostic['scan_settings'])
            if self.diagnostic['target_angle_deg'] != settings.max_angle_deg:
                raise ValueError('旋转目标与保存的扫描范围不一致')
        elif self.diagnostic['target_angle_deg'] != TaskParameters().unlock_angle_deg:
            raise ValueError('历史旋转目标与合同不一致')
        for relative, expected in self.diagnostic.get('analysis_source_hashes', {}).items():
            source = (ROOT/relative).resolve()
            if not source.is_relative_to(ROOT) or hashlib.sha256(source.read_bytes()).hexdigest() != expected:
                raise ValueError('旋转诊断算法来源已变更；历史结果不可当作当前计算')
        count = self.fcl['passed_samples']
        checked = self.fcl['checked_samples']
        rows = self.diagnostic['samples']
        if (type(count) is not int or not 0 < count <= len(rows)
                or checked != count + (1 if self.fcl['failure'] else 0)
                or (not self.fcl['failure'] and count != len(rows))):
            raise ValueError('FCL 检查前缀无效或起点已碰撞')
        self.rows = rows[:count]
        self.angles = np.array([r['angle_deg'] for r in self.rows])
        self.points = np.array([r['arm_qpos_rad'] for r in self.rows])
        if (self.points.shape != (count, 6) or not np.isfinite(self.points).all()
                or not np.isfinite(self.angles).all() or self.angles[0] != 0
                or np.any(np.diff(self.angles) <= 0)
                or self.angles[-1] > self.diagnostic['target_angle_deg']
                or not np.allclose(self.points[0], self.source.q0, atol=1e-12, rtol=0)
                or abs(self.angles[-1]-self.fcl['clear_prefix_angle_deg']) > 1e-12):
            raise ValueError('旋转路点或起点不一致')
        self.model, self.data = self.source.preview.model, self.source.preview.data
        # 数值回放复核记录中的盖体位姿、手根误差、碰撞与限位，不重新搜索 IK。
        for row in self.rows:
            q = np.array(row['arm_qpos_rad'])
            if np.any(q < self.source.preview.limits[:, 0]) or np.any(q > self.source.preview.limits[:, 1]):
                raise ValueError('旋转路点越界')
            expected_row = self.source.row(q, row['angle_deg'], detailed='scan_settings' in self.diagnostic)
            check_snapshot({'objects': row['cap_objects']}, {'objects': expected_row['cap_objects']})
            if 'scan_settings' in self.diagnostic:
                for key in ('joint_lower_margin_rad', 'joint_upper_margin_rad', 'joint_margin_rad',
                            'minimum_joint_margin_rad'):
                    if not np.allclose(row[key], expected_row[key], atol=1e-12, rtol=0):
                        raise ValueError('关节余量与记录的关节角不一致')
            actual = body_transform(self.data, 'base')
            target = hand_target(self.source.cap0, self.source.relative, np.deg2rad(row['angle_deg']))
            if (np.linalg.norm(actual[:3, 3]-target[:3, 3]) > .0001
                    or np.linalg.norm(rotation_error(target[:3, :3], actual[:3, :3])) > .002
                    or any(c['penetration_m'] > .0001 for c in self.source.preview.contacts())):
                raise ValueError('旋转前缀复核失败')
        if count > 1 and np.max(np.abs(np.diff(self.points, axis=0))) > .02000001:
            raise ValueError('旋转路点不连续')
        # 复用现有查看器只需要模型、状态和目标参考位置；不复用接近运动的回执。
        self.report = self.source.replay.report
        self.set_progress(0)

    def set_progress(self, fraction):
        angle = float(np.clip(fraction, 0, 1))*self.angles[-1]
        q = np.array([np.interp(angle, self.angles, self.points[:, j]) for j in range(6)])
        self.source.set_state(q, angle)
        return f'固定深度旋转初筛 {angle:.1f}°；手保持接近预型'
