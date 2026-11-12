"""一次性小试：一个靠端拇指锚点、四指可放弃的对侧网络引导。

只在本目录保存结果。求解中消去根平移以保持拇指指面点，接触仿真
没有黏接；固定握梁不具有自由度。长度用米，角用弧度，力用牛顿。
"""
import importlib.util
import json
from pathlib import Path
import sys
import time

import mujoco
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from threadpoolctl import threadpool_limits

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from fuel_cap_task.fixed_contact_kinematics import NativeHandKinematics, skew
from fuel_cap_task.joint_collision_update import solve_joint_residual_step
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE
from fuel_cap_task.saved_sdk_input import digest_file, save_json
from fuel_cap_task.signed_hand_clearance import SignedHandClearance


def load_helper(name, path):
    """仅复用旧小试的模型与受力函数，不运行其main或改写旧结果。"""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


OLD = OUT.parent / 'cap_thumb_anchor_spike_20261112_001'
physics = load_helper('anchor_spike_physics', OLD / 'spike.py')
forces = load_helper('anchor_spike_forces', OLD / 'fixed_forces.py')
transform = physics.transform
FINGERS = ['pinky', 'ring', 'middle', 'index']


class Trial:
    """一个锚点共享一个状态；四指目标只取原网络分数合格的对侧格。"""
    def __init__(self):
        self.run = OUT.parent / 'cap_confirmed_grasps_20261112_002/run_candidates'
        self.ref_file = OUT.parent / 'cap_single_reference_20261111_001/result.json'
        self.side_file = OUT.parent / 'cap_confirmed_sides_20261112_001/confirmed_geometry.npz'
        self.ref = json.loads(self.ref_file.read_text())
        self.vol = dict(np.load(self.run / 'network_volume.npz'))
        geo = dict(np.load(self.run / 'geometry_1mm.npz'))
        self.pred = dict(np.load(self.run / 'contacts.npz'))
        self.confirmed = dict(np.load(self.side_file))
        self.frame = self.confirmed['volume_from_manual'].copy()
        self.frame[:3, :3] = Rotation.from_matrix(self.frame[:3, :3]).as_matrix()
        self.inverse_frame = np.linalg.inv(self.frame)
        self.points = transform(geo['points_volume_m'], np.linalg.inv(self.vol['reference_from_volume']) @ geo['reference_from_volume'])
        self.mask = geo['target_point_mask']
        self.kin = NativeHandKinematics(self.ref['model_dir'], DEFAULT_SCENE)
        self.check = SignedHandClearance(self.kin, self.points, self.mask)
        self.reference_pose = np.linalg.inv(self.vol['reference_from_volume']) @ np.asarray(self.ref['reference_from_volume']) @ np.asarray(self.ref['final']['pose'])
        self.reference_drivers = np.asarray(self.ref['final']['drivers'])
        self.thumb = self.ref['contacts'][0]
        # 直接复用先前靠端点，避免本轮重新筛选多个拇指候选。
        self.old_end = json.loads((OLD / 'placement_probe/candidate_0.json').read_text())
        self.anchor_local = np.asarray(self.old_end['anchor_manual_m'])
        self.anchor = transform(self.anchor_local[None], self.frame)[0]
        self.normal = self.frame[:3, :3] @ np.array([0., 1., 1 / 17])
        self.normal /= np.linalg.norm(self.normal)
        self.hold_point = self.anchor + .0013 * self.normal
        self.ids = np.argwhere(self.pred['quality'] >= .1)
        self.raw = self.ids * .001
        local = transform(self.raw, self.inverse_frame)
        valid = (local[:, 1] < 0) & (abs(local[:, 0]) <= .03) & (local[:, 2] >= -.017) & (local[:, 2] <= 0)
        self.pool_ids = self.ids[valid]
        self.pool_local = local[valid].copy()
        self.pool_local[:, 1] = -(.0085 - self.pool_local[:, 2] / 17)
        self.pool = transform(self.pool_local, self.frame)
        self.pool_tree = cKDTree(self.pool)
        assert len(self.pool) and self.pred['quality'][tuple(self.old_end['feature_index_xyz'])] == self.old_end['raw_network_score']
        self.pads = {}
        for finger in FINGERS:
            regions = [r for r in self.kin.metadata['contact_regions'] if r['finger'] == finger]
            self.pads[finger] = [dict(link=r['link'], point_link_m=np.mean(r['vertices_link_m'][:3], axis=0), normal_link=r['normal_link']) for r in regions]
            self.pads[finger] += [dict(link=regions[0]['link'], point_link_m=regions[0]['tip_site_link_m'], normal_link=regions[0]['closing_direction_link'])]
            self.pads[finger] += [p for p in self.ref['contacts'] if p.get('finger') == finger]
        self.rotation = self.reference_pose[:3, :3].copy()
        self.drivers = self.reference_drivers.copy()
        self.drivers[:4] = np.maximum(self.kin.drives.lower[:4], self.drivers[:4] - .075)
        self.install(self.rotation, self.drivers)

    def install(self, rotation, drivers):
        """保持拇指同一物质点的位置；根平移由当前拇指FK精确算出。"""
        self.kin.set_state(np.eye(4), drivers)
        p = self.kin.point(self.thumb['link'], self.thumb['point_link_m'], self.thumb['normal_link'])[0]
        pose = np.eye(4)
        pose[:3, :3] = rotation
        pose[:3, 3] = self.hold_point - rotation @ p
        self.kin.set_state(pose, drivers)
        return pose

    def anchor_basis(self):
        """12维原增量到9维绕锚点旋转/六驱动增量的解析映射。"""
        p, _, jac, _ = self.kin.point(self.thumb['link'], self.thumb['point_link_m'], self.thumb['normal_link'])
        basis = np.zeros((12, 9))
        basis[:3, :3] = skew(p - self.kin.pose[:3, 3])
        basis[:3, 3:] = -jac[:, 6:]
        basis[3:6, :3] = np.eye(3)
        basis[6:, 3:] = np.eye(6)
        return basis

    def objective(self):
        """每指挑最近的指面/对侧目标；20mm外饱和且梯度为零，可放弃。"""
        residual, jacobians, rows = [], [], []
        for finger in FINGERS:
            options = []
            for pad_id, pad in enumerate(self.pads[finger]):
                point, _, jac, _ = self.kin.point(pad['link'], pad['point_link_m'], pad['normal_link'])
                distance, index = self.pool_tree.query(point)
                options.append((distance, int(index), pad_id, point, jac))
            distance, index, pad_id, point, jac = min(options, key=lambda v: v[0])
            active = distance < .020
            residual.extend((point - self.pool[index]) / .005 if active else [4., 0., 0.])
            jacobians.extend(jac / .005 if active else np.zeros((3, 12)))
            rows.append(dict(finger=finger, active=bool(active), distance_m=float(distance), pad_id=pad_id,
                             feature_index_xyz=self.pool_ids[index].tolist(),
                             raw_network_score=float(self.pred['quality'][tuple(self.pool_ids[index])]),
                             projected_target_manual_m=self.pool_local[index].tolist(),
                             pad_point_manual_m=transform(point[None], self.inverse_frame)[0].tolist()))
        return np.asarray(residual), np.asarray(jacobians), rows


def collision_severity(checked):
    """只用于放置修复排序；成功仍要求原0.1mm容差全部通过。"""
    return sum((v / .001) ** 2 for v in checked['maxima'].values())


def optimize(trial):
    """最多24次局部更新；先修复放置，再在可行区域拉近可达四指。"""
    kin = trial.kin
    trace = []
    lower = np.maximum(kin.drives.lower, trial.reference_drivers - .4)
    upper = np.minimum(kin.drives.upper, trial.reference_drivers + .4)
    scales = np.array([.07] * 3 + [.10] * 6)
    stalled = 0
    for iteration in range(24):
        trial.install(trial.rotation, trial.drivers)
        checked = trial.check.evaluate(linearize=True)
        r, J, finger_rows = trial.objective()
        basis = trial.anchor_basis()
        gaps, A = checked['gaps'], checked['jacobian'] @ basis
        lo = np.r_[-scales[:3], np.maximum(-scales[3:], lower - trial.drivers)]
        hi = np.r_[scales[:3], np.minimum(scales[3:], upper - trial.drivers)]
        # 全局只允许绕参考姿态小范围变化，防止一次小试变成全局搜索。
        if checked['feasible']:
            step, info = solve_joint_residual_step(r, J @ basis, gaps, A, lo, hi, scales)
        else:
            bad = gaps < 0
            repair_r = np.r_[-gaps[bad] / .001, r * .01]
            repair_J = np.row_stack([-A[bad] / .001, (J @ basis) * .01])
            step, info = solve_joint_residual_step(repair_r, repair_J, np.empty(0), np.empty((0, 9)), lo, hi, scales)
        before = collision_severity(checked)
        accepted, chosen_scale = False, 0.
        old_rotation, old_drivers = trial.rotation.copy(), trial.drivers.copy()
        for scale in [1., .5, .25, .125]:
            new_rotation = Rotation.from_rotvec(step[:3] * scale).as_matrix() @ old_rotation
            if Rotation.from_matrix(new_rotation @ trial.reference_pose[:3, :3].T).magnitude() > .6:
                continue
            new_drivers = old_drivers + step[3:] * scale
            trial.install(new_rotation, new_drivers)
            after = trial.check.evaluate()
            new_r, _, _ = trial.objective()
            good = (after['feasible'] and new_r @ new_r < r @ r - 1e-9 if checked['feasible']
                    else collision_severity(after) < before - 1e-10)
            if good:
                trial.rotation, trial.drivers = new_rotation, new_drivers
                accepted, chosen_scale = True, scale
                break
        if not accepted:
            trial.install(old_rotation, old_drivers)
        current = trial.check.evaluate()
        _, _, current_fingers = trial.objective()
        p = kin.point(trial.thumb['link'], trial.thumb['point_link_m'], trial.thumb['normal_link'])[0]
        row = dict(iteration=iteration, accepted=accepted, scale=chosen_scale, solver=info,
                   feasible=current['feasible'], collision=current['maxima'], fingers=current_fingers,
                   thumb_anchor_error_m=float(np.linalg.norm(p - trial.hold_point)),
                   volume_from_hand=kin.pose.tolist(), drivers=trial.drivers.tolist())
        trace.append(row)
        print(json.dumps({k: row[k] for k in ['iteration', 'accepted', 'feasible', 'collision']}) , flush=True)
        stalled = 0 if accepted else stalled + 1
        if stalled >= 3:
            break
    return trace


def close_once(trial, row):
    """只初始化一次qpos，之后用ctrl/mj_step；不固定拇指到握梁。"""
    kin = trial.kin
    pose = kin.pose.copy()
    planned = trial.drivers.copy()
    _, _, targets = trial.objective()
    initial = planned.copy()
    # 四指各自预张开；不让未被选作接触的指也必须追点。
    for i, target in enumerate(targets):
        if not target['active']:
            continue
        for opening in [.10, .05, .025, 0.]:
            proposed = initial.copy()
            proposed[i] = max(kin.drives.lower[i], planned[i] - opening)
            kin.set_state(pose, proposed)
            if trial.check.evaluate()['feasible']:
                initial = proposed
                break
    kin.set_state(pose, initial)
    model, data, xml = physics.dynamics(kin, np.linalg.inv(pose) @ trial.frame, trial.confirmed['vertices_manual_m'])
    assert model.nq == 12 and not any(model.eq_type == mujoco.mjtEq.mjEQ_WELD)
    (OUT / 'candidate.xml').write_text(xml)
    for name, value in zip(kin.drives.joint_names, kin.drives.expand(initial)):
        data.joint(name).qpos[0] = value
    controls = np.array([model.actuator(n).id for n in ['pinky', 'ring', 'middle', 'index', 'thumb_proximal', 'thumb_yaw']])
    data.ctrl[controls] = initial
    mujoco.mj_forward(model, data)
    grip_id = model.geom('spike_grip_collision').id
    initial_depth = max([-float(c.dist) for c in data.contact if grip_id in c.geom] + [0.])
    target_ctrl = planned.copy()
    for i, target in enumerate(targets):
        if target['active']:
            target_ctrl[i] = min(kin.drives.upper[i], planned[i] + .06)
    # 拇指以原六驱动中的两路靠近接触；方向从该点的位置Jacobian确定。
    kin.set_state(pose, initial)
    _, _, jp, _ = kin.point(trial.thumb['link'], trial.thumb['point_link_m'], trial.thumb['normal_link'])
    closing = -trial.normal @ jp[:, 10:12]
    delta = np.clip(closing * .0025 / max(closing @ closing, 1e-12), -.08, .08)
    target_ctrl[4:] = np.clip(initial[4:] + delta, kin.drives.lower[4:], kin.drives.upper[4:])
    row.update(volume_from_hand=pose.tolist(), initial_drivers=initial.tolist(), target_drivers=target_ctrl.tolist(),
               initial_proxy_penetration_m=initial_depth, selected_targets=targets,
               contact_trace=[], geometry_trace=[], force_trace=[], physics_steps=0)
    if initial_depth > .0001:
        row['status'] = 'initial_proxy_penetration'
        return
    held = None
    first = None
    failure = None
    qstates, ctrlstates, times = [], [], []
    warning = data.warning.number.copy()
    for step in range(2800):
        data.ctrl[controls] = initial + min(1., (step + 1) / 2000) * (target_ctrl - initial) if held is None else held
        mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or np.any(data.warning.number > warning):
            failure = 'numerical_failure'
            break
        if step % 50 == 49:
            mujoco.mj_forward(model, data)
            actual = np.array([data.joint(n).qpos[0] for n in kin.drives.joint_names])
            kin.pose = pose.copy()
            kin.hand.place(kin.drives.joint_names, actual)
            checked = trial.check.evaluate()
            obs = physics.contacts(model, data)
            force = forces.extract(model, data)
            if not force['contacts']:
                for key in ['hand_net_force_on_grip_manual_n', 'hand_net_moment_on_grip_manual_nm', 'required_support_force_manual_n', 'required_support_moment_manual_nm']:
                    force[key] = [0., 0., 0.]
            p = kin.point(trial.thumb['link'], trial.thumb['point_link_m'], trial.thumb['normal_link'])[0]
            obs['thumb_pad_manual_m'] = transform(p[None], trial.inverse_frame)[0].tolist()
            if obs['opposed'] and held is None:
                first, held = float(data.time), data.ctrl[controls].copy()
            row['contact_trace'].append(dict(time_s=float(data.time), **obs))
            row['geometry_trace'].append(dict(time_s=float(data.time), maxima=checked['maxima'], feasible=checked['feasible']))
            row['force_trace'].append(force)
            qstates.append(data.qpos.copy()); ctrlstates.append(data.ctrl.copy()); times.append(float(data.time))
            if not checked['feasible']:
                failure = 'closure_collision'
                break
            if first is not None and data.time >= first + .2 - 1e-8:
                break
    row['physics_steps'] = step + 1
    tail = [v for v in row['contact_trace'] if first is not None and v['time_s'] >= first - 1e-8]
    row['sustained_opposed_contact'] = bool(first is not None and tail and all(v['opposed'] for v in tail) and tail[-1]['time_s'] >= first + .2 - 1e-8 and failure is None)
    row['status'] = failure or ('fixed_opposed_contact' if row['sustained_opposed_contact'] else 'no_sustained_opposed_contact')
    row['first_opposed_s'] = first
    row['final_forces'] = row['force_trace'][-1] if row['force_trace'] else None
    np.savez_compressed(OUT / 'states.npz', qpos=np.array(qstates), ctrl=np.array(ctrlstates), time_s=np.array(times),
                        joint_names=np.array(kin.drives.joint_names), volume_from_hand=pose, final_qpos=data.qpos, final_qvel=data.qvel)


def main():
    """来源冻结、单锚点求解、至多一次闭合；所有失败也完整保存。"""
    if (OUT / 'result.json').exists():
        raise FileExistsError('拒绝覆盖本次结果')
    started = time.monotonic()
    trial = Trial()
    files = [Path(__file__), OLD / 'spike.py', OLD / 'fixed_forces.py', OLD / 'placement_probe/candidate_0.json',
             trial.ref_file, trial.side_file, Path(DEFAULT_SCENE), Path(trial.ref['model_dir']) / 'model.json',
             trial.run / 'contacts.npz', trial.run / 'network_volume.npz', trial.run / 'geometry_1mm.npz']
    files += trial.kin.hand.mesh_paths
    files += [ROOT / 'src/fuel_cap_task' / f for f in ['fixed_contact_kinematics.py', 'signed_hand_clearance.py', 'observed_hand_geometry.py', 'joint_collision_update.py']]
    hashes = {str(p): digest_file(p) for p in files}
    manifest = dict(input_sha256=hashes, source_run=str(trial.run), candidate_budget=1, geometric_update_budget=24,
                    closure_budget=1, network_forwards=0, hardware_execution_performed=False, mujoco_version=mujoco.__version__,
                    contact_attraction_cutoff_m=.020, initial_thumb_outward_gap_m=.0013,
                    physics_hypotheses=dict(grip_fixed=True, no_hand_grip_weld=True, gravity=[0, 0, 0], friction=[1, .005, .001],
                        actuator_force_limit_nm=1., servo_kp=300., calibrated=False))
    save_json(OUT / 'manifest.json', manifest)
    row = dict(feature_index_xyz=trial.old_end['feature_index_xyz'], raw_network_score=trial.old_end['raw_network_score'],
               anchor_manual_m=trial.anchor_local.tolist(), raw_feature_manual_m=trial.old_end['raw_feature_manual_m'],
               opposite_pool_count=len(trial.pool), status='no_safe_geometric_pose', sustained_opposed_contact=False,
               physical_grasp_verified=False, motion_target_valid=False, physics_steps=0)
    row['optimization_trace'] = optimize(trial)
    if trial.check.evaluate()['feasible']:
        close_once(trial, row)
    row['elapsed_seconds'] = time.monotonic() - started
    assert all(digest_file(Path(p)) == sha for p, sha in hashes.items())
    row['input_unchanged'] = True
    save_json(OUT / 'result.json', row)
    print(json.dumps({k: row[k] for k in ['status', 'sustained_opposed_contact', 'physics_steps', 'elapsed_seconds']}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    with threadpool_limits(limits=1):
        main()
