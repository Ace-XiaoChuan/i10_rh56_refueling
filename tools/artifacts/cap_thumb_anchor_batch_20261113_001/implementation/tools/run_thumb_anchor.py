#!/usr/bin/env python3
"""从本次网络与握梁定位自动选锚点，进行有预算的几何和固定梁接触验证。"""
import argparse
import copy
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from threadpoolctl import threadpool_limits

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from fuel_cap_task.cap_reference import camera_surface_pose_candidates
from fuel_cap_task.cap_geometry import estimate_cap_geometry
from fuel_cap_task.fixed_contact_kinematics import NativeHandKinematics, skew
from fuel_cap_task.joint_collision_update import project_joint_step, solve_joint_residual_step
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE
from fuel_cap_task.pcd_point_cloud import load_pcd
from fuel_cap_task.saved_sdk_input import digest_file, save_json
from fuel_cap_task.signed_hand_clearance import SignedHandClearance
from fuel_cap_task.thumb_anchor import ContactSequence, GripShape, anchored_pose, rank_anchors, rigid, transform

FINGERS = ['pinky', 'ring', 'middle', 'index']
ACTUATORS = ['pinky', 'ring', 'middle', 'index', 'thumb_proximal', 'thumb_yaw']


def load_npz(path):
    """数组不允许pickle；输入由同一次观测的清单绑定。"""
    with np.load(path, allow_pickle=False) as data:
        return dict(data)


def normalized_frame(matrix):
    """只修正历史保存旋转的舍入误差，超出刚体容差直接拒绝。"""
    value = rigid(matrix)
    value[:3, :3] = Rotation.from_matrix(value[:3, :3]).as_matrix()
    return value


def observation_frame(config, volume):
    """当前定位转入网络体素系；人工侧面与自动相机估计保留不同来源。"""
    modes = [key for key in ['cap_cloud', 'grip_estimate', 'confirmed_geometry'] if config.get(key)]
    if len(modes) != 1:
        raise ValueError('定位来源须在 cap_cloud、grip_estimate、confirmed_geometry 中选一个')
    if config.get('grip_estimate') or config.get('cap_cloud'):
        binding = config.get('estimate_network_sha256')
        if binding != digest_file(Path(config['run_dir']) / 'network_volume.npz'):
            raise ValueError('相机定位需要 estimate_network_sha256 绑定本次网络体素')
        camera_from_grip = np.eye(4)
        if config.get('cap_cloud'):
            path = Path(config['cap_cloud'])
            if config.get('cap_cloud_unit') not in ('m', 'mm'):
                raise ValueError('整盖PCD需要显式 cap_cloud_unit=m 或 mm')
            points = load_pcd(path).xyz_raw * (.001 if config['cap_cloud_unit'] == 'mm' else 1.)
            estimate = estimate_cap_geometry(points)
            camera_from_grip[:3, :3] = estimate.rotation
            camera_from_grip[:3, 3] = estimate.surface_reference_m
            source = 'current_cap_cloud_geometry_fit_with_known_grip_dimensions'
        else:
            path = Path(config['grip_estimate'])
            report = json.loads(path.read_text())
            if report.get('geometric_checks_passed') is not True:
                raise ValueError('相机握梁定位没有通过几何检查')
            poses = camera_surface_pose_candidates(report)
            index = config.get('pose_index', 0)
            if type(index) is not int or not 0 <= index < len(poses):
                raise ValueError('定位候选编号无效')
            pose = poses[index]
            camera_from_grip[:3, :3] = Rotation.from_quat(pose['orientation_xyzw']).as_matrix()
            camera_from_grip[:3, 3] = pose['position_m']
            source = 'camera_geometry_estimate_with_known_grip_dimensions'
        return (normalized_frame(np.linalg.inv(volume['camera_from_volume']) @ camera_from_grip),
                source, path)
    path = Path(config['confirmed_geometry'])
    geometry = load_npz(path)
    if (not np.allclose(geometry['reference_from_volume'], volume['reference_from_volume'], atol=1e-9)
            or not np.array_equal(geometry['confirmed_occupied_surface_grid'],
                                  volume['confirmed_occupied_surface_grid'])):
        raise ValueError('人工侧面与当前网络坐标/占据来源不一致')
    return normalized_frame(geometry['volume_from_manual']), 'human_confirmed_geometry', path


class AnchorTrial:
    """复用相对握梁预型；场景、网络、锚点均来自本次输入。"""
    def __init__(self, config):
        self.config = config
        self.run = Path(config['run_dir'])
        self.ref_path = Path(config['template_reference'])
        self.ref = json.loads(self.ref_path.read_text())
        self.volume = load_npz(self.run / 'network_volume.npz')
        self.geometry = load_npz(self.run / 'geometry_1mm.npz')
        self.pred = load_npz(self.run / 'contacts.npz')
        record_path = self.run / 'contacts.json'
        record = json.loads(record_path.read_text())
        volume_path = self.run / 'volume.json'
        vol_record = json.loads(volume_path.read_text())
        if (record['observation_id'] != vol_record['observation_id']
                or record['frame_id'] != str(self.volume['frame_id'])
                or record['parent_sha256'] != digest_file(volume_path)
                or record.get('prediction_evidence_sufficient') is not True
                or record['files_sha256'].get(str(self.run / 'contacts.npz')) != digest_file(self.run / 'contacts.npz')):
            raise ValueError('网络预测、观测或上游摘要不一致')
        if str(self.geometry['frame_id']) != str(self.volume['frame_id']):
            raise ValueError('完整场景与网络参考系不一致')
        if self.pred['quality'].shape != tuple(self.volume['resolution_xyz']):
            raise ValueError('预测数组与当前网络尺寸不一致')
        hashes = vol_record.get('files_sha256', {})
        for filename in ['network_volume.npz', 'geometry_1mm.npz']:
            if hashes.get(str(self.run / filename)) != digest_file(self.run / filename):
                raise ValueError('体素来源摘要不一致：' + filename)
        self.observation_id = record['observation_id']
        self.frame, self.geometry_source, self.frame_path = observation_frame(config, self.volume)
        self.inverse = np.linalg.inv(self.frame)
        self.shape = GripShape(**config.get('grip_shape', {}))
        self.points = transform(self.geometry['points_volume_m'],
                                np.linalg.inv(self.volume['reference_from_volume']) @ self.geometry['reference_from_volume'])
        self.kin = NativeHandKinematics(config.get('model_dir', self.ref['model_dir']), DEFAULT_SCENE)
        assumed = self.geometry.get('point_source_kind', np.zeros(len(self.points))) != 0
        self.check = SignedHandClearance(self.kin, self.points[~assumed], self.geometry['target_point_mask'][~assumed],
                                         assumed_points=self.points[assumed])
        # 模板保存的是握梁相对姿态；新观测转动/平移后，整手随之变化。
        self.grip_from_hand = np.linalg.inv(normalized_frame(self.ref['volume_from_manual'])) @ np.asarray(self.ref['final']['pose'])
        self.reference_pose = self.frame @ self.grip_from_hand
        self.reference_drivers = np.asarray(self.ref['final']['drivers'])
        self.thumb = next(p for p in self.ref['contacts'] if p['finger'] == 'thumb')
        self.kin.set_state(self.reference_pose, self.reference_drivers)
        thumb_local = transform(self.point(self.thumb), self.inverse)
        offsets = []
        self.pads = {}
        for finger in FINGERS:
            regions = [r for r in self.kin.metadata['contact_regions'] if r['finger'] == finger]
            tip = dict(link=regions[0]['link'], point_link_m=regions[0]['tip_site_link_m'], normal_link=regions[0]['normal_link'])
            contacts = [p for p in self.ref['contacts'] if p.get('finger') == finger]
            sample = contacts[0] if contacts else tip
            offsets.append(float(transform(self.point(sample), self.inverse)[0] - thumb_local[0]))
            self.pads[finger] = [dict(link=r['link'], point_link_m=np.mean(r['vertices_link_m'][:3], axis=0), normal_link=r['normal_link']) for r in regions]
            self.pads[finger] += [tip] + contacts
        self.side = 1 if thumb_local[1] > 0 else -1
        self.anchors = rank_anchors(self.pred['quality'], float(self.volume['voxel_size_m']), self.frame, offsets,
                                    side=self.side, shape=self.shape, budget=config.get('candidate_budget', 3))
        ids = np.argwhere(self.pred['quality'] >= .1)
        local = transform(ids * float(self.volume['voxel_size_m']), self.inverse)
        valid = ((local[:, 1] * self.side < 0) & (abs(local[:, 0]) <= self.shape.half_length_m)
                 & (local[:, 2] >= -self.shape.height_m) & (local[:, 2] <= 0)
                 & (np.linalg.norm(self.shape.project(local, -self.side) - local, axis=1) <= .003))
        self.pool_ids = ids[valid]
        self.pool_local = self.shape.project(local[valid], -self.side)
        self.pool = transform(self.pool_local, self.frame)
        self.pool_tree = cKDTree(self.pool)
        if not len(self.pool):
            raise ValueError('另一侧没有有效网络候选，不能生成对握')
        self.normal = self.frame[:3, :3] @ self.shape.normal(self.side)

    def point(self, pad):
        """返回当前场景中的拇指/指面材料点。"""
        return self.kin.point(pad['link'], pad['point_link_m'], pad['normal_link'])[0]

    def install(self, rotation, drivers):
        """几何求解时消去根平移；接触积分阶段不使用此约束。"""
        self.kin.set_state(np.eye(4), drivers)
        pose = anchored_pose(rotation, self.point(self.thumb), self.hold_point)
        self.kin.set_state(pose, drivers)
        return pose

    def basis(self):
        """把12维根/驱动微分缩为保持拇指点的9维。"""
        point, _, jac, _ = self.kin.point(self.thumb['link'], self.thumb['point_link_m'], self.thumb['normal_link'])
        result = np.zeros((12, 9))
        result[:3, :3] = skew(point - self.kin.pose[:3, 3])
        result[:3, 3:] = -jac[:, 6:]
        result[3:6, :3], result[6:, 3:] = np.eye(3), np.eye(6)
        return result

    def objective(self):
        """近邻指面用于微调；远指暂不拉动，但仍必须进入独立展开/闭合阶段。"""
        residual, jacobians, rows = [], [], []
        for finger in FINGERS:
            options = []
            for pad in self.pads[finger]:
                point, _, jac, _ = self.kin.point(pad['link'], pad['point_link_m'], pad['normal_link'])
                distance, index = self.pool_tree.query(point)
                options.append((float(distance), int(index), point, jac))
            distance, index, point, jac = min(options, key=lambda v: v[0])
            near = distance < .020
            residual.extend((point - self.pool[index]) / .005 if near else [4., 0., 0.])
            jacobians.extend(jac / .005 if near else np.zeros((3, 12)))
            rows.append(dict(finger=finger, locally_attracted=near, distance_m=distance,
                             feature_index_xyz=self.pool_ids[index].tolist(),
                             pad_point_grip_m=transform(point, self.inverse).tolist()))
        return np.asarray(residual), np.asarray(jacobians), rows

    def solve(self, anchor, budget):
        """一个锚点最多24步，末步单独收敛碰撞边界；完整场景决定接受。"""
        self.hold_point = np.array(anchor['anchor_volume_m']) + self.normal * .0013
        rotation = self.reference_pose[:3, :3].copy()
        drivers = self.reference_drivers.copy()
        drivers[:4] = np.maximum(self.kin.drives.lower[:4], drivers[:4] - .075)
        self.install(rotation, drivers)
        trace, stalled = [], 0
        lower = np.maximum(self.kin.drives.lower, self.reference_drivers - .4)
        upper = np.minimum(self.kin.drives.upper, self.reference_drivers + .4)
        for iteration in range(budget):
            checked = self.check.evaluate(linearize=True)
            r, J, _ = self.objective()
            basis = self.basis()
            scales = np.array([.07] * 3 + [.10] * 6)
            lo = np.r_[-scales[:3], np.maximum(-scales[3:], lower - drivers)]
            hi = np.r_[scales[:3], np.minimum(scales[3:], upper - drivers)]
            gaps, A = checked['gaps'], checked['jacobian'] @ basis
            boundary = iteration == budget - 1
            if boundary:
                step, info = project_joint_step(np.zeros(9), gaps, A, lo, hi, scales)
            elif checked['feasible']:
                step, info = solve_joint_residual_step(r, J @ basis, gaps, A, lo, hi, scales)
            else:
                bad = gaps < 0
                step, info = solve_joint_residual_step(np.r_[-gaps[bad] / .001, r * .01],
                    np.vstack([-A[bad] / .001, (J @ basis) * .01]), np.empty(0), np.empty((0, 9)), lo, hi, scales)
            accepted = False
            severity = lambda c: sum((v / .001) ** 2 for v in c['maxima'].values())
            for scale in [1., .5, .25, .125]:
                candidate_r = Rotation.from_rotvec(step[:3] * scale).as_matrix() @ rotation
                if Rotation.from_matrix(candidate_r @ self.reference_pose[:3, :3].T).magnitude() > .6:
                    continue
                candidate_d = drivers + step[3:] * scale
                self.install(candidate_r, candidate_d)
                after = self.check.evaluate()
                new_r, _, _ = self.objective()
                good = (after['feasible'] if boundary else
                        (after['feasible'] and new_r @ new_r < r @ r - 1e-9 if checked['feasible']
                         else severity(after) < severity(checked) - 1e-10))
                if good:
                    rotation, drivers, accepted = candidate_r, candidate_d, True
                    break
            self.install(rotation, drivers)
            current = self.check.evaluate()
            trace.append(dict(iteration=iteration, accepted=accepted, collision=current['maxima'],
                feasible=current['feasible'], volume_from_hand=self.kin.pose.tolist(), drivers=drivers.tolist(),
                thumb_anchor_error_m=float(np.linalg.norm(self.point(self.thumb) - self.hold_point))))
            stalled = 0 if accepted else stalled + 1
            if current['feasible'] and (stalled >= 3 or boundary):
                break
            if stalled >= 3 and max(current['maxima'].values()) > 1e-6:
                break
        return trace

    def open_fingers(self):
        """四指逐一沿限位内展开；检查完整路径采样，受阻便停，不跳过远指。"""
        pose, initial = self.kin.pose.copy(), self.kin.drivers.copy()
        attempts = []
        for i, finger in enumerate(FINGERS):
            start = float(initial[i])
            blocked = None
            count = max(1, int(np.ceil((start - self.kin.drives.lower[i]) / .05)))
            for angle in np.linspace(start, self.kin.drives.lower[i], count + 1)[1:]:
                proposed = initial.copy()
                proposed[i] = angle
                self.kin.set_state(pose, proposed)
                checked = self.check.evaluate()
                if not checked['feasible']:
                    blocked = checked['maxima']
                    break
                initial = proposed
            self.kin.set_state(pose, initial)
            attempts.append(dict(finger=finger, start_driver_rad=start, opened_driver_rad=float(initial[i]),
                                 opening_delta_rad=float(initial[i] - start), blocked_by=blocked,
                                 full_open_reached=bool(abs(initial[i] - self.kin.drives.lower[i]) < 1e-8)))
        return initial, attempts


def dynamics(trial, pose):
    """同源手模型加固定握梁；假设参数显式保存，没有拇指黏接约束。"""
    tree = ET.fromstring(trial.kin.hand.xml_string)
    source = ET.parse(DEFAULT_SCENE).getroot()
    tree.find("./default/default[@class='inspire']").insert(0, copy.deepcopy(source.find("./default/default[@class='inspire']/position")))
    ET.SubElement(tree, 'option', timestep='0.0005', integrator='implicitfast', gravity='0 0 0', iterations='100', tolerance='1e-10')
    actuator = copy.deepcopy(source.find('actuator'))
    for item in actuator:
        item.set('forcerange', '-1 1')
        item.set('forcelimited', 'true')
    tree.append(actuator)
    tree.append(copy.deepcopy(source.find('equality')))
    ET.SubElement(tree.find('asset'), 'mesh', name='anchor_grip_mesh', vertex=' '.join(map(str, trial.shape.vertices().ravel())))
    hand_from_grip = np.linalg.inv(pose) @ trial.frame
    quat = Rotation.from_matrix(hand_from_grip[:3, :3]).as_quat()[[3, 0, 1, 2]]
    body = ET.SubElement(tree.find('worldbody'), 'body', name='anchor_grip',
        pos=' '.join(map(str, hand_from_grip[:3, 3])), quat=' '.join(map(str, quat)))
    ET.SubElement(body, 'geom', name='anchor_grip_collision', type='mesh', mesh='anchor_grip_mesh', mass='.05',
        condim='4', friction='1 .005 .001', solref='.002 1', solimp='.9 .95 .001')
    xml = ET.tostring(tree, encoding='unicode')
    model = mujoco.MjModel.from_xml_string(xml)
    if model.nq != 12 or np.any(model.eq_type == mujoco.mjtEq.mjEQ_WELD):
        raise ValueError('接触模型不能包含手梁黏接或额外根自由度')
    return model, mujoco.MjData(model), xml


def contact_feedback(model, data, shape, thumb_side):
    """仿真接触反馈：只把指定两个侧面的实际正压力算作对握。"""
    gid, body = model.geom('anchor_grip_collision').id, data.body('anchor_grip')
    rotation = body.xmat.reshape(3, 3)
    rows = []
    loads = {side: {f: 0. for f in ['thumb'] + FINGERS} for side in [-1, 1]}
    for i, contact in enumerate(data.contact):
        if gid not in contact.geom:
            continue
        other = int(contact.geom[1] if contact.geom[0] == gid else contact.geom[0])
        name = model.body(int(model.geom_bodyid[other])).name
        finger = next((f for f in ['thumb'] + FINGERS if name.startswith(f + '_')), None)
        if finger is None:
            continue
        local = rotation.T @ (contact.pos - body.xpos)
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, wrench)
        sign = -1 if contact.geom[0] == gid else 1
        force = rotation.T @ (sign * contact.frame.reshape(3, 3).T @ wrench[:3])
        side = 1 if local[1] > 0 else -1
        normal_on_grip = rotation.T @ (sign * contact.frame.reshape(3, 3)[0])
        lateral = shape.lateral_side(local, normal_on_grip) == side
        normal = max(0., float(wrench[0]))
        if lateral:
            loads[side][finger] += normal
        rows.append(dict(finger=finger, side=side, lateral=lateral, point_grip_m=local.tolist(),
                         normal_force_n=normal, force_on_grip_n=force.tolist()))
    thumb = loads[thumb_side]['thumb'] > .01
    opposed = bool(thumb and max(loads[-thumb_side][f] for f in FINGERS) > .01)
    return dict(thumb_contact=bool(thumb), opposed_contact=opposed, contacts=rows, loads_n=loads)


def simulate(trial, row, output):
    """一次初始化后只用ctrl/mj_step；先拇指、后四指、最后保持0.2秒。"""
    kin = trial.kin
    pose, planned = kin.pose.copy(), kin.drivers.copy()
    initial, attempts = trial.open_fingers()
    model, data, xml = dynamics(trial, pose)
    (output / 'candidate.xml').write_text(xml)
    for name, value in zip(kin.drives.joint_names, kin.drives.expand(initial)):
        data.joint(name).qpos[0] = value
    ids = np.array([model.actuator(n).id for n in ACTUATORS])
    data.ctrl[ids] = initial
    mujoco.mj_forward(model, data)
    gid = model.geom('anchor_grip_collision').id
    depth = max([-float(c.dist) for c in data.contact if gid in c.geom] + [0.])
    kin.set_state(pose, initial)
    _, _, jac, _ = kin.point(trial.thumb['link'], trial.thumb['point_link_m'], trial.thumb['normal_link'])
    closing = -trial.normal @ jac[:, 10:12]
    delta = np.clip(closing * .0025 / max(closing @ closing, 1e-12), -.08, .08)
    target = planned.copy()
    # 全部四指获得闭合指令；取消旧小试的20mm参与门槛。
    target[:4] = np.minimum(kin.drives.upper[:4], planned[:4] + .06)
    target[4:] = np.clip(initial[4:] + delta, kin.drives.lower[4:], kin.drives.upper[4:])
    row.update(initial_drivers=initial.tolist(), target_drivers=target.tolist(),
               finger_opening_attempts=attempts, initial_proxy_penetration_m=depth,
               contact_trace=[], geometry_trace=[])
    if depth > .0001:
        row['status'] = 'initial_proxy_penetration'
        return
    sequence = ContactSequence()
    qstates, controls, times, phases = [], [], [], []
    thumb_hold = None
    hand_hold = None
    warnings = data.warning.number.copy()
    for step in range(5000):
        now = float(data.time)
        ctrl = initial.copy()
        if sequence.phase == 'seek_thumb':
            ctrl[4:] = initial[4:] + min(1., now / .6) * (target[4:] - initial[4:])
        elif sequence.phase == 'close_fingers':
            ctrl[4:] = thumb_hold
            ctrl[:4] += min(1., (now - sequence.entered_s) / 1.) * (target[:4] - initial[:4])
        else:
            ctrl = hand_hold.copy()
        data.ctrl[ids] = ctrl
        mujoco.mj_step(model, data)
        if not np.isfinite(data.qpos).all() or np.any(data.warning.number > warnings):
            row['status'] = 'numerical_failure'
            break
        if step % 50 != 49:
            continue
        mujoco.mj_forward(model, data)
        # 使用实际积分的十二关节，不能将六驱动展开结果冒充实际状态。
        actual = np.array([data.joint(n).qpos[0] for n in kin.drives.joint_names])
        kin.pose = pose.copy()
        kin.hand.place(kin.drives.joint_names, actual)
        checked = trial.check.evaluate()
        obs = contact_feedback(model, data, trial.shape, trial.side)
        before = sequence.phase
        phase = sequence.update(float(data.time), obs['thumb_contact'], obs['opposed_contact'], feasible=checked['feasible'])
        if before == 'seek_thumb' and phase == 'close_fingers':
            thumb_hold = data.ctrl[ids][4:].copy()
        if before == 'close_fingers' and phase == 'hold':
            hand_hold = data.ctrl[ids].copy()
        row['contact_trace'].append(dict(time_s=float(data.time), phase=phase, **obs))
        row['geometry_trace'].append(dict(time_s=float(data.time), feasible=checked['feasible'], maxima=checked['maxima']))
        qstates.append(data.qpos.copy()); controls.append(data.ctrl.copy()); times.append(float(data.time)); phases.append(phase)
        if phase in ('done', 'failed'):
            row['status'] = 'fixed_opposed_contact' if phase == 'done' else sequence.reason
            break
    else:
        row['status'] = 'simulation_budget_exhausted'
    row['physics_steps'] = step + 1
    row['sustained_opposed_contact'] = sequence.phase == 'done'
    row['final_contacts'] = row['contact_trace'][-1] if row['contact_trace'] else None
    row['finger_participation'] = [dict(finger=f, geometric_opening_checked=True,
        closing_command_delta_rad=float(target[i] - initial[i]),
        final_command_delta_rad=float(data.ctrl[ids][i] - initial[i]),
        actual_closing_motion_rad=float(data.joint(kin.drives.joint_names[np.flatnonzero(kin.drives.matrix[:, i])[0]]).qpos[0]
                                       - kin.drives.expand(initial)[np.flatnonzero(kin.drives.matrix[:, i])[0]]),
        contact_observed=any(any(c['finger'] == f and c['normal_force_n'] > .01 for c in item['contacts']) for item in row['contact_trace']))
        for i, f in enumerate(FINGERS)]
    np.savez_compressed(output / 'states.npz', qpos=np.asarray(qstates), ctrl=np.asarray(controls), time_s=np.asarray(times),
                        phase=np.asarray(phases), joint_names=np.asarray(kin.drives.joint_names), volume_from_hand=pose,
                        final_qpos=data.qpos, final_qvel=data.qvel)


def run(config, output, *, simulate_contacts=True):
    """所有输入显式来源绑定；成功或失败均保存，不发送真机动作。"""
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    trial = AnchorTrial(config)
    files = [trial.ref_path, trial.frame_path, Path(DEFAULT_SCENE), Path(__file__),
             ROOT / 'tools/thumb_anchor_viewer.py', Path(trial.kin.model_dir) / 'model.json']
    files += [ROOT / 'src/fuel_cap_task' / name for name in ['thumb_anchor.py', 'cap_reference.py', 'cap_geometry.py',
              'fixed_contact_kinematics.py', 'signed_hand_clearance.py', 'observed_hand_geometry.py',
              'joint_collision_update.py', 'rh56_drive_map.py']]
    files += [trial.run / name for name in ['contacts.json', 'contacts.npz', 'volume.json', 'network_volume.npz', 'geometry_1mm.npz']]
    files += trial.kin.hand.mesh_paths
    manifest = dict(observation_id=trial.observation_id, config=config, geometry_source=trial.geometry_source,
                    input_sha256={str(path): digest_file(path) for path in files},
                    volume_from_grip=trial.frame.tolist(), reference_from_volume=trial.volume['reference_from_volume'].tolist(),
                    candidate_budget=config.get('candidate_budget', 3), geometric_update_budget=config.get('update_budget', 24),
                    network_forwards=0, hardware_execution_performed=False, arm_ik_computed=False, approach_path_checked=False,
                    physics_hypotheses=dict(grip_fixed=True, gravity=[0, 0, 0], friction=[1, .005, .001],
                                            actuator_force_limit_nm=1., servo_kp=300., calibrated=False))
    save_json(output / 'manifest.json', manifest)
    save_json(output / 'anchors.json', trial.anchors)
    rows = []
    for i, anchor in enumerate(trial.anchors):
        folder = output / ('candidate_%02d' % i)
        folder.mkdir()
        row = dict(**anchor, status='no_safe_geometric_pose', sustained_opposed_contact=False, physics_steps=0,
                   physical_grasp_verified=False, motion_target_valid=False)
        row['optimization_trace'] = trial.solve(anchor, config.get('update_budget', 24))
        row['volume_from_hand'] = trial.kin.pose.tolist()
        row['reference_from_hand'] = (trial.volume['reference_from_volume'] @ trial.kin.pose).tolist()
        row['drivers'] = trial.kin.drivers.tolist()
        if trial.check.evaluate()['feasible']:
            row['status'] = 'geometric_pregrasp_ready'
            if simulate_contacts:
                simulate(trial, row, folder)
        save_json(folder / 'result.json', row)
        rows.append(row)
        print(json.dumps(dict(candidate=i, anchor_grip_m=anchor['anchor_grip_m'], status=row['status']), ensure_ascii=False), flush=True)
        # 几何候选可以换点；进入接触试验后本轮结束，不自动重新接触。
        if (row.get('contact_trace') is not None or row['status'] == 'initial_proxy_penetration'
                or (not simulate_contacts and row['status'] == 'geometric_pregrasp_ready')):
            break
    unchanged = all(digest_file(Path(p)) == sha for p, sha in manifest['input_sha256'].items())
    result = dict(status=rows[-1]['status'] if rows else 'no_anchor', candidates_tried=len(rows),
                  selected_candidate=next((i for i, row in enumerate(rows) if row['sustained_opposed_contact']
                                           or row['status'] == 'geometric_pregrasp_ready'), None),
                  elapsed_seconds=time.monotonic() - started, input_unchanged=unchanged,
                  hardware_execution_performed=False, motion_target_valid=False,
                  pending=['arm_ik_and_collision_free_approach', 'verified_hardware_contact_feedback_and_compliance'],
                  results=rows)
    if not unchanged:
        raise ValueError('运行中来源发生变化')
    save_json(output / 'result.json', result)
    # 可视化只读取保存状态，不重新优化或积分。
    from thumb_anchor_viewer import write_viewer
    write_viewer(trial, result, output)
    print(str(output / 'index.html'), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--plan-only', action='store_true')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    for key in ['run_dir', 'template_reference', 'confirmed_geometry', 'grip_estimate', 'cap_cloud', 'model_dir']:
        if config.get(key):
            path = Path(config[key])
            config[key] = str((ROOT / path if not path.is_absolute() else path).resolve())
    budget = config.get('update_budget', 24)
    if type(budget) is not int or not 1 <= budget <= 24:
        parser.error('update_budget 必须为1至24')
    try:
        with threadpool_limits(limits=1):
            result = run(config, args.output_dir.resolve(), simulate_contacts=not args.plan_only)
    except (ValueError, FileExistsError, KeyError) as error:
        parser.exit(2, str(error) + '\n')
    return 0 if result['status'] in ('fixed_opposed_contact', 'geometric_pregrasp_ready') else 1


if __name__ == '__main__':
    raise SystemExit(main())
