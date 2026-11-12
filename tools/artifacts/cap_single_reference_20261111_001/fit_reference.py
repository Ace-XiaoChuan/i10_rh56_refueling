"""在当前多视角点云中复用一组人工整手参考，仅做毫米范围的静态修正。"""
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import lsq_linear
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from threadpoolctl import threadpool_limits

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'tools')]
from run_fixed_contact_ik import load_inputs
from fuel_cap_task.cap_hand_overlay import HandOnlyScene
from fuel_cap_task.fixed_grasp import load_fixed_grasp
from fuel_cap_task.fixed_contact_kinematics import NativeHandKinematics, sample_pad_candidates, retract
from fuel_cap_task.fixed_contact_ik import contact_state, constrained_least_squares
from fuel_cap_task.signed_hand_clearance import SignedHandClearance
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE
from fuel_cap_task.saved_sdk_input import save_json, digest_file


def inputs():
    """沿用已绑定来源的完整634610点，不使用原网络的逐指目标。"""
    return load_inputs(ROOT/'tools/artifacts/cap_five_finger_guard_20261111_001/cases/guarded/run_candidates',
                       ROOT/'tools/artifacts/cap_grip_scores_20261111_001/manual_sides.npz')


def select_contacts(kin, data, manual, frame):
    """只在已接受假设侧面的内部选目标；保存真实显示三角面的固定指面点。"""
    pads = sample_pad_candidates(data['model_dir'], 512)
    local = (data['assumed']-frame[:3, 3])@frame[:3, :3]
    contacts = []
    for finger, side in [('thumb', 1), ('index', -1), ('middle', -1)]:
        keep = ((local[:, 1]*side > 0) & (abs(local[:, 0]) <= .029)
                & (local[:, 2] <= -.001) & (local[:, 2] >= -.016))
        points = data['assumed'][keep]
        if not len(points):
            raise ValueError('该侧缺少已接受的内部假设面样本')
        normal = frame[:3, :3]@np.array([0., side, 1/17])
        normal /= np.linalg.norm(normal)
        tree = cKDTree(points)
        choices = []
        for pad in pads[finger]:
            point, pad_normal = kin.point(pad['link'], pad['point_link_m'], pad['normal_link'])[:2]
            distance, row = tree.query(point)
            angle = np.degrees(np.arccos(np.clip(-pad_normal@normal, -1, 1)))
            if angle <= 45:
                choices.append((distance, dict(pad, finger=finger, feature_id='manual_'+finger,
                    target_m=points[row].tolist(), surface_point_m=points[row].tolist(),
                    target_normal=normal.tolist(), side=side, geometry_source='hypothetical_manual_side',
                    network_feature=False)))
        if not choices:
            raise ValueError('参考手型没有朝向该侧面的指面样本：'+finger)
        contacts.append(min(choices, key=lambda row: row[0])[1])
    return contacts


def state(kin, clearance, contacts, label):
    """保存实际六驱动FK与完整分类检查，后续显示不再求解。"""
    check = clearance.evaluate()
    metrics = contact_state(kin, contacts)['metrics']
    return dict(label=label, pose=kin.pose.tolist(), drivers=kin.drivers.tolist(), metrics=metrics,
                collision=check['maxima'], counts=check['counts'], geometry_pass=check['feasible'])


def refine(kin, clearance, contacts):
    """局部约束最小二乘，全部接受状态重查几何；总改变量受参考范围约束。"""
    origin, original = kin.pose.copy(), kin.drivers.copy()
    states = [state(kin, clearance, contacts, '闭合参考（严格六驱动耦合）')]
    if not states[0]['geometry_pass']:
        raise ValueError('闭合参考不是可行初态，停止局部修正')
    trust = np.array([.001]*3+[.01]*3+[.02]*6)
    records = []
    for iteration in range(20):
        current = contact_state(kin, contacts)
        if all(m['distance_m'] <= .001 and m['angle_deg'] <= 45 for m in current['metrics']):
            break
        pose, drivers = kin.pose.copy(), kin.drivers.copy()
        active = clearance.evaluate(linearize=True)
        gaps = np.concatenate([active['gaps'], current['gaps']])/.001
        A = np.concatenate([active['jacobian'], current['gap_jacobian']])*trust/.001
        lower, upper = -np.ones(12), np.ones(12)
        # 单组只允许平移各轴±5mm、旋转总量≤5°、各驱动相对参考±0.15rad。
        lower[:3] = np.maximum(lower[:3], (origin[:3, 3]-.005-pose[:3, 3])/trust[:3])
        upper[:3] = np.minimum(upper[:3], (origin[:3, 3]+.005-pose[:3, 3])/trust[:3])
        lower[6:] = np.maximum(lower[6:], (np.maximum(kin.drives.lower, original-.15)-drivers)/trust[6:])
        upper[6:] = np.minimum(upper[6:], (np.minimum(kin.drives.upper, original+.15)-drivers)/trust[6:])
        step, qp = constrained_least_squares(current['residual'], current['jacobian']*trust,
                                             gaps, A, lower, upper)
        record = dict(iteration=iteration, qp=qp, accepted=False, trials=[])
        if qp['feasible']:
            for trial in range(8):
                pp, dd = retract(pose, drivers, step*trust*(.5**trial))
                rotation_change = Rotation.from_matrix(pp[:3, :3]@origin[:3, :3].T).magnitude()
                if rotation_change > np.deg2rad(5):
                    continue
                kin.set_state(pp, dd)
                checked = clearance.evaluate()
                proposed = contact_state(kin, contacts)
                score = float(proposed['residual']@proposed['residual'])
                record['trials'].append(dict(scale=.5**trial, collision=checked['maxima'], score=score))
                if (checked['feasible'] and np.all(proposed['gaps'] >= -1e-10)
                        and score < float(current['residual']@current['residual'])-1e-9):
                    states.append(state(kin, clearance, contacts, '局部修正'))
                    record['accepted'] = True
                    break
        records.append(record)
        if not record['accepted']:
            kin.set_state(pose, drivers)
            break
    states[-1]['label'] = '本组最终抓姿'
    return states, records


def main():
    """原抓姿配置只读；保存一组参考、预型对照与局部修正的全部接受状态。"""
    data = inputs()
    manual = dict(np.load(data['manual_file'], allow_pickle=False))
    frame = manual['volume_from_manual'].copy()
    original_frame = frame.copy()
    # 原旋转有4.1e-8正交误差，构造刚体旋转；原始矩阵和变化单独留存。
    frame[:3, :3] = Rotation.from_matrix(frame[:3, :3]).as_matrix()
    grasp = load_fixed_grasp(ROOT/'config/fixed_grasp.json')
    relative = np.eye(4)
    relative[:3, :3] = Rotation.from_quat(grasp.cap_from_hand.orientation_xyzw).as_matrix()
    relative[:3, 3] = grasp.cap_from_hand.position_m
    pose = frame@relative
    kin = NativeHandKinematics(data['model_dir'], DEFAULT_SCENE)
    scene = HandOnlyScene(grasp.provenance.source_scene_path)
    raw = json.loads((ROOT/'config/fixed_grasp.json').read_text())['contact_trial']['final_qpos']
    by_name = {scene.model.joint(i).name: raw[scene.model.jnt_qposadr[i]] for i in range(scene.model.njnt)}
    historical_full = np.array([by_name[n] for n in kin.drives.joint_names])
    drives = kin.drives
    # 历史动力学终态有微小耦合偏差与超限；明确拟合成合法六驱动，不静默截断。
    drivers = lsq_linear(drives.matrix, historical_full-drives.offset,
                        bounds=(drives.lower, drives.upper), tol=1e-13).x
    kin.set_state(pose, drivers)
    contacts = select_contacts(kin, data, manual, frame)
    clearance = SignedHandClearance(kin, data['scene'], data['geometry']['target_point_mask'],
                                    assumed_points=data['assumed'])
    scene.hand.apply(scene.data, grasp.pregrasp_channels)
    pre = np.array([scene.data.joint(n).qpos[0] for n in drives.driver_names])
    kin.set_state(pose, pre)
    preform = state(kin, clearance, contacts, '650预张开（静态对照）')
    kin.set_state(pose, drivers)
    states, iterations = refine(kin, clearance, contacts)
    final = states[-1]
    passes = final['geometry_pass'] and all(m['distance_m'] <= .001 and m['angle_deg'] <= 45 for m in final['metrics'])
    translation = frame[:3, :3].T@(kin.pose[:3, 3]-pose[:3, 3])
    sources = [Path(__file__), ROOT/'config/fixed_grasp.json', Path(DEFAULT_SCENE), Path(data['manual_file']),
               data['model_dir']/'model.json']+kin.hand.mesh_paths
    sources += [data['source_run']/n for n in ['observation.json','volume.json','contacts.json',
                 'candidates.json','geometry_1mm.npz','network_volume.npz','contacts.npz']]
    sources += [ROOT/'src/fuel_cap_task'/n for n in ['fixed_grasp.py','cap_hand_overlay.py','mujoco_scene.py',
        'fixed_contact_kinematics.py','fixed_contact_ik.py','signed_hand_clearance.py',
        'observed_hand_geometry.py','rh56_drive_map.py']]
    sources += [ROOT/'tools/run_fixed_contact_ik.py']
    sources += [data['model_dir']/n for n in kin.metadata['outputs']]
    report = dict(mode='single_confirmed_reference_local_static_fit', passes=passes,
        source_run=str(data['source_run']), model_dir=str(data['model_dir']), manual_file=str(data['manual_file']),
        original_volume_from_manual=original_frame.tolist(), volume_from_manual=frame.tolist(),
        frame_rotation_maximum_change=float(abs(frame-original_frame).max()),
        reference_from_volume=data['volume']['reference_from_volume'].tolist(),
        historical_qpos_by_joint=by_name, historical_full_in_drive_order=historical_full.tolist(),
        coupled_reference_full=drives.expand(drivers).tolist(),
        historical_projection_maximum_change_rad=float(abs(drives.expand(drivers)-historical_full).max()),
        contacts=contacts, preform=preform, states=states, iterations=iterations,
        final=final, translation_change_manual_mm=(translation*1000).tolist(),
        rotation_change_deg=float(np.rad2deg(Rotation.from_matrix(kin.pose[:3,:3]@pose[:3,:3].T).magnitude())),
        driver_changes_rad=(kin.drivers-drivers).tolist(), driver_names=list(drives.driver_names),
        scene_point_count=len(data['scene']), target_point_count=int(data['geometry']['target_point_mask'].sum()),
        assumed_sample_count=len(data['assumed']), contact_distance_m=.001, maximum_angle_deg=45.,
        penetration_tolerance_m=.0001, physical_grasp_verified=False, motion_target_valid=False,
        network_features_used=False, hardware_execution_performed=False, physics_steps=0,
        sources_sha256={str(p):digest_file(p) for p in sources})
    save_json(OUT/'result.json', report)
    print(json.dumps(dict(passes=passes, accepted_updates=len(states)-1,
        distances_mm=[m['distance_m']*1000 for m in final['metrics']],
        angles_deg=[m['angle_deg'] for m in final['metrics']],
        collision=final['collision'], translation_change_mm=report['translation_change_manual_mm'],
        rotation_change_deg=report['rotation_change_deg']), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    with threadpool_limits(limits=1):
        main()
