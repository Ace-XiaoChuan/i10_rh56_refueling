"""独立复核换端结果：来源、完整几何和同一闭合的精确重放。"""
import importlib.util
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from threadpoolctl import threadpool_limits

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from fuel_cap_task.fixed_contact_kinematics import NativeHandKinematics
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE
from fuel_cap_task.saved_sdk_input import digest_file, save_json


def transform(points, matrix):
    """米制按行刚体变换。"""
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def main():
    """不增加候选；重放只用于核对已保存的实际关节和接触力。"""
    row = json.loads((OUT / 'final_result.json').read_text())
    manifest = json.loads((OUT / 'manifest.json').read_text())
    hashes = {**manifest['input_sha256'], **row['finish_input_sha256']}
    assert all(digest_file(Path(p)) == sha for p, sha in hashes.items())
    assert len(row['optimization_trace']) == 24 and row['sustained_opposed_contact']
    assert row['anchor_manual_m'][0] > .015
    run = Path(manifest['source_run'])
    pred, vol, geo = [dict(np.load(run / f)) for f in ['contacts.npz', 'network_volume.npz', 'geometry_1mm.npz']]
    assert float(pred['quality'][tuple(row['feature_index_xyz'])]) == row['raw_network_score'] >= .1
    for t in row['optimization_trace']:
        assert t['thumb_anchor_error_m'] < 1e-12
        for f in t['fingers']:
            assert float(pred['quality'][tuple(f['feature_index_xyz'])]) == f['raw_network_score'] >= .1
    assert all(t['feasible'] for t in row['geometry_trace'])
    model = mujoco.MjModel.from_xml_path(str(OUT / 'candidate.xml'))
    data = mujoco.MjData(model)
    assert model.nq == 12 and not any(model.eq_type == mujoco.mjtEq.mjEQ_WELD)
    states = np.load(OUT / 'states.npz')
    ref = json.loads((OUT.parent / 'cap_single_reference_20261111_001/result.json').read_text())
    kin = NativeHandKinematics(ref['model_dir'], DEFAULT_SCENE)
    pose = np.asarray(row['volume_from_hand'])
    scene = transform(geo['points_volume_m'], np.linalg.inv(vol['reference_from_volume']) @ geo['reference_from_volume'])
    independent = []
    first_index = int(round(row['first_opposed_s'] / .025)) - 1
    for index in [first_index, len(states['qpos']) - 1]:
        data.qpos[:] = states['qpos'][index]
        mujoco.mj_forward(model, data)
        angles = [data.joint(n).qpos[0] for n in kin.drives.joint_names]
        kin.pose = pose.copy()
        kin.hand.place(kin.drives.joint_names, angles)
        hits, occupied = kin.hand.surface_penetrations(transform(scene, np.linalg.inv(pose)), geo['target_point_mask'], np.arange(len(scene)), .0001)
        self_hits = kin.hand.self_penetrations(.0001)
        assert not occupied.any() and not self_hits
        independent.append(dict(time_s=float(states['time_s'][index]), checked_scene_points=len(scene),
                                occupied_points=int(occupied.sum()), self_collisions=len(self_hits)))
    first = row['first_opposed_s']
    tail = [t for t in row['contact_trace'] if t['time_s'] >= first - 1e-8]
    assert tail[-1]['time_s'] - first >= .2 - 1e-8
    for t in tail:
        thumb = sum(c['normal_force_n'] for c in t['contacts'] if c['finger'] == 'thumb' and c['side'] == 1 and c['lateral'])
        partner = sum(c['normal_force_n'] for c in t['contacts'] if c['finger'] != 'thumb' and c['side'] == -1 and c['lateral'])
        assert thumb > .01 and partner > .01
    # 按原控制时序重放，核对每50步保存的实际12关节，禁止运动学重置。
    data = mujoco.MjData(model)
    initial, target = np.asarray(row['initial_drivers']), np.asarray(row['target_drivers'])
    for name, angle in zip(kin.drives.joint_names, kin.drives.expand(initial)):
        data.joint(name).qpos[0] = angle
    controls = np.array([model.actuator(n).id for n in ['pinky', 'ring', 'middle', 'index', 'thumb_proximal', 'thumb_yaw']])
    data.ctrl[controls] = initial
    mujoco.mj_forward(model, data)
    latch = round(first / model.opt.timestep)
    held = initial + min(1., latch / 2000) * (target - initial)
    error = 0.
    for step in range(row['physics_steps']):
        data.ctrl[controls] = initial + min(1., (step + 1) / 2000) * (target - initial) if step < latch else held
        mujoco.mj_step(model, data)
        if step % 50 == 49:
            mujoco.mj_forward(model, data)
            error = max(error, float(np.max(abs(data.qpos - states['qpos'][step // 50]))))
    assert error < 1e-12
    spec = importlib.util.spec_from_file_location('fixed_force_extract', OUT.parent / 'cap_thumb_anchor_spike_20261112_001/fixed_forces.py')
    force_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(force_module)
    replay_force = force_module.extract(model, data)
    for f in ['thumb', 'middle']:
        assert abs(replay_force['fingers'][f]['normal_force_n'] - row['final_forces']['fingers'][f]['normal_force_n']) < 1e-10
    report = dict(passes=True, input_hashes_checked=len(hashes), candidate_count=1, geometric_steps=24,
                  corrected_anchor_manual_m=row['anchor_manual_m'], original_network_scores_unchanged=True,
                  fixed_grip_has_no_dofs=True, no_hand_object_weld=True, independent_full_scene_states=independent,
                  physics_checkpoint_count=len(states['qpos']), sustained_opposed_duration_s=tail[-1]['time_s'] - first,
                  exact_replay_max_qpos_error=error, contact_forces_reproduced=True,
                  final_normal_forces_n={f: replay_force['fingers'][f]['normal_force_n'] for f in ['thumb', 'middle']},
                  physical_forces_calibrated=False, physical_grasp_verified=False, motion_target_valid=False)
    save_json(OUT / 'verification.json', report)
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    with threadpool_limits(limits=1):
        main()
