#!/usr/bin/env python3
"""独立重放一次已保存闭合，核对真实关节、接触力及完整点云侵入。"""
import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from threadpoolctl import threadpool_limits

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from fuel_cap_task.fixed_contact_kinematics import NativeHandKinematics
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE
from fuel_cap_task.saved_sdk_input import digest_file, save_json
from fuel_cap_task.thumb_anchor import transform


def verify(output):
    """按保存的阶段切换时刻重建ctrl，不重新选点、优化或改变接触参数。"""
    manifest = json.loads((output / 'manifest.json').read_text())
    assert all(digest_file(Path(p)) == sha for p, sha in manifest['input_sha256'].items())
    result = json.loads((output / 'result.json').read_text())
    selected = result['selected_candidate']
    if selected is None or result['status'] != 'fixed_opposed_contact':
        raise ValueError('当前报告没有持续对握结果可重放')
    folder = output / ('candidate_%02d' % selected)
    row = result['results'][selected]
    saved = np.load(folder / 'states.npz')
    model = mujoco.MjModel.from_xml_path(str(folder / 'candidate.xml'))
    data = mujoco.MjData(model)
    ref = json.loads(Path(manifest['config']['template_reference']).read_text())
    kin = NativeHandKinematics(manifest['config'].get('model_dir', ref['model_dir']), DEFAULT_SCENE)
    initial, target = np.array(row['initial_drivers']), np.array(row['target_drivers'])
    for name, value in zip(kin.drives.joint_names, kin.drives.expand(initial)):
        data.joint(name).qpos[0] = value
    ids = np.array([model.actuator(n).id for n in ['pinky', 'ring', 'middle', 'index', 'thumb_proximal', 'thumb_yaw']])
    data.ctrl[ids] = initial
    mujoco.mj_forward(model, data)
    trace = row['contact_trace']
    thumb_index = next(i for i, item in enumerate(trace) if item['phase'] == 'close_fingers')
    hold_index = next(i for i, item in enumerate(trace) if item['phase'] == 'hold')
    thumb_time, hold_time = trace[thumb_index]['time_s'], trace[hold_index]['time_s']
    thumb_tick, hold_tick = [round(t / model.opt.timestep) for t in [thumb_time, hold_time]]
    thumb_control, held_control = saved['ctrl'][thumb_index, ids][4:], saved['ctrl'][hold_index, ids]
    max_error = 0.
    for step in range(row['physics_steps']):
        ctrl = initial.copy()
        if step < thumb_tick:
            ctrl[4:] += min(1., data.time / .6) * (target[4:] - initial[4:])
        elif step < hold_tick:
            ctrl[4:] = thumb_control
            ctrl[:4] += min(1., (data.time - thumb_time) / 1.) * (target[:4] - initial[:4])
        else:
            ctrl = held_control
        data.ctrl[ids] = ctrl
        mujoco.mj_step(model, data)
        if step % 50 == 49:
            index = step // 50
            max_error = max(max_error, float(np.max(abs(data.qpos - saved['qpos'][index]))))
            np.testing.assert_allclose(data.ctrl, saved['ctrl'][index], atol=1e-12, rtol=0)
    assert max_error < 1e-12
    mujoco.mj_forward(model, data)
    gid = model.geom('anchor_grip_collision').id
    final_forces = []
    for i, contact in enumerate(data.contact):
        if gid not in contact.geom:
            continue
        other = int(contact.geom[1] if contact.geom[0] == gid else contact.geom[0])
        body = model.body(int(model.geom_bodyid[other])).name
        finger = next((f for f in ['thumb', 'index', 'middle', 'ring', 'pinky'] if body.startswith(f + '_')), None)
        if finger is None:
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        final_forces.append(dict(finger=finger, normal_force_n=float(force[0])))
    for finger in ['thumb', 'index', 'middle', 'ring', 'pinky']:
        actual = sum(c['normal_force_n'] for c in final_forces if c['finger'] == finger)
        expected = sum(c['normal_force_n'] for c in row['final_contacts']['contacts'] if c['finger'] == finger)
        assert abs(actual - expected) < 1e-10
    run = Path(manifest['config']['run_dir'])
    geo, volume = np.load(run / 'geometry_1mm.npz'), np.load(run / 'network_volume.npz')
    pose = np.array(row['volume_from_hand'])
    scene = transform(geo['points_volume_m'], np.linalg.inv(volume['reference_from_volume']) @ geo['reference_from_volume'])
    hand_points = transform(scene, np.linalg.inv(pose))
    checks = []
    # 与近邻带求解器不同，此处逐实体直接检查全部原始点。
    for index in sorted({thumb_index, hold_index, len(trace) - 1}):
        data.qpos[:] = saved['qpos'][index]
        mujoco.mj_forward(model, data)
        angles = np.array([data.joint(n).qpos[0] for n in kin.drives.joint_names])
        kin.pose = pose.copy()
        kin.hand.place(kin.drives.joint_names, angles)
        hits, occupied = kin.hand.surface_penetrations(hand_points, geo['target_point_mask'], np.arange(len(scene)), .0001)
        self_hits = kin.hand.self_penetrations(.0001)
        checks.append(dict(time_s=trace[index]['time_s'], points_checked=len(scene),
                           excess_points=int(occupied.sum()), self_collisions=len(self_hits)))
        assert not hits and not self_hits
    assert all(item['feasible'] for item in row['geometry_trace'])
    assert all(item['opposed_contact'] for item in trace[hold_index:])
    assert trace[-1]['time_s'] - hold_time >= .2
    assert not np.any(model.eq_type == mujoco.mjtEq.mjEQ_WELD)
    report = dict(passes=True, input_hashes_checked=len(manifest['input_sha256']),
                  candidates_tried=result['candidates_tried'], network_forwards=0,
                  exact_replay_max_qpos_error=max_error, force_replay_matches=True,
                  all_geometry_checkpoints_passed=len(trace), independent_full_scene_checks=checks,
                  thumb_first_contact_s=thumb_time, first_opposed_s=hold_time,
                  opposed_duration_s=trace[-1]['time_s'] - hold_time, final_contact_forces=final_forces,
                  physical_forces_calibrated=False, physical_grasp_verified=False,
                  hardware_execution_performed=False, motion_target_valid=False,
                  source_sha256={str(output / 'result.json'): digest_file(output / 'result.json'),
                                 str(Path(__file__)): digest_file(Path(__file__))})
    save_json(output / 'verification.json', report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output_dir', type=Path)
    args = parser.parse_args()
    with threadpool_limits(limits=1):
        verify(args.output_dir.resolve())


if __name__ == '__main__':
    main()
