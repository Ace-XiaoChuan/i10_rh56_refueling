"""复核预登记的全部组，失败样本也重放并检查实际末态。"""
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from threadpoolctl import threadpool_limits

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from fuel_cap_task.fixed_contact_kinematics import NativeHandKinematics
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE
from fuel_cap_task.saved_sdk_input import digest_file, save_json
from fuel_cap_task.thumb_anchor import transform


def replay(row, model, saved, kin):
    """由阶段切换时刻和原公式重建完整ctrl，适用于成功及接触超时。"""
    data = mujoco.MjData(model)
    initial, target = np.array(row['initial_drivers']), np.array(row['target_drivers'])
    for name, value in zip(kin.drives.joint_names, kin.drives.expand(initial)):
        data.joint(name).qpos[0] = value
    ids = np.array([model.actuator(n).id for n in ['pinky', 'ring', 'middle', 'index', 'thumb_proximal', 'thumb_yaw']])
    data.ctrl[ids] = initial
    mujoco.mj_forward(model, data)
    trace = row['contact_trace']
    thumb_i = next((i for i, item in enumerate(trace) if item['phase'] == 'close_fingers'), None)
    hold_i = next((i for i, item in enumerate(trace) if item['phase'] == 'hold'), None)
    thumb_time = trace[thumb_i]['time_s'] if thumb_i is not None else None
    thumb_tick = round(thumb_time / model.opt.timestep) if thumb_i is not None else float('inf')
    hold_tick = round(trace[hold_i]['time_s'] / model.opt.timestep) if hold_i is not None else float('inf')
    thumb_ctrl = saved['ctrl'][thumb_i, ids][4:] if thumb_i is not None else None
    held = saved['ctrl'][hold_i, ids] if hold_i is not None else None
    error = 0.
    for step in range(row['physics_steps']):
        ctrl = initial.copy()
        if step < thumb_tick:
            ctrl[4:] += min(1., data.time / .6) * (target[4:] - initial[4:])
        elif step < hold_tick:
            ctrl[4:] = thumb_ctrl
            ctrl[:4] += min(1., (data.time - thumb_time) / 1.) * (target[:4] - initial[:4])
        else:
            ctrl = held
        data.ctrl[ids] = ctrl
        mujoco.mj_step(model, data)
        if step % 50 == 49:
            i = step // 50
            error = max(error, float(np.max(abs(data.qpos - saved['qpos'][i]))))
            np.testing.assert_allclose(data.ctrl, saved['ctrl'][i], atol=1e-12, rtol=0)
    np.testing.assert_allclose(data.qpos, saved['final_qpos'], atol=1e-12, rtol=0)
    assert error < 1e-12
    mujoco.mj_forward(model, data)
    gid = model.geom('anchor_grip_collision').id
    loads = {finger: 0. for finger in ['thumb', 'index', 'middle', 'ring', 'pinky']}
    for i, contact in enumerate(data.contact):
        if gid not in contact.geom:
            continue
        other = int(contact.geom[1] if contact.geom[0] == gid else contact.geom[0])
        body = model.body(int(model.geom_bodyid[other])).name
        finger = next((f for f in loads if body.startswith(f + '_')), None)
        if finger is None:
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        loads[finger] += max(0., float(force[0]))
    for finger, force in loads.items():
        expected = sum(c['normal_force_n'] for c in trace[-1]['contacts'] if c['finger'] == finger)
        assert abs(force - expected) < 1e-10
    return dict(max_qpos_error=error, force_matches=True, total_normal_forces_n=loads,
                thumb_index=thumb_i, hold_index=hold_i)


def main():
    """共享数据只读；结果与独立检查另存，不改写任一组轨迹。"""
    manifest = json.loads((OUT / 'manifest.json').read_text())
    selection = json.loads((OUT / 'selection.json').read_text())
    summary = json.loads((OUT / 'summary.json').read_text())
    assert summary['count'] == selection['candidate_count'] == 8
    assert all(digest_file(Path(p)) == sha for p, sha in manifest['input_sha256'].items())
    config = manifest['config']
    ref = json.loads(Path(config['template_reference']).read_text())
    kin = NativeHandKinematics(ref['model_dir'], DEFAULT_SCENE)
    run = Path(config['run_dir'])
    geo, volume, prediction = [np.load(run / name) for name in ['geometry_1mm.npz', 'network_volume.npz', 'contacts.npz']]
    points = transform(geo['points_volume_m'], np.linalg.inv(volume['reference_from_volume']) @ geo['reference_from_volume'])
    checked = []
    for group in summary['groups']:
        folder = OUT / group['folder']
        group_manifest = json.loads((folder / 'manifest.json').read_text())
        assert all(digest_file(Path(p)) == sha for p, sha in group_manifest['input_sha256'].items())
        row = json.loads((folder / 'result.json').read_text())['results'][0]
        anchor = selection['anchors'][group['group']]
        assert row['feature_index_xyz'] == anchor['feature_index_xyz']
        assert row['raw_network_score'] == float(prediction['quality'][tuple(row['feature_index_xyz'])])
        assert group_manifest['config'] == config
        last = row['optimization_trace'][-1]
        pose = np.array(row['volume_from_hand'])
        hand_points = transform(points, np.linalg.inv(pose))
        kin.set_state(pose, np.array(row['drivers']))
        hits, occupied = kin.hand.surface_penetrations(hand_points, geo['target_point_mask'], np.arange(len(points)), .0001)
        self_hits = kin.hand.self_penetrations(.0001)
        if last['feasible']:
            assert not hits and not self_hits
        geometry = dict(point_count=len(points), excess_points=int(occupied.sum()), self_collisions=len(self_hits),
                        maximum_inside_depth_m=max([item['maximum_inside_depth_m'] for item in hits] + [0.]))
        result = dict(group=group['group'], status=group['status'], raw_score_matches=True,
                      geometric_final=geometry, physics_replayed=False)
        states_path = folder / 'candidate_00/states.npz'
        if states_path.exists():
            saved = np.load(states_path)
            model = mujoco.MjModel.from_xml_path(str(folder / 'candidate_00/candidate.xml'))
            assert model.nq == 12 and not np.any(model.eq_type == mujoco.mjtEq.mjEQ_WELD)
            assert model.body('anchor_grip').dofnum == 0
            result['replay'] = replay(row, model, saved, kin)
            result['physics_replayed'] = True
            indices = {len(row['contact_trace']) - 1}
            indices.update(i for i in [result['replay']['thumb_index'], result['replay']['hold_index']] if i is not None)
            states_checks = []
            data = mujoco.MjData(model)
            for index in sorted(indices):
                data.qpos[:] = saved['qpos'][index]
                mujoco.mj_forward(model, data)
                actual = np.array([data.joint(name).qpos[0] for name in kin.drives.joint_names])
                kin.pose = pose.copy()
                kin.hand.place(kin.drives.joint_names, actual)
                hits, occupied = kin.hand.surface_penetrations(hand_points, geo['target_point_mask'], np.arange(len(points)), .0001)
                self_hits = kin.hand.self_penetrations(.0001)
                states_checks.append(dict(time_s=float(saved['time_s'][index]), excess_points=int(occupied.sum()),
                                          self_collisions=len(self_hits), point_count=len(points)))
                if row['geometry_trace'][index]['feasible']:
                    assert not hits and not self_hits
            result['independent_physical_checks'] = states_checks
            if group['success']:
                first = result['replay']['hold_index']
                assert first is not None
                tail = row['contact_trace'][first:]
                assert tail[-1]['time_s'] - tail[0]['time_s'] >= .2
                assert all(item['opposed_contact'] and item['thumb_contact'] for item in tail)
                assert all(item['feasible'] for item in row['geometry_trace'])
                result['opposed_duration_s'] = tail[-1]['time_s'] - tail[0]['time_s']
        checked.append(result)
        save_json(folder / 'batch_verification.json', result)
        print(json.dumps(dict(group=group['group'], verified=True, physics_replayed=result['physics_replayed']), ensure_ascii=False), flush=True)
    assert sum(g['success'] for g in summary['groups']) == summary['success_count']
    save_json(OUT / 'verification.json', dict(passes=True, count=8, success_count=summary['success_count'],
        input_sha256_matches=True, all_registered_groups_present=True, checks=checked,
        physical_forces_calibrated=False, hardware_execution_performed=False,
        source_sha256={str(OUT / 'summary.json'): digest_file(OUT / 'summary.json'), str(Path(__file__)): digest_file(Path(__file__))}))


if __name__ == '__main__':
    with threadpool_limits(limits=1):
        main()
