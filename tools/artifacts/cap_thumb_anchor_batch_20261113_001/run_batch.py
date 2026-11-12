"""同一观测的八个预登记锚点，原样调用新入口逐组独立求解和接触仿真。"""
import copy
import json
from pathlib import Path
import sys
import time

import numpy as np
from threadpoolctl import threadpool_limits

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'src')]
import run_thumb_anchor as pipeline
from fuel_cap_task.saved_sdk_input import digest_file, save_json
from fuel_cap_task.thumb_anchor import rank_anchors, transform


def choose_batch(trial, count):
    """复用原排序器逐次排除已选4mm邻域，不改写原网络或其分数。"""
    kin = trial.kin
    kin.set_state(trial.reference_pose, trial.reference_drivers)
    thumb_x = transform(trial.point(trial.thumb), trial.inverse)[0]
    offsets = []
    for finger in pipeline.FINGERS:
        contacts = [p for p in trial.ref['contacts'] if p.get('finger') == finger]
        region = next(p for p in kin.metadata['contact_regions'] if p['finger'] == finger)
        pad = contacts[0] if contacts else dict(link=region['link'], point_link_m=region['tip_site_link_m'], normal_link=region['normal_link'])
        offsets.append(float(transform(trial.point(pad), trial.inverse)[0] - thumb_x))
    remaining = trial.pred['quality'].copy()
    ids = np.argwhere(remaining >= .1)
    local = transform(ids * float(trial.volume['voxel_size_m']), trial.inverse)
    projected = trial.shape.project(local, trial.side)
    rows = []
    for i in range(count):
        proposed = rank_anchors(remaining, float(trial.volume['voxel_size_m']), trial.frame, offsets,
                                side=trial.side, shape=trial.shape, budget=1)
        if not proposed:
            break
        anchor = proposed[0]
        assert anchor['raw_network_score'] == float(trial.pred['quality'][tuple(anchor['feature_index_xyz'])])
        rows.append(dict(batch_index=i, **anchor))
        nearby = np.linalg.norm(projected - np.array(anchor['anchor_grip_m']), axis=1) < .004
        remaining[tuple(ids[nearby].T)] = 0
    if len(rows) != count:
        raise ValueError('当前规则下不足预定数量的分离锚点，不临时改变规则补点')
    assert all(np.linalg.norm(np.array(a['anchor_grip_m']) - b['anchor_grip_m']) >= .004
               for i, a in enumerate(rows) for b in rows[:i])
    return rows


def main():
    """八组在运行前固定；每组新建模型，原入口的成功即停只作用于该组。"""
    if (OUT / 'selection.json').exists():
        raise FileExistsError('本轮已登记，不覆盖或自动重跑')
    config = json.loads((ROOT / 'config/thumb_anchor_trial.json').read_text())
    for name in ['run_dir', 'template_reference', 'confirmed_geometry', 'model_dir']:
        if config.get(name):
            path = Path(config[name])
            config[name] = str((ROOT / path if not path.is_absolute() else path).resolve())
    config['candidate_budget'] = 1
    original_type = pipeline.AnchorTrial
    trial = original_type(config)
    selected = choose_batch(trial, 8)
    files = [ROOT / 'tools/run_thumb_anchor.py', ROOT / 'tools/thumb_anchor_viewer.py',
             ROOT / 'tools/verify_thumb_anchor.py', Path(__file__),
             Path(config['template_reference']), Path(config['confirmed_geometry']), Path(pipeline.DEFAULT_SCENE)]
    files += [ROOT / 'src/fuel_cap_task' / name for name in ['thumb_anchor.py', 'fixed_contact_kinematics.py',
              'signed_hand_clearance.py', 'observed_hand_geometry.py', 'joint_collision_update.py', 'rh56_drive_map.py']]
    files += [trial.run / name for name in ['contacts.json', 'contacts.npz', 'volume.json', 'network_volume.npz', 'geometry_1mm.npz']]
    files += trial.kin.hand.mesh_paths + [trial.kin.model_dir / 'model.json']
    hashes = {str(p): digest_file(p) for p in files}
    save_json(OUT / 'selection.json', dict(candidate_count=8, minimum_anchor_separation_m=.004,
        selection_method='原rank_anchors排序，逐次排除已选4mm区域；保留原分数', anchors=selected))
    save_json(OUT / 'manifest.json', dict(config=config, observation_id=trial.observation_id, input_sha256=hashes,
        geometry_source='点云推定握梁坐标与用户确认的双侧补全，未新增实物标定',
        method='同一观测八个锚点独立测验，原新脚本求解与接触流程不改',
        update_budget_per_group=24, closure_budget_per_group=1, network_forwards=0,
        hardware_execution_performed=False, motion_target_valid=False))
    print(json.dumps(dict(registered_anchors=[a['feature_index_xyz'] for a in selected]), ensure_ascii=False), flush=True)
    started, groups = time.monotonic(), []
    try:
        for anchor in selected:
            index = anchor['batch_index']
            # 夹具只指定本组已登记的程序选点；不替换求解、碰撞、物理或接触反馈。
            class RegisteredAnchorTrial(original_type):
                def __init__(self, config):
                    super().__init__(config)
                    self.anchors = [copy.deepcopy(anchor)]
            pipeline.AnchorTrial = RegisteredAnchorTrial
            folder = OUT / ('group_%02d' % index)
            begin = time.monotonic()
            result = pipeline.run(config, folder)
            row = result['results'][0]
            group = dict(group=index, folder=folder.name, feature_index_xyz=anchor['feature_index_xyz'],
                anchor_grip_m=anchor['anchor_grip_m'], raw_network_score=anchor['raw_network_score'],
                status=row['status'], success=row['sustained_opposed_contact'],
                geometry_feasible=bool(row['optimization_trace'][-1]['feasible']),
                geometric_updates=len(row['optimization_trace']), physics_steps=row['physics_steps'],
                elapsed_seconds=time.monotonic() - begin,
                final_contacts=row.get('final_contacts'), finger_participation=row.get('finger_participation'))
            groups.append(group)
            save_json(OUT / 'progress.json', groups)
            print(json.dumps(dict(group=index, status=group['status'], success=group['success'],
                                   elapsed_seconds=group['elapsed_seconds']), ensure_ascii=False), flush=True)
    finally:
        pipeline.AnchorTrial = original_type
    assert all(digest_file(Path(p)) == sha for p, sha in hashes.items())
    summary = dict(groups=groups, count=len(groups), success_count=sum(g['success'] for g in groups),
                   geometry_feasible_count=sum(g['geometry_feasible'] for g in groups),
                   elapsed_seconds=time.monotonic() - started, input_unchanged=True,
                   observation_id=trial.observation_id, network_forwards=0,
                   hardware_execution_performed=False, motion_target_valid=False)
    save_json(OUT / 'summary.json', summary)
    print(json.dumps({k:v for k,v in summary.items() if k!='groups'}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    with threadpool_limits(limits=1):
        main()
