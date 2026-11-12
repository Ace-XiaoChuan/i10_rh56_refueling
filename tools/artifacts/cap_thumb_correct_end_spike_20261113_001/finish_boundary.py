"""预算内第24步：去掉与追点项竞争的边界残差，再至多闭合一次。"""
import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from threadpoolctl import threadpool_limits

OUT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('correct_end_spike', OUT / 'spike.py')
trial_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trial_module)
base = trial_module.base
from fuel_cap_task.joint_collision_update import project_joint_step
from fuel_cap_task.saved_sdk_input import digest_file, save_json


def main():
    """原23步结果冻结；最后一步和唯一闭合另存，不放宽几何容差。"""
    if (OUT / 'final_result.json').exists():
        raise FileExistsError('拒绝覆盖最后一步结果')
    original_sha = digest_file(OUT / 'result.json')
    row = json.loads((OUT / 'result.json').read_text())
    assert len(row['optimization_trace']) == 23 and row['physics_steps'] == 0
    trial = trial_module.CorrectedTrial()
    last = row['optimization_trace'][-1]
    trial.rotation = np.asarray(last['volume_from_hand'])[:3, :3].copy()
    trial.drivers = np.asarray(last['drivers'])
    trial.install(trial.rotation, trial.drivers)
    checked = trial.check.evaluate(linearize=True)
    before = checked['maxima'].copy()
    basis = trial.anchor_basis()
    scales = np.full(9, .01)
    lo = np.r_[-scales[:3], np.maximum(-scales[3:], trial.kin.drives.lower - trial.drivers)]
    hi = np.r_[scales[:3], np.minimum(scales[3:], trial.kin.drives.upper - trial.drivers)]
    step, info = project_joint_step(np.zeros(9), checked['gaps'], checked['jacobian'] @ basis,
                                   lo, hi, scales, interior_margin_m=1e-6)
    old_rotation, old_drivers = trial.rotation.copy(), trial.drivers.copy()
    accepted = False
    chosen_scale = 0.
    for scale in [1., .5, .25, .125]:
        rotation = Rotation.from_rotvec(step[:3] * scale).as_matrix() @ old_rotation
        drivers = old_drivers + step[3:] * scale
        trial.install(rotation, drivers)
        after = trial.check.evaluate()
        if after['feasible']:
            trial.rotation, trial.drivers = rotation, drivers
            accepted, chosen_scale = True, scale
            break
    if not accepted:
        trial.install(old_rotation, old_drivers)
    final_check = trial.check.evaluate()
    _, _, fingers = trial.objective()
    thumb = trial.thumb
    p = trial.kin.point(thumb['link'], thumb['point_link_m'], thumb['normal_link'])[0]
    row['optimization_trace'].append(dict(iteration=23, accepted=accepted, scale=chosen_scale,
        phase='仅最小位移消除碰撞边界残差，不追点', solver=info, feasible=final_check['feasible'],
        collision=final_check['maxima'], fingers=fingers, volume_from_hand=trial.kin.pose.tolist(),
        drivers=trial.drivers.tolist(), thumb_anchor_error_m=float(np.linalg.norm(p - trial.hold_point))))
    row['boundary_finish'] = dict(before=before, after=final_check['maxima'], tolerance_m=.0001,
        contact_pull_temporarily_disabled=True, global_budget_used=24,
        reason='软修复与微弱追点残差竞争，停在容差外约36nm；最后一步采用原生间隙约束投影')
    if final_check['feasible']:
        base.close_once(trial, row)
    manifest = json.loads((OUT / 'manifest.json').read_text())
    assert all(digest_file(Path(p)) == sha for p, sha in manifest['input_sha256'].items())
    assert digest_file(OUT / 'result.json') == original_sha
    row['finish_input_sha256'] = {str(OUT / 'result.json'): original_sha, str(Path(__file__)): digest_file(Path(__file__))}
    save_json(OUT / 'final_result.json', row)
    print(json.dumps({k: row[k] for k in ['status', 'sustained_opposed_contact', 'physics_steps', 'boundary_finish']}, ensure_ascii=False), flush=True)
    if row.get('final_forces'):
        print(json.dumps(row['final_forces'], ensure_ascii=False), flush=True)


if __name__ == '__main__':
    with threadpool_limits(limits=1):
        main()
