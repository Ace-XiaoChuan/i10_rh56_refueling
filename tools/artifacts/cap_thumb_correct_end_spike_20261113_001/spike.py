"""仅换拇指锚点的长轴端部，复用上一小试求解与接触设置。"""
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np
from threadpoolctl import threadpool_limits

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
PREVIOUS = OUT.parent / 'cap_thumb_optional_targets_spike_20261112_001'
sys.path.insert(0, str(ROOT / 'src'))
from fuel_cap_task.saved_sdk_input import digest_file, save_json

spec = importlib.util.spec_from_file_location('optional_target_spike', PREVIOUS / 'spike.py')
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
# 只将本次函数输出重定向到新目录；旧数据与实现不改写。
base.OUT = OUT


class CorrectedTrial(base.Trial):
    """选同一侧面的另一端原网络点，保留手根旋转、驱动和所有阈值。"""
    def __init__(self):
        super().__init__()
        self.previous_end = self.old_end
        goal = self.anchor_local.copy()
        goal[0] = -goal[0]
        local = base.transform(self.raw, self.inverse_frame)
        valid = np.flatnonzero((local[:, 1] > 0) & (local[:, 0] > .010) &
                               (local[:, 2] >= -.014) & (local[:, 2] <= -.001))
        cost = (local[valid, 0] - goal[0]) ** 2 + (local[valid, 2] - goal[2]) ** 2
        index = int(valid[np.argmin(cost)])
        self.anchor_local = local[index].copy()
        self.anchor_local[1] = .0085 - self.anchor_local[2] / 17
        self.anchor = base.transform(self.anchor_local[None], self.frame)[0]
        self.hold_point = self.anchor + self.normal * .0013
        self.old_end = dict(feature_index_xyz=self.ids[index].tolist(),
                            raw_network_score=float(self.pred['quality'][tuple(self.ids[index])]),
                            raw_feature_manual_m=local[index].tolist(), anchor_manual_m=self.anchor_local.tolist())
        self.install(self.rotation, self.drivers)


def main():
    """先保存换端证据，再进行同预算单点小试；无可行预型便停止。"""
    if (OUT / 'result.json').exists():
        raise FileExistsError('拒绝覆盖已保存试验')
    started = time.monotonic()
    trial = CorrectedTrial()
    prior = json.loads((PREVIOUS / 'result.json').read_text())
    old_anchor = np.asarray(prior['anchor_manual_m'])
    shift = trial.anchor_local - old_anchor
    predicted = [{**f, 'translated_pad_manual_m': (np.array(f['pad_point_manual_m']) + shift).tolist()}
                 for f in prior['optimization_trace'][-1]['fingers']]
    assert old_anchor[0] < -.015 and trial.anchor_local[0] > .015
    assert all(abs(f['translated_pad_manual_m'][0]) <= .030 for f in predicted)
    diagnosis = dict(previous_anchor_manual_m=old_anchor.tolist(), corrected_anchor_manual_m=trial.anchor_local.tolist(),
                     translation_only_prediction=predicted,
                     note='平移预测只核对四指长轴排列，不是可达、碰撞或接触验证；求解另行执行')
    save_json(OUT / 'end_diagnosis.json', diagnosis)
    files = [Path(__file__), PREVIOUS / 'spike.py', PREVIOUS / 'result.json',
             base.OLD / 'spike.py', base.OLD / 'fixed_forces.py',
             base.OLD / 'placement_probe/candidate_0.json', trial.ref_file, trial.side_file,
             Path(base.DEFAULT_SCENE), Path(trial.ref['model_dir']) / 'model.json',
             trial.run / 'contacts.npz', trial.run / 'network_volume.npz', trial.run / 'geometry_1mm.npz']
    files += trial.kin.hand.mesh_paths
    files += [ROOT / 'src/fuel_cap_task' / name for name in ['fixed_contact_kinematics.py', 'observed_hand_geometry.py',
                                                          'signed_hand_clearance.py', 'joint_collision_update.py']]
    hashes = {str(p): digest_file(p) for p in files}
    old_manifest = json.loads((PREVIOUS / 'manifest.json').read_text())
    manifest = {**old_manifest, 'input_sha256': hashes, 'only_changed_experiment_factor': '拇指网络目标换到长轴另一端',
                'previous_result': str(PREVIOUS / 'result.json'), 'source_run': str(trial.run)}
    save_json(OUT / 'manifest.json', manifest)
    row = dict(**trial.old_end, opposite_pool_count=len(trial.pool), status='no_safe_geometric_pose',
               sustained_opposed_contact=False, physical_grasp_verified=False, motion_target_valid=False, physics_steps=0)
    _, _, initial_fingers = trial.objective()
    row['initial_fingers'] = initial_fingers
    row['optimization_trace'] = base.optimize(trial)
    if trial.check.evaluate()['feasible']:
        base.close_once(trial, row)
    row['elapsed_seconds'] = time.monotonic() - started
    assert all(digest_file(Path(p)) == sha for p, sha in hashes.items())
    row['input_unchanged'] = True
    save_json(OUT / 'result.json', row)
    print(json.dumps({k: row[k] for k in ['anchor_manual_m', 'status', 'sustained_opposed_contact', 'physics_steps', 'elapsed_seconds']}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    with threadpool_limits(limits=1):
        main()
