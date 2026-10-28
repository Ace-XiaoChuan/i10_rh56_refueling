#!/usr/bin/env python3
"""根据盖体估计和当前候选生成绑定来源的 MTC 规划请求。"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from fuel_cap_task.arm_ik import ArmIKPreview
from fuel_cap_task.fixed_grasp import DEFAULT_FIXED_GRASP
from fuel_cap_task.mujoco_scene import ARM_JOINTS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', nargs=6, type=float, required=True)
    parser.add_argument('--distance', type=float, required=True)
    parser.add_argument('--candidate', type=Path, default=DEFAULT_FIXED_GRASP)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--random-seed', type=int, help='Prepare exactly one new mixed random scene')
    parser.add_argument('--observed-scene', type=Path, help='实测拆分场景的 octomap_scene.json')
    parser.add_argument('--direction', choices=['direction_000','direction_180'], default='direction_000')
    args = parser.parse_args()
    if args.observed_scene and args.random_seed is not None:
        parser.error('实测场景不能与随机占位场景混用')
    import numpy as np
    if not np.isfinite(args.start).all() or not np.isfinite(args.distance) or args.distance <= 0:
        parser.error('Finite start and positive distance required')
    from check_moveit_model_sources import check
    check()
    # 默认使用当前场景；指定种子时，先固定一个随机场景，再计算目标。
    # 此处不搜索 IK 或路径，不会因规划失败而换样本。
    preview = ArmIKPreview(args.candidate)
    variation = None
    if args.random_seed is not None:
        from fuel_cap_task.random_scenes import SceneSampler, SamplingSettings
        sampler = SceneSampler(args.candidate, SamplingSettings(random_seed=args.random_seed))
        sample = sampler.single_sample()
        variation = {'random_seed': args.random_seed, 'sample': sample}
        preview = sampler.preview(sample)
    # 起点只设置六个臂关节（rad）；手仍保持候选预型。
    preview.forward(args.start)
    model, data = preview.model, preview.data
    positions = {model.joint(j).name: float(data.qpos[model.jnt_qposadr[j]]) for j in range(model.njnt)
                 if model.joint(j).name in ARM_JOINTS or j in preview.scene.hand.joint_ids}
    root = Path(__file__).resolve().parents[1]
    paths = [preview.scene_path, args.candidate.resolve(), root/'src/i10_rh56_moveit_config/config/generated_hashes.json',
             root/'src/i10_rh56_moveit_config/config/provenance.json']
    source_hashes = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    # 使用预览持有的同一份候选和观测，只增加接近距离；不再次变换或换算法兰。
    prepared = preview.preparation.with_approach(args.distance)
    candidate_key = str(args.candidate.resolve().relative_to(root))
    source_hashes[candidate_key] = prepared.grasp.provenance.candidate_sha256
    request = {'schema_version': 1, 'arm_joints': list(ARM_JOINTS), 'initial_joint_positions': positions,
               **prepared.mtc_fields(), 'source_hashes': source_hashes,
               'candidate_path': candidate_key}
    if variation is not None:
        # 目标变化时，障碍也必须同步；仅改变目标会在错误的油箱位置上规划。
        from fuel_cap_task.planning_scene_snapshot import scene_snapshot
        baseline = json.loads((root/'src/i10_rh56_moveit_config/config/scene.json').read_text())
        request['scene_variation'] = variation
        request['planning_scene'] = scene_snapshot(baseline, model, data)
    if args.observed_scene:
        from fuel_cap_task.observed_mtc import observed_fields
        scene_config = json.loads((root/'config/active_point_cloud_scene.json').read_text())
        request.update(observed_fields(args.observed_scene, root/scene_config['grasp_candidates'],
                                       args.candidate, scene_config['collision_scene'], args.direction, args.distance))
        request['initial_joint_positions'].update(request.get('fixed_left_joint_positions',{}))
        if request.get('robot_model_profile') == 'dual_arm':
            from fuel_cap_task.observed_mtc import model_directory
            model_directory(root/'src/i10_rh56_moveit_config',request)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(request, indent=2, allow_nan=False)+'\n')
    print(args.output)

if __name__ == '__main__':
    main()
