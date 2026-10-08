#!/usr/bin/env python3
"""根据盖体估计和当前候选生成绑定来源的 MTC 规划请求。"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from fuel_cap_task.arm_ik import ArmIKPreview
from fuel_cap_task.grasp_search import quaternion
from fuel_cap_task.mujoco_scene import ARM_JOINTS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', nargs=6, type=float, required=True)
    parser.add_argument('--distance', type=float, required=True)
    parser.add_argument('--candidate', type=Path, default=Path('artifacts/manual_grasp/user_selected_20260930.json'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--random-seed', type=int, help='Prepare exactly one new mixed random scene')
    args = parser.parse_args()
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
    # world_from_cap 是盖体系到世界系的位姿矩阵。
    # [:3, 2] 取盖的外向 Z 轴在世界系中的方向；[:3, 3] 取位置（m）。
    axis = preview.world_from_cap[:3, 2]
    # target 是最终抓取手根位姿，不是盖中心位姿。
    target = preview.world_from_hand
    # 抓取前位置 = 最终手根位置 + 外退距离 × 盖外向轴；方向保持抓取方向。
    # 六个臂关节及手预型也写入请求，供 MTC 设置完整起始状态。
    request = {'schema_version': 1, 'arm_joints': list(ARM_JOINTS), 'initial_joint_positions': positions,
               'pregrasp_position': (target[:3, 3]+args.distance*axis).tolist(),
               'hand_position': target[:3, 3].tolist(), 'hand_orientation_xyzw': list(quaternion(target[:3, :3])),
               'outward_axis_world': axis.tolist(), 'approach_distance_m': args.distance,
               'pregrasp_channels': preview.candidate['pregrasp_channels'],
               'cap_estimate': asdict(preview.estimate), 'source_hashes': source_hashes}
    if variation is not None:
        # 目标变化时，障碍也必须同步；仅改变目标会在错误的油箱位置上规划。
        from fuel_cap_task.planning_scene_snapshot import scene_snapshot
        baseline = json.loads((root/'src/i10_rh56_moveit_config/config/scene.json').read_text())
        request['scene_variation'] = variation
        request['candidate_path'] = str(args.candidate.resolve().relative_to(root))
        request['planning_scene'] = scene_snapshot(baseline, model, data)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(request, indent=2)+'\n')
    print(args.output)

if __name__ == '__main__':
    main()
