#!/usr/bin/env python3
"""Create a provenance-bound MTC request from an estimated cap and current candidate."""
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
    args = parser.parse_args()
    import numpy as np
    if not np.isfinite(args.start).all() or not np.isfinite(args.distance) or args.distance <= 0:
        parser.error('Finite start and positive distance required')
    from check_moveit_model_sources import check
    check()
    preview = ArmIKPreview(args.candidate)
    preview.forward(args.start)
    model, data = preview.model, preview.data
    positions = {model.joint(j).name: float(data.qpos[model.jnt_qposadr[j]]) for j in range(model.njnt)
                 if model.joint(j).name in ARM_JOINTS or j in preview.scene.hand.joint_ids}
    root = Path(__file__).resolve().parents[1]
    paths = [preview.scene_path, args.candidate.resolve(), root/'src/i10_rh56_moveit_config/config/generated_hashes.json',
             root/'src/i10_rh56_moveit_config/config/provenance.json']
    source_hashes = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    axis = preview.world_from_cap[:3, 2]
    target = preview.world_from_hand
    request = {'schema_version': 1, 'arm_joints': list(ARM_JOINTS), 'initial_joint_positions': positions,
               'pregrasp_position': (target[:3, 3]+args.distance*axis).tolist(),
               'hand_position': target[:3, 3].tolist(), 'hand_orientation_xyzw': list(quaternion(target[:3, :3])),
               'outward_axis_world': axis.tolist(), 'approach_distance_m': args.distance,
               'pregrasp_channels': preview.candidate['pregrasp_channels'],
               'cap_estimate': asdict(preview.estimate), 'source_hashes': source_hashes}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(request, indent=2)+'\n')
    print(args.output)

if __name__ == '__main__':
    main()
