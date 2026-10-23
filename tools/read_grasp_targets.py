#!/usr/bin/env python3
"""读取当前盖体与固定候选，打印手根、法兰目标及坐标链复核数值。"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import mujoco
import numpy as np

from fuel_cap_task.arm_ik import body_transform
from fuel_cap_task.fixed_grasp import DEFAULT_FIXED_GRASP, load_fixed_grasp
from fuel_cap_task.grasp_targets import inverse, transform
from fuel_cap_task.grasp_preparation import prepare_grasp
from fuel_cap_task.grasp_search import pose_arrays
from fuel_cap_task.interfaces import TaskFailure
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE, MujocoCapScene


def main() -> int:
    """串起第 2～5 步；只更新运动学，不求解 IK，不推进仿真时间。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path, default=DEFAULT_FIXED_GRASP)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--distance", type=float,
                        help="沿盖体外向轴退开的距离（m，必须为有限正数）；传入时打印接近目标")
    args = parser.parse_args()

    try:
        grasp = load_fixed_grasp(args.candidate, source_scene_path=args.scene,
                                 expected_mujoco_version=mujoco.__version__)
        scene = MujocoCapScene(args.scene)
        cap = scene.estimator.read()
        # read() 已更新运动学；安装链、基座和盖体都来自这一份场景状态。
        world_from_base = body_transform(scene.data, "base_link")
        flange_from_hand = (inverse(body_transform(scene.data, "wrist3_Link"))
                            @ body_transform(scene.data, "base"))
        prepared = prepare_grasp(
            grasp, cap,
            flange_from_hand=flange_from_hand, world_from_base=world_from_base,
            approach_distance_m=args.distance,
        )
        targets, approach = prepared.targets, prepared.approach
    except (OSError, ValueError, TaskFailure) as exc:
        parser.exit(1, f"无法生成抓姿目标：{exc}\n")

    relative = transform(*pose_arrays(asdict(grasp.cap_from_hand)))
    # 残差是整个 4×4 矩阵的最大元素差，用来复核坐标链，不是实际到位误差。
    residuals = {
        "cap_relative_max_abs": float(np.max(np.abs(
            inverse(targets.world_from_cap) @ targets.world_from_hand - relative))),
        "flange_mount_max_abs": float(np.max(np.abs(
            targets.world_from_flange @ flange_from_hand - targets.world_from_hand))),
        "base_hand_max_abs": float(np.max(np.abs(
            world_from_base @ targets.base_from_hand - targets.world_from_hand))),
        "base_flange_max_abs": float(np.max(np.abs(
            world_from_base @ targets.base_from_flange - targets.world_from_flange))),
    }
    report = {
        "mode": "fixed_grasp_target_calculation",
        "cap_estimate": asdict(cap),
        "candidate_provenance": asdict(grasp.provenance),
        "cap_from_hand": relative.tolist(),
        "flange_from_hand": flange_from_hand.tolist(),
        "world_from_base": world_from_base.tolist(),
        "targets": {name: value.tolist() for name, value in asdict(targets).items()},
        "chain_residuals": residuals,
    }
    if approach is not None:
        report["approach_targets"] = prepared.approach_record()
        residuals["pregrasp_mount_max_abs"] = float(np.max(np.abs(
            approach.world_from_pregrasp_flange @ flange_from_hand
            - approach.world_from_pregrasp_hand)))
    print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
