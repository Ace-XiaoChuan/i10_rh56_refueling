#!/usr/bin/env python3
"""从独立 RGB-D 与整盖标注生成 RH56 离线候选；不读取盖体真值、不控制机械臂。"""

import argparse
from contextlib import redirect_stderr, redirect_stdout
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import random
import shutil
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "dependencies/SpaHybGen/src"))
os.environ.setdefault("MPLCONFIGDIR", "/tmp/cap-spahybgen-matplotlib")

import numpy as np
from PIL import Image, ImageDraw
import torch
from spahybgen import inference
from spahybgen.networks import load_network
from spahybgen.observation import VoxelVolume, CameraIntrinsic
from spahybgen.utils.utils_trans_np import Transform

from fuel_cap_task.cap_spahybgen_input import local_geometry, observed_free_grid, restrict_prediction, target_ellipse
from fuel_cap_task.cap_spahybgen_optimizer import CapRH56Optimization
from fuel_cap_task.rgbd_point_cloud import digest, load_observation
from fuel_cap_task.rh56_spahybgen import audit_hand_model
from generate_rh56_grasps import check_model


def save_json(path, value):
    """标准 JSON 拒绝非有限数值，保留中文说明。"""
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def prepare(args, config, output):
    """使用原 VoxelVolume 深度雕刻输入；目标标注只决定局部位置与接触范围。"""
    observation = load_observation(args.observation_dir)
    annotation = config["target_annotation"]
    if (observation.input_sha256["rgb.png"] != annotation["image_sha256"]
            or observation.metadata["image_size_wh"] != annotation["image_size_wh"]):
        raise ValueError("整盖标注与 RGB 图片不匹配")
    with np.load(args.observation_dir / "depth.npz", allow_pickle=False) as data:
        depth, valid = data["depth_m"], data["valid"]
    mask = target_ellipse(depth.shape, annotation["ellipse_xyxy"])
    geometry = local_geometry(depth, valid, observation.metadata["camera_matrix"],
                              observation.metadata["world_from_optical"], mask,
                              config["volume_size_m"], config["resolution"],
                              config["contact_radius_m"], config["center_radius_m"])
    width, height = observation.metadata["image_size_wh"]
    k = np.asarray(observation.metadata["camera_matrix"])
    intrinsic = CameraIntrinsic(width, height, k[0, 0], k[1, 1], k[0, 2], k[1, 2])
    volume = VoxelVolume(config["volume_size_m"], config["resolution"])
    # 原输入生成器读取的是体积到相机的变换，不能误传其逆矩阵。
    measured_depth = np.where(valid & np.isfinite(depth) & (depth > 0), depth, 0).astype(np.float32)
    volume.integrate(measured_depth, intrinsic, Transform.from_matrix(geometry["camera_from_volume"]))
    grid = volume.get_grid()[0].astype(np.float32)
    if not (0 < grid.sum() < grid.size):
        raise ValueError("雕刻体积全空或全满，拒绝推理")
    free = observed_free_grid(depth, valid, k, geometry["camera_from_volume"],
                              config["volume_size_m"], config["resolution"])
    geometry["grid"] = grid
    geometry["observed_free_grid"] = free
    geometry["target_mask"] = mask
    pixels = geometry["pixels_uv"]
    with Image.open(args.observation_dir / "rgb.png") as image:
        rgb = np.asarray(image.convert("RGB"))
        annotated = image.convert("RGB")
    geometry["colors_rgb"] = rgb[pixels[:, 1], pixels[:, 0]]
    ImageDraw.Draw(annotated).ellipse(annotation["ellipse_xyxy"], outline=(225, 60, 40), width=3)
    annotated.save(output / "target_annotation.png")
    np.savez_compressed(output / "scene_input.npz", **geometry)
    return geometry, observation


def verify(output):
    """只读复核输入、来源和输出；不复用不匹配的预测或旧手模型。"""
    result = json.loads((output / "result.json").read_text())
    for name, expected in result["source_sha256"].items():
        if digest(ROOT / name) != expected:
            raise ValueError("源码或网络权重已变化：" + name)
    for name, expected in result["inputs_sha256"].items():
        if digest(output / name) != expected:
            raise ValueError("运行输入已变化：" + name)
    for name, expected in result["outputs_sha256"].items():
        if digest(output / name) != expected:
            raise ValueError("运行输出已变化：" + name)
    check_model(output / result["model_dir"])
    print("油箱盖观测、网络、RH56 模型及生成结果来源复核通过")


def run(args):
    """固定随机源，完整保存推理、优化与输入；失败不覆盖其他运行。"""
    output = args.output_dir.resolve()
    config = json.loads(args.config.read_text())
    if (config["volume_size_m"] != .4 or config["resolution"] != 80
            or config["network_mode"] not in ("eval", "upstream_train")):
        raise ValueError("当前预训练基线仅接受 0.4 m、80³ 和明确记录的网络模式")
    if min(config["batch"], config["iterations"], config["threads"]) <= 0:
        raise ValueError("候选数、迭代数和线程数须为正")
    if not (0 < config.get("init_random_scale", .2) <= 1 and 0 < config.get("learning_rate", .001) <= .01):
        raise ValueError("初始关节采样须在可行范围内，学习率须在 (0,0.01] 内")
    check_model(args.model_dir)
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(args.config, output / "settings.json")
    started = time.monotonic()
    torch.set_num_threads(config["threads"])
    torch.manual_seed(config["seed"])
    np.random.seed(config["seed"])
    random.seed(config["seed"])
    checkpoint = ROOT / "dependencies/SpaHybGen/assets/trained_models/spahybgen_unet_64_voxel.pt"
    sources = [Path(__file__), ROOT / "src/fuel_cap_task/cap_spahybgen_input.py",
               ROOT / "src/fuel_cap_task/cap_spahybgen_optimizer.py",
               ROOT / "src/fuel_cap_task/rh56_spahybgen.py", ROOT / "src/fuel_cap_task/rh56_drive_map.py",
               ROOT / "src/fuel_cap_task/rgbd_point_cloud.py", ROOT / "tools/generate_rh56_grasps.py",
               ROOT / "dependencies/SpaHybGen/src/spahybgen/observation.py",
               ROOT / "dependencies/SpaHybGen/src/spahybgen/optimization.py", checkpoint]
    source_hashes = {str(p.relative_to(ROOT)): digest(p) for p in sources}
    try:
        with (output / "runtime.log").open("w") as log, redirect_stdout(log), redirect_stderr(log):
            geometry, observation = prepare(args, config, output)
            save_json(output / "model_audit.json", audit_hand_model(args.model_dir))
            network = load_network(checkpoint, torch.device("cpu"),
                                   {"voxel_discreteness": 80, "orientation": "quat", "augment": False})
            # 对照发现该权重的 eval 输出退化，迁移基线沿用作者默认 BN 模式。
            # predict() 使用 no_grad；不创建网络优化器，不更新或保存网络权重。
            network.train(config["network_mode"] == "upstream_train")
            quality, rotations, centers = inference.predict(geometry["grid"][None], network, torch.device("cpu"))
            if not np.isfinite(rotations).all():
                raise ValueError("网络朝向预测包含非有限数值")
            target_quality, target_centers = restrict_prediction(quality, centers, geometry)
            np.savez_compressed(output / "inference.npz", quality_raw=quality, rotation_xyzw=rotations,
                                center_score_raw=centers, quality=target_quality, center_score=target_centers)
            scene = np.concatenate([geometry["grid"][None], target_quality[None], rotations, target_centers[None]])
            optimizer = CapRH56Optimization(args.model_dir, batch=config["batch"])
            optimizer.init_random_scale = config.get("init_random_scale", .2)
            optimizer.learning_rate = config.get("learning_rate", .001)
            trajectory, losses = optimizer.run_optimization(scene, max_iter=config["iterations"], tqdm_disable=True)
            final = optimizer.evaluate_final()
            q = optimizer.get_opt_q()
            optimizer.handmodel.update_kinematics(q)
            rotation = optimizer.handmodel.global_rotation.detach().numpy()
            full = optimizer.handmodel.expand_drivers(q[:, 9:]).detach().numpy()
            contacts, normals = optimizer.handmodel.sample_contact_points_and_normal()
            q_np = q.detach().numpy()
            hard = optimizer.handmodel.drives.validate(q_np[:, 9:], full)
            hard &= np.isfinite(trajectory).all(axis=(1, 2))
            hard &= np.max(np.abs(rotation.transpose(0, 2, 1) @ rotation - np.eye(3)), axis=(1, 2)) < 1e-5
            hard &= np.abs(np.linalg.det(rotation) - 1) < 1e-5
            for value in final.values():
                hard &= np.isfinite(value).reshape(config["batch"], -1).all(axis=1)
            rank = np.argsort(final["loss_opti"])
            np.savez_compressed(output / "optimization.npz", q_trajectory=trajectory, q_final=q_np,
                                full_joints_rad=full, hard_valid=hard, rank=rank,
                                contact_points_volume_m=contacts.detach().numpy(),
                                contact_normals_volume=normals.detach().numpy(),
                                **{"history_" + key: value for key, value in losses.items()},
                                **{"final_" + key: value for key, value in final.items()})
            candidates = []
            for index in range(config["batch"]):
                volume_from_hand = np.eye(4)
                volume_from_hand[:3, :3], volume_from_hand[:3, 3] = rotation[index], q_np[index, :3]
                candidates.append({"id": index, "hard_valid": bool(hard[index]),
                                   "volume_from_hand": volume_from_hand.tolist(),
                                   "camera_from_hand": (geometry["camera_from_volume"] @ volume_from_hand).tolist(),
                                   "world_from_hand": (geometry["world_from_volume"] @ volume_from_hand).tolist(),
                                   "driver_angles_rad": q_np[index, 9:].tolist(),
                                   "full_joint_angles_rad": full[index].tolist(),
                                   "objective": float(final["loss_opti"][index]),
                                   "physical_grasp_verified": False})
            save_json(output / "candidates.json", {
                "schema_version": 1, "mode": "cap_rgbd_offline_candidates",
                "driver_names": optimizer.handmodel.metadata["mapping"]["driver_names"],
                "full_joint_names": optimizer.handmodel.metadata["mapping"]["joint_names"],
                "rank": rank.tolist(), "candidates": candidates,
                "model_dir": os.path.relpath(args.model_dir, output)})
        inputs = {}
        for parent, names in [(args.observation_dir, ("rgb.png", "depth.npz", "observation.json")),
                              (args.model_dir, ("model.json",))]:
            for name in names:
                path = parent / name
                inputs[os.path.relpath(path, output)] = digest(path)
        environment = {p.metadata["Name"]: p.version for p in importlib.metadata.distributions()}
        save_json(output / "environment.json", environment)
        result = {"status": "completed_cap_rgbd_offline_generation", "python": sys.version,
                  "model_dir": os.path.relpath(args.model_dir, output), "elapsed_s": time.monotonic() - started,
                  "settings": config, "hard_valid_count": int(hard.sum()),
                  "driver_limits_all_iterations": bool(optimizer.handmodel.drives.validate(trajectory[:, :, 9:]).all()),
                  "network_training_mode": network.training, "network_weights_optimized": False,
                  "checkpoint_sha256": digest(checkpoint),
                  "target_annotation_source": config["target_annotation"]["source"],
                  "evaluation_truth_used_for_generation": False, "scene_geometry_used_for_generation": False,
                  "grid_occupied_count": int(geometry["grid"].sum()),
                  "target_contact_peak": float(target_quality.max()),
                  "target_center_peak": float(target_centers.max()),
                  "prediction_evidence_sufficient": bool(target_quality.max() >= config["minimum_contact_score"]
                                                          and target_centers.max() >= config["minimum_center_score"]),
                  "method_changes": ["whole_cap_observation_neighborhood", "single_target_predicted_weighted_center",
                                     "six_driver_hard_limits_and_final_recompute", "recorded_network_mode"],
                  "arm_planning_performed": False, "hardware_execution_performed": False,
                  "physical_grasp_verified": False, "source_sha256": source_hashes,
                  "inputs_sha256": inputs,
                  "outputs_sha256": {p.name: digest(p) for p in output.iterdir() if p.is_file()}}
        if not result["driver_limits_all_iterations"]:
            raise ValueError("优化迭代中存在驱动越界")
        save_json(output / "result.json", result)
        print(json.dumps({key: result[key] for key in ("status", "elapsed_s", "hard_valid_count",
                         "target_contact_peak", "target_center_peak")}, ensure_ascii=False))
    except Exception:
        (output / "failure.txt").write_text(traceback.format_exc())
        raise


def main():
    """所有运行显式传入观测与模型；已存在的输出目录只允许只读核对。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--observation-dir", type=Path)
    parser.add_argument("--model-dir", type=Path)
    parser.add_argument("--config", type=Path, default=ROOT / "config/spahybgen_cap_demo.json")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    if args.verify_only:
        return verify(args.output_dir.resolve())
    if args.observation_dir is None or args.model_dir is None:
        parser.error("生成时须提供 --observation-dir 和 --model-dir")
    args.observation_dir, args.model_dir = args.observation_dir.resolve(), args.model_dir.resolve()
    run(args)


if __name__ == "__main__":
    main()
