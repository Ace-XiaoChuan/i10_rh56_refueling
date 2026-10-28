#!/usr/bin/env python3
"""用已验证的原示例接触预测生成 RH56 离线候选，不接任务控制。"""

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
os.environ.setdefault("MPLCONFIGDIR", "/tmp/rh56-matplotlib")

import numpy as np
import torch
from reproduce_spahybgen import verify as verify_original
from fuel_cap_task.rh56_spahybgen import RH56Optimization, audit_hand_model


def digest(path):
    """计算实际来源或产物哈希。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, data):
    """不把 NaN/Infinity 写为貌似有效的候选数值。"""
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def check_model(model_dir):
    """核对模型生成输入及几何产物，禁止只按手名接受。"""
    model = json.loads((model_dir / "model.json").read_text())
    for name, expected in model["inputs"].items():
        if digest(ROOT / name) != expected:
            raise ValueError("model input changed: " + name)
    for name, expected in model["outputs"].items():
        if digest(model_dir / name) != expected:
            raise ValueError("derived model changed: " + name)
    return model


def check_result(output):
    """只读核对保存的环境、实现和输入输出；不重新优化。"""
    report = json.loads((output / "result.json").read_text())
    if report["status"] != "completed_offline_hand_adaptation":
        raise ValueError("result is not a completed offline hand adaptation")
    if sys.version != report["python"]:
        raise ValueError("Python environment changed")
    packages = {d.metadata["Name"]: d.version for d in importlib.metadata.distributions()}
    if packages != json.loads((output / "environment.json").read_text()):
        raise ValueError("package versions changed")
    for group in ("inputs", "outputs"):
        for name, expected in report[group].items():
            path = ROOT / name if group == "inputs" else output / name
            if digest(path) != expected:
                raise ValueError(group + " changed: " + name)
    check_model(ROOT / report["model_dir"])
    verify_original(ROOT / report["original_result_dir"], ROOT / "dependencies/SpaHybGen")
    print("RH56 离线结果的输入、实现、环境和输出复核通过：", output)


def save_views(output, hand, q, grid, rank):
    """保存候选切换网页及三视图；着色不代表力或抓稳。"""
    import plotly.graph_objects as go
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    points = np.argwhere(grid > .5) * .005
    points = points[::max(1, len(points) // 6000)]
    hand.update_kinematics(q)
    contacts, normals = hand.sample_contact_points_and_normal()
    contacts, normals = contacts.detach().numpy(), normals.detach().numpy()
    best = int(rank[0])
    figure = go.Figure(go.Scatter3d(x=points[:, 0], y=points[:, 1], z=points[:, 2],
        mode="markers", name="occupied voxels", marker=dict(size=2, color=points[:, 2],
        colorscale="Viridis", opacity=.2, colorbar=dict(title="voxel z / m"))))
    groups = []
    for index, candidate in enumerate(rank[:4]):
        begin = len(figure.data)
        for mesh in hand.get_meshes_from_q(i=int(candidate)):
            vertices, faces = mesh.vertices, mesh.faces
            figure.add_trace(go.Mesh3d(x=vertices[:, 0], y=vertices[:, 1], z=vertices[:, 2],
                i=faces[:, 0], j=faces[:, 1], k=faces[:, 2], color="lightgray", opacity=.85,
                name=f"RH56 candidate {candidate}", visible=index == 0, showlegend=False))
        cp = contacts[candidate]
        figure.add_trace(go.Scatter3d(x=cp[:, 0], y=cp[:, 1], z=cp[:, 2], mode="markers",
            name="finger surface samples", marker=dict(size=3, color="crimson"), visible=index == 0))
        # 每面取一个采样点画外法向；红点只表示可接触指面，非物理接触确认。
        start, end = cp[::5], cp[::5] + .012 * normals[candidate, ::5]
        arrows = np.stack([start, end, np.full_like(start, np.nan)], axis=1).reshape(-1, 3)
        figure.add_trace(go.Scatter3d(x=arrows[:, 0], y=arrows[:, 1], z=arrows[:, 2], mode="lines",
            name="outward normals", line=dict(color="cyan", width=4), visible=index == 0))
        groups.append((begin, len(figure.data), int(candidate)))
    buttons = []
    for begin, end, candidate in groups:
        visibility = [False] * len(figure.data)
        visibility[0] = True
        visibility[begin:end] = [True] * (end - begin)
        buttons.append(dict(label=f"candidate {candidate}", method="update", args=[{"visible": visibility}]))
    figure.update_layout(title="RH56 offline geometric candidates — original SpaHybGen scene",
        scene=dict(aspectmode="data", xaxis_title="x / m", yaxis_title="y / m", zaxis_title="z / m"),
        updatemenus=[dict(buttons=buttons, x=0, y=1.12)])
    figure.write_html(str(output / "candidates.html"), include_plotlyjs=True)
    meshes = hand.get_meshes_from_q(i=best)
    preview = plt.figure(figsize=(15, 5))
    for column, azimuth in enumerate([-60, 30, 120], start=1):
        axis = preview.add_subplot(1, 3, column, projection="3d")
        axis.scatter(*points.T, c=points[:, 2], cmap="viridis", s=1, alpha=.15)
        for mesh in meshes:
            axis.add_collection3d(Poly3DCollection(mesh.vertices[mesh.faces], facecolor="gray", alpha=.6))
        axis.scatter(*contacts[best].T, c="red", s=4)
        axis.quiver(*contacts[best, ::5].T, *normals[best, ::5].T, length=.012, color="cyan")
        axis.set(xlim=(0, .4), ylim=(0, .4), zlim=(0, .4), xlabel="x / m", ylabel="y / m", zlabel="z / m")
        axis.set_box_aspect((1, 1, 1))
        axis.view_init(elev=25, azim=azimuth)
    preview.suptitle(f"RH56 candidate {best}: gray hand, red finger samples, cyan outward normals")
    preview.tight_layout()
    preview.savefig(output / "preview.png", dpi=130)
    plt.close(preview)

    # 接触区域另在可行中位手型显示，便于复核五指选面与法向。
    middle = torch.zeros_like(q)
    middle[:, 3], middle[:, 7] = 1, 1
    middle[:, 9:] = (hand.joints_q_lower + hand.joints_q_upper) / 2
    hand.update_kinematics(middle)
    region_view = go.Figure()
    for mesh in hand.get_meshes_from_q(i=0):
        v, f = mesh.vertices, mesh.faces
        region_view.add_trace(go.Mesh3d(x=v[:, 0], y=v[:, 1], z=v[:, 2], i=f[:, 0], j=f[:, 1],
            k=f[:, 2], color="lightgray", opacity=.45, showlegend=False))
    for region in hand.metadata["contact_regions"]:
        transform = hand.current_status[region["link"]].get_matrix()[0].numpy()
        vertices = np.array(region["vertices_link_m"])[:3] @ transform[:3, :3].T + transform[:3, 3]
        region_view.add_trace(go.Mesh3d(x=vertices[:, 0], y=vertices[:, 1], z=vertices[:, 2],
            i=[0], j=[1], k=[2], color="crimson", name=region["finger"], showlegend=False))
        center = vertices.mean(axis=0)
        end = center + .012 * (transform[:3, :3] @ region["normal_link"])
        region_view.add_trace(go.Scatter3d(x=[center[0], end[0]], y=[center[1], end[1]],
            z=[center[2], end[2]], mode="lines", line=dict(color="cyan", width=6), showlegend=False))
    region_view.update_layout(title="RH56 geometric finger patches and outward normals (middle driver pose)",
                             scene=dict(aspectmode="data"))
    region_view.write_html(str(output / "contact_regions.html"), include_plotlyjs=True)


def run(args):
    """一次有限预算运行；目录不可覆盖，异常保留实现快照和失败记录。"""
    output, model_dir, original = args.output_dir.resolve(), args.model_dir.resolve(), args.inference_dir.resolve()
    if output.exists():
        raise FileExistsError("output directory already exists")
    model = check_model(model_dir)
    verify_original(original, ROOT / "dependencies/SpaHybGen")
    implementation = ["tools/generate_rh56_grasps.py", "tools/reproduce_spahybgen.py",
                      "src/fuel_cap_task/__init__.py", "src/fuel_cap_task/rh56_drive_map.py",
                      "src/fuel_cap_task/rh56_spahybgen.py", "dependencies/spahybgen-reproduction.txt"]
    inputs = {name: digest(ROOT / name) for name in implementation}
    for folder, files in ((model_dir, ["model.json", *model["outputs"]]),
                          (original, ["result.json", "inference.npz", "source_manifest.json", "environment.json"])):
        for name in files:
            path = folder / name
            inputs[str(path.relative_to(ROOT))] = digest(path)
    output.mkdir(parents=True)
    for name in implementation:
        path = output / "implementation_snapshot" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, path)
    dump(output / "environment.json", {d.metadata["Name"]: d.version for d in importlib.metadata.distributions()})
    dump(output / "inputs.json", inputs)
    torch.set_num_threads(args.threads)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    random.seed(args.seed)
    started = time.monotonic()
    try:
        with (output / "runtime.log").open("w") as log, redirect_stdout(log), redirect_stderr(log):
            audit = audit_hand_model(model_dir)
            dump(output / "model_audit.json", audit)
            # 重用保存的预测，不能把这次手优化写成重新推理的结果。
            with np.load(original / "inference.npz") as data:
                scene = np.concatenate([data["grid"][None], data["quality"][None],
                                        data["rotation_xyzw"], data["wrench_center_score"][None]], axis=0)
            if scene.shape != (7, 80, 80, 80) or not np.isfinite(scene).all():
                raise ValueError("invalid original prediction")
            optimizer = RH56Optimization(model_dir, batch=args.batch)
            trajectory, history = optimizer.run_optimization(scene, max_iter=args.iterations, tqdm_disable=True)
            final = optimizer.evaluate_final()
            q = optimizer.get_opt_q()
            drivers = q[:, 9:].numpy()
            full = optimizer.handmodel.expand_drivers(q[:, 9:]).numpy()
            rotation = optimizer.handmodel.global_rotation.numpy()
            finite = np.isfinite(trajectory).all(axis=(1, 2))
            finite &= np.logical_and.reduce([np.isfinite(value).reshape(args.batch, -1).all(axis=1)
                                             for value in final.values()])
            rotation_error = np.max(np.abs(rotation.transpose(0, 2, 1) @ rotation - np.eye(3)), axis=(1, 2))
            joint_valid = optimizer.handmodel.drives.validate(drivers, full)
            pose_valid = (rotation_error < 1e-5) & (np.abs(np.linalg.det(rotation) - 1) < 1e-5)
            valid = finite & joint_valid & pose_valid
            rank_all = np.argsort(final["loss_opti"])
            rank = rank_all[valid[rank_all]]
            if not len(rank):
                raise ValueError("no finite, joint-feasible, valid-pose candidate")
            optimizer.handmodel.update_kinematics(q)
            contacts, normals = optimizer.handmodel.sample_contact_points_and_normal()
            np.savez_compressed(output / "optimization.npz", q_trajectory=trajectory,
                **{"history_" + k: v for k, v in history.items()},
                **{"final_" + k: v for k, v in final.items()}, full_joints_rad=full,
                base_rotation_matrix=rotation, contact_points_scene_m=contacts.numpy(),
                contact_normals_scene=normals.numpy(), hard_valid=valid, rank_all=rank_all, rank_valid=rank)
            candidates = []
            for i in range(args.batch):
                candidates.append({"id": i, "hard_valid": bool(valid[i]), "joint_valid": bool(joint_valid[i]),
                    "pose_valid": bool(pose_valid[i]), "finite": bool(finite[i]),
                    "scene_from_hand": {"translation_m": q[i, :3].numpy().tolist(), "rotation_matrix": rotation[i].tolist()},
                    "driver_angles_rad": drivers[i].tolist(), "full_joint_angles_rad": full[i].tolist(),
                    "final_losses": {k: np.asarray(v[i]).tolist() for k, v in final.items()},
                    "physical_grasp_validated": False, "self_collision_validated": False})
            dump(output / "candidates.json", {"root_frame": model["root_frame"], "scene_frame": "original_example_volume",
                "driver_names": model["mapping"]["driver_names"], "full_joint_names": model["mapping"]["joint_names"],
                "rank_valid": rank.tolist(), "best_candidate_id": int(rank[0]), "candidates": candidates})
            save_views(output, optimizer.handmodel, q, scene[0], rank)
        report = {"status": "completed_offline_hand_adaptation", "python": sys.version,
            "model_dir": str(model_dir.relative_to(ROOT)), "original_result_dir": str(original.relative_to(ROOT)),
            "hand": model["hand"], "batch": args.batch, "iterations": args.iterations,
            "seed": args.seed, "torch_threads": args.threads, "elapsed_s": time.monotonic() - started,
            "trajectory_shape": list(trajectory.shape), "hard_valid_count": int(valid.sum()),
            "all_trajectory_driver_limits_valid": bool(optimizer.handmodel.drives.validate(trajectory[:, :, 9:]).all()),
            "best_candidate_id": int(rank[0]), "best_final_objective": float(final["loss_opti"][rank[0]]),
            "ranking": "末态损失重算，使用最后一轮的运行均值与上游权重；未更新归一化器",
            "model_audit": audit, "network_rerun": False, "algorithm_modified": True,
            "changes": ["六驱动线性耦合展开", "扰动评价前和 Adam 更新后驱动硬投影", "末态重新计算并排序"],
            "self_collision_validated": False, "physical_grasp_validated": False,
            "cap_task_validated": False, "arm_reachability_validated": False, "hardware_calibrated": False,
            "inputs": inputs, "outputs": {str(p.relative_to(output)): digest(p)
                                           for p in output.rglob("*") if p.is_file()}}
        if not report["all_trajectory_driver_limits_valid"]:
            raise ValueError("intermediate trajectory violated driver/follower constraints")
        dump(output / "result.json", report)
        print(json.dumps({k: report[k] for k in ("status", "hard_valid_count", "best_candidate_id", "elapsed_s")}, ensure_ascii=False))
    except Exception:
        (output / "failure.txt").write_text(traceback.format_exc())
        dump(output / "result.json", {"status": "failed_offline_hand_adaptation", "inputs": inputs,
                                     "physical_grasp_validated": False})
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", type=Path, default=ROOT / "artifacts/rh56_spahybgen/model_20261008")
    parser.add_argument("--inference-dir", type=Path, default=ROOT / "artifacts/spahybgen/original_20261008_v2")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--iterations", type=int, default=120)
    parser.add_argument("--seed", type=int, default=20261008)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    if min(args.batch, args.iterations, args.threads) <= 0:
        parser.error("batch, iterations and threads must be positive")
    if args.verify_only:
        check_result(args.output_dir.resolve())
    else:
        run(args)


if __name__ == "__main__":
    main()
