#!/usr/bin/env python3
"""运行未修改的上游完整示例，仅增加来源核对和结果保存。"""

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import random
import runpy
import subprocess
import sys
import time
import traceback


ROOT = Path(__file__).resolve().parents[1]
SOURCE_COMMIT = "39e5794506bbbf2fd41804f73c93b7b393be550e"


def digest(path):
    """绑定实际文件内容，避免把替换后的权重或源码当作本次输入。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def verify(output, source):
    """只读复核已有结果的输入、实现和输出绑定，不重新推理或优化。"""
    report = json.loads((output / "result.json").read_text())
    if report["status"] != "completed_original_example":
        raise ValueError("已有记录不是完整示例成功结果")
    if report["source_commit"] != SOURCE_COMMIT or report["runner_sha256"] != digest(__file__):
        raise ValueError("实现版本与已有结果不一致")
    if report["requirements_sha256"] != digest(ROOT / "dependencies/spahybgen-reproduction.txt"):
        raise ValueError("依赖合同与已有结果不一致")
    packages = {dist.metadata["Name"]: dist.version for dist in importlib.metadata.distributions()}
    if packages != json.loads((output / "environment.json").read_text()) or sys.version != report["python"]:
        raise ValueError("当前 Python 环境与本次复现记录不一致")
    for name, expected in json.loads((output / "source_manifest.json").read_text()).items():
        if digest(source / name) != expected:
            raise ValueError("上游输入或源码已改变：" + name)
    for name, expected in report["outputs"].items():
        if digest(output / name) != expected:
            raise ValueError("复现输出已改变：" + name)
    print("已有复现来源和输出复核通过：", output)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, default=ROOT / "dependencies/SpaHybGen")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--max-iter", type=int, default=120)
    parser.add_argument("--seed", type=int, default=20261008)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    if args.batch_size < 5 or args.max_iter < 1:
        parser.error("batch-size 至少为 5（上游视图展示前 20%），max-iter 必须为正数")
    source = args.source_dir.resolve()
    output = args.output_dir.resolve()

    # 固定上游版本并拒绝覆盖；失败也会留下独立记录。
    commit = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    dirty = subprocess.check_output(
        ["git", "-C", str(source), "status", "--porcelain", "--untracked-files=no"], text=True
    ).strip()
    if commit != SOURCE_COMMIT or dirty:
        raise ValueError("上游版本不匹配或已跟踪源码有修改，拒绝复现")
    if args.verify_only:
        return verify(output, source)
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    report = {
        "status": "running", "started_utc": datetime.now(timezone.utc).isoformat(),
        "source_url": "https://github.com/wangzivector/SpaHybGen", "source_commit": commit,
        "entrypoint": "scripts/generation_pipeline_test.py", "hand": "robotiq2f",
        "batch_size": args.batch_size, "max_iter": args.max_iter, "seed": args.seed,
        "python": sys.version, "platform": platform.platform(),
        "runner_sha256": digest(__file__),
        "requirements_sha256": digest(ROOT / "dependencies/spahybgen-reproduction.txt"),
        "algorithm_modified": False, "physical_grasp_validated": False,
        "rh56_validated": False, "cap_task_validated": False,
        "instrumentation": ["固定随机种子及 CPU 线程数", "保存推理及优化数组", "将浏览器展示保存为 HTML"],
    }
    tracked = subprocess.check_output(["git", "-C", str(source), "ls-files", "-z"]).decode().split("\0")
    save_json(output / "source_manifest.json", {name: digest(source / name) for name in tracked if name})
    packages = {dist.metadata["Name"]: dist.version for dist in importlib.metadata.distributions()}
    save_json(output / "environment.json", packages)
    save_json(output / "result.json", report)
    stage = "imports"
    original_cwd = Path.cwd()
    original_argv = sys.argv[:]
    patches = []
    try:
        # 直接从固定 checkout 导入；原 catkin 安装流程不进入 ROS 2 项目。
        sys.path.insert(0, str(source / "src"))
        os.chdir(source)
        import numpy as np
        import torch
        from plotly import graph_objects as go
        from spahybgen import networks
        from spahybgen.pipeline.grasp_optimization import GraspOptimization

        random.seed(args.seed)
        np.random.seed(args.seed)
        torch.manual_seed(args.seed)
        torch.set_num_threads(min(4, os.cpu_count() or 1))
        if torch.cuda.is_available():
            raise ValueError("此入口记录 CPU 复现，请使用锁定的 CPU 环境")
        report["device"] = "cpu"
        report["torch_threads"] = torch.get_num_threads()
        timings = {}
        captured = {}
        original_load = networks.load_network
        original_run = GraspOptimization.run_optimization
        original_show = go.Figure.show

        def load_and_record(*positional, **keywords):
            """保留上游网络模式，记录而不暗中改成 eval。"""
            net = original_load(*positional, **keywords)
            report["network_training_mode"] = bool(net.training)
            return net

        def run_and_record(optimizer, *positional, **keywords):
            """原方法只执行一次，旁路保存进入优化器的预测和返回数组。"""
            nonlocal stage
            scene = keywords.get("scene_infer_map_np")
            if scene is None:
                scene = positional[0]
            np.savez_compressed(output / "inference.npz", grid=scene[0], quality=scene[1],
                                rotation_xyzw=scene[2:6], wrench_center_score=scene[6])
            stage = "optimization"
            tick = time.perf_counter()
            trajectory, losses = original_run(optimizer, *positional, **keywords)
            timings["optimization_seconds"] = time.perf_counter() - tick
            np.savez_compressed(output / "optimization.npz", q_trajectory=trajectory, **losses)
            expected = (args.batch_size, args.max_iter + 1, 9 + len(optimizer.handmodel.joints))
            if trajectory.shape != expected or not np.isfinite(trajectory).all():
                raise ValueError("优化轨迹维度错误或包含非有限值")
            if any(not np.isfinite(value).all() for value in losses.values()):
                raise ValueError("优化损失包含非有限值")
            if scene.shape != (7, 80, 80, 80) or not np.isfinite(scene).all():
                raise ValueError("预测结果维度错误或包含非有限值")
            best = int(losses["sort_ids"][0])
            report["trajectory_shape"] = list(trajectory.shape)
            report["prediction_shape"] = list(scene.shape)
            report["best_candidate_id"] = best
            report["best_final_losses"] = {key: float(value[best, -1]) for key, value in losses.items()
                                           if key != "sort_ids"}
            report["loss_state_timing"] = "上游最后一次更新前的损失；保存的轨迹末态是更新后的 q，接入时需复核末态"
            report["all_candidates_retained"] = True
            captured["optimizer"] = optimizer
            report["joint_names"] = list(optimizer.handmodel.robot.get_joint_parameter_names())
            report["pose_representation"] = "xyz_m + R6d + URDF joints; 不是机械臂执行轨迹"
            lower = optimizer.q_joint_lower.cpu().numpy()
            upper = optimizer.q_joint_upper.cpu().numpy()
            joint_q = trajectory[:, -1, 9:]
            violation = np.maximum(np.maximum(lower - joint_q, joint_q - upper), 0)
            report["final_joint_limit_violation_max"] = float(violation.max())
            report["best_joint_limit_violation_max"] = float(violation[best].max())
            report["candidates_with_strict_joint_limit_violation"] = int((violation.max(axis=1) > 0).sum())
            stage = "visualization"
            return trajectory, losses

        def save_view(figure, *unused, **unused_keywords):
            """保留原图内容，保存自包含网页而不依赖浏览器自动打开。"""
            figure.write_html(str(output / "grasps.html"), include_plotlyjs=True, auto_open=False)
            report["visualization_trace_count"] = len(figure.data)
            optimizer = captured["optimizer"]
            best_meshes = optimizer.get_current_plotly_data(index=report["best_candidate_id"])
            best_view = go.Figure(data=list(figure.data[:2]) + best_meshes)
            best_view.write_html(str(output / "best_grasp.html"), include_plotlyjs=True, auto_open=False)

            # 静态图仅帮助检查实际输出；不把图中的接近形状称作物理抓稳。
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d.art3d import Poly3DCollection
            volume = optimizer.scene_grid.cpu().numpy()
            points = np.argwhere(volume > 0.5) * optimizer.voxel_size
            points = points[::max(1, len(points) // 5000)]
            preview = plt.figure(figsize=(10, 5))
            for column, azimuth in enumerate([-60, 35], start=1):
                axis = preview.add_subplot(1, 2, column, projection="3d")
                axis.scatter(*points.T, c=points[:, 2], cmap="viridis", s=1, alpha=0.2)
                for mesh in best_meshes:
                    vertices = np.column_stack([mesh.x, mesh.y, mesh.z])
                    faces = np.column_stack([mesh.i, mesh.j, mesh.k]).astype(int)
                    axis.add_collection3d(Poly3DCollection(vertices[faces], facecolor="crimson", alpha=0.65))
                axis.set(xlim=(0, 0.4), ylim=(0, 0.4), zlim=(0, 0.4), xlabel="x / m", ylabel="y / m", zlabel="z / m")
                axis.set_box_aspect((1, 1, 1))
                axis.view_init(elev=25, azim=azimuth)
            preview.suptitle("SpaHybGen original example: best ranked Robotiq-2F candidate")
            preview.tight_layout()
            preview.savefig(output / "preview.png", dpi=150)
            plt.close(preview)

        patches = [(networks, "load_network", original_load),
                   (GraspOptimization, "run_optimization", original_run), (go.Figure, "show", original_show)]
        networks.load_network = load_and_record
        GraspOptimization.run_optimization = run_and_record
        go.Figure.show = save_view

        # 上游脚本负责网络、预处理、原手模型和所有优化步骤。
        stage = "upstream_pipeline"
        sys.argv = [str(source / report["entrypoint"]), "--hand", "robotiq2f",
                    "--max_iter", str(args.max_iter), "--batch_size", str(args.batch_size)]
        runpy.run_path(sys.argv[0], run_name="__main__")
        if not all((output / name).is_file() for name in ["inference.npz", "optimization.npz", "grasps.html"]):
            raise ValueError("上游示例返回但缺失必需结果")
        report["timings"] = timings
        report["status"] = "completed_original_example"
        report["outputs"] = {name: digest(output / name) for name in [
            "inference.npz", "optimization.npz", "grasps.html", "best_grasp.html", "preview.png",
            "environment.json", "source_manifest.json"]}
        print("复现完成：", output, flush=True)
        return 0
    except Exception as error:
        report["status"] = "failed"
        report["failed_stage"] = stage
        report["error"] = repr(error)
        (output / "failure.txt").write_text(traceback.format_exc())
        traceback.print_exc()
        return 1
    finally:
        for target, name, original in patches:
            setattr(target, name, original)
        os.chdir(original_cwd)
        sys.argv = original_argv
        report["elapsed_seconds"] = time.perf_counter() - started
        save_json(output / "result.json", report)


if __name__ == "__main__":
    raise SystemExit(main())
