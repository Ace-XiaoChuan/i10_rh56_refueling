#!/usr/bin/env python3
"""顺序运行并保存油箱盖离线迁移实验；支持在来源一致时续跑已完成阶段。"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / ".venv/bin/python"
GRASP = ROOT / ".venv-spahybgen/bin/python"


def digest(path):
    """核对完成记录绑定的实际文件，不按目录存在就跳过。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save_json(path, value):
    """保存可独立读取的执行状态和摘要。"""
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def check_hashes(base, records):
    """拒绝续用输入、实现或产物已变化的运行。"""
    for relative, expected in records.items():
        if digest(base / relative) != expected:
            raise ValueError("已有结果来源或产物已变化：" + str(base / relative))


def main():
    """所有数据直接保存在项目实验目录，失败时保留日志；不做机械臂或真机操作。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists() and not args.resume:
        parser.error("输出目录已存在，需核对后显式使用 --resume 或选择新目录")
    output.mkdir(parents=True, exist_ok=True)
    logs = output / "logs"
    logs.mkdir(exist_ok=True)
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", OPENBLAS_NUM_THREADS="4",
                       MPLCONFIGDIR="/tmp/cap-spahybgen-matplotlib", MUJOCO_GL="egl",
                       PYTHONPATH=str(ROOT / "src") + ":" + str(ROOT / "dependencies/SpaHybGen/src"))

    def step(name, command):
        save_json(output / "progress.json", {"stage": name, "state": "running"})
        print("开始：" + name, flush=True)
        log = logs / (name + ".log")
        try:
            with log.open("w") as stream:
                subprocess.run([str(item) for item in command], cwd=ROOT, env=environment,
                               stdout=stream, stderr=subprocess.STDOUT, check=True)
        except subprocess.CalledProcessError:
            save_json(output / "progress.json", {"stage": name, "state": "failed", "log": str(log.relative_to(output))})
            print(log.read_text()[-3000:], flush=True)
            raise
        print("完成：" + name, flush=True)

    model, observation = output / "hand_model", output / "observation"
    if (model / "model.json").exists():
        record = json.loads((model / "model.json").read_text())
        check_hashes(ROOT, record["inputs"])
        check_hashes(model, record["outputs"])
    else:
        step("hand_model", [SIM, ROOT / "tools/build_rh56_spahybgen_model.py", "--output-dir", model])
    if (observation / "observation.json").exists():
        check_hashes(observation, json.loads((observation / "observation.json").read_text())["files"])
    else:
        step("rgbd_capture", [SIM, ROOT / "tools/capture_cap_rgbd.py", "--width", 1280, "--height", 960,
                             "--lookat-world-m", 0, .94, .62, "--distance-m", .9, "--azimuth-deg", 90,
                             "--elevation-deg", -20, "--output-dir", observation])
    default = json.loads((ROOT / "config/spahybgen_cap_demo.json").read_text())
    settings = output / "experiment_settings"
    settings.mkdir(exist_ok=True)
    for number, mode, scale, rate, iterations in ((1, "eval", .2, .001, 120),
                                                 (2, "upstream_train", .2, .001, 120),
                                                 (3, "upstream_train", 1., .003, 240)):
        config = dict(default, network_mode=mode, init_random_scale=scale,
                      learning_rate=rate, iterations=iterations)
        config_path = settings / f"run_{number:03d}.json"
        if config_path.exists() and json.loads(config_path.read_text()) != config:
            raise ValueError("实验设置已变化，请选择新目录")
        if not config_path.exists():
            save_json(config_path, config)
        run, geometry = output / f"run_{number:03d}", output / f"geometry_{number:03d}"
        if (run / "result.json").exists():
            record = json.loads((run / "result.json").read_text())
            check_hashes(ROOT, record["source_sha256"])
            check_hashes(run, record["inputs_sha256"])
            check_hashes(run, record["outputs_sha256"])
        else:
            step(f"generation_{number:03d}", [GRASP, ROOT / "tools/generate_cap_grasps.py",
                 "--observation-dir", observation, "--model-dir", model,
                 "--config", config_path, "--output-dir", run])
        if (geometry / "geometry.json").exists():
            record = json.loads((geometry / "geometry.json").read_text())
            check_hashes(ROOT, record["source_sha256"])
            check_hashes(geometry, record["outputs_sha256"])
            if record["generation_result_sha256"] != digest(run / "result.json"):
                raise ValueError("几何结果与候选不匹配")
        else:
            command = [SIM, ROOT / "tools/check_cap_grasp_geometry.py", "--result-dir", run,
                       "--output-dir", geometry]
            if number < 3:
                command.append("--no-render")
            step(f"geometry_{number:03d}", command)
    latest, geometry, views = output / "run_003", output / "geometry_003", output / "views"
    comparison = output / "network_mode_comparison.json"
    if not comparison.exists():
        step("network_mode_comparison", [GRASP, ROOT / "tools/compare_cap_network_modes.py",
             "--scene-input", latest / "scene_input.npz", "--output", comparison])
    if not (views / "views.json").exists():
        step("render_views", [GRASP, ROOT / "tools/render_cap_grasps.py", "--result-dir", latest,
                              "--geometry-dir", geometry, "--output-dir", views])
    else:
        record = json.loads((views / "views.json").read_text())
        check_hashes(views, record["outputs_sha256"])
        if record["renderer_sha256"] != digest(ROOT / "tools/render_cap_grasps.py"):
            raise ValueError("已有展示使用其他版本渲染器，请选择新目录")
    step("tests_input", [SIM, "-m", "pytest", ROOT / "tools/tests/test_cap_spahybgen_input.py",
                         "-q", "-p", "no:cacheprovider"])
    step("tests_adapter", [GRASP, "-m", "unittest", "discover", "-s", ROOT / "tests/spahybgen", "-v"])
    step("verify_latest", [GRASP, ROOT / "tools/generate_cap_grasps.py", "--output-dir", latest, "--verify-only"])
    input_log = (logs / "tests_input.log").read_text()
    adapter_log = (logs / "tests_adapter.log").read_text()
    counts = [int(re.search(r"(\d+) passed", input_log)[1]), int(re.search(r"Ran (\d+) tests", adapter_log)[1])]
    report = json.loads((latest / "result.json").read_text())
    checks = json.loads((geometry / "geometry.json").read_text())
    save_json(output / "validation.json", {"latest_run": "run_003", "test_count_passed": sum(counts),
              "candidate_count": len(checks["candidates"]), "geometry_pass_count": checks["geometrically_clear_count"],
              "source_and_outputs_verified": True, "network_weights_optimized": False,
              "arm_planning_performed": False, "hardware_execution_performed": False})
    snapshot = output / "implementation_snapshot"
    names = set(report["source_sha256"]) | {
        "tools/check_cap_grasp_geometry.py", "tools/render_cap_grasps.py", "tools/run_cap_spahybgen_demo.py",
        "tools/compare_cap_network_modes.py", "config/spahybgen_cap_demo.json",
        "tools/tests/test_cap_spahybgen_input.py", "tests/spahybgen/test_cap_target_optimizer.py",
        "tests/spahybgen/test_cap_volume_carving.py"}
    for name in names:
        if Path(name).suffix == ".pt":
            continue
        target = snapshot / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, target)
    save_json(output / "progress.json", {"stage": "complete", "state": "complete"})
    print(json.dumps({"output": str(output), "tests_passed": sum(counts),
                      "geometry_pass_count": checks["geometrically_clear_count"]}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
