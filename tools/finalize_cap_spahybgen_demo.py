#!/usr/bin/env python3
"""核对已完成的离线实验，生成汇报入口、结果说明和完整文件摘要。"""

import argparse
import hashlib
from html.parser import HTMLParser
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np
from PIL import Image


def digest(path):
    """绑定实际文件，避免记录只反映文件名和目录存在。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_hashes(base, records):
    """逐份核对来源与产物，不改写已保存的实验记录。"""
    for name, expected in records.items():
        if digest(base / name) != expected:
            raise ValueError("文件摘要不一致：" + str(base / name))


class Resources(HTMLParser):
    """核对独立三维页没有外部脚本、图片或样式文件依赖。"""

    def __init__(self):
        super().__init__()
        self.external = []

    def handle_starttag(self, tag, attributes):
        if tag in ("script", "img", "iframe", "link"):
            self.external.extend(value for key, value in attributes if key in ("src", "href"))


def main():
    """所有计算检查通过后才写总览；该操作不重新推理、优化或执行。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result-dir", type=Path, required=True)
    args = parser.parse_args()
    base = args.result_dir.resolve()
    if json.loads((base / "progress.json").read_text())["state"] != "complete":
        raise ValueError("实验流程尚未完成，拒绝生成完成说明")
    runs = []
    for number in (1, 2, 3):
        folder = base / f"run_{number:03d}"
        report = json.loads((folder / "result.json").read_text())
        check_hashes(ROOT, report["source_sha256"])
        check_hashes(folder, report["inputs_sha256"])
        check_hashes(folder, report["outputs_sha256"])
        geometry_folder = base / f"geometry_{number:03d}"
        geometry = json.loads((geometry_folder / "geometry.json").read_text())
        check_hashes(ROOT, geometry["source_sha256"])
        check_hashes(geometry_folder, geometry["outputs_sha256"])
        if geometry["generation_result_sha256"] != digest(folder / "result.json"):
            raise ValueError("几何检查没有绑定同一次生成结果")
        if geometry["arm_joint_names_present"] or geometry["physics_steps"] != 0:
            raise ValueError("离线几何评价范围不一致")
        runs.append((report, geometry))
    latest, geometry = runs[-1]
    comparison = json.loads((base / "network_mode_comparison.json").read_text())
    if (comparison["checkpoint_sha256"] != latest["checkpoint_sha256"]
            or comparison["cap_scene_input_sha256"] != digest(base / "run_003/scene_input.npz")
            or comparison["implementation_sha256"] != digest(ROOT / "tools/compare_cap_network_modes.py")
            or not all(row["all_outputs_finite"] for row in comparison["comparisons"])):
        raise ValueError("网络模式对照与当前输入、权重或实现不一致")
    upstream = ROOT / "dependencies/SpaHybGen"
    git_environment = dict(os.environ, GIT_OPTIONAL_LOCKS="0")
    commit = subprocess.check_output(["git", "-C", str(upstream), "rev-parse", "HEAD"], text=True, env=git_environment).strip()
    dirty = subprocess.check_output(["git", "-C", str(upstream), "status", "--porcelain", "--untracked-files=no"],
                                    text=True, env=git_environment).strip()
    if commit != "39e5794506bbbf2fd41804f73c93b7b393be550e" or dirty:
        raise ValueError("上游来源与固定复现版本不一致")
    model = base / "hand_model"
    metadata = json.loads((model / "model.json").read_text())
    check_hashes(ROOT, metadata["inputs"])
    check_hashes(model, metadata["outputs"])
    with np.load(base / "run_003/scene_input.npz") as scene, \
            np.load(base / "run_003/inference.npz") as inference, \
            np.load(base / "run_003/optimization.npz") as optimization:
        if scene["grid"].shape != (80, 80, 80) or not set(np.unique(scene["grid"])) <= {0, 1}:
            raise ValueError("网络输入体积或类型不符合固定基线")
        if inference["quality"][~scene["contact_region"]].any() or inference["center_score"][~scene["center_region"]].any():
            raise ValueError("目标邻域外存在未限制的预测")
        trajectory = optimization["q_trajectory"]
        if trajectory.shape != (16, 241, 15) or not np.isfinite(trajectory).all():
            raise ValueError("最新迭代数组不符合保存的设置")
        mapping = metadata["mapping"]
        full = optimization["q_final"][:, 9:] @ np.asarray(mapping["matrix"]).T + mapping["offset"]
        np.testing.assert_allclose(full, optimization["full_joints_rad"], atol=2e-7, rtol=0)
        # 真值只在生成完成后的独立评价中读取，不属于候选输入。
        with np.load(base / "observation/evaluation_only.npz") as truth:
            mask, actual = scene["target_mask"], truth["target_mask"]
            annotation_iou = float((mask & actual).sum() / (mask | actual).sum())
    views = json.loads((base / "views/views.json").read_text())
    check_hashes(base / "views", views["outputs_sha256"])
    if views["geometry_sha256"] != digest(base / "geometry_003/geometry.json"):
        raise ValueError("展示与最终几何检查不一致")
    for name in ("index.html", "contact_map.html"):
        page = Resources()
        page.feed((base / "views" / name).read_text())
        if page.external:
            raise ValueError("三维页包含外部资源依赖")
    with Image.open(base / "geometry_003/optimization.gif") as animation:
        if not animation.is_animated:
            raise ValueError("优化过程动图无动画帧")
    import cv2
    capture = cv2.VideoCapture(str(base / "geometry_003/optimization.mp4"))
    decoded = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame.shape[:2] != (700, 900):
            raise ValueError("优化视频画幅变化")
        decoded += 1
    capture.release()
    if decoded != 91:
        raise ValueError("优化视频缺少预期帧")
    validation = json.loads((base / "validation.json").read_text())
    validation.update({"all_experiment_and_model_hashes_match": True, "standalone_views_checked": True,
                       "coupled_joint_arrays_checked": True, "mp4_decoded_frames": decoded,
                       "manual_annotation_iou_evaluation_only": annotation_iou, "upstream_commit": commit})
    (base / "validation.json").write_text(json.dumps(validation, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    count, clear = len(geometry["candidates"]), geometry["geometrically_clear_count"]
    summary = f"最新一轮生成 {count} 个满足关节范围和位姿数值条件的候选，几何筛选通过 {clear} 个。"
    if clear == 0:
        summary += "当前候选存在盖体或周边环境干涉，尚不能作为有效抓取结果。"
    rows = []
    for number, (report, checks) in enumerate(runs, 1):
        config = report["settings"]
        rows.append(f"| {number} | {config['network_mode']} | {config['init_random_scale']} | {config['iterations']} | "
                    f"{report['hard_valid_count']} | {checks['geometrically_clear_count']} |")
    report_text = "# 油箱盖离线抓姿迁移结果\n\n" + summary + "\n\n" + \
        "输入为仿真 RGB-D 与人工整盖标注，网络预测接触区域，优化器生成 RH56 抓姿。未开展机械臂规划或真机执行。\n\n" + \
        "[候选交互页](views/index.html) · [接触预测](views/contact_map.html) · [优化动画](geometry_003/optimization.gif) · [MP4](geometry_003/optimization.mp4)\n\n" + \
        "| 对照 | 网络模式 | 初始关节采样比例 | 迭代 | 数值有效候选 | 几何通过 |\n| --- | --- | --- | --- | --- | --- |\n" + \
        "\n".join(rows) + "\n\n" + \
        "网络模式对照见 network_mode_comparison.json。作者模式使用当前输入的 BatchNorm 统计，不训练或改写网络权重。\n\n" + \
        f"输入、手模型和产物摘要核对通过；{validation['test_count_passed']} 项检查通过。日志位于 logs/，源码快照位于 implementation_snapshot/。\n\n" + \
        "下一步优先改进手型与接触组合、凹槽和环境干涉约束，再评价抓稳与旋拧能力。复现说明见项目 doc/cap_spahybgen_migration.md。\n"
    (base / "README.md").write_text(report_text, encoding="utf-8")
    (base / "index.html").write_text('''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>油箱盖自动抓姿离线实验</title>
<style>body{font-family:system-ui;margin:32px auto;padding:0 20px;max-width:1100px;color:#243447;line-height:1.7}
img{max-width:100%;height:auto}a{color:#225d9b}.status{padding:16px;background:#fff4df;border-radius:8px}</style>
<h1>油箱盖自动抓姿离线实验</h1><p>仿真 RGB-D · 人工整盖标注 · RH56 独立手</p>
<p class="status">''' + summary + '''</p>
<p><a href="views/index.html">打开三维候选网页</a> · <a href="views/contact_map.html">查看三维接触预测</a> ·
<a href="geometry_003/optimization.mp4">播放优化过程视频</a> · <a href="README.md">结果说明</a></p>
<h2>预测接触区域</h2><p>彩色点表示整盖邻域内的网络接触预测；分数用于候选生成。</p>
<img src="views/contact_map.png" alt="油箱盖接触预测">
<h2>候选抓姿对比</h2><p>每个候选均标注独立几何检查状态。</p>
<img src="views/candidate_comparison.png" alt="RH56 候选抓姿对照">
<h2>优化过程</h2><p>动画展示数值求解过程中的手姿变化，不是实际执行轨迹。</p>
<img src="geometry_003/optimization.gif" alt="抓姿优化过程动画"></html>''', encoding="utf-8")
    target = base / "implementation_snapshot/tools" / Path(__file__).name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(Path(__file__), target)
    records = {str(p.relative_to(base)): digest(p) for p in sorted(base.rglob("*"))
               if p.is_file() and p != base / "archive.json"}
    (base / "archive.json").write_text(json.dumps({"all_files_verified": True, "files_sha256": records},
                                                 ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"viewer": str(base / "index.html"), "candidate_count": count,
                      "geometry_pass_count": clear, "verified_files": len(records)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
