#!/usr/bin/env python3
"""在原示例和油箱盖体积上对照网络模式；不训练或保存网络权重。"""

import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "dependencies/SpaHybGen/src"))
os.environ.setdefault("MPLCONFIGDIR", "/tmp/cap-spahybgen-matplotlib")

import numpy as np
import torch
from spahybgen.networks import load_network
from spahybgen.inference import predict
from fuel_cap_task.rgbd_point_cloud import digest


def main():
    """每次前向重新加载同一权重，避免 BatchNorm 状态从另一模式泄漏。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("输出已存在，请指定新文件")
    with np.load(args.scene_input, allow_pickle=False) as data:
        cap, region = data["grid"][None], data["contact_region"]
    observation = ROOT / "dependencies/SpaHybGen/assets/observations/scene_010_ann_0124_voxel.npz"
    with np.load(observation, allow_pickle=False) as data:
        original = data["grid"]
    if original.ndim == 3:
        original = original[None]
    torch.set_num_threads(4)
    checkpoint = ROOT / "dependencies/SpaHybGen/assets/trained_models/spahybgen_unet_64_voxel.pt"
    before = digest(checkpoint)
    comparisons = []
    for name, grid in (("original", original), ("cap", cap)):
        for mode in ("eval", "upstream_train"):
            torch.manual_seed(20261008)
            network = load_network(checkpoint, torch.device("cpu"),
                                   {"voxel_discreteness": 80, "orientation": "quat", "augment": False})
            network.train(mode == "upstream_train")
            quality, rotation, center = predict(grid, network, torch.device("cpu"))
            row = {"input": name, "mode": mode, "occupied_count": int(grid.sum()),
                   "quality_peak": float(quality.max()), "center_peak": float(center.max()),
                   "quality_quantiles": np.quantile(quality, [.5, .9, .99]).tolist(),
                   "all_outputs_finite": bool(np.isfinite(quality).all() and np.isfinite(rotation).all()
                                              and np.isfinite(center).all())}
            if name == "cap":
                row["target_quality_peak"] = float(quality[region].max())
            comparisons.append(row)
    if digest(checkpoint) != before:
        raise ValueError("前向模式对照意外改变了权重文件")
    report = {"checkpoint_sha256": before, "network_weights_optimized": False,
              "original_observation_sha256": digest(observation),
              "cap_scene_input_sha256": digest(args.scene_input),
              "implementation_sha256": digest(Path(__file__)), "comparisons": comparisons}
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(comparisons, ensure_ascii=False))


if __name__ == "__main__":
    main()
