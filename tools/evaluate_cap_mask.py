#!/usr/bin/env python3
"""推理完成后独立计算可见盖体 IoU；真值不能进入检测和分割。"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prediction-dir", type=Path, required=True)
    parser.add_argument("--evaluation-only", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prediction = np.load(args.prediction_dir / "candidates.npz", allow_pickle=False)
    truth = np.load(args.evaluation_only, allow_pickle=False)["target_mask"]
    if truth.dtype != np.bool_ or truth.ndim != 2 or not truth.any():
        raise ValueError("visible target ground truth must be nonempty boolean H x W")
    evaluations = []
    for index, mask in enumerate(prediction["masks"]):
        if mask.shape != truth.shape or mask.dtype != np.bool_:
            raise ValueError("mask dimensions or type do not match")
        intersection = int((mask & truth).sum())
        union = int((mask | truth).sum())
        evaluations.append({"index": index, "iou": intersection / union,
                            "precision": intersection / mask.sum() if mask.any() else 0.,
                            "recall": intersection / truth.sum()})
    report = {"evaluation_source": "simulation_visible_geometry_only",
              "prediction_result_sha256": hashlib.sha256((args.prediction_dir / "result.json").read_bytes()).hexdigest(),
              "evaluation_only_sha256": hashlib.sha256(args.evaluation_only.read_bytes()).hexdigest(),
              "evaluation_code_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "visible_target_pixels": int(truth.sum()), "candidates": evaluations,
              "acceptance_threshold": None,
              "limitations": "仅此帧可见几何评价；不自动改写推理结果或标为语义/泛化通过"}
    with args.output.open("x") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
        f.write("\n")
    print(json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    main()
