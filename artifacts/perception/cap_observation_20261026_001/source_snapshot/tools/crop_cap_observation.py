#!/usr/bin/env python3
"""用显式 ROI 和三维种子辅助裁剪当前观测，保留原始坐标和源点行号。"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from fuel_cap_task.cap_observation import extract_connected_crop
from fuel_cap_task.pcd_point_cloud import load_pcd


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", type=Path, required=True)
    parser.add_argument("--length-unit", choices=("mm", "m"), required=True)
    parser.add_argument("--bounds-mm", type=float, nargs=6, required=True)
    parser.add_argument("--seed-mm", type=float, nargs=3, action="append", required=True)
    parser.add_argument("--connection-radius-mm", type=float, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists():
        parser.error("输出目录已存在，请使用新目录保留既有结果")
    cloud = load_pcd(args.pcd)
    indices, diagnostics = extract_connected_crop(cloud.xyz_raw, length_unit=args.length_unit,
        bounds_mm=args.bounds_mm, seeds_mm=args.seed_mm, radius_mm=args.connection_radius_mm)
    points = cloud.xyz_raw[indices]
    output.mkdir(parents=True)
    # 用 float64 导出 load_pcd 的原值；不因重写文件产生 float32 舍入，也不改变点序。
    header = (f"# length_unit {args.length_unit}; frame camera_sdk_native; source subset only\n"
              "VERSION .7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\n"
              f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n")
    path = output / "cap_crop.pcd"
    path.write_bytes(header.encode("ascii") + np.asarray(points, dtype="<f8").tobytes())
    np.savez_compressed(output / "source_indices.npz", source_indices=indices)
    report = {**diagnostics, "schema_version": 1, "source_path": str(args.pcd.resolve()),
        "source_sha256": cloud.sha256, "source_point_count": len(cloud.xyz_raw),
        "length_unit": args.length_unit, "frame_id": "camera_sdk_native",
        "crop_path": path.name, "crop_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "source_indices_path": "source_indices.npz", "visual_review_required": True,
        "interpretation": "基于当前场景选取的ROI与种子，非通用自动盖体检测；没有添加拟合点或用圆半径截断观测"}
    (output / "crop.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
