#!/usr/bin/env python3
"""按固定相机系三维框排除油枪点，保留原始文件、字段、点序及逐点来源。"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from fuel_cap_task.pcd_point_cloud import load_pcd

DEFAULT_CONFIG = ROOT / "config/vizum_point_cloud_filter.json"


def digest(path):
    """绑定实际读取的文件内容。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def exclusion_mask(points, config):
    """输入为相机 SDK 原生毫米坐标，三个闭区间须同时满足才删除。

    非有限点不参与盒内判定，保留在输出中并单独统计，避免隐式改变原始数据。
    """
    if (config.get("schema_version") != 1 or config.get("mode") != "exclude_axis_aligned_box"
            or config.get("frame_id") != "camera_sdk_native" or config.get("length_unit") != "mm"
            or config.get("interval") != "closed" or config.get("combine_axes") != "and"
            or config.get("preserve_raw") is not True or type(config.get("enabled")) is not bool):
        raise ValueError("过滤配置须声明相机 SDK 原生毫米坐标、闭区间交集和原始数据保留")
    lower = np.asarray(config["bounds_min_xyz"], dtype=float)
    upper = np.asarray(config["bounds_max_xyz"], dtype=float)
    if (lower.shape != (3,) or upper.shape != (3,) or not np.isfinite(np.r_[lower, upper]).all()
            or not np.all(lower < upper)):
        raise ValueError("三维框上下界须为有限三维向量，且每轴下界小于上界")
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("点云坐标须为 N×3 数组")
    return (np.isfinite(points).all(axis=1) & np.all((points >= lower) & (points <= upper), axis=1)
            if config["enabled"] else np.zeros(len(points), dtype=bool))


def write_records(path, cloud, indices):
    """按源结构体字节布局保存选中点，所有字段及其类型保持一致。"""
    if not len(indices):
        return None
    header = cloud.header
    fields = [("VERSION", header["VERSION"]), ("FIELDS", header["FIELDS"]),
              ("SIZE", header["SIZE"]), ("TYPE", header["TYPE"]),
              ("COUNT", header.get("COUNT", ["1"] * len(header["FIELDS"]))),
              ("WIDTH", [str(len(indices))]), ("HEIGHT", ["1"]),
              ("VIEWPOINT", header.get("VIEWPOINT", ["0", "0", "0", "1", "0", "0", "0"])),
              ("POINTS", [str(len(indices))]), ("DATA", ["binary"])]
    with path.open("xb") as stream:
        stream.write(("# Original point records; camera SDK native coordinates in mm\n" +
                      "\n".join(key + " " + " ".join(values) for key, values in fields) + "\n").encode("ascii"))
        stream.write(cloud.records[indices].tobytes())
    return path.name


def filter_cloud(source, destination, config_path=DEFAULT_CONFIG):
    """只在新目录保存过滤结果，不切换当前规划场景。"""
    source, destination, config_path = Path(source).resolve(), Path(destination).resolve(), Path(config_path).resolve()
    config = json.loads(config_path.read_text())
    cloud = load_pcd(source)
    removed_mask = exclusion_mask(cloud.xyz_raw, config)
    kept = np.flatnonzero(~removed_mask)
    removed = np.flatnonzero(removed_mask)
    if not len(kept):
        raise ValueError("过滤后没有保留点，请检查坐标、单位和过滤范围")
    destination.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(source, destination / "cloud_raw.pcd")
    shutil.copyfile(config_path, destination / "filter_config.json")
    if digest(destination / "cloud_raw.pcd") != cloud.sha256:
        raise ValueError("复制期间原始点云发生变化")
    write_records(destination / "cloud_filtered.pcd", cloud, kept)
    removed_path = write_records(destination / "cloud_removed.pcd", cloud, removed)
    np.savez_compressed(destination / "point_indices.npz", kept_source_indices=kept, removed_source_indices=removed)
    report = {"schema_version": 1, "mode": config["mode"],
              "created_utc": datetime.now(timezone.utc).isoformat(),
              "source_path": str(source), "source_sha256": cloud.sha256,
              "config_path": str(config_path), "config_sha256": digest(config_path),
              "frame_id": config["frame_id"], "length_unit": config["length_unit"],
              "byte_order": cloud.byte_order, "input_point_count": len(cloud.xyz_raw),
              "kept_point_count": len(kept), "removed_point_count": len(removed),
              "nonfinite_points_retained": int((~np.isfinite(cloud.xyz_raw).all(axis=1)).sum()),
              "removed_pcd": removed_path, "coordinate_transform_applied": False,
              "outputs_sha256": {p.name: digest(p) for p in destination.iterdir() if p.is_file()},
              "implementation_sha256": {str(p.relative_to(ROOT)): digest(p) for p in
                  [Path(__file__).resolve(), ROOT / "src/fuel_cap_task/pcd_point_cloud.py"]}}
    (destination / "filter_result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    return report


def main():
    """每份观测另存完整原始点和盒内、盒外子集。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    result = filter_cloud(args.pcd, args.output_dir, args.config)
    print(json.dumps({k: result[k] for k in ["input_point_count", "kept_point_count", "removed_point_count"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
