#!/usr/bin/env python3
"""校验只读采集结果，导出数据质量摘要和便于查看的 CSV，不改写原始记录。"""

import argparse
import csv
import hashlib
import json
import math
import statistics
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def finite(value):
    """将缺失值和非有限值保留为无效，不用零替代。"""
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def rotation(axis, angle):
    """按 URDF 轴角定义构造旋转矩阵，独立于控制器正运动学实现。"""
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def rpy_matrix(angles):
    """固定轴 XYZ 的 RPY，组合矩阵为 Rz(yaw) Ry(pitch) Rx(roll)。"""
    return rotation([0, 0, 1], angles[2]) @ rotation([0, 1, 0], angles[1]) @ rotation([1, 0, 0], angles[0])


def check_model(queries):
    """比较保存的纯数学探针与 URDF，不把模型一致性解释为实物精度。"""
    fk_record = queries.get("RobotAlgorithm.forwardToolKinematics_model_probes", {})
    basis_record = queries.get("Math.poseTrans_flange_basis_computation", {})
    if not fk_record.get("ok") or not basis_record.get("ok"):
        return {"available": False, "physical_calibration_verified": False}
    basis_probes = basis_record["value"]
    pose = np.array(basis_probes[0]["flange_pose_raw"], dtype=float)
    basis = np.column_stack([(np.array(probe["transformed_pose_raw"])[:3] - pose[:3]) / 0.01 for probe in basis_probes])
    rpy_error = float(np.max(np.abs(basis - rpy_matrix(pose[3:]))))
    angle = np.linalg.norm(pose[3:])
    rotvec_matrix = np.eye(3) if angle == 0 else rotation(pose[3:] / angle, angle)
    rotvec_error = float(np.max(np.abs(basis - rotvec_matrix)))
    result = {
        "available": True,
        "physical_calibration_verified": False,
        "sdk_basis_rotation_matrix": basis.tolist(),
        "basis_vs_RPY_RzRyRx_max_abs": rpy_error,
        "basis_vs_rotation_vector_max_abs": rotvec_error,
        "pose_convention_supported_by_probes": "RPY_fixed_XYZ_RzRyRx" if rpy_error < 1e-8 else "unresolved",
    }
    if rpy_error >= 1e-8:
        return result
    project = Path(__file__).resolve().parents[2]
    urdf = project / "assets/robots/aubo_i10/urdf/aubo_i10_JY.urdf"
    tree = ET.parse(urdf).getroot()
    by_child = {joint.find("child").attrib["link"]: joint for joint in tree.findall("joint")}
    chain, child = [], "wrist3_Link"
    while child != "base_link":
        joint = by_child[child]
        chain.append(joint)
        child = joint.find("parent").attrib["link"]
    chain.reverse()
    errors = []
    for probe in fk_record["value"]:
        if probe["result_code"] != 0:
            continue
        matrix, index = np.eye(4), 0
        for joint in chain:
            origin = joint.find("origin")
            transform = np.eye(4)
            if origin is not None:
                transform[:3, 3] = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
                transform[:3, :3] = rpy_matrix([float(v) for v in origin.attrib.get("rpy", "0 0 0").split()])
            matrix = matrix @ transform
            if joint.attrib["type"] in ("revolute", "continuous"):
                axis = [float(v) for v in joint.find("axis").attrib.get("xyz", "1 0 0").split()]
                transform = np.eye(4)
                transform[:3, :3] = rotation(axis, probe["q_rad"][index])
                matrix = matrix @ transform
                index += 1
        sdk_pose = np.array(probe["flange_pose_raw"])
        errors.append({
            "q_rad": probe["q_rad"],
            "position_error_m": float(np.linalg.norm(matrix[:3, 3] - sdk_pose[:3])),
            "rotation_matrix_max_abs_error": float(np.max(np.abs(matrix[:3, :3] - rpy_matrix(sdk_pose[3:])))),
        })
    result.update({
        "urdf": str(urdf.relative_to(project)),
        "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
        "base_link": "base_link", "flange_link": "wrist3_Link",
        "joint_names_in_order": [joint.attrib["name"] for joint in chain if joint.attrib["type"] in ("revolute", "continuous")],
        "samples": errors,
        "maximum_position_error_m": max(record["position_error_m"] for record in errors) if errors else None,
        "maximum_rotation_matrix_error": max(record["rotation_matrix_max_abs_error"] for record in errors) if errors else None,
    })
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    directory = args.directory
    metadata = json.loads((directory / "metadata.json").read_text())
    queries = json.loads((directory / "configuration.json").read_text())
    rows = [json.loads(line) for line in (directory / "rtde_samples.jsonl").read_text().splitlines() if line]
    good = [row for row in rows if row.get("parse_ok")]
    fields = metadata.get("rtde", {}).get("selected_fields", [])
    vectors = [name for name in fields if any(isinstance(row.get(name), list) for row in good)]
    summary = {
        "schema_version": 1,
        "rpc_queries": len(queries),
        "rpc_successes": sum(record.get("ok") is True for record in queries.values()),
        "rpc_failed_methods": [name for name, record in queries.items() if not record.get("ok")],
        "rpc_empty_results": [name for name, record in queries.items() if record.get("ok") and record.get("value") in ([], {}, None, "")],
        "rtde_rows": len(rows),
        "rtde_parse_ok": len(good),
        "rtde_parse_errors": len(rows) - len(good),
        "requested_hz": metadata.get("requested_rtde_hz"),
        "vector_fields": {},
    }
    times = [row["host_monotonic_s"] for row in good if finite(row.get("host_monotonic_s"))]
    if len(times) > 1:
        intervals = [b - a for a, b in zip(times, times[1:])]
        positive = [value for value in intervals if value > 0]
        span = times[-1] - times[0]
        summary["host_timing"] = {
            "span_s": span,
            "mean_hz": (len(times) - 1) / span if span > 0 else None,
            "nonpositive_intervals": len(intervals) - len(positive),
            "minimum_interval_s": min(intervals),
            "median_interval_s": statistics.median(intervals),
            "maximum_interval_s": max(intervals),
        }
    controller_times = [row["timestamp"] for row in good if finite(row.get("timestamp"))]
    summary["controller_timestamp"] = {
        "finite_rows": len(controller_times),
        "missing_or_invalid_rows": len(good) - len(controller_times),
        "usable_for_clock_alignment": len(controller_times) == len(good) and len(good) > 1 and all(b > a for a, b in zip(controller_times, controller_times[1:])),
    }
    for name in vectors:
        values = [row.get(name) for row in good]
        valid = [value for value in values if isinstance(value, list) and len(value) == 6 and all(finite(x) for x in value)]
        entry = {
            "valid_six_value_rows": len(valid),
            "empty_rows": sum(value == [] for value in values),
            "missing_or_malformed_rows": len(values) - len(valid) - sum(value == [] for value in values),
        }
        if valid:
            columns = list(zip(*valid))
            entry.update({
                "first": valid[0], "last": valid[-1],
                "minimum": [min(col) for col in columns],
                "maximum": [max(col) for col in columns],
                "span": [max(col) - min(col) for col in columns],
                "mean": [statistics.fmean(col) for col in columns],
                "population_std": [statistics.pstdev(col) for col in columns],
            })
        summary["vector_fields"][name] = entry
    q = summary["vector_fields"].get("R1_actual_q", {})
    if q.get("first"):
        summary["first_joint_positions_deg"] = [math.degrees(v) for v in q["first"]]
    summary["interpretation"] = {
        "pose": "原始姿态保留控制器原值；数学探针的编码核对见 model_check",
        "force": "保存控制器读数，不据此认定接触力已去除重力、负载与零偏",
        "kinematics_probes": "正运动学探针是控制器模型计算，不是多个实物姿态观测",
    }
    summary["model_check"] = check_model(queries)
    (directory / "analysis.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    header = ["host_received_utc", "host_monotonic_s", "timestamp", "parse_ok"]
    for name in vectors:
        header.extend(f"{name}_{axis + 1}" for axis in range(6))
    with (directory / "rtde_samples.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for row in rows:
            result = [row.get(name) for name in header[:4]]
            for name in vectors:
                vector = row.get(name)
                result.extend(vector if isinstance(vector, list) and len(vector) == 6 else [None] * 6)
            writer.writerow(result)
    print(json.dumps({key: summary[key] for key in ["rpc_queries", "rpc_successes", "rpc_failed_methods", "rtde_rows", "rtde_parse_errors", "host_timing", "controller_timestamp"] if key in summary}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
