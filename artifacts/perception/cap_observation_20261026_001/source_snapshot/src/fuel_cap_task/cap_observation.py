"""绑定静止观测与盖体几何，保存基座系诊断候选；不提供机器人执行接口。"""

from datetime import datetime, timezone
import hashlib
import json
import re
from pathlib import Path

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .pcd_point_cloud import load_pcd


def digest(path):
    """用文件内容绑定观测、外参和裁剪，不以文件名或修改时间代替。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def checked_transform(value, name):
    """验证米制刚体变换；拒绝缩放、镜像和非齐次矩阵。"""
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{name} 必须是有限 4×4 矩阵")
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-8, rtol=0)):
        raise ValueError(f"{name} 不是右手刚体变换，不能隐式应用缩放或镜像")
    return matrix


def extract_connected_crop(points_raw, *, length_unit, bounds_mm, seeds_mm, radius_mm):
    """在显式毫米 ROI 内按三维连通分量选点，返回原始行号及选区诊断。

    输入保留文件原单位；仅为邻域查询换成毫米，不给输出点做几何变换。
    ROI 与种子由操作者或 agent 查看当前观测后指定，不作为自动盖体检测器。
    """
    points = np.asarray(points_raw, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or length_unit not in ("mm", "m"):
        raise ValueError("需要 (N,3) 点与显式 mm/m 单位")
    bounds = np.asarray(bounds_mm, dtype=float).reshape(3, 2)
    seeds = np.asarray(seeds_mm, dtype=float)
    radius = float(radius_mm)
    if (not np.isfinite(bounds).all() or np.any(bounds[:, 0] >= bounds[:, 1])
            or seeds.ndim != 2 or seeds.shape[1] != 3 or not len(seeds)
            or not np.isfinite(seeds).all() or not np.isfinite(radius) or radius <= 0):
        raise ValueError("ROI、种子或连通距离无效")
    if np.any(seeds <= bounds[:, 0]) or np.any(seeds >= bounds[:, 1]):
        raise ValueError("种子必须位于 ROI 内")
    xyz_mm = points * (1000 if length_unit == "m" else 1)
    keep = np.isfinite(xyz_mm).all(axis=1)
    keep &= np.all((xyz_mm > bounds[:, 0]) & (xyz_mm < bounds[:, 1]), axis=1)
    source_indices = np.flatnonzero(keep)
    roi = xyz_mm[keep]
    if not len(roi):
        raise ValueError("ROI 中没有有效点")
    tree = cKDTree(roi)
    pairs = tree.query_pairs(radius, output_type="ndarray")
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])),
                       shape=(len(roi), len(roi)))
    component_count, labels = connected_components(graph, directed=False)
    distances, nearest = tree.query(seeds)
    if np.any(distances > radius):
        raise ValueError("种子离观测点过远，应从当前点云选取种子")
    selected_labels = np.unique(labels[nearest])
    selected = source_indices[np.isin(labels, selected_labels)]
    crop_mm = xyz_mm[selected]
    margins = np.minimum(crop_mm - bounds[:, 0], bounds[:, 1] - crop_mm)
    minimum_margin = float(margins.min())
    if minimum_margin <= radius:
        raise ValueError("所选连通区域贴近 ROI 截断边界，应扩大 ROI 后检查")
    return selected, {
        "method": "agent_or_operator_seeded_3d_connected_components",
        "bounds_mm": bounds.ravel().tolist(), "seeds_mm": seeds.tolist(),
        "connection_radius_mm": radius, "roi_point_count": len(roi),
        "roi_component_count": component_count,
        "selected_component_counts": [int(np.count_nonzero(labels == i)) for i in selected_labels],
        "selected_point_count": len(selected),
        "seed_snap_distances_mm": distances.tolist(),
        "seed_source_indices": source_indices[nearest].tolist(),
        "minimum_selected_margin_to_roi_mm": minimum_margin,
        "circle_or_radius_used_to_cut_points": False,
    }


def _utc(value):
    """解析含时区的主机时间，不把它解释成曝光硬件时间。"""
    # Python 3.10 的 fromisoformat 对小数位数有限制；统一到微秒并兼容 UTC 的 Z 写法。
    value = value.replace("Z", "+00:00")
    value = re.sub(r"\.(\d+)(?=[+-]\d{2}:\d{2}$)",
                   lambda match: "." + match.group(1)[:6].ljust(6, "0"), value)
    parsed = datetime.fromisoformat(value)
    if parsed.tzinfo is None:
        raise ValueError("观测主机时间必须包含时区")
    return parsed.astimezone(timezone.utc)


def bind_observation(report, points_camera_m, observation_path, *, calibration_path=None,
                     crop_manifest_path=None):
    """核对数据绑定后计算 base_from_camera 与两个基座系模型参考位姿。

    仅在“SDK 未翻转 XYZ 与已有外参相机轴相同”的显式假设下换算。
    物理轴约定、多视角一致性和工具安装仍需复核，结果保持 motion_target_valid=false。
    """
    path = Path(observation_path).resolve()
    observation = json.loads(path.read_text(encoding="utf-8"))
    if not observation.get("capture_complete"):
        raise ValueError("观测没有完成采集")
    source_record = observation["point_cloud"]
    if source_record.get("mirror") != "none" or source_record.get("frame") != "camera_sdk_native":
        raise ValueError("当前适配器只接受已声明未翻转的 SDK 原生点云")
    raw_path = path.parent / source_record["path"]
    if digest(raw_path) != source_record["sha256"]:
        raise ValueError("原始点云摘要与观测记录不一致")
    raw = load_pcd(raw_path)
    # 原始 SDK 数值按项目确认的毫米约定解释；独立实物测距验证另行记录。
    source_m = raw.xyz_raw * .001
    finite_source = source_m[np.isfinite(source_m).all(axis=1)]
    points = np.asarray(points_camera_m, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError("裁剪必须包含有限三维点")
    if not len(finite_source):
        raise ValueError("源点云没有有限点")
    distances, _ = cKDTree(finite_source).query(points)
    # 这是文件导出舍入容差 0.0001 mm，不是相机精度或机器人到位容差。
    if float(distances.max()) > 1e-7:
        raise ValueError("裁剪点不是该次采集的原始子集，不能绑定其他观测或变换后的点")
    crop_record = None
    if crop_manifest_path is not None:
        crop_record = json.loads(Path(crop_manifest_path).read_text(encoding="utf-8"))
        if (crop_record["source_sha256"] != source_record["sha256"]
                or crop_record["crop_sha256"] != report["input_sha256"]):
            raise ValueError("裁剪清单与输入文件或源点云摘要不匹配")

    calibration_path = Path(calibration_path) if calibration_path else path.parent / "fuel_cap_camera_calibration.json"
    if digest(calibration_path) != observation["calibration_sha256"]:
        raise ValueError("外参不是采集时保存的版本")
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    extrinsic = calibration["extrinsic"]
    if extrinsic["name"] != "flange_from_camera" or extrinsic["matrix_translation_unit"] != "m":
        raise ValueError("外参必须显式为相机到法兰且平移单位为米")
    flange_from_camera = checked_transform(extrinsic["flange_from_camera_matrix"], "flange_from_camera")
    transforms = []
    for key in ("state_before", "state_after"):
        state = observation[key]
        if state.get("steady") is not True or state.get("simulation") is not False:
            raise ValueError("前后机器人状态必须报告真机且静止")
        q = np.asarray(state["q_rad"], dtype=float)
        pose = np.asarray(state["flange_pose_m_rpy_rad"], dtype=float)
        if q.shape != (6,) or pose.shape != (6,) or not np.isfinite(q).all() or not np.isfinite(pose).all():
            raise ValueError("机器人状态必须包含六个有限关节角和六维法兰位姿")
        matrix = checked_transform(state["base_from_flange"], key + ".base_from_flange")
        if (not np.allclose(matrix[:3, 3], pose[:3], atol=1e-10, rtol=0)
                or not np.allclose(matrix[:3, :3], Rotation.from_euler("xyz", pose[3:]).as_matrix(), atol=1e-10, rtol=0)):
            raise ValueError("保存的法兰矩阵与米制固定轴 XYZ RPY 不一致")
        transforms.append(matrix)
    before_end = _utc(observation["robot_before"]["end_utc"])
    camera_start = _utc(observation["camera_cloud"]["start_utc"])
    camera_end = _utc(observation["camera_cloud"]["end_utc"])
    after_start = _utc(observation["robot_after"]["start_utc"])
    if not before_end <= camera_start <= camera_end <= after_start:
        raise ValueError("状态快照没有在时间上包围点云采集调用")
    base_from_camera = transforms[0] @ flange_from_camera
    base_from_camera_after = transforms[1] @ flange_from_camera
    candidates = []
    for camera_pose, orientation in zip(report["pose_candidates_camera_model_based"], report["orientation_candidates"]):
        camera_from_cap = np.eye(4)
        camera_from_cap[:3, :3] = orientation["rotation_columns_xyz"]
        camera_from_cap[:3, 3] = camera_pose["position_m"]
        base_from_cap = base_from_camera @ checked_transform(camera_from_cap, "camera_from_cap")
        candidates.append({"frame_id": "base", "reference_frame": "cap_grip_center",
            "position_m": base_from_cap[:3, 3].tolist(),
            "orientation_xyzw": Rotation.from_matrix(base_from_cap[:3, :3]).as_quat().tolist(),
            "base_from_cap_grip_center": base_from_cap.tolist(),
            "ambiguity_yaw_deg": camera_pose["ambiguity_yaw_deg"]})
    def base_point(point):
        return (base_from_camera[:3, :3] @ np.asarray(point) + base_from_camera[:3, 3]).tolist()
    result = dict(report)
    result.update({"motion_target_valid": False, "robot_state_paired": True,
        "source": crop_record["method"] if crop_record else "crop_verified_as_observation_subset",
        "pose_candidates_base_model_based": candidates,
        "boundary_center_base_m": base_point(report["boundary_center_m"]),
        "visible_bridge_surface_reference_base_m": base_point(report["visible_bridge_surface_reference_m"]),
        "outward_axis_base": (base_from_camera[:3, :3] @ report["outward_axis_camera"]).tolist(),
        "bridge_axis_base_undirected": (base_from_camera[:3, :3] @ report["bridge_axis_camera_undirected"]).tolist(),
        "observation_binding": {
            "observation_path": str(path), "observation_sha256": digest(path),
            "point_cloud_sha256": source_record["sha256"], "calibration_sha256": digest(calibration_path),
            "crop_to_source_max_nearest_error_m": float(distances.max()),
            "robot_before": observation["state_before"], "robot_after": observation["state_after"],
            "base_from_camera": base_from_camera.tolist(), "base_from_camera_after": base_from_camera_after.tolist(),
            "host_acquisition_window_utc": [camera_start.isoformat(), camera_end.isoformat()],
            "time_meaning": observation["time_meaning"],
            "camera_axis_assumption": "SDK native unflipped XYZ equals calibration camera axes",
            "camera_axis_assumption_verified_against_legacy_export": False,
            "crop_manifest": crop_record,
        }})
    result["unresolved"] = [s for s in report["unresolved"] if "缺少本次采集的机器人姿态与采集时间" not in s]
    result["unresolved"] += ["SDK未翻转XYZ与旧导出PCD/外参相机轴约定仍需独立核对",
                              "主机前后RPC快照不等于曝光硬件时间或连续静止监测",
                              "外参实物精度、工具安装及多视角一致性仍待核对"]
    if crop_record is not None:
        result["assumptions"] = ["ROI和三维种子已在当前观测中选取，整盖选区仍需结合预览复核"] + report.get("assumptions", [])[1:]
    result["implementation_sha256"] = {**report["implementation_sha256"], "cap_observation.py": digest(__file__)}
    return result
