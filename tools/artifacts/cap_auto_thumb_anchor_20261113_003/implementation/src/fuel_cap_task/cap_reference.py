"""区分握梁顶面与内部中心，显式换算旧抓姿和历史观测。"""

from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

TOP_FRAME = "cap_grip_top_center"
CENTER_FRAME = "cap_grip_center"
DEFAULT_CAP_URDF = Path(__file__).resolve().parents[2] / "assets/objects/fuel_cap/urdf/fuel_cap.urdf"


def cap_reference_transform(target_frame, source_frame, *, urdf_path=DEFAULT_CAP_URDF):
    """返回模型中的 target_from_source，平移为米；仅用于已有抓姿的参考迁移。"""
    if target_frame not in (TOP_FRAME, CENTER_FRAME) or source_frame not in (TOP_FRAME, CENTER_FRAME):
        raise ValueError("握梁参考帧必须显式为顶面或内部中心")
    if target_frame == source_frame:
        return np.eye(4)
    root = ET.parse(urdf_path).getroot()

    def body_from(frame):
        joint = root.find(f"./joint[@name='{frame}_joint']")
        if joint is None or joint.find("parent").get("link") != "cap_body":
            raise ValueError("握梁参考帧须直接定义在 cap_body 中")
        origin = joint.find("origin")
        position = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        if position.shape != (3,) or rpy.shape != (3,) or not np.isfinite(np.r_[position, rpy]).all():
            raise ValueError("模型参考变换必须使用有限三维位置与 RPY")
        value = np.eye(4)
        value[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        value[:3, 3] = position
        return value

    return np.linalg.inv(body_from(target_frame)) @ body_from(source_frame)


def reexpress_hand_pose(pose, source_frame, target_frame, *, urdf_path=DEFAULT_CAP_URDF):
    """保留手的实际位置与朝向，将相对抓姿改用另一模型参考帧表达。"""
    position = np.asarray(pose["position_m"], dtype=float)
    quaternion = np.asarray(pose["orientation_xyzw"], dtype=float)
    if (position.shape != (3,) or quaternion.shape != (4,)
            or not np.isfinite(np.r_[position, quaternion]).all()
            or not np.isclose(np.linalg.norm(quaternion), 1., atol=1e-6, rtol=0)):
        raise ValueError("抓姿须包含米制三维位置和 XYZW 单位四元数")
    source_from_hand = np.eye(4)
    source_from_hand[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    source_from_hand[:3, 3] = position
    model_path = Path(urdf_path).resolve()
    model_digest = hashlib.sha256(model_path.read_bytes()).hexdigest()
    target_from_source = cap_reference_transform(target_frame, source_frame, urdf_path=model_path)
    if hashlib.sha256(model_path.read_bytes()).hexdigest() != model_digest:
        raise ValueError("参考模型在抓姿换算期间发生变化")
    target_from_hand = target_from_source @ source_from_hand
    return {**pose, "frame_id": target_frame,
            "position_m": target_from_hand[:3, 3].tolist(),
            "orientation_xyzw": Rotation.from_matrix(target_from_hand[:3, :3]).as_quat().tolist(),
            "reference_conversion": {"source_frame": source_frame, "target_frame": target_frame,
                "reference_urdf_path": str(model_path), "reference_urdf_sha256": model_digest,
                "target_from_source_matrix": target_from_source.tolist()}}


def cap_pose_in_reference(parent_from_source, source_frame, target_frame):
    """保持父系和实物位置不变，将被定位的参考点换为另一模型帧。"""
    matrix = np.asarray(parent_from_source, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-8, rtol=0)):
        raise ValueError("参考位姿必须是右手刚体 4×4 变换")
    return matrix @ cap_reference_transform(source_frame, target_frame)


def camera_surface_pose_candidates(report):
    """读取显式顶面候选；旧报告只使用已保存的观测表面，不推测缺失高度。"""
    current = "pose_candidates_camera" in report
    if current:
        if report.get("reference_frame") != TOP_FRAME:
            raise ValueError("新几何结果须声明 cap_grip_top_center")
        poses = report["pose_candidates_camera"]
    else:
        if report.get("reference_frame", CENTER_FRAME) != CENTER_FRAME:
            raise ValueError("缺少与声明参考帧一致的相机系候选")
        poses = report.get("pose_candidates_camera_model_based", [])
        surface = np.asarray(report.get("visible_bridge_surface_reference_m"), dtype=float)
        if surface.shape != (3,) or not np.isfinite(surface).all():
            raise ValueError("旧几何结果缺少已观测的握梁表面参考点，不能换算")
    if not isinstance(poses, list) or not poses:
        raise ValueError("几何结果缺少相机系姿态候选")
    result = []
    for index, pose in enumerate(poses):
        expected = TOP_FRAME if current else CENTER_FRAME
        if pose.get("reference_frame", expected if not current else None) != expected:
            raise ValueError("相机系候选参考帧与结果声明不一致")
        if pose.get("frame_id", "camera" if not current else None) != "camera":
            raise ValueError("所选姿态必须在 camera 中表达")
        position = np.asarray(pose["position_m"] if current else surface, dtype=float)
        quaternion = pose.get("orientation_xyzw")
        if quaternion is None and not current:
            quaternion = Rotation.from_matrix(report["orientation_candidates"][index]["rotation_columns_xyz"]).as_quat()
        quaternion = np.asarray(quaternion, dtype=float)
        if (position.shape != (3,) or quaternion.shape != (4,)
                or not np.isfinite(np.r_[position, quaternion]).all()
                or not np.isclose(np.linalg.norm(quaternion), 1., rtol=0, atol=1e-6)):
            raise ValueError("顶面候选必须包含有限位置与 XYZW 单位四元数")
        result.append({**pose, "frame_id": "camera", "reference_frame": TOP_FRAME,
                       "position_m": position.tolist(), "orientation_xyzw": quaternion.tolist()})
    return result


def candidate_reference_frames(report):
    """校验整份抓姿候选的版本与参考帧；仅旧版本允许省略内部中心声明。"""
    version = report.get("schema_version", 1)
    if type(version) is not int or version not in (1, 2):
        raise ValueError("抓姿候选版本必须是 1 或 2")
    expected = CENTER_FRAME if version == 1 else TOP_FRAME
    default = CENTER_FRAME if version == 1 else None
    if report.get("reference_frame", default) != expected:
        raise ValueError("抓姿候选版本与顶层参考帧不一致")
    candidates = report.get("candidates")
    if not isinstance(candidates, list) or not candidates:
        raise ValueError("需要非空抓姿候选列表")
    frames = []
    for candidate in candidates:
        if not isinstance(candidate, dict) or candidate.get("reference_frame", default) != expected:
            raise ValueError("每个抓姿候选的参考帧须与版本和顶层声明一致")
        frames.append(expected)
    return frames


def world_cap_candidates(report, world_from_camera):
    """转换整份候选到世界系，并保留内部中心或顶面的明确含义。"""
    frames = candidate_reference_frames(report)
    return [{"candidate_id": candidate["candidate_id"], "reference_frame": frame,
             "world_from_cap": (np.asarray(world_from_camera) @ cap_pose_in_reference(
                 candidate["camera_from_cap"], frame, frame)).tolist()}
            for candidate, frame in zip(report["candidates"], frames)]
