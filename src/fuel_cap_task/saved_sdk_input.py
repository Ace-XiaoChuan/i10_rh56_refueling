"""将保存的 SDK 点云适配为理想针孔深度和一毫米表面格；兼容优化环境。"""

import hashlib
import json
from pathlib import Path

import numpy as np

from .pcd_point_cloud import load_pcd
from .rgbd_point_cloud import transform_points


def digest_file(path):
    """分块计算文件摘要，避免大点云额外驻留内存。"""
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def save_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def source_indices(value, count):
    """只接受原点云范围内的不重复整数点号，拒绝截断或隐式转型。"""
    indices = np.asarray(value)
    if (indices.ndim != 1 or indices.dtype.kind not in "iu" or not len(indices)
            or np.any(indices < 0) or np.any(indices >= count) or len(np.unique(indices)) != len(indices)):
        raise ValueError("原始点号无效、重复或越界")
    return indices.astype(np.int64)


def project_sdk_depth(xyz_mm, kept_indices, target_indices, camera_matrix, rgb_from_native_m, width, height):
    """毫米 XYZ 转米并投影；Z-buffer 保留最近原点号，不补洞或借用遮挡点标签。"""
    xyz = np.asarray(xyz_mm, dtype=float)
    kept = source_indices(kept_indices, len(xyz))
    target = source_indices(target_indices, len(xyz))
    k = np.asarray(camera_matrix, dtype=float)
    if (k.shape != (3, 3) or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0
            or width <= 0 or height <= 0):
        raise ValueError("针孔内参或图像尺寸无效")
    points = transform_points(xyz[kept] * .001, rgb_from_native_m)
    finite = np.isfinite(points).all(axis=1) & (points[:, 2] > 0)
    points, kept = points[finite], kept[finite]
    uv = np.rint(points[:, :2] / points[:, 2, None] * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]])
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < width) & (uv[:, 1] >= 0) & (uv[:, 1] < height)
    uv, points, kept = uv[inside].astype(np.int64), points[inside], kept[inside]
    linear = uv[:, 1] * width + uv[:, 0]
    # 每个像素先按深度排序；同深度以原点号保证结果可复现。
    order = np.lexsort((kept, points[:, 2], linear))
    _, first = np.unique(linear[order], return_index=True)
    selected = order[first]
    source = np.full(height * width, -1, dtype=np.int64)
    depth = np.zeros(height * width, dtype=np.float32)
    source[linear[selected]], depth[linear[selected]] = kept[selected], points[selected, 2]
    valid = source >= 0
    target_mask = valid & np.isin(source, target)
    return tuple(a.reshape(height, width) for a in (depth, valid, target_mask, source))


def sparse_surface_voxels(points_volume_m, target_point_mask, size_m=.4, voxel_size_m=.001):
    """保存原始实测表面所在的一毫米格和点数；缺失格不等于空闲。"""
    if not np.isclose(voxel_size_m, .001, rtol=0, atol=1e-12):
        raise ValueError("当前几何体素须为 0.001 m")
    points = np.asarray(points_volume_m)
    selected = np.asarray(target_point_mask, dtype=bool)
    inside = np.isfinite(points).all(axis=1) & np.all((points >= 0) & (points < size_m), axis=1)
    indices = np.floor(points[inside] / voxel_size_m).astype(np.int32)
    unique, inverse, counts = np.unique(indices, axis=0, return_inverse=True, return_counts=True)
    target_count = np.bincount(inverse, weights=selected[inside].astype(int), minlength=len(unique)).astype(int)
    return unique, counts, target_count


def load_sdk_snapshot(manifest):
    """校验文件摘要、子集关系和配对状态，返回生成器需要的实测输入。"""
    for path, expected in manifest["inputs_sha256"].items():
        if digest_file(path) != expected:
            raise ValueError("观测输入摘要变化：" + path)
    capture = manifest["capture"]
    metadata = json.loads(Path(capture["observation"]).read_text())
    frame = manifest.get("frame_id", "world")
    if frame not in ("world", "rgb_camera_optical"):
        raise ValueError("未知的离线参考坐标系")
    if metadata.get("source_kind") != "vizum_linux_sdk" or not metadata.get("processing_complete"):
        raise ValueError("需要完整 SDK 处理观测")
    if frame == "world" and (not metadata.get("robot_state_paired") or not metadata["state_before"].get("steady")
            or not metadata["state_after"].get("steady")
            or not np.allclose(metadata["state_before"]["q_rad"], metadata["state_after"]["q_rad"], rtol=0, atol=1e-8)):
        raise ValueError("需要完整 SDK 观测和静止的前后配对机器人状态")
    for name in ("point_cloud", "target_cloud", "filtered_cloud"):
        record = metadata[name]
        if record.get("frame") != "camera_sdk_native" or record.get("xyz_unit") != "mm":
            raise ValueError("仅接受 SDK 原生毫米点云")
    if metadata["point_cloud"].get("mirror") != "none":
        raise ValueError("原生点云不能预先镜像")
    cloud = load_pcd(capture["raw_cloud"])
    target_ids = source_indices(np.load(capture["target_indices"], allow_pickle=False), len(cloud.xyz_raw))
    with np.load(capture["filtered_indices"], allow_pickle=False) as data:
        kept = source_indices(data["kept_source_indices"], len(cloud.xyz_raw))
    for path, indices in ((capture["target_cloud"], target_ids), (capture["filtered_cloud"], kept)):
        subset = load_pcd(path).xyz_raw
        if subset.shape != cloud.xyz_raw[indices].shape or not np.allclose(subset, cloud.xyz_raw[indices], atol=1e-5, rtol=0, equal_nan=True):
            raise ValueError("点云与原始点号不对应：" + path)
    calibration = json.loads(Path(capture["rgb_calibration"]).read_text())
    rgb_from_native = np.asarray(calibration["transform_flat"], dtype=float).reshape(4, 4)
    rgb_from_native[:3, 3] *= .001
    k = np.asarray(calibration["intrinsic_flat"]).reshape(3, 3)
    reference_from_rgb = np.eye(4)
    if frame == "world":
        hand_eye = json.loads(Path(capture["hand_eye"]).read_text())
        if hand_eye["extrinsic"].get("camera_frame_matches_pcd") is not True:
            raise ValueError("手眼标定未绑定原生点云系")
        model = json.loads(Path(capture["mount_model"]).read_text())
        reference_from_rgb = (np.asarray(model["geometry"]["world_from_left_base"])
                             @ np.asarray(metadata["state_before"]["base_from_flange"])
                             @ np.asarray(hand_eye["extrinsic"]["flange_from_camera_matrix"])
                             @ np.linalg.inv(rgb_from_native))
    depth, valid, target_mask, pixel_source = project_sdk_depth(cloud.xyz_raw, kept, target_ids, k,
                                                               rgb_from_native, calibration["width"], calibration["height"])
    result = dict(depth=depth, valid=valid, target_mask=target_mask, pixel_source=pixel_source,
                camera_matrix=k, reference_from_optical=reference_from_rgb, frame_id=frame,
                points_optical_m=transform_points(cloud.xyz_raw[kept] * .001, rgb_from_native),
                point_is_target=np.isin(kept, target_ids), kept_source_indices=kept)
    if frame == "world":
        result["world_from_optical"] = reference_from_rgb
    return result
