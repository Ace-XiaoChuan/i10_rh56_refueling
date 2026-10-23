"""将已配准的针孔 RGB-D 观测还原成可见表面点，不读取仿真几何。"""

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image


@dataclass(frozen=True)
class PointCloudObservation:
    """同一观测的三维点、RGB 与像素索引；坐标单位为米，像素顺序为 u、v。"""

    camera_xyz_m: np.ndarray
    world_xyz_m: np.ndarray
    rgb: np.ndarray
    pixel_uv: np.ndarray
    metadata: dict
    input_sha256: dict


def digest(path: Path) -> str:
    """记录实际读取文件的摘要，避免后续把点云与另一份观测混用。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def backproject_depth(depth_m, valid, camera_matrix):
    """将光轴 Z 深度反投影到 X 右、Y 下、Z 前的相机光学系。

    输入图像须已去畸变；内参不含斜切项。返回 N×3 米制点和 N×2
    像素索引，跳过无效、非有限和非正深度，不解释未观测区域是否为空。
    """
    depth = np.asarray(depth_m, dtype=float)
    mask = np.asarray(valid)
    k = np.asarray(camera_matrix, dtype=float)
    if depth.ndim != 2 or mask.shape != depth.shape or mask.dtype != np.bool_:
        raise ValueError("深度须为二维数组，valid 须为同形状布尔数组")
    if (k.shape != (3, 3) or not np.isfinite(k).all()
            or k[0, 0] <= 0 or k[1, 1] <= 0):
        raise ValueError("内参须为有限的 3×3 矩阵，且焦距为正")
    expected = np.array([[k[0, 0], 0., k[0, 2]], [0., k[1, 1], k[1, 2]], [0., 0., 1.]])
    if not np.allclose(k, expected, atol=1e-12, rtol=0):
        raise ValueError("当前入口要求无斜切的针孔内参")

    # 先筛有效像素，再还原相机系坐标；深度是 Z，不是到相机的斜距。
    v, u = np.nonzero(mask & np.isfinite(depth) & (depth > 0))
    z = depth[v, u]
    x = (u - k[0, 2]) * z / k[0, 0]
    y = (v - k[1, 2]) * z / k[1, 1]
    points = np.column_stack((x, y, z))
    if not np.isfinite(points).all():
        raise ValueError("反投影坐标超出有限数值范围")
    return points, np.column_stack((u, v)).astype(np.int32)


def transform_points(camera_xyz_m, world_from_optical):
    """将相机系点变换到世界系；矩阵表示相机光学系在世界系中的位姿。"""
    points = np.asarray(camera_xyz_m, dtype=float)
    transform = np.asarray(world_from_optical, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("点须为有限的 N×3 数组")
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8, rtol=0)):
        raise ValueError("world_from_optical 须为有限的 4×4 齐次矩阵")
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or abs(np.linalg.det(rotation) - 1) > 1e-6):
        raise ValueError("world_from_optical 的旋转须正交且为右手系")
    return points @ rotation.T + transform[:3, 3]


def load_observation(folder):
    """读取统一 RGB-D 文件，只使用观测与标定，真值文件不参与重建。"""
    folder = Path(folder).resolve()
    paths = {name: folder / name for name in ("observation.json", "rgb.png", "depth.npz")}
    metadata = json.loads(paths["observation.json"].read_text())
    for field in ("source", "frame_id", "timestamp_clock"):
        if not isinstance(metadata.get(field), str) or not metadata[field].strip():
            raise ValueError(f"观测缺少明确的 {field}")
    timestamp = metadata.get("timestamp_s")
    if (isinstance(timestamp, bool) or not isinstance(timestamp, (int, float))
            or not np.isfinite(timestamp)):
        raise ValueError("观测时间戳须为有限数值")
    if metadata.get("depth_unit") != "m" or metadata.get("depth_convention") != "optical_z":
        raise ValueError("请先将深度转换为以米为单位的光轴 Z 深度")
    size = metadata.get("image_size_wh")
    if (not isinstance(size, list) or len(size) != 2
            or any(type(value) is not int or value <= 0 for value in size)):
        raise ValueError("image_size_wh 须为正整数宽、高")
    input_sha256 = {name: digest(path) for name, path in paths.items()}
    for name in ("rgb.png", "depth.npz"):
        expected = metadata.get("files", {}).get(name)
        if expected is not None and expected != input_sha256[name]:
            raise ValueError(f"观测文件摘要不符：{name}")
    with Image.open(paths["rgb.png"]) as image:
        rgb = np.asarray(image.convert("RGB"))
    with np.load(paths["depth.npz"], allow_pickle=False) as arrays:
        depth, valid = arrays["depth_m"], arrays["valid"]
    if depth.shape != (size[1], size[0]) or rgb.shape[:2] != depth.shape:
        raise ValueError("RGB、深度和标定分辨率不一致，不能按像素直接配对")
    camera, pixel_uv = backproject_depth(depth, valid, metadata["camera_matrix"])
    if not len(camera):
        raise ValueError("没有可重建的有效深度")
    world = transform_points(camera, metadata["world_from_optical"])
    colors = rgb[pixel_uv[:, 1], pixel_uv[:, 0]].copy()
    return PointCloudObservation(camera, world, colors, pixel_uv, metadata, input_sha256)


def write_ply(path, xyz_m, rgb):
    """保存完整彩色点云为二进制小端 PLY；单点坐标使用 float32，单位为米。"""
    xyz, colors = np.asarray(xyz_m), np.asarray(rgb)
    if (xyz.ndim != 2 or xyz.shape[1] != 3 or not np.isfinite(xyz).all()
            or colors.shape != xyz.shape or colors.dtype != np.uint8):
        raise ValueError("PLY 要求有限的 N×3 坐标及同形状 uint8 RGB")
    if np.any(np.abs(xyz) > np.finfo(np.float32).max):
        raise ValueError("PLY 坐标超出 float32 范围")
    records = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                       ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    for column, name in enumerate(("x", "y", "z")):
        records[name] = xyz[:, column]
    for column, name in enumerate(("red", "green", "blue")):
        records[name] = colors[:, column]
    header = (f"ply\nformat binary_little_endian 1.0\ncomment length_unit m\n"
              f"element vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
    with Path(path).open("xb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(records.tobytes())
