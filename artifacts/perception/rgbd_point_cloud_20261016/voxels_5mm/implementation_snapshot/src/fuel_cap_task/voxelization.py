"""将世界系观测点归入固定网格，只记录被表面点命中的体素。"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SurfaceVoxels:
    """稀疏表面体素；未列出的格子未知，不表示自由空间。"""

    indices_xyz: np.ndarray
    centers_world_m: np.ndarray
    point_counts: np.ndarray
    mean_rgb: np.ndarray
    point_to_voxel: np.ndarray
    origin_world_m: np.ndarray
    size_m: float


def voxelize_points(world_xyz_m, rgb, *, size_m, origin_world_m):
    """按半开格子归组，返回每格点数、平均颜色及逐点对应关系。

    网格原点固定在世界系，不随每帧点云包围盒变化。格子 i 占据
    [origin+i*size, origin+(i+1)*size)，负坐标也使用向下取整。
    不过滤机器人自身，不推断物体内部、遮挡区域或自由空间。
    """
    points, colors = np.asarray(world_xyz_m, dtype=float), np.asarray(rgb)
    origin = np.asarray(origin_world_m, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or not len(points)
            or not np.isfinite(points).all()):
        raise ValueError("世界系点云须为非空且有限的 N×3 数组")
    if colors.shape != points.shape or colors.dtype != np.uint8:
        raise ValueError("RGB 须为与点云同形状的 uint8 数组")
    if (isinstance(size_m, (bool, np.bool_)) or not np.isscalar(size_m)
            or not np.isfinite(size_m) or size_m <= 0):
        raise ValueError("体素边长须为有限的正数，单位米")
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError("网格原点须为世界系三个有限米制坐标")

    # 第一步：坐标减去原点，再除以格子边长；floor 使负坐标正确分格。
    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        scaled = (points - origin) / size_m
    if not np.isfinite(scaled).all() or np.any(np.abs(scaled) >= 2.**63):
        raise ValueError("体素索引超出 int64 可表示范围")
    point_indices = np.floor(scaled).astype(np.int64)

    # 第二步：合并相同 XYZ 索引，inverse 保存每个原点归属哪一个体素。
    indices, inverse, counts = np.unique(point_indices, axis=0, return_inverse=True,
                                          return_counts=True)
    color_sums = np.zeros((len(indices), 3), dtype=np.float64)
    np.add.at(color_sums, inverse, colors)
    mean_rgb = np.rint(color_sums / counts[:, None]).astype(np.uint8)

    # 第三步：还原格子中心供显示；完整体素由索引、原点和边长共同定义。
    centers = origin + (indices + .5) * size_m
    return SurfaceVoxels(indices, centers, counts, mean_rgb, inverse, origin, float(size_m))


def cube_mesh(centers_world_m, size_m):
    """把每个中心扩展成八顶点、十二三角面，仅供按实际边长显示立方格。"""
    centers = np.asarray(centers_world_m, dtype=float)
    if centers.ndim != 2 or centers.shape[1] != 3:
        raise ValueError("立方格中心须为 N×3 数组")
    corners = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
                        [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]], dtype=float)
    faces = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
                      [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
                      [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])
    vertices = (centers[:, None, :] + (corners - .5) * size_m).reshape(-1, 3)
    triangles = (faces[None, :, :] + 8 * np.arange(len(centers))[:, None, None]).reshape(-1, 3)
    return vertices, triangles
