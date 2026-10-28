"""校验原始体素与整图摆放；地图局部格子不随规划系的选择重新离散化。"""

import numpy as np


def rigid_matrix(value):
    """读取将 B 系映射到 A 系的 A_from_B；平移为米，旋转须为右手正交矩阵。"""
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-8, rtol=0)):
        raise ValueError("需要有限的右手刚体变换 A_from_B")
    return matrix


def load_occupied_voxels(path):
    """从 NPZ 读取整数格号，核对边长、原点和中心；返回值保留原有盖体标记。"""
    with np.load(path, allow_pickle=False) as arrays:
        indices = arrays["indices_xyz"].copy()
        centers = arrays["centers_camera_m"].copy()
        size = float(arrays["voxel_size_m"])
        origin = arrays["grid_origin_camera_m"].copy()
        cap = arrays["cap_voxel_mask"].copy()
    if (indices.ndim != 2 or indices.shape[1] != 3 or not len(indices)
            or not np.issubdtype(indices.dtype, np.integer)):
        raise ValueError("格号须为非空 N×3 整数数组")
    if (not np.isfinite(size) or size <= 0 or origin.shape != (3,)
            or not np.isfinite(origin).all()):
        raise ValueError("网格边长和原点无效")
    if indices.min() < -32768 or indices.max() >= 32768:
        raise ValueError("格号超出 OctoMap 16 层键范围")
    if len(np.unique(indices, axis=0)) != len(indices):
        raise ValueError("重复占据格")
    if (centers.shape != indices.shape or not np.isfinite(centers).all()
            or not np.allclose(centers, origin+(indices+.5)*size, atol=1e-12, rtol=0)):
        raise ValueError("中心与网格原点、格号、边长不一致")
    if cap.shape != (len(indices),) or cap.dtype != np.bool_:
        raise ValueError("盖体标记须与格号逐行对应")
    return indices, centers, size, origin, cap


def align_map(world_from_cap, camera_from_cap, grid_origin_camera_m):
    """盖体系对齐得到整图变换；非零网格原点另计入 camera_from_octomap。"""
    target, observed = rigid_matrix(world_from_cap), rigid_matrix(camera_from_cap)
    world_from_camera = target @ np.linalg.inv(observed)
    camera_from_octomap = np.eye(4)
    origin = np.asarray(grid_origin_camera_m, dtype=float)
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError("网格原点无效")
    camera_from_octomap[:3, 3] = origin
    return world_from_camera, camera_from_octomap, world_from_camera @ camera_from_octomap
