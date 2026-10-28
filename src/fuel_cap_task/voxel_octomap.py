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


def split_voxel_membership(indices_xyz, point_counts, point_to_voxel, cap_point_rows):
    """按完整点归属分格：0 为环境，1 为纯盖体，2 为留在环境中的混合格。

    混合格含裁剪外点，语义尚未确定；保留为环境，避免接触放行时漏掉障碍。
    返回的分组只引用原整数格号，不移动、重新体素化或丢弃任何观测格。
    """
    indices = np.asarray(indices_xyz)
    counts = np.asarray(point_counts)
    membership = np.asarray(point_to_voxel)
    rows = np.asarray(cap_point_rows)
    n = len(indices)
    if (counts.shape != (n,) or not np.issubdtype(counts.dtype, np.integer)
            or np.any(counts <= 0) or membership.ndim != 1
            or not np.issubdtype(membership.dtype, np.integer)
            or not len(membership) or membership.min() < 0 or membership.max() >= n):
        raise ValueError("完整点到格号归属无效")
    if not np.array_equal(np.bincount(membership, minlength=n), counts):
        raise ValueError("每格点数与完整归属不一致")
    if (rows.ndim != 1 or not np.issubdtype(rows.dtype, np.integer) or not len(rows)
            or rows.min() < 0 or rows.max() >= len(membership)
            or len(np.unique(rows)) != len(rows)):
        raise ValueError("盖体点行号无效或重复")
    cap_counts = np.bincount(membership[rows], minlength=n)
    pure_cap = cap_counts == counts
    mixed = (cap_counts > 0) & ~pure_cap
    if not pure_cap.any() or pure_cap.all():
        raise ValueError("需要非空独立盖体与环境")
    components = np.zeros(n, dtype=np.uint8)
    components[pure_cap] = 1
    components[mixed] = 2
    return components, cap_counts


def cap_collision_object(centers_camera_m, size_m, world_from_camera, *, object_id, frame_id):
    """生成标准 CollisionObject 字段；全部 primitive_poses 在规划系中表达。

    object.pose 使用单位变换，兼容本机 Humble 对该字段的处理；每个 box 的
    中心与朝向均由同一个 world_from_camera 得到，原 2 mm 格子不会轴对齐重采样。
    """
    from scipy.spatial.transform import Rotation

    centers = np.asarray(centers_camera_m, dtype=float)
    transform = rigid_matrix(world_from_camera)
    if (centers.ndim != 2 or centers.shape[1] != 3 or not len(centers)
            or not np.isfinite(centers).all() or not np.isfinite(size_m) or size_m <= 0
            or not object_id or not frame_id):
        raise ValueError("独立盖体几何或标识无效")
    quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
    orientation = dict(zip(("x", "y", "z", "w"), map(float, quaternion)))
    world = centers @ transform[:3, :3].T + transform[:3, 3]
    # Humble 的 byte 字段在 Python 中用单字节 bytes 表示；JSON 数组 [0] 可还原为 ADD。
    return {"header": {"frame_id": frame_id}, "id": object_id, "operation": [0],
            "pose": {"orientation": {"w": 1.0}},
            "primitives": [{"type": 1, "dimensions": [float(size_m)]*3} for _ in centers],
            "primitive_poses": [{"position": dict(zip(("x", "y", "z"), map(float, p))),
                                 "orientation": orientation.copy()} for p in world]}
