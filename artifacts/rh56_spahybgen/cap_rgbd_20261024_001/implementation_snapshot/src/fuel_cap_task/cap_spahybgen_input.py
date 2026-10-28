"""仅由 RGB-D 和人工整盖标注构建 SpaHybGen 局部体积，不读取模型真值。"""

import numpy as np
from scipy.ndimage import distance_transform_edt

from .rgbd_point_cloud import backproject_depth, transform_points


def target_ellipse(shape, bounds):
    """按原图像素生成整盖椭圆标注；bounds 为左、上、右、下，单位像素。"""
    box = np.asarray(bounds, dtype=float)
    if (len(shape) != 2 or box.shape != (4,) or not np.isfinite(box).all()
            or not (0 <= box[0] < box[2] < shape[1] and 0 <= box[1] < box[3] < shape[0])):
        raise ValueError("整盖椭圆须在图像范围内且宽、高为正")
    v, u = np.indices(shape)
    center = (box[:2] + box[2:]) / 2
    radius = (box[2:] - box[:2]) / 2
    return ((u - center[0]) / radius[0]) ** 2 + ((v - center[1]) / radius[1]) ** 2 <= 1


def local_geometry(depth, valid, camera_matrix, world_from_optical, target_mask,
                   size_m=.4, resolution=80, contact_radius_m=.01, center_radius_m=.02):
    """以整盖可见点的中位数放置局部体积，保留到相机和世界的刚体变换。

    体积 X 沿相机 X，Y 沿相机 -Y，Z 沿相机 -Z，因此 Z 朝向观察者。
    体积中心只是观测区域的参考位置，不作为油箱盖完整位姿估计。
    物理尺寸不缩放；体素索引采用原优化器的 index * voxel_size 坐标约定。
    """
    mask = np.asarray(target_mask)
    if mask.shape != np.shape(depth) or mask.dtype != np.bool_:
        raise ValueError("目标标注须为与深度同形状的布尔数组")
    settings = np.array([size_m, contact_radius_m, center_radius_m], dtype=float)
    if (not np.isfinite(settings).all() or np.any(settings <= 0)
            or type(resolution) is not int or resolution < 2):
        raise ValueError("体积、邻域尺寸和网格分辨率须有效")
    camera, pixels = backproject_depth(depth, valid, camera_matrix)
    selected = mask[pixels[:, 1], pixels[:, 0]]
    if selected.sum() < 30:
        raise ValueError("整盖区域内有效深度不足 30 点")
    reference = np.median(camera[selected], axis=0)
    camera_from_volume = np.eye(4)
    camera_from_volume[:3, :3] = np.diag([1., -1., -1.])
    camera_from_volume[:3, 3] = reference - camera_from_volume[:3, :3] @ np.full(3, size_m / 2)
    volume_from_camera = np.linalg.inv(camera_from_volume)
    points = transform_points(camera, volume_from_camera)
    voxel_size = size_m / resolution
    indices = np.floor(points / voxel_size).astype(np.int64)
    inside = np.all((indices >= 0) & (indices < resolution), axis=1)
    if not inside[selected].all():
        raise ValueError("局部体积未覆盖完整目标观测；请调整标注或体积")
    target_surface = np.zeros((resolution,) * 3, dtype=bool)
    target_indices = indices[selected]
    target_surface[tuple(target_indices.T)] = True
    distances = distance_transform_edt(~target_surface) * voxel_size
    world_from_volume = np.asarray(world_from_optical, dtype=float) @ camera_from_volume
    # 同时验证世界变换，避免把镜像或尺度误当刚体位姿传到显示与几何检查。
    transform_points(np.zeros((1, 3)), world_from_volume)
    return {"points_volume_m": points[inside], "pixels_uv": pixels[inside],
            "target_point_mask": selected[inside], "target_surface_grid": target_surface,
            "contact_region": distances <= contact_radius_m + 1e-12,
            "center_region": distances <= center_radius_m + 1e-12,
            "camera_from_volume": camera_from_volume, "world_from_volume": world_from_volume,
            "reference_camera_m": reference, "voxel_size_m": np.array(voxel_size)}


def observed_free_grid(depth, valid, camera_matrix, camera_from_volume, size_m=.4, resolution=80):
    """独立保存由有效深度支持的空闲格；未观测区域不因网络网格为零就变为空闲。"""
    axis = (np.arange(resolution) + .5) * (size_m / resolution)
    centers = np.stack(np.meshgrid(axis, axis, axis, indexing="ij"), axis=-1).reshape(-1, 3)
    camera = transform_points(centers, camera_from_volume)
    k = np.asarray(camera_matrix)
    positive = camera[:, 2] > 0
    u = np.zeros(len(camera), dtype=int)
    v = np.zeros(len(camera), dtype=int)
    u[positive] = np.rint(k[0, 0] * camera[positive, 0] / camera[positive, 2] + k[0, 2]).astype(int)
    v[positive] = np.rint(k[1, 1] * camera[positive, 1] / camera[positive, 2] + k[1, 2]).astype(int)
    inside = positive & (u >= 0) & (u < depth.shape[1]) & (v >= 0) & (v < depth.shape[0])
    rows = np.flatnonzero(inside)
    rows = rows[valid[v[rows], u[rows]] & np.isfinite(depth[v[rows], u[rows]])
                & (depth[v[rows], u[rows]] > 0)]
    free = np.zeros(len(camera), dtype=bool)
    free[rows] = camera[rows, 2] < depth[v[rows], u[rows]] - size_m / resolution
    return free.reshape((resolution,) * 3)


def restrict_prediction(quality, center_score, geometry):
    """保留网络原始得分，仅限制整盖邻域；不添加人工接触点或替换网络预测。"""
    quality = np.asarray(quality)
    center_score = np.asarray(center_score)
    if (quality.shape != geometry["contact_region"].shape or center_score.shape != quality.shape
            or not np.isfinite(quality).all() or not np.isfinite(center_score).all()):
        raise ValueError("预测形状或数值无效")
    target_quality = np.where(geometry["contact_region"], quality, 0).astype(np.float32)
    target_center = np.where(geometry["center_region"], center_score, 0).astype(np.float32)
    if target_quality.max() <= 0 or target_center.max() <= 0:
        raise ValueError("整盖邻域内没有正的接触或中心预测")
    return target_quality, target_center
