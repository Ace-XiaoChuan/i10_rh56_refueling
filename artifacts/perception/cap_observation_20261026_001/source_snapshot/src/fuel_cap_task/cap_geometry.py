"""从已裁剪的圆形盖体点云估计几何方向，不把可见表面冒充握梁几何中心。"""

from dataclasses import dataclass
from itertools import product
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull, QhullError
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class CapGeometry:
    """相机系米制几何结果；X 沿握梁，Z 朝观察相机，X 的正负仍有歧义。

    boundary_center_m 是观测外轮廓近似圆所在平面的圆心；surface_reference_m
    是从该圆心沿估计轴与握梁可见表面拟合相交得到的参考点。二者均不直接
    等于固定抓姿要求的 cap_grip_center。数组索引对应输入点的原始顺序。
    """

    boundary_center_m: np.ndarray
    surface_reference_m: np.ndarray
    rotation: np.ndarray
    radius_m: float
    face_indices: np.ndarray
    bridge_indices: np.ndarray
    boundary_indices: np.ndarray
    diagnostics: dict


def read_grip_model_reference(urdf_path, *, top_width_reduction_m):
    """读取占位 URDF 的握梁尺寸和既有参考点，用户补充顶面较根部收窄量。

    宽度沿 cap_grip_center 的 Y，高度沿其 Z，长度沿其 X。URDF 当前以 box
    包络近似梯形根部；本函数只解释尺寸，不改动模型或把参考点移到体积质心。
    返回值均使用米。顶面到中心距离来自已有参考帧与几何的交叉核对。
    """
    root = ET.parse(urdf_path).getroot()
    visual = root.find("./link[@name='cap_body']/visual[@name='cap_grip']")
    center = root.find("./joint[@name='cap_grip_center_joint']/origin")
    top = root.find("./joint[@name='cap_grip_top_center_joint']/origin")
    if visual is None or center is None or top is None:
        raise ValueError("URDF 缺少握梁 box 或中心/顶面参考帧")
    box = visual.find("./geometry/box")
    if box is None:
        raise ValueError("此占位尺寸读取器要求握梁使用 box 包络")

    def vector(element, field, default="0 0 0"):
        value = np.fromstring(element.get(field, default), sep=" ")
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError("URDF 的尺寸、位置或旋转必须是有限三维向量")
        return value

    sizes = vector(box, "size")
    if np.any(sizes <= 0):
        raise ValueError("握梁尺寸必须为正")
    visual_origin = visual.find("origin")
    if visual_origin is None:
        visual_origin = ET.Element("origin")
    center_rotation = Rotation.from_euler("xyz", vector(center, "rpy")).as_matrix()
    visual_rotation = Rotation.from_euler("xyz", vector(visual_origin, "rpy")).as_matrix()
    corners = np.array(list(product([-.5, .5], repeat=3))) * sizes
    body_points = corners @ visual_rotation.T + vector(visual_origin, "xyz")
    local = (body_points - vector(center, "xyz")) @ center_rotation
    minimum, maximum = local.min(axis=0), local.max(axis=0)
    top_local = (vector(top, "xyz") - vector(center, "xyz")) @ center_rotation
    if (not np.allclose(top_local[:2], 0., atol=1e-10)
            or not np.isclose(top_local[2], maximum[2], atol=1e-10)
            or not np.allclose(minimum, -maximum, atol=1e-10)):
        raise ValueError("握梁中心、顶面参考点与 box 包络不一致，需重新确认参考定义")
    length, base_width, height = maximum - minimum
    reduction = float(top_width_reduction_m)
    if not np.isfinite(reduction) or reduction < 0 or reduction >= base_width:
        raise ValueError("顶面宽度收窄量必须非负且小于根部宽度")
    return {"length_m": float(length), "base_width_m": float(base_width),
            "top_width_m": float(base_width - reduction), "height_m": float(height),
            "top_width_reduction_m": reduction,
            "top_surface_to_reference_m": float(top_local[2]),
            "reference_frame": "cap_grip_center", "reference_definition": "existing_height_midpoint",
            "source_kind": "provisional_urdf_and_user_width_relation"}


def _plane_basis(normal):
    """构造平面内正交基，坐标轴的选取只用于计算，不赋予物理朝向。"""
    seed = np.eye(3)[np.argmin(np.abs(normal))]
    first = np.cross(normal, seed)
    first /= np.linalg.norm(first)
    return np.column_stack((first, np.cross(normal, first)))


def _towards_camera(normal, position, camera_origin):
    """已知观察的是盖外侧时，用点到相机原点的方向确定法向正负。"""
    if normal @ (camera_origin - position) < 0:
        normal = -normal
    return normal


def _split_heights(points, normal):
    """对沿法向的高度做确定性二聚类，并拒绝没有明显分层的输入。"""
    heights = (points - points.mean(axis=0)) @ normal
    centers = np.percentile(heights, [10, 90])
    for _ in range(60):
        labels = np.argmin(np.abs(heights[:, None] - centers), axis=1)
        if any(np.count_nonzero(labels == i) < 30 for i in range(2)):
            raise ValueError("盖面与握梁任一分组的支持点不足")
        updated = np.array([heights[labels == i].mean() for i in range(2)])
        if np.max(np.abs(updated - centers)) < 1e-10:
            break
        centers = updated
    bridge = labels == np.argmax(centers)
    face = ~bridge
    gap = np.percentile(heights[bridge], 10) - np.percentile(heights[face], 90)
    separation = float(np.ptp(centers))
    # 这是几何基线的拒绝条件，不是设备精度指标。单平面或纯噪声不能冒充握梁。
    if separation <= 0 or gap < 0.2 * separation:
        raise ValueError("未发现足够清楚的盖面—握梁深度分层")
    return np.flatnonzero(face), np.flatnonzero(bridge), separation, float(gap)


def _boundary_fit(points, normal, face_indices, camera_origin):
    """从盖面投影凸包获取观测外轮廓，拟合其平面和圆；不填补缺失表面。"""
    face = points[face_indices]
    origin = face.mean(axis=0)
    basis = _plane_basis(normal)
    try:
        hull = ConvexHull((face - origin) @ basis)
    except QhullError as exc:
        raise ValueError("盖面投影退化，无法拟合外轮廓") from exc
    boundary_indices = face_indices[hull.vertices]
    boundary = points[boundary_indices]
    if len(boundary) < 8:
        raise ValueError("观测外轮廓支持点不足")
    origin = boundary.mean(axis=0)
    _, _, vectors = np.linalg.svd(boundary - origin, full_matrices=False)
    normal = _towards_camera(vectors[-1], origin, camera_origin)
    basis = _plane_basis(normal)
    uv = (boundary - origin) @ basis
    solution = np.linalg.lstsq(
        np.column_stack((2 * uv, np.ones(len(uv)))), np.sum(uv**2, axis=1), rcond=None,
    )[0]
    radius = np.sqrt(max(0., solution[2] + solution[:2] @ solution[:2]))
    if radius <= 0:
        raise ValueError("观测轮廓的圆半径退化")
    fitted = least_squares(
        lambda x: np.linalg.norm(uv - x[:2], axis=1) - x[2],
        [*solution[:2], radius], loss="soft_l1", f_scale=radius * 0.015,
        bounds=([-np.inf, -np.inf, radius * 0.2], [np.inf, np.inf, radius * 5]),
    )
    if not fitted.success:
        raise ValueError("观测轮廓圆拟合未收敛")
    center = origin + basis @ fitted.x[:2]
    radius = float(fitted.x[2])
    centered = (boundary - center) @ basis
    angles = np.sort(np.mod(np.arctan2(centered[:, 1], centered[:, 0]), 2 * np.pi))
    max_gap = np.max(np.diff(np.r_[angles, angles[0] + 2 * np.pi]))
    plane_rms = float(np.sqrt(np.mean(((boundary - center) @ normal)**2)))
    circle_rms = float(np.sqrt(np.mean((np.linalg.norm(centered, axis=1) - radius)**2)))
    coverage = float(360 - np.rad2deg(max_gap))
    if plane_rms > radius * 0.04 or circle_rms > radius * 0.06 or coverage < 270:
        raise ValueError("外轮廓的共面、近圆或角度覆盖条件不足，需检查选区与遮挡")
    return center, normal, radius, boundary_indices, plane_rms, circle_rms, coverage


def estimate_cap_geometry(points_m, *, camera_origin_m=(0., 0., 0.)):
    """估计已裁剪整盖的方向、观测圆和可见表面参考点。

    输入 points_m 为 (N,3) 有限实数，单位米，camera_origin_m 与点处于同一
    坐标系。要求盖外侧朝向观察相机、近圆外轮廓覆盖充分、凸起长握梁具有
    清楚的高度分层。返回 CapGeometry；条件不足时抛出 ValueError。
    首版只适用于此类几何，不输出有效运动目标或伪造采集时间。
    """
    points = np.asarray(points_m, dtype=float)
    camera = np.asarray(camera_origin_m, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 128:
        raise ValueError("需要至少 128 个三维点，形状为 (N,3)")
    if not np.isfinite(points).all() or camera.shape != (3,) or not np.isfinite(camera).all():
        raise ValueError("点坐标与相机原点必须是有限三维实数")
    origin = points.mean(axis=0)
    _, singular, vectors = np.linalg.svd(points - origin, full_matrices=False)
    if singular[1] < 1e-12:
        raise ValueError("输入点退化为直线或单点")
    normal = _towards_camera(vectors[-1], origin, camera)

    # 先用整体 PCA 获得粗法向，再用近圆外轮廓细化；分组与法向交替更新。
    for _ in range(3):
        face_indices, bridge_indices, separation, gap = _split_heights(points, normal)
        (center, normal, radius, boundary_indices, plane_rms,
         circle_rms, coverage) = _boundary_fit(points, normal, face_indices, camera)
    face_indices, bridge_indices, separation, gap = _split_heights(points, normal)
    bridge = points[bridge_indices]
    heights = (bridge - center) @ normal
    # 去掉靠近盖面的少量连接/侧壁点，保留朝外的握梁表面作为朝向支持。
    core_indices = bridge_indices[heights >= np.percentile(heights, 10)]
    core = points[core_indices]
    basis = _plane_basis(normal)
    uv = (core - center) @ basis
    values, directions = np.linalg.eigh(np.cov(uv, rowvar=False))
    elongation = float(values[-1] / max(values[0], 1e-20))
    if elongation < 4:
        raise ValueError("凸起区域缺少明确长轴，无法确定握梁朝向")
    along = basis @ directions[:, -1]
    # 正负只采用确定性的坐标约定，两个 180° 候选都要保留。
    if along[np.argmax(np.abs(along))] < 0:
        along = -along
    across = np.cross(normal, along)
    rotation = np.column_stack((along, across, normal))

    # 用归一化局部坐标拟合可见握梁表面的二次高度，减少曲率与采样密度的偏置。
    local = (core - center) @ rotation[:, :2] / radius
    s, t = local.T
    design = np.column_stack((np.ones(len(s)), s, t, s*s, s*t, t*t))
    target = (core - center) @ normal
    if np.linalg.matrix_rank(design) < 6:
        raise ValueError("握梁表面支持点退化，无法估计表面参考点")
    coefficients = np.linalg.lstsq(design, target, rcond=None)[0]
    fitted = least_squares(lambda x: design @ x - target, coefficients,
                           loss="soft_l1", f_scale=radius * 0.015)
    if not fitted.success:
        raise ValueError("握梁表面拟合未收敛")
    if not (s.min() <= 0 <= s.max() and t.min() <= 0 <= t.max()):
        raise ValueError("观测圆心的投影位于握梁支持范围外")
    reference = center + normal * fitted.x[0]
    diagnostics = {
        "point_count": len(points), "face_count": len(face_indices),
        "bridge_count": len(bridge_indices), "bridge_core_count": len(core_indices),
        "boundary_count": len(boundary_indices),
        "height_separation_m": separation, "height_quantile_gap_m": gap,
        "boundary_plane_rms_m": plane_rms, "boundary_circle_rms_m": circle_rms,
        "boundary_angular_coverage_deg": coverage, "bridge_eigenvalue_ratio": elongation,
        "bridge_surface_rms_m": float(np.sqrt(np.mean((design @ fitted.x - target)**2))),
        "surface_height_from_boundary_m": float(fitted.x[0]),
    }
    return CapGeometry(center, reference, rotation, radius, face_indices, bridge_indices,
                       boundary_indices, diagnostics)
