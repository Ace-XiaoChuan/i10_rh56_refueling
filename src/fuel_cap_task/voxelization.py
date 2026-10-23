"""将世界系观测点归入固定网格，只记录被表面点命中的体素。"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SurfaceVoxels:
    """保存被观测点命中的表面格子及逐点归属，未列出的格子保持未知。

    字段：
        indices_xyz：形状 (M, 3) 的整数数组，每行为一个格子的 (ix, iy, iz)。
            它是网格索引，不是米制坐标；M 为不同的已命中格子数。
        centers_world_m：形状 (M, 3) 的浮点数组，各格几何中心的世界系坐标，
            单位米；不是格内点坐标的平均值。
        point_counts：形状 (M,) 的整数数组，第 j 项为第 j 个格子内的点数。
        mean_rgb：形状 (M, 3)、类型 uint8 的数组，各格内点的平均 RGB，
            通道依次为红、绿、蓝，按最近整数舍入到 0～255。
        point_to_voxel：形状 (N,) 的整数数组，N 为输入点数；第 i 项为输入
            点 i 在上述 M 行格子列表中的行号 j，即该点属于 indices_xyz[j]。
            这个一维列表行号与三维格号 (ix, iy, iz) 是不同的概念。
        origin_world_m：形状 (3,) 的浮点数组，网格原点在世界系的位置，单位米。
        size_m：正浮点数，立方格的边长，单位米。

    同一行 j 的格号、中心、点数、颜色描述同一个格子。完整格子的范围由
    indices_xyz、origin_world_m、size_m 共同定义，而非只有一个中心点。
    """

    indices_xyz: np.ndarray
    centers_world_m: np.ndarray
    point_counts: np.ndarray
    mean_rgb: np.ndarray
    point_to_voxel: np.ndarray
    origin_world_m: np.ndarray
    size_m: float


def voxelize_points(world_xyz_m, rgb, *, size_m, origin_world_m):
    """按半开格子归组，返回每格点数、平均颜色及逐点对应关系。

    参数：
        world_xyz_m：形状 (N, 3) 的有限数值数组，每行为一个观测点的世界系
            (X, Y, Z)，单位米；N 必须大于 0。通常来自点云重建的 world_xyz_m。
        rgb：形状 (N, 3)、类型 uint8 的 RGB 数组，通道顺序为红、绿、蓝，
            各通道取值 0～255，第 i 行颜色对应 world_xyz_m 的第 i 个点。
        size_m：有限正数，所有轴采用相同的立方格边长，单位米；
            例如 0.005 表示边长 5 mm，而不是 5 m 或 5 个格子。
        origin_world_m：形状 (3,) 的有限数值数组，网格原点的世界系坐标，
            单位米。它规定格子边界的位置，不是相机原点或点云中心。
            改变此值只改变格子划分，不移动输入点。
        函数签名中的 * 表示 size_m、origin_world_m 必须写参数名传入。

    返回：
        SurfaceVoxels，含 M 个已命中格子的索引、中心、点数、平均颜色和
        N 个输入点各自的归属。原始点坐标不被修改，也不在此返回值中复制保存。

    异常：
        ValueError：输入点为空、形状或颜色类型不符、边长非正、数值非有限，
            或计算得到的网格索引超出 int64 的可表示范围。

    网格原点固定在世界系，不随每帧点云包围盒变化。格子 i 占据
    [origin+i*size, origin+(i+1)*size)，各轴均包含下边界、不包含上边界；
    负坐标也使用向下取整，例如 floor(-0.2)=-1，而不是截断为 0。
    不过滤机器人自身，不推断物体内部、遮挡区域或自由空间。
    """
    # points 是 (N, 3) 的观测点；colors 与其逐行对应；origin 是网格原点 (3,)。
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

    # 第一步：每行点坐标都减去同一个 origin（NumPy 广播），再除以格子边长。
    # scaled 仍为 (N, 3)，但其单位已变为“格子数”，例如 2.4 表示离原点 2.4 格。
    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        scaled = (points - origin) / size_m
    if not np.isfinite(scaled).all() or np.any(np.abs(scaled) >= 2.**63):
        raise ValueError("体素索引超出 int64 可表示范围")
    # 向下取整后，每个输入点有一个三维整数格号；同一格里的多个点格号相同。
    point_indices = np.floor(scaled).astype(np.int64)

    # 第二步：axis=0 表示以整行 (ix, iy, iz) 为单位去重，而不是单个数字去重。
    # indices 为 (M, 3) 的不同格号；inverse 为 (N,) 的逐点所属格子列表行号；
    # counts 为 (M,) 的每格点数。恒有 indices[inverse] == point_indices。
    indices, inverse, counts = np.unique(point_indices, axis=0, return_inverse=True,
                                          return_counts=True)
    # 同格点的颜色累加到同一行；add.at 能正确处理 inverse 中重复的格子编号。
    color_sums = np.zeros((len(indices), 3), dtype=np.float64)
    np.add.at(color_sums, inverse, colors)
    # counts[:, None] 将 (M,) 变为 (M, 1)，使三个颜色通道都除以对应格子的点数。
    mean_rgb = np.rint(color_sums / counts[:, None]).astype(np.uint8)

    # 第三步：格号给出下边界位置，加半格得到几何中心；它不是格内点的均值。
    centers = origin + (indices + .5) * size_m
    return SurfaceVoxels(indices, centers, counts, mean_rgb, inverse, origin, float(size_m))


def cube_mesh(centers_world_m, size_m):
    """把每格中心扩展成八顶点、十二三角面，仅供显示真实大小的立方格。

    参数：
        centers_world_m：形状 (M, 3) 的数值数组，各格几何中心的世界系坐标，
            单位米；调用方应保证数值有限，通常传 SurfaceVoxels 的同名字段。
        size_m：立方格边长，单位米；调用方应保证为有限正数。

    返回：
        (vertices, triangles) 元组：
        vertices 为形状 (8*M, 3) 的世界系顶点坐标数组，单位米；
        triangles 为形状 (12*M, 3) 的整数数组，每行是一个三角面的三个顶点
        在 vertices 中的行号。每格单独保留八个顶点，不合并相邻格子的公共面。

    异常：
        ValueError：中心数组的形状不是 (M, 3)。

    该函数只构造显示网格，不改变表面体素的观测含义，也不执行物体内部填充。
    """
    centers = np.asarray(centers_world_m, dtype=float)
    if centers.ndim != 2 or centers.shape[1] != 3:
        raise ValueError("立方格中心须为 N×3 数组")
    corners = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
                        [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]], dtype=float)
    faces = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
                      [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
                      [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])
    # [0, 1] 角点先减去 0.5，变为相对中心的 ±半格偏移；每个中心加八种偏移。
    vertices = (centers[:, None, :] + (corners - .5) * size_m).reshape(-1, 3)
    # 第 j 个立方格的顶点从 8*j 行开始，面索引须加上对应偏移。
    triangles = (faces[None, :, :] + 8 * np.arange(len(centers))[:, None, None]).reshape(-1, 3)
    return vertices, triangles


def pixel_to_voxel_lookup(pixel_uv, point_to_voxel, image_size_wh):
    """建立 H×W 像素到体素编号的查表，无有效观测的像素保留 -1。

    参数：
        pixel_uv：形状 (N, 2) 的整数数组，每行为原图像素 (u, v)，即列、行。
            索引从 0 开始，须在图像范围内，且同一像素不能重复出现。
        point_to_voxel：形状 (N,) 的非负整数数组，第 i 项为 pixel_uv[i]
            对应点在体素列表中的行号；须与输入像素逐项对应并能用 int32 表示。
            通常使用 voxelize_points() 返回值中的同名字段。
        image_size_wh：长度为 2 的序列 (W, H)，依次为图像宽、高，均为正整数。
            注意此处宽在前，而返回数组形状及数组索引采用行在前。

    返回：
        形状 (H, W)、小端 int32 类型的数组 lookup。lookup[v, u] 为该像素
        所属体素在列表中的行号；没有有效观测的像素为 -1，不能据此认为无障碍。

    异常：
        ValueError：形状、整数类型、图像尺寸不符，像素越界或重复，
            或体素编号为负数、超出 int32 范围。

    使用原观测的逐点对应，不通过颜色猜测物体，也不以最近三维点补无效深度。
    小端 int32 编码便于页面按原图点击后直接查找。
    """
    uv, ids = np.asarray(pixel_uv), np.asarray(point_to_voxel)
    if (len(image_size_wh) != 2
            or any(type(value) is not int or value <= 0 for value in image_size_wh)):
        raise ValueError("图像宽高须为两个正整数")
    width, height = image_size_wh
    if (uv.ndim != 2 or uv.shape[1] != 2 or not np.issubdtype(uv.dtype, np.integer)
            or ids.shape != (len(uv),) or not np.issubdtype(ids.dtype, np.integer)):
        raise ValueError("像素须为整数 N×2 数组，体素编号须为对应的整数 N 数组")
    if (np.any(uv < 0) or np.any(uv[:, 0] >= width) or np.any(uv[:, 1] >= height)
            or np.any(ids < 0) or np.any(ids > np.iinfo(np.int32).max)):
        raise ValueError("像素或体素编号超出查表范围")
    # 将 (u, v) 换成一维像素编号 v*W+u，以检查是否重复提供同一个像素。
    flat = uv[:, 1].astype(np.int64)*width + uv[:, 0]
    if len(np.unique(flat)) != len(flat):
        raise ValueError("同一个观测像素不能重复对应多个输入点")
    # 先把整幅图标为无有效观测，再仅填写实际存在的像素到体素的对应。
    lookup = np.full((height, width), -1, dtype="<i4")
    lookup[uv[:, 1], uv[:, 0]] = ids
    return lookup
