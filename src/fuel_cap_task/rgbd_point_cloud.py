"""将已配准的针孔 RGB-D 观测还原成可见表面点，不读取仿真几何。"""

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image


@dataclass(frozen=True)
class PointCloudObservation:
    """保存同一帧观测的点云及来源，所有逐点数组的第 i 行对应同一个点。

    字段：
        camera_xyz_m：形状 (N, 3) 的浮点数组，依次为相机光学系 X、Y、Z，
            单位米；相机 X 向右、Y 向下、Z 向前。
        world_xyz_m：形状 (N, 3) 的浮点数组，同一批点在世界系中的坐标，
            单位米。此世界系不自动等同于机械臂基座坐标系。
        rgb：形状 (N, 3)、类型 uint8 的颜色数组，通道顺序为红、绿、蓝，
            各通道取值 0～255。
        pixel_uv：形状 (N, 2)、类型 int32 的原图像素索引，每行为 (u, v)，
            即列号、行号；它把三维点追溯到 RGB、深度或后续分割掩膜。
        metadata：原始 observation.json 的字典，包含内参、坐标变换、
            分辨率、时间戳、观测来源等信息。
        input_sha256：字典，键为实际读取的文件名，值为文件的 SHA-256 摘要，
            用于核对点云是否仍与同一份观测对应。

    N 是通过有效性检查后保留的像素数量，不一定等于图像总像素数。
    """

    camera_xyz_m: np.ndarray
    world_xyz_m: np.ndarray
    rgb: np.ndarray
    pixel_uv: np.ndarray
    metadata: dict
    input_sha256: dict


def digest(path: Path) -> str:
    """计算文件内容摘要，避免后续把点云与另一份观测混用。

    参数：
        path：pathlib.Path，待读取文件的路径。

    返回：
        str，文件内容的 SHA-256 十六进制字符串。文件内容相同则摘要相同；
        仅移动或重命名文件不会改变此摘要。

    异常：
        文件不存在或不可读时，向上传递对应的文件读取异常。
    """
    return hashlib.sha256(path.read_bytes()).hexdigest()


def backproject_depth(depth_m, valid, camera_matrix):
    """将光轴 Z 深度反投影到 X 右、Y 下、Z 前的相机光学系。

    参数：
        depth_m：形状 (H, W) 的数值数组，H 为图像行数，W 为列数。
            depth_m[v, u] 是像素 (u, v) 处表面的光轴 Z 深度，单位米，
            不是点到相机原点的欧氏距离；输入图像须已去畸变。
        valid：形状 (H, W)、类型 bool 的有效性数组。True 表示允许使用
            该像素，False 表示排除；这是上游深度有效标记，不是物体类别。
            即使为 True，非有限或非正深度仍会被下面的计算排除。
        camera_matrix：形状 (3, 3) 的针孔内参矩阵 K，须对应当前深度图：
            [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]。
            fx、fy 为正的像素焦距；cx、cy 为主点的列、行坐标，单位像素。
            当前实现不接受斜切项。

    返回：
        (points, pixel_uv) 元组：
        points 为形状 (N, 3) 的浮点数组，每行为相机系 (X, Y, Z)，单位米；
        pixel_uv 为形状 (N, 2) 的 int32 数组，每行为对应原像素的 (u, v)。
        N 为满足全部有效条件的像素数；没有有效像素时，两者可为空。

    异常：
        ValueError：数组形状、有效标记类型、内参不符合约定，或计算溢出。

    只生成有效像素所观测到的表面点，不补全无效或遮挡部分，也不把它们
    解释为空闲空间。坐标来自深度和标定，不读取 MuJoCo 物体几何。
    """
    # 将输入统一为数组；depth 和 k 使用浮点数，mask 保持布尔类型。
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

    # 三个同为 (H, W) 的布尔数组逐像素相与：上游允许、数值有限、深度为正。
    # nonzero 返回行号在前、列号在后，因此这里写成 v, u；各自形状为 (N,)。
    v, u = np.nonzero(mask & np.isfinite(depth) & (depth > 0))
    # 成对索引取得各有效像素的深度；这里的深度就是相机系 Z，单位米。
    z = depth[v, u]
    # (u-cx)/fx、(v-cy)/fy 给出归一化观察方向，再乘 Z 得到 X、Y。
    x = (u - k[0, 2]) * z / k[0, 0]
    y = (v - k[1, 2]) * z / k[1, 1]
    # 按列拼成 (N, 3)，每行一个三维点；返回时同时保留其原像素 (u, v)。
    points = np.column_stack((x, y, z))
    if not np.isfinite(points).all():
        raise ValueError("反投影坐标超出有限数值范围")
    return points, np.column_stack((u, v)).astype(np.int32)


def transform_points(camera_xyz_m, world_from_optical):
    """用同一份外参，把相机系点的坐标改写为世界系坐标。

    参数：
        camera_xyz_m：形状 (N, 3) 的有限数值数组，每行为相机光学系中的
            (X, Y, Z)，单位米；通常来自 backproject_depth() 的第一个返回值。
        world_from_optical：形状 (4, 4) 的齐次变换矩阵，方向为“相机光学系
            到世界系”，结构为 [[R, t], [0, 0, 0, 1]]。
            左上角 R 为 (3, 3) 旋转矩阵，无长度单位；右上角 t 为 (3,) 平移，
            表示相机原点在世界系中的位置，单位米。R 须正交且行列式为 +1，
            因而不允许缩放或镜像。

    返回：
        形状 (N, 3) 的浮点数组，逐行对应输入点的世界系坐标，单位米。
        点的顺序和物理位置不变，只改变坐标的描述方式。

    异常：
        ValueError：点坐标或变换矩阵的形状、有限性、刚体变换条件不符。

    单点公式为 p_world = R @ p_camera + t。世界系由外参定义，接入机械臂前
    还须核对它与基座坐标系的关系，不能直接把两者当作同一个坐标系。
    """
    points = np.asarray(camera_xyz_m, dtype=float)
    transform = np.asarray(world_from_optical, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("点须为有限的 N×3 数组")
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8, rtol=0)):
        raise ValueError("world_from_optical 须为有限的 4×4 齐次矩阵")
    rotation = transform[:3, :3]  # 取左上角的 (3, 3) 旋转部分 R。
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or abs(np.linalg.det(rotation) - 1) > 1e-6):
        raise ValueError("world_from_optical 的旋转须正交且为右手系")
    # points 将各点存为行向量，因此批量旋转写成 points @ R.T；不是求逆变换。
    # transform[:3, 3] 是平移 t，NumPy 会将它分别加到每一行旋转后的坐标上。
    return points @ rotation.T + transform[:3, 3]


def load_observation(folder):
    """读取一帧 RGB-D 与标定，重建点云并保留像素、颜色和来源对应。

    参数：
        folder：str 或 pathlib.Path，观测目录。目录须包含：
            observation.json：来源、时间、图像宽高、内参、相机到世界系外参；
            rgb.png：与深度逐像素配准的彩色图；
            depth.npz：包含形状 (H, W) 的 depth_m 和同形状 bool 数组 valid。
            元数据须明确 depth_unit="m"、depth_convention="optical_z"，
            图像须已去畸变且符合无斜切针孔模型。

    返回：
        PointCloudObservation，包含有效点的相机坐标、世界坐标、RGB、原像素
        索引、观测元数据和实际读取文件的摘要。所有逐点数组顺序一致。

    异常：
        ValueError：来源、时间、单位、分辨率、摘要或几何数据不符合约定，
            或本帧没有有效点。缺失文件、字段或文件解析失败时传递原异常。

    仅读取上述三个文件；evaluation_only.npz 和仿真物体真值不参与重建。
    """
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
    # 先由像素与深度生成相机系点，同时记住每个点来自原图的哪个像素。
    camera, pixel_uv = backproject_depth(depth, valid, metadata["camera_matrix"])
    if not len(camera):
        raise ValueError("没有可重建的有效深度")
    # 再将同一批点变换到世界系；用 (行 v, 列 u) 取出一一对应的 RGB。
    world = transform_points(camera, metadata["world_from_optical"])
    colors = rgb[pixel_uv[:, 1], pixel_uv[:, 0]].copy()
    return PointCloudObservation(camera, world, colors, pixel_uv, metadata, input_sha256)


def write_ply(path, xyz_m, rgb):
    """将完整彩色点云保存为二进制小端 PLY，不抽样或改变坐标系。

    参数：
        path：str 或 pathlib.Path，待创建的 PLY 文件路径；父目录须已存在。
            采用独占创建模式，已有同名文件不会被覆盖。
        xyz_m：形状 (N, 3) 的有限数值数组，每行为 (X, Y, Z)，单位米。
            相机系或世界系均可，但本函数不执行坐标变换；坐标系由调用方说明。
        rgb：形状 (N, 3)、类型 uint8 的颜色数组，依次为红、绿、蓝，
            取值 0～255，第 i 行颜色须对应 xyz_m 的第 i 个点。

    返回：
        None。文件按点保存 float32 的 X、Y、Z 与 uint8 的红、绿、蓝。

    异常：
        ValueError：形状、颜色类型、坐标有限性不符或坐标超出 float32 范围；
        FileExistsError：输出文件已存在。其他写入错误向上传递。
    """
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
