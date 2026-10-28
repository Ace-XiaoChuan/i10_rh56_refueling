"""读取 ASCII 或未压缩二进制 PCD，保留文件坐标、字段和点序。"""

from dataclasses import dataclass
import hashlib
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class PcdCloud:
    """逐点数组保持原始顺序，非有限坐标也保留，供检查器显式统计。

    xyz_raw 为 (N, 3) float64，单位和坐标轴由采集方说明；rgb 为
    (N, 3) uint8 或 None，只表示按 PCL 规则解出的字段，不保证是真实颜色。
    records 保留全部原字段。header 保存 PCD 头，不将 VIEWPOINT 当作机器人外参。
    """

    xyz_raw: np.ndarray
    rgb: np.ndarray | None
    records: np.ndarray
    header: dict
    byte_order: str
    sha256: str


def _ascii_records(payload, dtype, count):
    """按头声明逐行读取文本；整数单独解析，避免颜色或整数大值经过浮点中转。"""
    try:
        lines = payload.decode("ascii").splitlines()
    except UnicodeDecodeError as error:
        raise ValueError("PCD ASCII 载荷包含非 ASCII 字符") from error
    specs = []
    for name in dtype.names:
        field = dtype.fields[name][0]
        base = field.base
        number = int(np.prod(field.shape)) if field.shape else 1
        limits = np.iinfo(base) if base.kind in "iu" else np.finfo(base)
        specs.append((name, field.shape, base.kind, number, limits))
    columns = sum(spec[3] for spec in specs)
    records = np.empty(count, dtype=dtype)
    row = 0
    for line in lines:
        tokens = line.split("#", 1)[0].split()
        if not tokens:
            continue
        if row >= count:
            raise ValueError("PCD ASCII 点数超过头声明")
        if len(tokens) != columns:
            raise ValueError(f"PCD ASCII 第 {row + 1} 点的列数与字段 COUNT 不符")
        cursor = 0
        for name, shape, kind, number, limits in specs:
            values = []
            for token in tokens[cursor:cursor + number]:
                try:
                    value = int(token) if kind in "iu" else float(token)
                except ValueError as error:
                    raise ValueError(f"PCD ASCII 字段 {name} 的数值格式错误") from error
                if kind in "iu" and not limits.min <= value <= limits.max:
                    raise ValueError(f"PCD ASCII 字段 {name} 的整数超出声明范围")
                if kind == "f" and np.isfinite(value) and abs(value) > float(limits.max):
                    raise ValueError(f"PCD ASCII 字段 {name} 的浮点数超出声明范围")
                values.append(value)
            records[name][row] = (np.asarray(values, dtype=dtype.fields[name][0].base).reshape(shape)
                                  if shape else values[0])
            cursor += number
        row += 1
    if row != count:
        raise ValueError("PCD ASCII 点数少于头声明")
    records.setflags(write=False)
    return records


def load_pcd(path, *, byte_order="little"):
    """读取 DATA ascii/binary 的 PCD 0.7，按声明字段检查完整载荷。

    输入为文件路径；byte_order 显式记录字节序，默认 little 适用于常见
    Windows/Linux x86 二进制导出；ASCII 没有文件字节序，参数仅指定数组表示。
    PCD 头没有长度单位或机器人坐标系信息。压缩格式明确报错。
    输出 PcdCloud 不缩放、不滤点、不变换坐标，也不根据 WIDTH 编造像素索引。
    """
    if byte_order not in ("little", "big"):
        raise ValueError("byte_order 须为 little 或 big")
    content = Path(path).read_bytes()
    header, offset = {}, 0
    # 头部有界扫描；不将缺少 DATA 行的二进制载荷误当作文本头。
    while offset < min(len(content), 65536):
        end = content.find(b"\n", offset)
        if end < 0 or end >= 65536:
            break
        try:
            line = content[offset:end].decode("ascii").split("#", 1)[0].strip()
        except UnicodeDecodeError as error:
            raise ValueError("PCD 头须为 ASCII，且须以 DATA 行结束") from error
        offset = end + 1
        if not line:
            continue
        key, *values = line.split()
        key = key.upper()
        if key in header or not values:
            raise ValueError(f"PCD 头重复字段或缺少取值：{key}")
        header[key] = values
        if key == "DATA":
            break
    else:
        raise ValueError("PCD 缺少有界的 DATA 头")
    required = ("VERSION", "FIELDS", "SIZE", "TYPE", "WIDTH", "HEIGHT", "POINTS", "DATA")
    if any(key not in header for key in required):
        raise ValueError("PCD 缺少必需头字段")
    if header["VERSION"] not in (["0.7"], [".7"]):
        raise ValueError("当前读取器仅支持 PCD 0.7")
    if header["DATA"] not in (["binary"], ["ascii"]):
        raise ValueError("当前读取器仅支持 DATA binary 或 DATA ascii，不能读取压缩 PCD")

    fields = header["FIELDS"]
    if len(set(fields)) != len(fields):
        raise ValueError("PCD FIELDS 不能重复")
    try:
        sizes = [int(value) for value in header["SIZE"]]
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        width, height, count = (int(header[key][0]) for key in ("WIDTH", "HEIGHT", "POINTS"))
    except ValueError as error:
        raise ValueError("PCD 尺寸与点数须为整数") from error
    if (any(len(header[key]) != 1 for key in ("WIDTH", "HEIGHT", "POINTS"))
            or min(width, height, count) <= 0 or width * height != count):
        raise ValueError("PCD 点数须为正，且 WIDTH × HEIGHT 须等于 POINTS")
    types = header["TYPE"]
    if not (len(fields) == len(sizes) == len(types) == len(counts)) or min(counts) <= 0:
        raise ValueError("PCD 字段、大小、类型与 COUNT 须逐项对应且 COUNT 为正")
    order = "<" if byte_order == "little" else ">"
    layout = []
    for name, size, kind, number in zip(fields, sizes, types, counts):
        supported = ((kind == "F" and size in (4, 8))
                     or (kind in ("I", "U") and size in (1, 2, 4, 8)))
        if not supported:
            raise ValueError(f"不支持的 PCD 字段类型：{name} {kind}{size}")
        dtype = np.dtype(f"{order}{kind.lower()}{size}")
        layout.append((name, dtype) if number == 1 else (name, dtype, (number,)))
    dtype = np.dtype(layout, align=False)
    if header["DATA"] == ["binary"]:
        if len(content) - offset != count * dtype.itemsize:
            raise ValueError("PCD 二进制载荷长度与头声明不符，文件可能截断或布局不同")
        records = np.frombuffer(content, dtype=dtype, count=count, offset=offset)
    else:
        records = _ascii_records(content[offset:], dtype, count)
    for name in ("x", "y", "z"):
        if name not in fields or records[name].ndim != 1:
            raise ValueError("PCD 须包含标量 x、y、z 字段")
    xyz = np.column_stack([records[name] for name in ("x", "y", "z")]).astype(np.float64)

    # PCL 的颜色是一个 32 位位模式。TYPE F 时须重解释位，不能做数值转整。
    rgb = None
    color_field = "rgb" if "rgb" in fields else "rgba" if "rgba" in fields else None
    if color_field is not None:
        packed = records[color_field]
        if packed.ndim != 1 or packed.dtype.itemsize != 4:
            raise ValueError("打包 rgb/rgba 须为 32 位标量")
        bits = packed.view(np.dtype(f"{order}u4"))
        rgb = np.column_stack(((bits >> 16) & 255, (bits >> 8) & 255, bits & 255)).astype(np.uint8)
    return PcdCloud(xyz, rgb, records, header, byte_order, hashlib.sha256(content).hexdigest())


def load_binary_pcd(path, *, byte_order="little"):
    """保留旧的二进制读取入口；需要兼容 ASCII 时使用 load_pcd。"""
    cloud = load_pcd(path, byte_order=byte_order)
    if cloud.header["DATA"] != ["binary"]:
        raise ValueError("load_binary_pcd 仅支持 DATA binary；ASCII 请使用 load_pcd")
    return cloud
