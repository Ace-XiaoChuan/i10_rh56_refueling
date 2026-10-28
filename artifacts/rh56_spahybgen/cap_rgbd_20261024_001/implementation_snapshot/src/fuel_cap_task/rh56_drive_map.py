"""RH56 仿真驱动到完整关节的线性展开；不换算硬件通道指令。"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class RH56DriveMap:
    """列为六个驱动，行为完整运动学关节；偏移含模型参考位置。"""

    driver_names: tuple
    joint_names: tuple
    matrix: np.ndarray
    offset: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    full_lower: np.ndarray
    full_upper: np.ndarray

    @classmethod
    def from_dict(cls, data):
        """读取派生元数据，拒绝维数、非有限值或倒置区间。"""
        result = cls(tuple(data["driver_names"]), tuple(data["joint_names"]),
                     *(np.asarray(data[key], dtype=float) for key in
                       ("matrix", "offset", "lower", "upper", "full_lower", "full_upper")))
        n, k = len(result.joint_names), len(result.driver_names)
        if k != 6 or len(set(result.driver_names)) != k or len(set(result.joint_names)) != n:
            raise ValueError("expected six unique drivers and unique full joint names")
        if result.matrix.shape != (n, k) or result.offset.shape != (n,):
            raise ValueError("invalid affine drive mapping dimensions")
        if result.lower.shape != (k,) or result.upper.shape != (k,):
            raise ValueError("invalid driver limits")
        if result.full_lower.shape != (n,) or result.full_upper.shape != (n,):
            raise ValueError("invalid full joint limits")
        arrays = (result.matrix, result.offset, result.lower, result.upper,
                  result.full_lower, result.full_upper)
        if not all(np.isfinite(x).all() for x in arrays):
            raise ValueError("non-finite mapping")
        if np.any(result.lower >= result.upper) or np.any(result.full_lower >= result.full_upper):
            raise ValueError("empty or inverted joint range")
        for i, name in enumerate(result.driver_names):
            row = result.joint_names.index(name)
            if not np.array_equal(result.matrix[row], np.eye(k)[i]) or result.offset[row] != 0:
                raise ValueError("driver row is not an identity mapping")
        # 线性区间的极值可直接算出，保证整个驱动盒都满足从动关节限位。
        extrema_a = result.matrix * result.lower
        extrema_b = result.matrix * result.upper
        minimum = np.minimum(extrema_a, extrema_b).sum(axis=1) + result.offset
        maximum = np.maximum(extrema_a, extrema_b).sum(axis=1) + result.offset
        if np.any(minimum < result.full_lower - 1e-9) or np.any(maximum > result.full_upper + 1e-9):
            raise ValueError("driver range permits a follower limit violation")
        return result

    def expand(self, drivers):
        """输入 (...,6) rad，输出 (...,N) rad；不隐式裁剪或修复候选。"""
        q = np.asarray(drivers)
        if q.shape[-1] != 6 or not np.isfinite(q).all():
            raise ValueError("invalid driver array")
        return q @ self.matrix.T + self.offset

    def validate(self, drivers, full=None, tolerance=1e-6):
        """硬检查；返回逐候选有效性，保留失败候选而非替换其数值。"""
        q = np.asarray(drivers)
        expected = self.expand(q)
        actual = expected if full is None else np.asarray(full)
        if actual.shape != expected.shape:
            raise ValueError("full joint shape mismatch")
        return (np.isfinite(actual).all(axis=-1)
                & (q >= self.lower - tolerance).all(axis=-1)
                & (q <= self.upper + tolerance).all(axis=-1)
                & (actual >= self.full_lower - tolerance).all(axis=-1)
                & (actual <= self.full_upper + tolerance).all(axis=-1)
                & (np.abs(actual - expected) <= tolerance).all(axis=-1))
