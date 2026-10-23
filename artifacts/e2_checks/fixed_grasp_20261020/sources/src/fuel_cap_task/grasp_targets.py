"""把固定手盖关系放到当前盖体上，计算手根与法兰目标，不求 IK 或执行运动。

A_from_B 将 B 系坐标转换到 A 系；位置单位为米，输入四元数采用 XYZW。
盖体系是 cap_grip_center，手根是 base，法兰是 wrist3_Link，臂基座是 base_link。
"""

from dataclasses import asdict, dataclass

import numpy as np

from .grasp_search import pose_arrays
from .interfaces import CapEstimate, Pose, TaskFailure
from .validation import finite_real, validate_estimate


def transform(p, r):
    """复用原 IK 实现，把位置（m）和 3×3 旋转拼成 4×4 齐次变换。"""
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = r, p
    return t


def inverse(t):
    """复用原 IK 的刚体逆变换：[R, p] 的逆为 [R.T, -R.T @ p]。"""
    return transform(-t[:3, :3].T @ t[:3, 3], t[:3, :3].T)


@dataclass(frozen=True)
class GraspTargets:
    """同一次计算的 4×4 位姿矩阵；保留中间结果，便于检查坐标链。"""

    world_from_cap: np.ndarray
    world_from_hand: np.ndarray
    world_from_flange: np.ndarray
    base_from_hand: np.ndarray
    base_from_flange: np.ndarray


@dataclass(frozen=True)
class ApproachTargets:
    """沿盖体外向轴退开的世界系接近目标；朝向与最终抓取目标一致。"""

    approach_distance_m: float
    outward_axis_world: np.ndarray
    world_from_pregrasp_hand: np.ndarray
    world_from_pregrasp_flange: np.ndarray


def _rigid_transform(value, name):
    """检查外部提供的安装或基座变换，拒绝非有限值、缩放、反射及错误形状。"""
    t = np.array(value, dtype=float, copy=True)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], rtol=0, atol=1e-9)):
        raise ValueError(f"{name} 必须是有限的 4×4 齐次变换")
    r = t[:3, :3]
    if (not np.allclose(r.T @ r, np.eye(3), rtol=0, atol=1e-6)
            or abs(np.linalg.det(r) - 1) > 1e-6):
        raise ValueError(f"{name} 的旋转必须正交且行列式为 +1")
    return t


def calculate_grasp_targets(
    cap: CapEstimate,
    cap_from_hand: Pose,
    *,
    flange_from_hand: np.ndarray,
    world_from_base: np.ndarray,
) -> GraspTargets:
    """由当前观测、固定相对位姿及装配关系计算空间目标。

    cap 必须在 world 中表达；cap_from_hand 使用 load_fixed_grasp() 返回的
    同名字段，在 cap_grip_center 中表达。两个矩阵由调用方从已更新的模型或
    标定结果提供，不把当前装配的 43.5 mm 写死在计算中，也不假定基座等于世界系。

    无效观测或观测参考系错误抛出 TaskFailure；相对位姿或装配矩阵错误抛出
    ValueError。函数不修改输入；来源和时间继续由调用方与 cap 快照一起保存。
    """
    # 1. 先确认输入的有效性和坐标含义，不能把其他参考系静默当成 world。
    validate_estimate(cap)
    if cap.pose.frame_id != "world":
        raise TaskFailure("抓姿计算要求 world 系 CapEstimate，请先转换观测参考系")
    if not isinstance(cap_from_hand, Pose) or cap_from_hand.frame_id != "cap_grip_center":
        raise ValueError("cap_from_hand 必须是在 cap_grip_center 中表达的 Pose")
    flange_from_hand = _rigid_transform(flange_from_hand, "flange_from_hand")
    world_from_base = _rigid_transform(world_from_base, "world_from_base")

    # 2. 复用现有 XYZW 转换函数；
    world_from_cap = transform(*pose_arrays(asdict(cap.pose)))
    cap_from_hand_matrix = transform(*pose_arrays(asdict(cap_from_hand)))

    # 3. 核心：手根 → 盖体 → 世界。位置满足 p_wh = p_wc + R_wc @ p_ch，
    #    所以盖体系中的手根偏移也会随盖体转动，不能只把两个位置直接相加。
    world_from_hand = world_from_cap @ cap_from_hand_matrix

    # 4. 安装链满足 world_from_hand = world_from_flange @ flange_from_hand。
    #    求法兰时在右侧乘逆；旋转和位置一起处理，不能只沿世界 Z 减安装高度。
    world_from_flange = world_from_hand @ inverse(flange_from_hand)

    # 5. 世界目标转入臂基座系：基座安装有旋转或平移时，同样需要完整逆变换。
    base_from_world = inverse(world_from_base)
    return GraspTargets(
        world_from_cap=world_from_cap,
        world_from_hand=world_from_hand,
        world_from_flange=world_from_flange,
        base_from_hand=base_from_world @ world_from_hand,
        base_from_flange=base_from_world @ world_from_flange,
    )


def calculate_approach_targets(
    targets: GraspTargets,
    *,
    approach_distance_m: float,
) -> ApproachTargets:
    """从已计算的抓取目标外退指定距离（m），不重新读取观测或修改原目标。

    targets 使用 calculate_grasp_targets() 的结果。外退沿当前盖体的局部 +Z，
    实际接近沿反方向；距离必须显式传入有限正数。返回世界系手根及法兰位姿，
    二者平移相同向量，保持安装关系及抓取朝向。无效距离抛出 ValueError。
    """
    if not finite_real(approach_distance_m) or approach_distance_m <= 0:
        raise ValueError("approach_distance_m 必须是以米为单位的有限正数")

    # 1. 第三列是盖体局部 +Z 在世界系中的方向，不能使用手根轴或固定世界轴。
    axis_world = targets.world_from_cap[:3, 2].copy()
    offset_world = float(approach_distance_m) * axis_world

    # 2. 复制原目标，只改平移列；手根和法兰都沿同一世界系向量外退。
    pregrasp_hand = targets.world_from_hand.copy()
    pregrasp_flange = targets.world_from_flange.copy()
    pregrasp_hand[:3, 3] += offset_world
    pregrasp_flange[:3, 3] += offset_world

    return ApproachTargets(
        approach_distance_m=float(approach_distance_m),
        outward_axis_world=axis_world,
        world_from_pregrasp_hand=pregrasp_hand,
        world_from_pregrasp_flange=pregrasp_flange,
    )
