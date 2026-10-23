"""把候选、观测和目标计算结果一起交给预览、MTC 及后续执行适配。"""

from dataclasses import asdict, dataclass, replace

import numpy as np

from .fixed_grasp import FixedGrasp
from .grasp_search import pose_arrays, quaternion
from .grasp_targets import (
    ApproachTargets, GraspTargets, calculate_approach_targets, calculate_grasp_targets,
)
from .interfaces import CapEstimate, Pose, TaskFailure, TaskParameters
from .validation import finite_real, validate_estimate


@dataclass(frozen=True)
class GraspPreparation:
    """同一次候选和盖体观测生成的几何结果；不包含轨迹或执行成功判断。"""

    grasp: FixedGrasp
    cap: CapEstimate
    targets: GraspTargets
    approach: ApproachTargets | None = None

    def with_approach(self, distance_m: float) -> "GraspPreparation":
        """根据已保存的抓取目标增加接近目标，不重新读取候选或观测。"""
        return replace(self, approach=calculate_approach_targets(
            self.targets, approach_distance_m=distance_m,
        ))

    def task_parameters(self, parameters: TaskParameters | None = None) -> TaskParameters:
        """分别绑定接近预型和闭合目标，保留调用方已有的旋拧、拔出等参数。

        E3 后端可连同本对象接收这些参数；此处只准备数据，不调用执行器。
        未知的物理参数继续保持 None。
        """
        return replace(parameters if parameters is not None else TaskParameters(),
                       grasp_pose_in_cap=self.grasp.cap_from_hand,
                       approach_channels=self.grasp.pregrasp_channels,
                       grasp_channels=self.grasp.grasp_channels)

    def approach_record(self) -> dict:
        """序列化接近目标供预览与结果查看；距离和位姿来自同一份计算。"""
        if self.approach is None:
            raise ValueError("请先显式指定接近距离")
        return {name: value.tolist() if isinstance(value, np.ndarray) else value
                for name, value in asdict(self.approach).items()}

    def mtc_fields(self) -> dict:
        """适配现有 MTC 请求：世界系中的手根 base 目标，四元数为 XYZW。

        MTC 已用 setIKFrame("base") 指定手根，因此此处直接发送手根结果。
        法兰目标保留在 targets 中供法兰 IK 或后续执行接口选择。
        """
        if self.approach is None:
            raise ValueError("生成 MTC 请求前必须显式指定接近距离")
        hand = self.targets.world_from_hand
        return {
            "target_frame_id": "world", "target_link": "base",
            "pregrasp_position": self.approach.world_from_pregrasp_hand[:3, 3].tolist(),
            "hand_position": hand[:3, 3].tolist(),
            "hand_orientation_xyzw": quaternion(hand[:3, :3]),
            "outward_axis_world": self.approach.outward_axis_world.tolist(),
            "approach_distance_m": self.approach.approach_distance_m,
            "pregrasp_channels": list(self.grasp.pregrasp_channels),
            "grasp_channels": list(self.grasp.grasp_channels),
            "cap_estimate": asdict(self.cap),
            "candidate_provenance": asdict(self.grasp.provenance),
        }


def prepare_grasp(
    grasp: FixedGrasp,
    cap: CapEstimate,
    *,
    flange_from_hand: np.ndarray,
    world_from_base: np.ndarray,
    approach_distance_m: float | None = None,
) -> GraspPreparation:
    """复用第 4、5 步计算，保留候选来源和观测时间，返回同一次准备的结果。"""
    targets = calculate_grasp_targets(
        cap, grasp.cap_from_hand,
        flange_from_hand=flange_from_hand, world_from_base=world_from_base,
    )
    prepared = GraspPreparation(grasp, cap, targets)
    return prepared if approach_distance_m is None else prepared.with_approach(approach_distance_m)


def check_mtc_fields(request: dict, prepared: GraspPreparation) -> None:
    """回放前核对请求的目标、轴、观测及两组手指指令，拒绝误传或重复变换。

    历史格式未写明参考系和目标 link 时沿用其 world/base 约定；闭合指令
    是新请求增加的字段，存在时必须与候选一致。
    """
    if request.get("target_frame_id", "world") != "world" or request.get("target_link", "base") != "base":
        raise ValueError("MTC 请求必须使用 world 系的手根 base 目标")
    expected = prepared.mtc_fields()
    if (not finite_real(request["approach_distance_m"])
            or request["approach_distance_m"] != expected["approach_distance_m"]):
        raise ValueError("MTC 接近距离与准备结果不一致")
    for name in ("hand_position", "pregrasp_position", "outward_axis_world"):
        value = np.asarray(request[name], dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all() or not np.allclose(
                value, expected[name], atol=1e-12, rtol=0):
            raise ValueError(f"MTC {name} 与当前盖体/候选的计算结果不一致")
    _, rotation = pose_arrays({"position_m": request["hand_position"],
                               "orientation_xyzw": request["hand_orientation_xyzw"]})
    if not np.allclose(rotation, prepared.targets.world_from_hand[:3, :3], atol=1e-12, rtol=0):
        raise ValueError("MTC 手根朝向与当前盖体/候选不一致")
    for name in ("pregrasp_channels", "grasp_channels"):
        if name == "grasp_channels" and name not in request:
            continue
        value = request[name]
        if (not isinstance(value, list) or len(value) != 6
                or any(type(v) is not int for v in value) or value != expected[name]):
            raise ValueError(f"MTC {name} 与固定候选不一致")
    if "cap_estimate" in request:
        record = dict(request["cap_estimate"])
        record["pose"] = Pose(**record["pose"])
        cap = CapEstimate(**record)
        try:
            validate_estimate(cap)
        except TaskFailure as exc:
            raise ValueError(f"MTC 盖体观测无效：{exc}") from exc
        position, rotation = pose_arrays(asdict(cap.pose))
        if (cap.pose.frame_id != "world"
                or not np.allclose(position, prepared.targets.world_from_cap[:3, 3], atol=1e-12, rtol=0)
                or not np.allclose(rotation, prepared.targets.world_from_cap[:3, :3], atol=1e-12, rtol=0)):
            raise ValueError("MTC 盖体观测与当前场景不一致")
    if "candidate_provenance" in request and request["candidate_provenance"] != expected["candidate_provenance"]:
        raise ValueError("MTC 候选来源与当前读取结果不一致")
