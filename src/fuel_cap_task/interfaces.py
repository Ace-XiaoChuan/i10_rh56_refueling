"""Task-level requests and feedback; no joint commands or simulator objects."""

from dataclasses import dataclass
from typing import Protocol


class TaskFailure(Exception):
    """An expected execution failure, with a reason suitable for the operator."""


@dataclass(frozen=True)
class Pose:
    """在 frame_id 坐标系下表达的位姿；位置单位为米，四元数顺序为 XYZW。"""

    frame_id: str
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class CapEstimate:
    """当前盖体位姿的快照，连同来源、采样时间和有效性一起传递。

    盖体系 +X 沿握梁、+Z 朝开口外侧，提供方负责统一轴向及对称性。
    timestamp_s 须结合 timestamp_clock 解释；valid=False 时不能生成运动目标。
    """

    pose: Pose
    timestamp_s: float
    source: str  # 例如 mock、simulation_ground_truth 或 vision。
    valid: bool
    confidence: float | None = None
    timestamp_clock: str = "unix"


@dataclass(frozen=True)
class TaskParameters:
    # User-confirmed values, mirrored from doc/assembly_contract.md.
    grasp_channels: tuple[int, int, int, int, int, int] = (0, 0, 440, 390, 660, 0)
    # Positive about cap-local outward +Z: CCW viewed from the hand.
    unlock_angle_deg: float = 225.0
    # Unconfirmed physical inputs stay unset. Real executors must check the
    # inputs they need in check_ready(), before starting any motion.
    thread_lead_m_per_turn: float | None = None
    thread_travel_m: float | None = None
    grasp_pose_in_cap: Pose | None = None
    approach_channels: tuple[int, int, int, int, int, int] | None = None
    retreat_distance_m: float | None = None
    hold_duration_s: float | None = None
    rotation_tolerance_deg: float | None = None


@dataclass(frozen=True)
class ApproachFeedback:
    at_grasp_pose: bool
    arm_stopped: bool
    local_control_acquired: bool


@dataclass(frozen=True)
class GraspFeedback:
    stable: bool


@dataclass(frozen=True)
class TwistFeedback:
    # Measured/estimated cap motion, not commanded wrist motion.
    cap_rotation_deg: float
    rotation_reached: bool
    thread_released: bool
    grasp_stable: bool
    axial_progress_m: float | None = None


@dataclass(frozen=True)
class RetreatFeedback:
    clear_of_opening: bool
    grasp_stable: bool


@dataclass(frozen=True)
class HoldFeedback:
    stable: bool


@dataclass(frozen=True)
class StopFeedback:
    # None means unknown/unconfirmed, never assumed successful.
    motion_stopped: bool | None
    grasp_held: bool | None
    detail: str = ""


class CapEstimator(Protocol):
    async def estimate(self) -> CapEstimate:
        """返回带来源、时间和有效性的当前盖体位姿。"""
        ...


class Executor(Protocol):
    """Operations must yield to asyncio and support cancellation.

    Cancellation ends the operation coroutine; stop_and_hold then requests and
    confirms controller stop. Controller ownership lives here, not in the task.
    Successful hold() leaves holding control active after the task returns.
    """

    name: str

    async def check_ready(self, parameters: TaskParameters) -> None:
        """Check parameters, scene and controllers; raise TaskFailure if unready.

        Physical adapters must reject missing required motion/contact inputs.
        """
        ...

    async def approach(self, cap: CapEstimate, parameters: TaskParameters) -> ApproachFeedback:
        """Plan/approach, stop coarse motion and hand over to local control."""
        ...

    async def grasp(self, parameters: TaskParameters) -> GraspFeedback: ...

    async def twist(self, cap: CapEstimate, parameters: TaskParameters) -> TwistFeedback: ...

    async def retreat(self, cap: CapEstimate, parameters: TaskParameters) -> RetreatFeedback: ...

    async def hold(self, parameters: TaskParameters) -> HoldFeedback:
        """Observe stable holding for the configured physical acceptance period."""
        ...

    async def stop_and_hold(self) -> StopFeedback:
        """Stop further motion, preserve existing grasp; never close or release anew."""
        ...
