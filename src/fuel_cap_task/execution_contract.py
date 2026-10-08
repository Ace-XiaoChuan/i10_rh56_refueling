"""反馈驱动的任务执行合同；复用位姿和参数，不绑定规划模型来源。"""

from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from .interfaces import CapEstimate, StopFeedback, TaskParameters


@dataclass(frozen=True)
class RunLimits:
    """软件运行预算；不表示允许的机械载荷或运动范围。"""

    max_twist_segments: int
    total_timeout_s: float


@dataclass(frozen=True)
class FeedbackContext:
    """记录判断来源与有效性；时间须和时钟域一起解释。"""

    source: str
    timestamp_s: float
    timestamp_clock: str
    valid: bool
    detail: str = ""


@dataclass(frozen=True, kw_only=True)
class Feedback:
    context: FeedbackContext


@dataclass(frozen=True)
class MotionProgress:
    """相对首次抓稳参考起点的累计盖体运动，单位为度和米。"""

    cap_rotation_deg: float = 0.0
    axial_progress_m: float | None = None


@dataclass(frozen=True)
class TwistRequest:
    cap: CapEstimate
    progress: MotionProgress
    segment_index: int


class TwistEndReason(str, Enum):
    DETACHED = "detached"
    SEGMENT_BOUNDARY = "segment_boundary"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class ApproachFeedback(Feedback):
    at_grasp_pose: bool | None
    arm_stopped: bool | None
    local_control_acquired: bool | None


@dataclass(frozen=True)
class GraspFeedback(Feedback):
    stable: bool | None


@dataclass(frozen=True)
class TwistFeedback(Feedback):
    end_reason: TwistEndReason
    progress: MotionProgress
    cap: CapEstimate
    thread_released: bool | None
    grasp_stable: bool | None
    motion_stopped: bool | None


@dataclass(frozen=True)
class SupportFeedback(Feedback):
    progress: MotionProgress
    cap: CapEstimate
    thread_released: bool | None
    release_allowed: bool | None
    grasp_stable: bool | None
    motion_stopped: bool | None


@dataclass(frozen=True)
class ReleaseFeedback(Feedback):
    released: bool | None
    cap_supported: bool | None
    motion_stopped: bool | None


@dataclass(frozen=True)
class WithdrawFeedback(Feedback):
    at_observation_pose: bool | None
    motion_stopped: bool | None
    planning_control_acquired: bool | None
    cap_supported: bool | None


@dataclass(frozen=True)
class RetreatFeedback(Feedback):
    clear_of_opening: bool | None
    grasp_stable: bool | None


@dataclass(frozen=True)
class HoldFeedback(Feedback):
    stable: bool | None


class Executor(Protocol):
    """任务层只编排；执行器负责有限动作、反馈判断及单一控制权。"""

    name: str

    async def check_ready(self, parameters: TaskParameters) -> None:
        """真实后端在运动前拒绝缺失的物理、判据或控制边界。"""
        ...

    async def approach(self, cap: CapEstimate, parameters: TaskParameters,
                       progress: MotionProgress) -> ApproachFeedback:
        """依据新观测更新目标与场景，到位停稳后交给局部控制。"""
        ...

    async def grasp(self, parameters: TaskParameters) -> GraspFeedback: ...

    async def twist(self, request: TwistRequest, parameters: TaskParameters) -> TwistFeedback:
        """取得本段有限规划边界；持续监测，在事件触发后停稳返回。"""
        ...

    async def check_support(self, request: TwistRequest,
                            parameters: TaskParameters) -> SupportFeedback:
        """保持抓握检查机构支撑；试拉和卸载本身不直接证明允许释放。"""
        ...

    async def release(self, support: SupportFeedback,
                      parameters: TaskParameters) -> ReleaseFeedback:
        """动作前再次核对允许释放条件；失败后不自动重新闭手。"""
        ...

    async def withdraw(self, cap: CapEstimate, parameters: TaskParameters) -> WithdrawFeedback:
        """释放后外退到观测区域，停稳并交回规划控制。"""
        ...

    async def retreat(self, cap: CapEstimate, parameters: TaskParameters) -> RetreatFeedback:
        """使用当前盖体观测，保持抓握取出；不复用最初目标。"""
        ...

    async def hold(self, parameters: TaskParameters) -> HoldFeedback:
        """确认持稳；任务返回后仍维持保持控制。"""
        ...

    async def stop_and_hold(self) -> StopFeedback:
        """先结束活动操作再停止，保留已有抓握，不新增闭合或释放。"""
        ...
