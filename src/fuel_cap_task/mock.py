"""合成阶段反馈与有限脚本，只验证编排，不驱动模型或硬件。"""

import asyncio
from dataclasses import replace
import time

from .interfaces import CapEstimate, Pose, StopFeedback, TaskFailure, TaskParameters
from .execution_contract import (
    ApproachFeedback, FeedbackContext, GraspFeedback, HoldFeedback, MotionProgress,
    ReleaseFeedback, RetreatFeedback, SupportFeedback, TwistEndReason,
    TwistFeedback, TwistRequest, WithdrawFeedback,
)


def mock_context(detail: str = "合成判断；没有物理测量或判据验证") -> FeedbackContext:
    return FeedbackContext("mock_no_physics", time.time(), "unix", True, detail)


def mock_cap() -> CapEstimate:
    """任意测试坐标，不表示真实场景或标定。"""
    return CapEstimate(Pose("mock_world", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
                       time.time(), "mock", True, 1.0)


class MockEstimator:
    async def estimate(self) -> CapEstimate:
        return mock_cap()


class MockExecutor:
    name = "mock_no_physics"

    def __init__(self, *, fail_at: str | None = None, delay_s: float = 0.01,
                 twist_script: tuple[TwistFeedback, ...] | None = None,
                 support_script: tuple[SupportFeedback, ...] = ()):
        self.fail_at = fail_at
        self.delay_s = delay_s
        self.calls: list[str] = []
        self.holding = False
        self.motion_active = False
        self.last_grasp_channels = None
        self.last_unlock_angle_deg = None
        self.twist_requests: list[TwistRequest] = []
        self.approach_inputs: list[tuple[CapEstimate, MotionProgress]] = []
        self.retreat_inputs: list[CapEstimate] = []
        self._twists = None if twist_script is None else iter(twist_script)
        self._supports = iter(support_script)

    async def _action(self, name: str) -> None:
        self.calls.append(name)
        await asyncio.sleep(self.delay_s)
        if self.fail_at == name:
            raise TaskFailure(f"mock failure at {name}")

    async def check_ready(self, parameters: TaskParameters) -> None:
        # 本后端允许物理参数为空，仅用于软件测试；真实后端必须检查必要输入。
        await self._action("check_ready")

    async def approach(self, cap: CapEstimate, parameters: TaskParameters,
                       progress: MotionProgress) -> ApproachFeedback:
        self.approach_inputs.append((cap, progress))
        self.motion_active = True
        await self._action("approach")
        self.motion_active = False
        return ApproachFeedback(True, True, True, context=mock_context())

    async def grasp(self, parameters: TaskParameters) -> GraspFeedback:
        self.last_grasp_channels = parameters.grasp_channels
        await self._action("grasp")
        self.holding = True
        return GraspFeedback(True, context=mock_context())

    async def twist(self, request: TwistRequest, parameters: TaskParameters) -> TwistFeedback:
        self.twist_requests.append(request)
        self.motion_active = True
        self.last_unlock_angle_deg = parameters.unlock_angle_deg
        await self._action("twist")
        self.motion_active = False
        if self._twists is not None:
            return self._next(self._twists, "twist")
        # 默认演示仅合成一次脱离；参考角度作为样例读数，任务不要求达到它。
        cap = replace(request.cap, source="mock", timestamp_s=time.time())
        return TwistFeedback(TwistEndReason.DETACHED,
                             MotionProgress(request.progress.cap_rotation_deg + parameters.unlock_angle_deg),
                             cap, True, self.holding, True, context=mock_context())

    @staticmethod
    def _next(script, phase):
        try:
            return next(script)
        except StopIteration as exc:
            raise TaskFailure(f"mock {phase} script exhausted") from exc

    async def check_support(self, request: TwistRequest,
                            parameters: TaskParameters) -> SupportFeedback:
        await self._action("check_support")
        # 必须显式提供支撑脚本，不默认批准松手。
        return self._next(self._supports, "support")

    async def release(self, support: SupportFeedback,
                      parameters: TaskParameters) -> ReleaseFeedback:
        if (support.context.valid is not True or support.thread_released is not False
                or support.release_allowed is not True or support.grasp_stable is not True
                or support.motion_stopped is not True):
            raise TaskFailure("mock release permission was not confirmed")
        await self._action("release")
        self.holding = False
        return ReleaseFeedback(True, True, True, context=mock_context())

    async def withdraw(self, cap: CapEstimate, parameters: TaskParameters) -> WithdrawFeedback:
        self.motion_active = True
        await self._action("withdraw")
        self.motion_active = False
        return WithdrawFeedback(True, True, True, True, context=mock_context())

    async def retreat(self, cap: CapEstimate, parameters: TaskParameters) -> RetreatFeedback:
        self.retreat_inputs.append(cap)
        self.motion_active = True
        await self._action("retreat")
        self.motion_active = False
        return RetreatFeedback(True, self.holding, context=mock_context())

    async def hold(self, parameters: TaskParameters) -> HoldFeedback:
        await self._action("hold")
        return HoldFeedback(self.holding, context=mock_context())

    async def stop_and_hold(self) -> StopFeedback:
        await self._action("stop_and_hold")
        self.motion_active = False
        return StopFeedback(True, self.holding, "synthetic stop; existing grasp preserved")


MOCK_SCENARIOS = ("default", "early_detach", "regrasp", "support_unknown", "detach_in_check")


def scenario_executor(scenario: str, **kwargs) -> MockExecutor:
    """有限演示脚本；数值仅是软件夹具，不是新运动参数。"""
    if scenario == "default":
        return MockExecutor(**kwargs)
    cap = mock_cap()

    def twist(rotation, reason):
        return TwistFeedback(reason, MotionProgress(rotation), cap,
                             reason is TwistEndReason.DETACHED, True, True,
                             context=mock_context())

    def support(released, allowed):
        return SupportFeedback(MotionProgress(30), cap, released, allowed, True, True,
                               context=mock_context())

    if scenario == "early_detach":
        return MockExecutor(twist_script=(twist(5, TwistEndReason.DETACHED),), **kwargs)
    boundary = twist(30, TwistEndReason.SEGMENT_BOUNDARY)
    if scenario == "regrasp":
        return MockExecutor(twist_script=(boundary, twist(45, TwistEndReason.DETACHED)),
                            support_script=(support(False, True),), **kwargs)
    if scenario == "support_unknown":
        return MockExecutor(twist_script=(boundary,),
                            support_script=(support(None, None),), **kwargs)
    if scenario == "detach_in_check":
        return MockExecutor(twist_script=(boundary,),
                            support_script=(support(True, False),), **kwargs)
    raise ValueError(f"unknown mock scenario: {scenario}")
