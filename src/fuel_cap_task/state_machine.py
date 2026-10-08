"""根据有效阶段反馈编排一次取盖任务，所有失败进入统一停止路径。"""

import asyncio
from dataclasses import asdict, dataclass
from enum import Enum
import logging
import time
from typing import Awaitable, Callable, TypeVar

from .interfaces import CapEstimate, CapEstimator, StopFeedback, TaskFailure, TaskParameters
from .execution_contract import (
    ApproachFeedback, Executor, GraspFeedback, HoldFeedback, MotionProgress,
    ReleaseFeedback, RetreatFeedback, RunLimits, SupportFeedback, TwistEndReason,
    TwistFeedback, TwistRequest, WithdrawFeedback,
)
from .validation import finite_real, validate_estimate, validate_feedback

LOG = logging.getLogger(__name__)
T = TypeVar("T")


class State(str, Enum):
    IDLE = "idle"
    CHECK_READY = "check_ready"
    ESTIMATE = "estimate"
    APPROACH = "approach"
    GRASP = "grasp"
    TWIST = "twist"
    CHECK_SUPPORT = "check_support"
    RELEASE = "release"
    WITHDRAW = "withdraw"
    RETREAT = "retreat"
    HOLD = "hold"
    COMPLETED = "completed"
    STOPPING = "stopping"
    WAITING_FOR_OPERATOR = "waiting_for_operator"


@dataclass(frozen=True)
class Event:
    elapsed_s: float
    state: State
    kind: str
    data: dict


@dataclass(frozen=True)
class TaskResult:
    state: State
    reason: str
    backend: str
    stop: StopFeedback | None = None


class CapTask:
    def __init__(
        self,
        estimator: CapEstimator,
        executor: Executor,
        parameters: TaskParameters | None = None,
        *,
        limits: RunLimits,
        step_timeout_s: float,
        stop_timeout_s: float,
    ):
        if not all(finite_real(value) and value > 0 for value in (step_timeout_s, stop_timeout_s)):
            raise ValueError("timeouts must be finite and positive")
        if (not isinstance(limits, RunLimits)
                or type(limits.max_twist_segments) is not int or limits.max_twist_segments <= 0
                or not finite_real(limits.total_timeout_s) or limits.total_timeout_s <= 0):
            raise ValueError("run limits require positive segments and a finite positive timeout")
        self.estimator = estimator
        self.executor = executor
        self.parameters = parameters or TaskParameters()
        self.step_timeout_s = step_timeout_s
        self.stop_timeout_s = stop_timeout_s
        self.limits = limits
        self.progress = MotionProgress()
        self._frame_id: str | None = None
        self.state = State.IDLE
        self.events: list[Event] = []
        self.result: TaskResult | None = None
        self._cancel = asyncio.Event()
        self._started_at = 0.0

    def cancel(self) -> None:
        """请求取消；活动操作结束后停止，不自动恢复或释放。"""
        self._cancel.set()

    def _record(self, kind: str, **data) -> None:
        self.events.append(Event(time.monotonic() - self._started_at, self.state, kind, data))

    def _transition(self, state: State) -> None:
        self.state = state
        self._record("state")

    async def _step(self, state: State, operation: Callable[..., Awaitable[T]], *args) -> T:
        if self._cancel.is_set():
            raise TaskFailure("cancelled")
        self._transition(state)
        try:
            # 总预算跨重新抓握累计；停止回执使用独立预算，不受其耗尽影响。
            remaining = self.limits.total_timeout_s - (time.monotonic() - self._started_at)
            if remaining <= 0:
                raise TaskFailure("task total timeout exhausted")
            result = await asyncio.wait_for(operation(*args), min(self.step_timeout_s, remaining))
        except asyncio.TimeoutError as exc:
            raise TaskFailure(f"{state.value} timed out") from exc
        if state == State.ESTIMATE:
            validate_estimate(result)
            self._check_frame(result)
        elif state == State.CHECK_READY:
            if result is not None:
                raise TaskFailure("interface contract: check_ready must return None")
        else:
            expected = {
                State.APPROACH: ApproachFeedback, State.GRASP: GraspFeedback,
                State.TWIST: TwistFeedback, State.RETREAT: RetreatFeedback,
                State.HOLD: HoldFeedback,
                State.CHECK_SUPPORT: SupportFeedback,
                State.RELEASE: ReleaseFeedback, State.WITHDRAW: WithdrawFeedback,
            }[state]
            validate_feedback(result, expected)
        self._record("feedback", **(asdict(result) if result is not None else {}))
        return result

    async def _sequence(self) -> None:
        p = self.parameters
        await self._step(State.CHECK_READY, self.executor.check_ready, p)
        cap = await self._step(State.ESTIMATE, self.estimator.estimate)
        await self._approach_and_grasp(cap)

        # 每段连续执行到提前脱离或规划边界；参考角度不决定完成。
        for segment_index in range(1, self.limits.max_twist_segments + 1):
            request = TwistRequest(cap, self.progress, segment_index)
            twist = await self._step(State.TWIST, self.executor.twist, request, p)
            self._accept_motion(twist)
            self._require_held_and_stopped(twist)
            cap = twist.cap
            if twist.end_reason is TwistEndReason.DETACHED:
                self._record("decision", action="take_out", reason="confirmed_detachment")
                await self._take_out(cap)
                return
            if twist.end_reason is not TwistEndReason.SEGMENT_BOUNDARY:
                raise TaskFailure("twist end state is unknown")

            # 支撑检查也可能确认提前脱离；此时保持抓握直接取出。
            support = await self._step(
                State.CHECK_SUPPORT, self.executor.check_support,
                TwistRequest(cap, self.progress, segment_index), p)
            self._accept_motion(support)
            self._require_held_and_stopped(support)
            cap = support.cap
            if support.thread_released is True:
                self._record("decision", action="take_out", reason="detachment_during_support_check")
                await self._take_out(cap)
                return
            if support.thread_released is not False or support.release_allowed is not True:
                raise TaskFailure("release permission or cap support was not confirmed")
            if segment_index == self.limits.max_twist_segments:
                raise TaskFailure("twist segment budget exhausted; keep existing grasp")

            # 释放仅属于成功阶段之间的计划动作；后续任何失败均停止，不重新闭手。
            self._record("decision", action="planned_regrasp", segment_index=segment_index)
            released = await self._step(State.RELEASE, self.executor.release, support, p)
            if not all(value is True for value in
                       (released.released, released.cap_supported, released.motion_stopped)):
                raise TaskFailure("release or continued cap support was not confirmed")
            withdrawn = await self._step(State.WITHDRAW, self.executor.withdraw, cap, p)
            if not all(value is True for value in
                       (withdrawn.at_observation_pose, withdrawn.motion_stopped,
                        withdrawn.planning_control_acquired, withdrawn.cap_supported)):
                raise TaskFailure("withdrawal, cap support or control handover was not confirmed")
            cap = await self._step(State.ESTIMATE, self.estimator.estimate)
            await self._approach_and_grasp(cap)

    def _check_frame(self, cap: CapEstimate) -> None:
        """任务中不混合参考系；显式转换由观测提供方负责。"""
        if self._frame_id is None:
            self._frame_id = cap.pose.frame_id
        elif cap.pose.frame_id != self._frame_id:
            raise TaskFailure("interface contract: cap frame changed during task")

    def _accept_motion(self, feedback: TwistFeedback | SupportFeedback) -> None:
        """接收同一起点的累计进度，不把累计值再次相加。"""
        self._check_frame(feedback.cap)
        if feedback.progress.cap_rotation_deg < self.progress.cap_rotation_deg:
            raise TaskFailure("interface contract: cumulative cap rotation regressed")
        self.progress = feedback.progress
        self._record("progress", **asdict(self.progress))

    @staticmethod
    def _require_held_and_stopped(feedback: TwistFeedback | SupportFeedback) -> None:
        if feedback.grasp_stable is not True or feedback.motion_stopped is not True:
            raise TaskFailure("stable grasp or stopped motion was not confirmed")

    async def _approach_and_grasp(self, cap: CapEstimate) -> None:
        p = self.parameters
        approach = await self._step(State.APPROACH, self.executor.approach, cap, p, self.progress)
        if not all(value is True for value in
                   (approach.at_grasp_pose, approach.arm_stopped, approach.local_control_acquired)):
            raise TaskFailure("approach or control handover was not confirmed")
        grasp = await self._step(State.GRASP, self.executor.grasp, p)
        if grasp.stable is not True:
            raise TaskFailure("stable grasp was not confirmed")

    async def _take_out(self, cap: CapEstimate) -> None:
        p = self.parameters
        retreat = await self._step(State.RETREAT, self.executor.retreat, cap, p)
        if retreat.clear_of_opening is not True or retreat.grasp_stable is not True:
            raise TaskFailure("cap clearance or stable grasp was not confirmed")
        hold = await self._step(State.HOLD, self.executor.hold, p)
        if hold.stable is not True:
            raise TaskFailure("stable final holding was not confirmed")

    async def _stop(self, reason: str) -> TaskResult:
        self._record("failure", reason=reason)
        self._transition(State.STOPPING)
        try:
            feedback = await asyncio.wait_for(self.executor.stop_and_hold(), self.stop_timeout_s)
            validate_feedback(feedback, StopFeedback)
        except asyncio.TimeoutError:
            feedback = StopFeedback(None, None, "stop acknowledgement timed out")
        except Exception as exc:
            LOG.exception("Executor stop failed")
            feedback = StopFeedback(None, None, f"{type(exc).__name__}: {exc}")
        self._record("feedback", **asdict(feedback))
        self._transition(State.WAITING_FOR_OPERATOR)
        return TaskResult(self.state, reason, self.executor.name, feedback)

    async def run(self) -> TaskResult:
        if self.state != State.IDLE:
            raise RuntimeError("create a new task after operator reset; a run cannot resume")
        self.state = State.CHECK_READY
        self._started_at = time.monotonic()
        self._record("request", backend=self.executor.name, parameters=asdict(self.parameters),
                     limits=asdict(self.limits))
        if self._cancel.is_set():
            self.result = await self._stop("cancelled")
            return self.result

        sequence = asyncio.create_task(self._sequence())
        cancelled = asyncio.create_task(self._cancel.wait())
        try:
            done, _ = await asyncio.wait((sequence, cancelled), return_when=asyncio.FIRST_COMPLETED)
            if cancelled in done or self._cancel.is_set():
                sequence.cancel()
                await asyncio.gather(sequence, return_exceptions=True)
                self.result = await self._stop("cancelled")
            else:
                await sequence  # 执行异常统一传递到停止路径。
                self._transition(State.COMPLETED)
                self.result = TaskResult(self.state, "completed; executor retains holding control", self.executor.name)
        except asyncio.CancelledError:
            # 未来适配器直接取消 run() 时，也先结束操作再请求停止。
            sequence.cancel()
            await asyncio.gather(sequence, return_exceptions=True)
            self.result = await self._stop("cancelled")
            raise
        except Exception as exc:
            if not isinstance(exc, TaskFailure):
                LOG.exception("Unexpected task failure")
            self.result = await self._stop(f"{type(exc).__name__}: {exc}")
        finally:
            cancelled.cancel()
            await asyncio.gather(cancelled, return_exceptions=True)
        return self.result
