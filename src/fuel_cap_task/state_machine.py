"""One task run, with explicit feedback gates and a single stop path."""

import asyncio
from dataclasses import asdict, dataclass
from enum import Enum
import logging
import time
from typing import Awaitable, Callable, TypeVar

from .interfaces import (
    ApproachFeedback, CapEstimator, Executor, GraspFeedback, HoldFeedback,
    RetreatFeedback, StopFeedback, TaskFailure, TaskParameters, TwistFeedback,
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
        step_timeout_s: float,
        stop_timeout_s: float,
    ):
        if not all(finite_real(value) and value > 0 for value in (step_timeout_s, stop_timeout_s)):
            raise ValueError("timeouts must be finite and positive")
        self.estimator = estimator
        self.executor = executor
        self.parameters = parameters or TaskParameters()
        self.step_timeout_s = step_timeout_s
        self.stop_timeout_s = stop_timeout_s
        self.state = State.IDLE
        self.events: list[Event] = []
        self.result: TaskResult | None = None
        self._cancel = asyncio.Event()
        self._started_at = 0.0

    def cancel(self) -> None:
        """Request cancellation on the task's event loop; no recovery or release."""
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
            result = await asyncio.wait_for(operation(*args), self.step_timeout_s)
        except asyncio.TimeoutError as exc:
            raise TaskFailure(f"{state.value} timed out") from exc
        if state == State.ESTIMATE:
            validate_estimate(result)
        elif state == State.CHECK_READY:
            if result is not None:
                raise TaskFailure("interface contract: check_ready must return None")
        else:
            expected = {
                State.APPROACH: ApproachFeedback, State.GRASP: GraspFeedback,
                State.TWIST: TwistFeedback, State.RETREAT: RetreatFeedback,
                State.HOLD: HoldFeedback,
            }[state]
            validate_feedback(result, expected)
        self._record("feedback", **(asdict(result) if result is not None else {}))
        return result

    async def _sequence(self) -> None:
        p = self.parameters
        await self._step(State.CHECK_READY, self.executor.check_ready, p)
        cap = await self._step(State.ESTIMATE, self.estimator.estimate)
        approach = await self._step(State.APPROACH, self.executor.approach, cap, p)
        if not (approach.at_grasp_pose and approach.arm_stopped and approach.local_control_acquired):
            raise TaskFailure("approach or control handover was not confirmed")
        grasp = await self._step(State.GRASP, self.executor.grasp, p)
        if not grasp.stable:
            raise TaskFailure("stable grasp was not confirmed")
        twist = await self._step(State.TWIST, self.executor.twist, cap, p)
        if not (twist.rotation_reached and twist.thread_released and twist.grasp_stable):
            raise TaskFailure("cap rotation, thread release or stable grasp was not confirmed")
        retreat = await self._step(State.RETREAT, self.executor.retreat, cap, p)
        if not (retreat.clear_of_opening and retreat.grasp_stable):
            raise TaskFailure("cap clearance or stable grasp was not confirmed")
        hold = await self._step(State.HOLD, self.executor.hold, p)
        if not hold.stable:
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
        self._record("request", backend=self.executor.name, parameters=asdict(self.parameters))
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
                await sequence  # propagate execution failures to the single stop path
                self._transition(State.COMPLETED)
                self.result = TaskResult(self.state, "completed; executor retains holding control", self.executor.name)
        except asyncio.CancelledError:
            # Also handle direct cancellation of run(), e.g. a future ROS adapter.
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
