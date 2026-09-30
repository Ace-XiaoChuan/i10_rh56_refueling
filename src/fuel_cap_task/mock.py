"""Synthetic feedback for orchestration tests. No physics, images or hardware."""

import asyncio
import time

from .interfaces import (
    ApproachFeedback, CapEstimate, GraspFeedback, HoldFeedback, Pose,
    RetreatFeedback, StopFeedback, TaskFailure, TaskParameters, TwistFeedback,
)


class MockEstimator:
    async def estimate(self) -> CapEstimate:
        # Arbitrary fixture coordinates, not a scene pose or calibration.
        return CapEstimate(Pose("mock_world", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
                           time.time(), "mock", True, 1.0)


class MockExecutor:
    name = "mock_no_physics"

    def __init__(self, *, fail_at: str | None = None, delay_s: float = 0.01):
        self.fail_at = fail_at
        self.delay_s = delay_s
        self.calls: list[str] = []
        self.holding = False
        self.motion_active = False
        self.last_grasp_channels = None
        self.last_unlock_angle_deg = None

    async def _action(self, name: str) -> None:
        self.calls.append(name)
        await asyncio.sleep(self.delay_s)
        if self.fail_at == name:
            raise TaskFailure(f"mock failure at {name}")

    async def check_ready(self, parameters: TaskParameters) -> None:
        # Deliberately accepts unconfirmed physical inputs: this backend only
        # exercises orchestration and must never be used for physical acceptance.
        await self._action("check_ready")

    async def approach(self, cap: CapEstimate, parameters: TaskParameters) -> ApproachFeedback:
        self.motion_active = True
        await self._action("approach")
        self.motion_active = False
        return ApproachFeedback(True, True, True)

    async def grasp(self, parameters: TaskParameters) -> GraspFeedback:
        self.last_grasp_channels = parameters.grasp_channels
        await self._action("grasp")
        self.holding = True
        return GraspFeedback(True)

    async def twist(self, cap: CapEstimate, parameters: TaskParameters) -> TwistFeedback:
        self.motion_active = True
        self.last_unlock_angle_deg = parameters.unlock_angle_deg
        await self._action("twist")
        self.motion_active = False
        return TwistFeedback(parameters.unlock_angle_deg, True, True, self.holding)

    async def retreat(self, cap: CapEstimate, parameters: TaskParameters) -> RetreatFeedback:
        self.motion_active = True
        await self._action("retreat")
        self.motion_active = False
        return RetreatFeedback(True, self.holding)

    async def hold(self, parameters: TaskParameters) -> HoldFeedback:
        await self._action("hold")
        return HoldFeedback(self.holding)

    async def stop_and_hold(self) -> StopFeedback:
        await self._action("stop_and_hold")
        self.motion_active = False
        return StopFeedback(True, self.holding, "synthetic stop; existing grasp preserved")
