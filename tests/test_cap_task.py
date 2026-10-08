"""验证阶段反馈、禁止动作、取消与统一停止行为。"""

import asyncio
from dataclasses import asdict, replace
import json
import unittest

from fuel_cap_task import CapTask, State
from fuel_cap_task.interfaces import StopFeedback, TaskFailure
from fuel_cap_task.execution_contract import (
    ApproachFeedback, GraspFeedback, HoldFeedback, RetreatFeedback, RunLimits,
    MotionProgress, TwistEndReason, TwistFeedback,
)
from fuel_cap_task.mock import MockEstimator, MockExecutor, mock_cap, mock_context


def twist_reply(rotation, *, axial=None, released=True, stable=True):
    return TwistFeedback(TwistEndReason.DETACHED, MotionProgress(rotation, axial),
                         mock_cap(), released, stable, True, context=mock_context())


class BlockingExecutor(MockExecutor):
    def __init__(self, blocked_at="twist"):
        super().__init__(delay_s=0)
        self.blocked_at = blocked_at
        self.entered = asyncio.Event()
        self.exited = asyncio.Event()

    async def _action(self, name):
        if name != self.blocked_at:
            await super()._action(name)
            return
        self.calls.append(name)
        self.entered.set()
        try:
            await asyncio.Event().wait()
        finally:
            self.exited.set()

    async def stop_and_hold(self):
        if self.blocked_at != "stop_and_hold" and not self.exited.is_set():
            raise AssertionError("stop overlaps the cancelled operation")
        return await super().stop_and_hold()


class TaskTests(unittest.IsolatedAsyncioTestCase):
    def make_task(self, executor=None, estimator=None, timeout=1.0):
        return CapTask(estimator or MockEstimator(), executor or MockExecutor(delay_s=0),
                       limits=RunLimits(1, 5.0), step_timeout_s=timeout, stop_timeout_s=timeout)

    async def test_success_keeps_grasp_and_records_source_and_commands(self):
        executor = MockExecutor(delay_s=0)
        task = self.make_task(executor)
        result = await task.run()
        self.assertEqual(result.state, State.COMPLETED)
        self.assertEqual(executor.calls, ["check_ready", "approach", "grasp", "twist", "retreat", "hold"])
        self.assertTrue(executor.holding)
        self.assertFalse(executor.motion_active)
        self.assertEqual(executor.last_grasp_channels, (0, 0, 440, 390, 660, 0))
        self.assertEqual(executor.last_unlock_angle_deg, 225.0)
        self.assertIsNone(task.parameters.thread_travel_m)
        estimate = next(e for e in task.events if e.state == State.ESTIMATE and e.kind == "feedback")
        self.assertEqual(estimate.data["source"], "mock")
        json.dumps([asdict(e) for e in task.events])
        with self.assertRaises(RuntimeError):
            await task.run()

    async def test_failures_stop_once_without_later_actions_or_retries(self):
        sequence = ["check_ready", "approach", "grasp", "twist", "retreat", "hold"]
        for index, phase in enumerate(sequence):
            with self.subTest(phase=phase):
                executor = MockExecutor(fail_at=phase, delay_s=0)
                task = self.make_task(executor)
                result = await task.run()
                self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
                self.assertEqual(executor.calls, sequence[:index + 1] + ["stop_and_hold"])
                self.assertTrue(result.stop.motion_stopped)
                self.assertEqual(executor.holding, index > 2)
                self.assertIn(phase, result.reason)

    async def test_invalid_estimate_never_starts_approach(self):
        class InvalidEstimator(MockEstimator):
            async def estimate(self):
                return replace(await super().estimate(), valid=False)

        executor = MockExecutor(delay_s=0)
        result = await self.make_task(executor, InvalidEstimator()).run()
        self.assertEqual(executor.calls, ["check_ready", "stop_and_hold"])
        self.assertIn("estimate is invalid", result.reason)

    async def test_malformed_estimates_stop_before_approach(self):
        base = await MockEstimator().estimate()
        cases = [
            None, replace(base, valid="true"), replace(base, source=" "),
            replace(base, timestamp_clock=""), replace(base, timestamp_s=float("nan")),
            replace(base, confidence=1.1), replace(base, confidence=True),
            replace(base, pose=replace(base.pose, position_m=(0, 0))),
            replace(base, pose=replace(base.pose, position_m=(0, float("inf"), 0))),
            replace(base, pose=replace(base.pose, orientation_xyzw=(0, 0, 0, 0))),
            replace(base, pose=replace(base.pose, orientation_xyzw=(0, 0, 0, 2))),
        ]
        for estimate in cases:
            with self.subTest(estimate=estimate):
                class Estimator:
                    async def estimate(self):
                        return estimate

                executor = MockExecutor(delay_s=0)
                result = await self.make_task(executor, Estimator()).run()
                self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
                self.assertIn("interface contract", result.reason)
                self.assertEqual(executor.calls, ["check_ready", "stop_and_hold"])

    async def test_simulation_clock_zero_and_negative_unit_quaternion_are_valid(self):
        base = await MockEstimator().estimate()

        class Estimator:
            async def estimate(self):
                return replace(base, timestamp_s=0, timestamp_clock="simulation",
                               source="simulation_ground_truth", confidence=None,
                               pose=replace(base.pose, orientation_xyzw=(0, 0, 0, -1)))

        result = await self.make_task(estimator=Estimator()).run()
        self.assertEqual(result.state, State.COMPLETED)

    async def test_malformed_feedback_cannot_advance(self):
        cases = [
            ("check_ready", GraspFeedback(True, context=mock_context())),
            ("approach", ApproachFeedback(True, "false", True, context=mock_context())),
            ("grasp", HoldFeedback(True, context=mock_context())),
            ("grasp", GraspFeedback(1, context=mock_context())),
            ("twist", twist_reply(float("nan"))),
            ("twist", twist_reply(225, axial=float("inf"))),
            ("retreat", RetreatFeedback(True, "yes", context=mock_context())),
            ("hold", None),
        ]
        sequence = ["check_ready", "approach", "grasp", "twist", "retreat", "hold"]
        for phase, feedback in cases:
            with self.subTest(phase=phase, feedback=feedback):
                executor = MockExecutor(delay_s=0)
                original = getattr(executor, phase)

                async def malformed(*args):
                    await original(*args)
                    return feedback

                setattr(executor, phase, malformed)
                result = await self.make_task(executor).run()
                self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
                self.assertIn("interface contract", result.reason)
                self.assertEqual(executor.calls,
                                 sequence[:sequence.index(phase) + 1] + ["stop_and_hold"])

    async def test_malformed_stop_receipt_becomes_unknown(self):
        for receipt in (None, GraspFeedback(True, context=mock_context()), StopFeedback("yes", True)):
            with self.subTest(receipt=receipt):
                class Executor(MockExecutor):
                    async def stop_and_hold(self):
                        self.calls.append("stop_and_hold")
                        return receipt

                executor = Executor(fail_at="twist", delay_s=0)
                with self.assertLogs("fuel_cap_task.state_machine", level="ERROR"):
                    result = await self.make_task(executor).run()
                self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
                self.assertIn("mock failure at twist", result.reason)
                self.assertIsNone(result.stop.motion_stopped)
                self.assertIsNone(result.stop.grasp_held)
                self.assertIn("interface contract", result.stop.detail)
                self.assertEqual(executor.calls.count("stop_and_hold"), 1)

    def test_timeouts_must_be_finite_positive_numbers(self):
        for invalid in (float("nan"), float("inf"), 0, -1, True, "1"):
            for field in ("step_timeout_s", "stop_timeout_s"):
                with self.subTest(invalid=invalid, field=field):
                    kwargs = dict(step_timeout_s=1, stop_timeout_s=1)
                    kwargs[field] = invalid
                    with self.assertRaises(ValueError):
                        CapTask(MockEstimator(), MockExecutor(), limits=RunLimits(1, 5.0), **kwargs)

    async def test_feedback_not_command_completion_gates_progress(self):
        cases = [
            ("approach", ApproachFeedback(True, False, True, context=mock_context())),
            ("approach", ApproachFeedback(True, True, False, context=mock_context())),
            ("grasp", GraspFeedback(False, context=mock_context())),
            ("twist", twist_reply(225, released=False)),
            ("twist", twist_reply(100, released=None)),
            ("twist", twist_reply(225, stable=False)),
            ("retreat", RetreatFeedback(False, True, context=mock_context())),
            ("hold", HoldFeedback(False, context=mock_context())),
        ]
        for phase, feedback in cases:
            with self.subTest(phase=phase, feedback=feedback):
                executor = MockExecutor(delay_s=0)
                original = getattr(executor, phase)

                async def unconfirmed(*args):
                    await original(*args)
                    return feedback

                setattr(executor, phase, unconfirmed)
                task = self.make_task(executor)
                result = await task.run()
                self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
                self.assertEqual(executor.calls[-2:], [phase, "stop_and_hold"])

    async def test_cancel_during_motion_waits_for_operation_then_stops_holding(self):
        executor = BlockingExecutor()
        task = self.make_task(executor)
        running = asyncio.create_task(task.run())
        await asyncio.wait_for(executor.entered.wait(), 1)
        task.cancel()
        result = await running
        self.assertEqual(result.reason, "cancelled")
        self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
        self.assertTrue(result.stop.grasp_held)
        self.assertEqual(executor.calls[-2:], ["twist", "stop_and_hold"])
        self.assertFalse(executor.motion_active)

    async def test_external_asyncio_cancel_also_stops(self):
        executor = BlockingExecutor()
        task = self.make_task(executor)
        running = asyncio.create_task(task.run())
        await asyncio.wait_for(executor.entered.wait(), 1)
        running.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await running
        self.assertEqual(task.result.state, State.WAITING_FOR_OPERATOR)
        self.assertTrue(task.result.stop.motion_stopped)
        self.assertTrue(executor.holding)

    async def test_cancel_at_stage_boundary_does_not_start_next_motion(self):
        class CancelAfterGrasp(MockExecutor):
            async def grasp(self, parameters):
                feedback = await super().grasp(parameters)
                task.cancel()
                return feedback

        executor = CancelAfterGrasp(delay_s=0)
        task = self.make_task(executor)
        result = await task.run()
        self.assertEqual(result.reason, "cancelled")
        self.assertEqual(executor.calls, ["check_ready", "approach", "grasp", "stop_and_hold"])
        self.assertTrue(result.stop.grasp_held)

    async def test_cancel_before_start_does_not_issue_motion(self):
        executor = MockExecutor(delay_s=0)
        task = self.make_task(executor)
        task.cancel()
        result = await task.run()
        self.assertEqual(executor.calls, ["stop_and_hold"])
        self.assertFalse(result.stop.grasp_held)

    async def test_timeout_cancels_operation_and_stops(self):
        executor = BlockingExecutor()
        result = await self.make_task(executor, timeout=0.02).run()
        self.assertIn("twist timed out", result.reason)
        self.assertTrue(result.stop.motion_stopped)
        self.assertTrue(executor.exited.is_set())
        self.assertNotIn("retreat", executor.calls)

    async def test_stop_timeout_records_unknown_without_recovery(self):
        executor = BlockingExecutor("stop_and_hold")
        executor.fail_at = "twist"
        result = await self.make_task(executor, timeout=0.02).run()
        self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
        self.assertIsNone(result.stop.motion_stopped)
        self.assertIsNone(result.stop.grasp_held)
        self.assertIn("timed out", result.stop.detail)
        self.assertEqual(executor.calls[-2:], ["twist", "stop_and_hold"])

    async def test_stop_error_records_unknown_and_original_failure(self):
        class StopError(MockExecutor):
            async def stop_and_hold(self):
                self.calls.append("stop_and_hold")
                raise OSError("controller disconnected")

        executor = StopError(fail_at="twist", delay_s=0)
        with self.assertLogs("fuel_cap_task.state_machine", level="ERROR"):
            result = await self.make_task(executor).run()
        self.assertIn("mock failure at twist", result.reason)
        self.assertIsNone(result.stop.motion_stopped)
        self.assertIn("controller disconnected", result.stop.detail)
        self.assertEqual(executor.calls.count("stop_and_hold"), 1)

    async def test_unexpected_adapter_error_still_stops(self):
        class BrokenEstimator(MockEstimator):
            async def estimate(self):
                raise ValueError("bad observation")

        executor = MockExecutor(delay_s=0)
        with self.assertLogs("fuel_cap_task.state_machine", level="ERROR"):
            result = await self.make_task(executor, BrokenEstimator()).run()
        self.assertIn("ValueError: bad observation", result.reason)
        self.assertEqual(executor.calls, ["check_ready", "stop_and_hold"])

    async def test_executor_readiness_rejects_missing_physical_inputs_before_motion(self):
        class RequiresTravel(MockExecutor):
            async def check_ready(self, parameters):
                self.calls.append("check_ready")
                if parameters.thread_travel_m is None:
                    raise TaskFailure("thread travel is required for this executor")

        executor = RequiresTravel(delay_s=0)
        result = await self.make_task(executor).run()
        self.assertIn("thread travel is required", result.reason)
        self.assertEqual(executor.calls, ["check_ready", "stop_and_hold"])

    async def test_concurrent_run_is_rejected(self):
        executor = BlockingExecutor("check_ready")
        task = self.make_task(executor)
        running = asyncio.create_task(task.run())
        await asyncio.wait_for(executor.entered.wait(), 1)
        with self.assertRaises(RuntimeError):
            await task.run()
        task.cancel()
        await running


if __name__ == "__main__":
    unittest.main()
