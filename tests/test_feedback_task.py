"""验证新路线分支、重新抓握及无效反馈下禁止动作。"""

import asyncio
from dataclasses import replace
import unittest

from fuel_cap_task import CapTask, State
from fuel_cap_task.execution_contract import (
    MotionProgress, RunLimits, SupportFeedback, TwistEndReason, TwistFeedback,
)
from fuel_cap_task.interfaces import GraspFeedback as LegacyGraspFeedback
from fuel_cap_task.mock import MockEstimator, MockExecutor, mock_cap, mock_context


def twist(rotation=30.0, *, boundary=False, cap=None):
    return TwistFeedback(
        TwistEndReason.SEGMENT_BOUNDARY if boundary else TwistEndReason.DETACHED,
        MotionProgress(rotation), cap or mock_cap(), not boundary, True, True,
        context=mock_context())


def support(rotation=30.0, *, allowed=True, released=False, cap=None):
    return SupportFeedback(MotionProgress(rotation), cap or mock_cap(), released,
                           allowed, True, True, context=mock_context())


class FeedbackTaskTests(unittest.IsolatedAsyncioTestCase):
    def task(self, executor, estimator=None, *, segments=2, total=5.0):
        return CapTask(estimator or MockEstimator(), executor, limits=RunLimits(segments, total),
                       step_timeout_s=1.0, stop_timeout_s=1.0)

    def regrasp_executor(self, **kwargs):
        return MockExecutor(delay_s=0, twist_script=(twist(boundary=True), twist(45)),
                            support_script=(support(),), **kwargs)

    async def test_early_detachment_does_not_require_reference_angle(self):
        executor = MockExecutor(delay_s=0, twist_script=(twist(5),))
        task = self.task(executor, segments=1)
        result = await task.run()
        self.assertEqual(result.state, State.COMPLETED)
        self.assertEqual(task.progress.cap_rotation_deg, 5)
        self.assertEqual(executor.calls, ['check_ready', 'approach', 'grasp', 'twist', 'retreat', 'hold'])
        self.assertTrue(executor.holding)

    async def test_regrasp_uses_new_observation_and_preserves_cumulative_progress(self):
        first = mock_cap()
        second = replace(first, pose=replace(first.pose, position_m=(0.1, 0.2, 0.3)))

        class Estimator:
            def __init__(self):
                self.inputs = iter((first, second))
                self.calls = 0

            async def estimate(self):
                self.calls += 1
                return next(self.inputs)

        estimator = Estimator()
        executor = self.regrasp_executor()
        task = self.task(executor, estimator)
        result = await task.run()
        self.assertEqual(result.state, State.COMPLETED)
        self.assertEqual(executor.calls, [
            'check_ready', 'approach', 'grasp', 'twist', 'check_support',
            'release', 'withdraw', 'approach', 'grasp', 'twist', 'retreat', 'hold'])
        self.assertEqual(estimator.calls, 2)
        self.assertIs(executor.approach_inputs[1][0], second)
        self.assertIs(executor.twist_requests[1].cap, second)
        self.assertEqual(executor.approach_inputs[1][1].cap_rotation_deg, 30)
        self.assertEqual(executor.twist_requests[1].progress.cap_rotation_deg, 30)
        self.assertEqual(executor.twist_requests[1].segment_index, 2)
        self.assertEqual(task.progress.cap_rotation_deg, 45)
        self.assertTrue(executor.holding)

    async def test_take_out_uses_current_feedback_pose(self):
        current = mock_cap()
        current = replace(current, pose=replace(current.pose, position_m=(1, 2, 3)))
        executor = MockExecutor(delay_s=0, twist_script=(twist(cap=current),))
        self.assertEqual((await self.task(executor).run()).state, State.COMPLETED)
        self.assertIs(executor.retreat_inputs[0], current)

    async def test_detachment_during_support_check_skips_release_even_at_last_segment(self):
        current = mock_cap()
        executor = MockExecutor(delay_s=0, twist_script=(twist(boundary=True),),
                                support_script=(support(35, released=True, allowed=False, cap=current),))
        task = self.task(executor, segments=1)
        self.assertEqual((await task.run()).state, State.COMPLETED)
        self.assertEqual(task.progress.cap_rotation_deg, 35)
        self.assertNotIn('release', executor.calls)
        self.assertNotIn('withdraw', executor.calls)
        self.assertIs(executor.retreat_inputs[0], current)

    async def test_unknown_or_denied_support_never_releases(self):
        cases = [support(allowed=None), support(allowed=False),
                 support(allowed=None, released=None)]
        for reply in cases:
            with self.subTest(reply=reply):
                executor = MockExecutor(delay_s=0, twist_script=(twist(boundary=True),),
                                        support_script=(reply,))
                result = await self.task(executor).run()
                self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
                self.assertEqual(executor.calls, ['check_ready', 'approach', 'grasp', 'twist',
                                                  'check_support', 'stop_and_hold'])
                self.assertTrue(result.stop.grasp_held)

    async def test_invalid_or_contradictory_support_never_releases(self):
        base = support()
        cases = [replace(base, thread_released=True), replace(base, thread_released=None),
                 replace(base, release_allowed=1), replace(base, grasp_stable=None),
                 replace(base, motion_stopped=False),
                 replace(base, context=replace(base.context, valid=False)),
                 replace(base, context=replace(base.context, detail=''))]
        for reply in cases:
            with self.subTest(reply=reply):
                executor = MockExecutor(delay_s=0, twist_script=(twist(boundary=True),),
                                        support_script=(reply,))
                result = await self.task(executor).run()
                self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
                self.assertNotIn('release', executor.calls)
                self.assertTrue(executor.holding)

    async def test_unknown_twist_or_unconfirmed_grasp_stop_without_release(self):
        base = twist()
        cases = [replace(base, end_reason=TwistEndReason.UNKNOWN, thread_released=None),
                 replace(base, thread_released=None), replace(base, grasp_stable=False),
                 replace(base, grasp_stable=None), replace(base, motion_stopped=None),
                 replace(base, end_reason='detached'),
                 replace(twist(boundary=True), thread_released=True)]
        for reply in cases:
            with self.subTest(reply=reply):
                executor = MockExecutor(delay_s=0, twist_script=(reply,))
                result = await self.task(executor).run()
                self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
                self.assertEqual(executor.calls[-2:], ['twist', 'stop_and_hold'])
                self.assertNotIn('retreat', executor.calls)

    async def test_feedback_context_and_motion_must_be_valid(self):
        base = twist()
        cases = [replace(base, context=None),
                 replace(base, context=replace(base.context, source=' ')),
                 replace(base, context=replace(base.context, timestamp_clock='')),
                 replace(base, context=replace(base.context, timestamp_s=float('nan'))),
                 replace(base, context=replace(base.context, valid=1)),
                 replace(base, progress=MotionProgress(float('inf'))),
                 replace(base, progress=MotionProgress(-1)),
                 replace(base, progress=MotionProgress(True)),
                 replace(base, progress=MotionProgress(30, float('nan'))),
                 replace(base, cap=replace(base.cap, valid=False))]
        for reply in cases:
            with self.subTest(reply=reply):
                executor = MockExecutor(delay_s=0, twist_script=(reply,))
                result = await self.task(executor).run()
                self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
                self.assertIn('interface contract', result.reason)
                self.assertNotIn('retreat', executor.calls)

    async def test_regressed_progress_in_twist_or_support_stops(self):
        for phase in ('twist', 'support'):
            with self.subTest(phase=phase):
                executor = MockExecutor(delay_s=0,
                                        twist_script=(twist(boundary=True), twist(10)),
                                        support_script=(support(10 if phase == 'support' else 30),))
                result = await self.task(executor).run()
                self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
                self.assertIn('regressed', result.reason)
                self.assertNotIn('retreat', executor.calls)
                if phase == 'support':
                    self.assertNotIn('release', executor.calls)

    async def test_changed_reference_frame_stops_before_next_motion(self):
        for phase in ('twist', 'support', 'estimate'):
            with self.subTest(phase=phase):
                changed = mock_cap()
                changed = replace(changed, pose=replace(changed.pose, frame_id='other'))
                executor = MockExecutor(delay_s=0,
                                        twist_script=(twist(boundary=True, cap=changed if phase == 'twist' else None),),
                                        support_script=(support(cap=changed if phase == 'support' else None),))
                class Estimator(MockEstimator):
                    calls = 0

                    async def estimate(self):
                        self.calls += 1
                        return changed if self.calls > 1 else mock_cap()

                result = await self.task(executor, Estimator()).run()
                self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
                self.assertIn('frame changed', result.reason)
                self.assertEqual(executor.calls.count('approach'), 1)

    async def test_invalid_reobservation_stops_without_reclosing(self):
        class Estimator(MockEstimator):
            calls = 0

            async def estimate(self):
                self.calls += 1
                return replace(mock_cap(), valid=self.calls == 1)

        executor = self.regrasp_executor()
        result = await self.task(executor, Estimator()).run()
        self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
        self.assertEqual(executor.calls[-2:], ['withdraw', 'stop_and_hold'])
        self.assertEqual(executor.calls.count('grasp'), 1)
        self.assertFalse(result.stop.grasp_held)

    async def test_failures_in_new_stages_stop_once_without_recovery(self):
        for phase in ('check_support', 'release', 'withdraw'):
            with self.subTest(phase=phase):
                executor = self.regrasp_executor(fail_at=phase)
                result = await self.task(executor).run()
                self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
                self.assertEqual(executor.calls[-2:], [phase, 'stop_and_hold'])
                self.assertEqual(executor.calls.count('stop_and_hold'), 1)
                self.assertEqual(executor.calls.count('grasp'), 1)
                self.assertEqual(result.stop.grasp_held, phase != 'withdraw')

    async def test_unconfirmed_release_or_withdrawal_has_no_later_actions(self):
        for phase in ('release', 'withdraw'):
            with self.subTest(phase=phase):
                executor = self.regrasp_executor()
                original = getattr(executor, phase)

                async def unconfirmed(*args):
                    reply = await original(*args)
                    return replace(reply, cap_supported=None)

                setattr(executor, phase, unconfirmed)
                result = await self.task(executor).run()
                self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
                self.assertEqual(executor.calls[-2:], [phase, 'stop_and_hold'])
                self.assertFalse(executor.holding)
                self.assertEqual(executor.calls.count('grasp'), 1)

    async def test_second_grasp_failure_does_not_start_second_twist(self):
        class Executor(MockExecutor):
            async def grasp(self, parameters):
                reply = await super().grasp(parameters)
                return replace(reply, stable=False) if self.calls.count('grasp') == 2 else reply

        executor = Executor(delay_s=0, twist_script=(twist(boundary=True), twist(45)),
                            support_script=(support(),))
        result = await self.task(executor).run()
        self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
        self.assertEqual(executor.calls.count('twist'), 1)
        self.assertEqual(executor.calls[-2:], ['grasp', 'stop_and_hold'])

    async def test_last_segment_budget_does_not_release_for_nonexistent_next_segment(self):
        executor = self.regrasp_executor()
        result = await self.task(executor, segments=1).run()
        self.assertIn('budget exhausted', result.reason)
        self.assertNotIn('release', executor.calls)
        self.assertTrue(executor.holding)

    async def test_exhausted_mock_script_does_not_repeat_previous_success(self):
        executor = MockExecutor(delay_s=0, twist_script=(twist(boundary=True),),
                                support_script=(support(),))
        result = await self.task(executor).run()
        self.assertIn('script exhausted', result.reason)
        self.assertEqual(executor.calls.count('twist'), 2)
        self.assertEqual(executor.calls.count('release'), 1)
        self.assertNotIn('retreat', executor.calls)

    async def test_missing_support_script_does_not_default_to_release_permission(self):
        executor = MockExecutor(delay_s=0, twist_script=(twist(boundary=True),))
        result = await self.task(executor).run()
        self.assertIn('support script exhausted', result.reason)
        self.assertNotIn('release', executor.calls)

    async def test_cancel_after_release_never_withdraws_or_recloses(self):
        class Executor(MockExecutor):
            async def release(self, reply, parameters):
                result = await super().release(reply, parameters)
                task.cancel()
                return result

        executor = Executor(delay_s=0, twist_script=(twist(boundary=True),),
                            support_script=(support(),))
        task = self.task(executor)
        result = await task.run()
        self.assertEqual(result.reason, 'cancelled')
        self.assertNotIn('withdraw', executor.calls)
        self.assertFalse(result.stop.grasp_held)

    async def test_total_budget_applies_across_regrasp_stages(self):
        class Executor(MockExecutor):
            async def check_support(self, request, parameters):
                self.calls.append('check_support')
                try:
                    await asyncio.Event().wait()
                finally:
                    self.support_exited = True

        executor = Executor(delay_s=0, twist_script=(twist(boundary=True),))
        executor.support_exited = False
        result = await self.task(executor, total=0.04).run()
        self.assertIn('timed out', result.reason)
        self.assertTrue(executor.support_exited)
        self.assertNotIn('release', executor.calls)
        self.assertTrue(result.stop.grasp_held)

    async def test_legacy_receipt_is_rejected_without_inventing_source(self):
        executor = MockExecutor(delay_s=0)
        original = executor.grasp

        async def legacy(parameters):
            await original(parameters)
            return LegacyGraspFeedback(True)

        executor.grasp = legacy
        result = await self.task(executor).run()
        self.assertEqual(result.state, State.WAITING_FOR_OPERATOR)
        self.assertNotIn('twist', executor.calls)

    async def test_logs_record_feedback_source_decision_and_run_limits(self):
        executor = self.regrasp_executor()
        task = self.task(executor)
        await task.run()
        request = next(e for e in task.events if e.kind == 'request')
        self.assertEqual(request.data['limits']['max_twist_segments'], 2)
        decisions = [e.data['action'] for e in task.events if e.kind == 'decision']
        self.assertEqual(decisions, ['planned_regrasp', 'take_out'])
        sourced = [e for e in task.events if e.kind == 'feedback'
                   and e.state not in (State.ESTIMATE, State.CHECK_READY)]
        self.assertTrue(sourced)
        self.assertTrue(all(e.data['context']['source'] == 'mock_no_physics' for e in sourced))

    def test_run_limits_are_explicit_finite_and_positive(self):
        for limits in (None, RunLimits(0, 1), RunLimits(True, 1), RunLimits(1.5, 1),
                       RunLimits(1, 0), RunLimits(1, float('inf')), RunLimits(1, True)):
            with self.subTest(limits=limits), self.assertRaises(ValueError):
                CapTask(MockEstimator(), MockExecutor(), limits=limits,
                        step_timeout_s=1, stop_timeout_s=1)


if __name__ == '__main__':
    unittest.main()
