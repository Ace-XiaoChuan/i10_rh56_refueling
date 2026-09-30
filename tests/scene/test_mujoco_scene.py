"""Checks against the actual MJCF, without dynamics or rendering."""

import asyncio
import unittest

import mujoco
import numpy as np

from fuel_cap_task import CapTask, State
from fuel_cap_task.interfaces import TaskParameters
from fuel_cap_task.mock import MockExecutor
from fuel_cap_task.mujoco_scene import MujocoCapScene, HandPreviewMapping


class SceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene = MujocoCapScene()

    def setUp(self):
        self.model = self.scene.model
        self.data = self.scene.data
        mujoco.mj_resetData(self.model, self.data)
        self.scene.hand.apply(self.data, TaskParameters().grasp_channels)

    def check_limits_and_couplings(self):
        for j in self.scene.hand.joint_ids:
            q = self.data.qpos[self.model.jnt_qposadr[j]]
            lower, upper = self.model.jnt_range[j]
            self.assertGreaterEqual(q, lower - 1e-12, self.model.joint(j).name)
            self.assertLessEqual(q, upper + 1e-12, self.model.joint(j).name)
        residual = self.data.efc_pos[self.data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY]
        self.assertEqual(len(residual), 6)
        np.testing.assert_allclose(residual, 0, atol=1e-12)

    def test_requested_pose_and_both_endpoints_satisfy_actual_constraints(self):
        for channels in [TaskParameters().grasp_channels, [0] * 6, [1000] * 6]:
            with self.subTest(channels=channels):
                self.scene.hand.apply(self.data, channels)
                self.check_limits_and_couplings()
                self.assertEqual(self.data.time, 0)

    def test_joint_mapping_and_nonhand_state_are_preserved(self):
        self.data.joint("cap_twist").qpos[0] = 0.7
        self.data.joint("fuel_door_hinge").qpos[0] = 0.3
        self.data.joint("upperArm_joint").qpos[0] = -0.4
        before = self.data.qpos.copy()
        self.scene.hand.apply(self.data, [1000] * 6)
        hand_addresses = self.model.jnt_qposadr[self.scene.hand.joint_ids]
        other_addresses = np.setdiff1d(np.arange(self.model.nq), hand_addresses)
        np.testing.assert_array_equal(self.data.qpos[other_addresses], before[other_addresses])
        self.assertEqual([c.joint for c in self.scene.hand.channels], [
            "pinky_proximal_joint", "ring_proximal_joint", "middle_proximal_joint",
            "index_proximal_joint", "thumb_proximal_pitch_joint", "thumb_proximal_yaw_joint"])
        for c in self.scene.hand.channels:
            self.assertAlmostEqual(self.data.ctrl[c.actuator_id], self.data.qpos[self.model.jnt_qposadr[c.joint_id]])

    def test_reference_offsets_are_included_in_linear_couplings(self):
        # MuJoCo equality is expressed relative to qpos0, not absolute zero.
        driver = self.model.joint("index_proximal_joint").id
        follower = self.model.joint("index_intermediate_joint").id
        addresses = self.model.jnt_qposadr[[driver, follower]]
        original = self.model.qpos0[addresses].copy()
        try:
            self.model.qpos0[addresses] = [0.1, 0.2]
            mapping = HandPreviewMapping(self.model)
            mapping.apply(self.data, TaskParameters().grasp_channels)
            self.check_limits_and_couplings()
        finally:
            self.model.qpos0[addresses] = original

    def test_bad_channels_fail_before_modifying_pose(self):
        for values in [[0] * 5, [0, 0, -1, 0, 0, 0], [1001] * 6, [float("nan")] * 6]:
            before = self.data.qpos.copy()
            with self.assertRaises(ValueError):
                self.scene.hand.apply(self.data, values)
            np.testing.assert_array_equal(self.data.qpos, before)

    def test_ground_truth_frame_and_clock(self):
        report = asyncio.run(self.scene.report(TaskParameters().grasp_channels))
        estimate = report["cap_estimate"]
        np.testing.assert_allclose(estimate["pose"]["position_m"], [0, 1.007, 0.4], atol=1e-12)
        np.testing.assert_allclose(report["cap_axis_world"], [0, -1, 0], atol=1e-12)
        np.testing.assert_allclose(report["grip_axis_world"], [0, 0, 1], atol=1e-12)
        self.assertEqual(estimate["source"], "simulation_ground_truth")
        self.assertEqual(estimate["timestamp_clock"], "simulation")
        self.assertIsNone(estimate["confidence"])
        self.assertEqual(estimate["timestamp_s"], self.data.time)

    def test_estimate_tracks_cap_rotation_and_is_a_snapshot(self):
        before = asyncio.run(self.scene.estimator.estimate())
        self.data.joint("cap_twist").qpos[0] = np.pi / 2
        self.data.time = 2.5
        report = asyncio.run(self.scene.report(TaskParameters().grasp_channels))
        np.testing.assert_allclose(report["grip_axis_world"], [-1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(report["cap_axis_world"], [0, -1, 0], atol=1e-12)
        self.assertEqual(report["cap_estimate"]["timestamp_s"], 2.5)
        self.assertEqual(before.timestamp_s, 0)
        self.assertNotEqual(before.pose.orientation_xyzw, report["cap_estimate"]["pose"]["orientation_xyzw"])

    def test_estimate_follows_changed_scene_position_and_orientation(self):
        mount = self.model.body("inspection_tank_mount")
        original_pos, original_quat = mount.pos.copy(), mount.quat.copy()
        before = asyncio.run(self.scene.estimator.estimate())
        rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        try:
            mount.pos[:] += [0.2, -0.1, 0.3]
            mount.quat[:] = [np.sqrt(0.5), 0, 0, np.sqrt(0.5)]
            report = asyncio.run(self.scene.report(TaskParameters().grasp_channels))
            expected = mount.pos + rotation @ (np.array(before.pose.position_m) - original_pos)
            np.testing.assert_allclose(report["cap_estimate"]["pose"]["position_m"], expected, atol=1e-12)
            np.testing.assert_allclose(report["cap_axis_world"], [1, 0, 0], atol=1e-12)
        finally:
            mount.pos[:] = original_pos
            mount.quat[:] = original_quat

    def test_estimator_can_feed_task_interface_with_explicit_mock_execution(self):
        executor = MockExecutor(delay_s=0)
        task = CapTask(self.scene.estimator, executor, step_timeout_s=1, stop_timeout_s=1)
        result = asyncio.run(task.run())
        self.assertEqual(result.state, State.COMPLETED)
        self.assertEqual(result.backend, "mock_no_physics")
        event = next(e for e in task.events if e.state == State.ESTIMATE and e.kind == "feedback")
        self.assertEqual(event.data["source"], "simulation_ground_truth")
        self.assertEqual(self.data.time, 0)


if __name__ == "__main__":
    unittest.main()
