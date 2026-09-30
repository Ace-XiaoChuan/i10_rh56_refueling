"""Geometric search contracts checked against the current scene, no dynamics."""

import math
import unittest

import mujoco
import numpy as np

from fuel_cap_task.grasp_search import GraspSearchScene, SearchSettings, pose_arrays, rotation, search
from fuel_cap_task.interfaces import TaskParameters


class GraspSearchTests(unittest.TestCase):
    def setUp(self):
        self.scene = GraspSearchScene(settings=SearchSettings(roll_samples=2, refinement_seeds=2,
                                                             closure_samples=3))

    def test_requested_pose_is_actual_hand_base_in_cap_frame(self):
        scene = self.scene
        p, r = np.array([0.01, -0.04, 0.15]), rotation(0, 0.4) @ rotation(2, -0.7)
        scene.place(p, r)
        cap_p, cap_r = scene.cap_frame()
        np.testing.assert_allclose(scene.data.body('base').xpos, cap_p + cap_r @ p, atol=1e-12)
        np.testing.assert_allclose(scene.data.body('base').xmat.reshape(3, 3), cap_r @ r, atol=1e-12)
        self.assertEqual(scene.closed, TaskParameters().grasp_channels)
        self.assertEqual(scene.data.time, 0)
        self.assertAlmostEqual(scene.data.joint('fuel_door_hinge').qpos[0], math.pi * 3 / 4)

    def test_cap_relative_placement_tracks_moving_tank(self):
        scene = self.scene
        p, r = np.array([0.01, -0.04, 0.15]), rotation(1, 0.4)
        first = scene.evaluate(p, r)
        mount = scene.model.body('inspection_tank_mount')
        mount.pos[:] += [0.3, -0.2, 0.1]
        mount.quat[:] = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
        mujoco.mj_forward(scene.model, scene.data)
        second = scene.evaluate(p, r)
        for a, b in zip(first['contacts'], second['contacts']):
            self.assertAlmostEqual(a['distance_m'], b['distance_m'], places=6)
            np.testing.assert_allclose(a['grip_point_cap_m'], b['grip_point_cap_m'], atol=1e-6)
        self.assertEqual(first['rejection_reasons'], second['rejection_reasons'])

    def test_rigid_pose_cannot_remove_frozen_self_penetration(self):
        for p, r in ((np.array([0, 0, 0.3]), np.eye(3)),
                     (np.array([0.2, 0, 0.3]), rotation(2, 1.2))):
            result = self.scene.evaluate(p, r)
            self.assertEqual(result['status'], 'rejected')
            self.assertIn('hand_self_penetration', result['rejection_reasons'])
            self.assertIn('fingers_not_both_near_grip', result['rejection_reasons'])
            collision = next(c for c in result['penetrations'] if c['kind'] == 'hand_self')
            self.assertLess(collision['distance_m'], -0.001)

    def test_closure_uses_authorized_endpoints_and_restores_frozen_pose(self):
        scene = self.scene
        candidate = scene.evaluate(np.array([0, 0, 0.3]), np.eye(3))
        before = scene.data.qpos.copy()
        closure = scene.closure(candidate['cap_from_hand'])
        self.assertEqual(closure['samples'][0]['channels'], [1000] * 6)
        self.assertEqual(closure['samples'][-1]['channels'], list(scene.closed))
        self.assertFalse(closure['physical_closure_verified'])
        np.testing.assert_allclose(scene.data.qpos, before)
        self.assertEqual(scene.data.time, 0)

    def test_search_returns_rejected_diagnostics_without_claiming_grasp(self):
        report = search(self.scene, keep=2)
        self.assertEqual(report['conclusion'], 'no_geometric_candidate_found')
        self.assertEqual(report['geometrically_feasible_evaluations'], 0)
        self.assertTrue(report['frozen_hand_self_penetrations'])
        self.assertEqual(len(report['candidates']), 2)
        for candidate in report['candidates']:
            self.assertEqual(candidate['status'], 'rejected')
            self.assertIn('hand_self_penetration', candidate['rejection_reasons'])
            p, r = pose_arrays(candidate['cap_from_hand'])
            repeated = self.scene.evaluate(p, r)
            self.assertAlmostEqual(candidate['score_m'], repeated['score_m'], places=8)
        self.assertEqual(self.scene.data.time, 0)


if __name__ == '__main__':
    unittest.main()
