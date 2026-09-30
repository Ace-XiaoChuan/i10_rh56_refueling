"""Contact exploration separates commanded target from integrated joint state."""

import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from fuel_cap_task.grasp_contact import ContactExplorer, ContactSettings


class ContactTests(unittest.TestCase):
    def test_integration_sets_ctrl_without_overwriting_actual_finger_pose(self):
        explorer = ContactExplorer(ContactSettings(ramp_s=0.05, observe_s=0.05, tail_s=0.02, trials=1))
        scene = explorer.scene
        pose = {'position_m': [0, 0, 0.3], 'orientation_xyzw': [0, 0, 0, 1]}
        with patch.object(scene.hand, 'apply', wraps=scene.hand.apply) as initialize:
            trial = explorer.trial(pose)
        self.assertEqual(initialize.call_count, 1)
        cap_p, cap_r = scene.cap_frame()
        np.testing.assert_allclose(scene.data.body('base').xpos,
                                   cap_p + cap_r @ np.array(pose['position_m']), atol=1e-12)
        self.assertEqual(trial['status'], 'insufficient_terminal_contact')
        self.assertFalse(trial['physical_grasp_verified'])
        self.assertAlmostEqual(trial['time_s'], 0.1, places=10)
        targets = scene.hand.targets(scene.closed)
        np.testing.assert_allclose(trial['driver_targets_rad'], targets)
        self.assertGreater(np.linalg.norm(trial['driver_errors_rad']), 1e-4)
        self.assertEqual(trial['tail_force_mean_n'], [0, 0, 0])
        before = scene.data.qpos.copy()
        candidate = {'cap_from_hand': pose, 'contact_trial': trial}
        explorer.restore(candidate)
        np.testing.assert_allclose(scene.data.qpos, before)
        self.assertEqual(scene.model.njnt, 12)
        self.assertEqual(scene.model.nu, 6)

    def test_fixed_fixture_keeps_door_open_and_cap_in_place(self):
        explorer = ContactExplorer(ContactSettings(ramp_s=0.02, observe_s=0.02, tail_s=0.01, trials=1))
        scene = explorer.scene
        cap_before = scene.data.site('cap_grip_center').xpos.copy()
        door_before = scene.data.body('fuel_door').xmat.copy()
        explorer.trial({'position_m': [0, 0, 0.3], 'orientation_xyzw': [0, 0, 0, 1]})
        np.testing.assert_allclose(scene.data.site('cap_grip_center').xpos, cap_before)
        np.testing.assert_allclose(scene.data.body('fuel_door').xmat, door_before)
        self.assertFalse(np.allclose(door_before, scene.data.body('fuel_tank_body').xmat))
        record = explorer.physics_record()
        self.assertEqual(record['source'], 'inherited_model_parameters_uncalibrated')


if __name__ == '__main__':
    unittest.main()
