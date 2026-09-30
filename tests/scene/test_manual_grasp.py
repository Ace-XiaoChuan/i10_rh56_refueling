"""Manual experiments preserve fixture poses and distinguish interrupted results."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from fuel_cap_task.grasp_contact import ContactExplorer, ContactSettings
from fuel_cap_task.manual_grasp import ManualGrasp


class ManualGraspTests(unittest.TestCase):
    def make_session(self):
        return ManualGrasp(ContactExplorer(ContactSettings(ramp_s=.02, observe_s=.02, tail_s=.01)))

    def test_incremental_closure_initializes_once_and_preserves_fixture(self):
        session = self.make_session()
        s = session.scene
        session.reset({'position_m': [0, 0, .3], 'orientation_xyzw': [0, 0, 0, 1]})
        cap = s.data.site('cap_grip_center').xpos.copy()
        session.start()
        with patch.object(s.hand, 'apply', wraps=s.hand.apply) as initialize:
            while session.run is not None:
                session.advance()
            self.assertEqual(initialize.call_count, 1)
        self.assertEqual(session.status, 'insufficient_terminal_contact')
        self.assertFalse(session.result['physical_grasp_verified'])
        cp, cr = s.cap_frame()
        np.testing.assert_allclose(s.data.body('base').xpos, cp + cr @ [0, 0, .3], atol=1e-12)
        np.testing.assert_allclose(s.data.site('cap_grip_center').xpos, cap)
        np.testing.assert_allclose(session.result['driver_targets_rad'], s.hand.targets(s.closed))

    def test_interrupted_save_load_restores_pregrasp_not_success(self):
        session = self.make_session()
        session.reset({'position_m': [0, 0, .3], 'orientation_xyzw': [0, 0, 0, 1]})
        session.start()
        session.advance()
        session.stop()
        self.assertEqual(session.status, 'interrupted')
        with tempfile.TemporaryDirectory() as tmp:
            path = session.save(tmp)
            saved = json.loads(path.read_text())
            self.assertIsNone(saved['contact_trial'])
            self.assertFalse(saved['physical_grasp_verified'])
            session.load(path)
            self.assertEqual(session.status, 'pregrasp')
            self.assertEqual(session.scene.data.time, 0)
            self.assertEqual(session.pose, saved['cap_from_hand'])
            saved['grasp_channels'][0] += 1
            Path(path).write_text(json.dumps(saved))
            with self.assertRaises(ValueError):
                session.load(path)

    def test_invalid_edit_leaves_applied_candidate_unchanged(self):
        session = self.make_session()
        before = session.scene.data.qpos.copy()
        with self.assertRaises(ValueError):
            session.reset(channels=[1001]*6)
        np.testing.assert_array_equal(before, session.scene.data.qpos)


if __name__ == '__main__':
    unittest.main()
