"""检查绕估计盖轴的目标生成、关节轴方向及首个失败后的停止行为。"""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from fuel_cap_task.arm_ik import body_transform, inverse, transform
from fuel_cap_task.grasp_search import rotation
from fuel_cap_task.rotation_diagnostic import hand_target, RotationDiagnostic, RotationReplay

ROOT = Path(__file__).resolve().parents[2]
TRAJECTORY = ROOT/'artifacts/planning/random_single/approach.json'


class RotationDiagnosticTests(unittest.TestCase):
    def test_off_axis_hand_orbits_cap_and_keeps_relative_transform(self):
        cap = transform(np.array([.3, -.7, .6]), rotation(0, .5) @ rotation(1, -.2))
        relative = transform(np.array([.08, .03, .05]), rotation(1, .7))
        hand = hand_target(cap, relative, np.pi/2)
        expected_cap = cap @ transform(np.zeros(3), rotation(2, np.pi/2))
        np.testing.assert_allclose(inverse(expected_cap) @ hand, relative, atol=1e-12)
        local_position = (inverse(cap) @ hand)[:3, 3]
        np.testing.assert_allclose(local_position, [-.03, .08, .05], atol=1e-12)
        self.assertGreater(np.linalg.norm(hand[:3, 3]-(cap @ relative)[:3, 3]), .05)

    def test_actual_model_cap_joint_matches_estimated_axis(self):
        diagnostic = RotationDiagnostic(TRAJECTORY)
        diagnostic.set_state(diagnostic.q0, 30.)
        # 模型参考标记与盖体系的轴定义已统一，可直接比较完整位姿。
        actual = body_transform(diagnostic.preview.data, 'cap_grip_center')
        expected = diagnostic.cap0 @ transform(np.zeros(3), rotation(2, np.deg2rad(30)))
        np.testing.assert_allclose(actual[:3, 3], expected[:3, 3], atol=1e-12)
        np.testing.assert_allclose(actual[:3, 2], expected[:3, 2], atol=1e-12)
        np.testing.assert_allclose(actual[:3, :3], expected[:3, :3], atol=1e-12)
        np.testing.assert_array_equal(diagnostic.preview.data.qpos[diagnostic.preview.addresses], diagnostic.q0)

    def test_jump_stops_without_new_seed_or_retry(self):
        diagnostic = RotationDiagnostic(TRAJECTORY)
        with tempfile.TemporaryDirectory() as folder:
            with patch('fuel_cap_task.rotation_diagnostic.solve_continuous',
                       return_value=(diagnostic.q0 + .2, np.zeros(6), True)) as solve:
                result = diagnostic.run(Path(folder)/'diagnostic.json')
            self.assertEqual(solve.call_count, 1)
            self.assertEqual(result['failure']['kind'], 'joint_branch_jump')
            self.assertEqual(result['failure']['angle_deg'], 1.)
            self.assertEqual(result['mujoco_prefix_angle_deg'], 0.)
            self.assertEqual(len(result['samples']), 1)
            saved = json.loads((Path(folder)/'diagnostic.json').read_text())
            self.assertTrue(saved['analysis_finished'])
            self.assertEqual(saved['failure'], result['failure'])

    def test_saved_prefix_preserves_hand_and_rejects_changed_reports(self):
        folder = ROOT/'artifacts/planning/rotation_single'
        replay = RotationReplay(folder/'diagnostic.json', folder/'fcl_validation.json')
        self.assertLess(replay.angles[-1], replay.diagnostic['target_angle_deg'])
        self.assertIsNotNone(replay.diagnostic['failure'])
        mask = np.ones(replay.model.nq, dtype=bool)
        mask[replay.source.preview.addresses] = False
        mask[replay.source.cap_address] = False
        for fraction in (0., .3, 1.):
            replay.set_progress(fraction)
            np.testing.assert_array_equal(replay.data.qpos[mask], replay.source.start[mask])
        with tempfile.TemporaryDirectory() as tmp:
            diagnostic = copy.deepcopy(replay.diagnostic)
            diagnostic['samples'][0]['angle_deg'] += 1
            path = Path(tmp)/'diagnostic.json'
            path.write_text(json.dumps(diagnostic))
            with self.assertRaises(ValueError):
                RotationReplay(path, folder/'fcl_validation.json')
            fcl = copy.deepcopy(replay.fcl)
            fcl['passed_samples'] = len(replay.diagnostic['samples'])+1
            path = Path(tmp)/'fcl.json'
            path.write_text(json.dumps(fcl))
            with self.assertRaises(ValueError):
                RotationReplay(folder/'diagnostic.json', path)


if __name__ == '__main__':
    unittest.main()
