"""检查绕数、有限候选预算和最终选择依据；真实路径证据另行复核。"""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from fuel_cap_task.arm_ik import body_transform
from fuel_cap_task.configuration_selection import (
    CandidateContext, check_joint_endpoint, generate_pool, ordered_candidates,
    run, unique_raw, verify, winding_variants,
)
from fuel_cap_task.regrasp_margin import RegraspContext
from fuel_cap_task.rotation_diagnostic import RotationDiagnostic
from fuel_cap_task.segmented_rotation import digest
from fuel_cap_task.trajectory_replay import ROOT


class ConfigurationSelectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = RegraspContext(ROOT/'artifacts/planning/segmented_single_tol_020')

    def test_winding_variants_stay_in_limits_and_change_one_joint(self):
        limits = np.array([[-2*np.pi, 2*np.pi]]*6)
        variants = list(winding_variants(np.zeros(6), limits))
        self.assertEqual(len(variants), 12)
        for q, origin in variants:
            self.assertEqual(np.count_nonzero(q), 1)
            self.assertEqual(q[origin['joint_index']], origin['turns']*2*np.pi)
            self.assertTrue(np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1]))
        with self.assertRaises(ValueError):
            list(winding_variants([float('nan')]*6, limits))

    def test_raw_dedup_preserves_integer_turns(self):
        q = np.zeros(6)
        rows = [{'arm_qpos_rad': q.tolist()}]
        self.assertFalse(unique_raw(q+.001, rows))
        q[5] += 2*np.pi
        self.assertTrue(unique_raw(q, rows))

    def test_same_fk_does_not_mean_same_winding(self):
        source = RotationDiagnostic(self.context.trajectory)
        q = self.context.after.copy()
        source.set_state(q, self.context.angle)
        pose = body_transform(source.preview.data, 'base').copy()
        q[5] += 2*np.pi
        source.set_state(q, self.context.angle)
        np.testing.assert_allclose(body_transform(source.preview.data, 'base'), pose, atol=1e-12)
        with self.assertRaises(ValueError):
            check_joint_endpoint(q, self.context.after)

    def test_ranking_uses_checked_cap_range_before_static_margin(self):
        def row(label, angle, margin, distance):
            return {'id': label, 'checked_additional_angle_deg': angle,
                    'minimum_joint_margin_rad': margin, 'distance_from_before_rad': distance}
        rows = [row('short', 73., 4., 1.), row('equal', 99., 5., 1.),
                row('unknown', None, 6., 0.), row('long', 135., .2, 10.),
                row('tie_far', 120., 2., 3.), row('tie_near', 120., 2., 1.)]
        self.assertEqual([r['id'] for r in ordered_candidates(rows, 99.)], ['long', 'tie_near', 'tie_far'])
        self.assertEqual(ordered_candidates(rows, None), [])

    def test_endpoint_check_is_raw_finite_and_bounded(self):
        self.assertAlmostEqual(check_joint_endpoint(np.zeros(6), np.full(6, .005)), .005)
        for values in ([0.]*5, [float('nan')]*6, [.02]*6):
            with self.assertRaises(ValueError):
                check_joint_endpoint(values, np.zeros(6))

    def test_failed_ik_seeds_are_logged_without_retry(self):
        def fail(preview, target, seed):
            return np.asarray(seed), np.ones(6), False
        with patch('fuel_cap_task.configuration_selection.solve_continuous', side_effect=fail) as solver:
            pool = generate_pool(self.context)
        self.assertEqual(solver.call_count, 24)
        self.assertEqual(len(pool['ik_attempts']), 24)
        self.assertTrue(all(not row['converged'] for row in pool['ik_attempts']))
        selected = [row for row in pool['candidates'] if row['scan_selected']]
        self.assertLessEqual(len(selected), 12)
        self.assertEqual({label for row in selected for label in row['labels']}, {'before', 'old_after'})
        self.assertTrue(any(row['origin']['kind'] == 'single_joint_winding' for row in pool['candidates']))

    def test_existing_output_rejected_before_search(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            with patch('fuel_cap_task.configuration_selection.RegraspContext') as context:
                with self.assertRaises(ValueError):
                    run(self.context.folder, folder)
                context.assert_not_called()

    def test_candidate_binding_detects_changed_pool(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as folder:
            path = Path(folder)/'pool.json'
            path.write_text('{}')
            binding = {str(path.relative_to(ROOT)): digest(path)}
            node = CandidateContext(self.context, self.context.after, candidate_id='test', bindings=binding, role='candidate')
            self.assertEqual(node.origin('after')['cumulative_angle_deg'], self.context.angle)
            path.write_text('{"changed":true}')
            with self.assertRaises(ValueError):
                CandidateContext(self.context, self.context.after, candidate_id='test', bindings=binding, role='candidate')

    def test_saved_real_selection_when_available(self):
        folder = ROOT/'artifacts/planning/regrasp_selection_single'
        if not (folder/'summary.json').exists():
            self.skipTest('本轮真实 ROS 产物尚未生成')
        summary = verify(folder)
        self.assertTrue(summary['analysis_finished'])
        if summary['selected']:
            self.assertGreater(summary['selected']['checked_range_change_deg'], 0.)
            self.assertLessEqual(summary['selected']['endpoint_joint_error_rad'], .01)
        self.assertFalse(summary['physical_regrasp_verified'])


if __name__ == '__main__':
    unittest.main()
