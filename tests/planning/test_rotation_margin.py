"""验证余量含义、失败证据、扫描预算和审计前缀；注入审计不计真实 FCL 验收。"""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from fuel_cap_task.rotation_diagnostic import RotationDiagnostic, RotationScanSettings, joint_margins
from fuel_cap_task.rotation_margin import summarize, write_outputs

ROOT = Path(__file__).resolve().parents[2]
TRAJECTORY = ROOT/'artifacts/planning/random_single/approach.json'


def fixture_audit(path, diagnostic, count=None):
    """只用于测试汇总边界的审计记录，不实际调用 FCL。"""
    count = len(diagnostic['samples']) if count is None else count
    failure = None if count == len(diagnostic['samples']) else {
        'kind': 'fcl_robot_collision', 'angle_deg': diagnostic['samples'][count]['angle_deg'], 'contacts': []}
    return {'mode': 'full_fcl_rotation_prefix_audit_not_execution',
            'diagnostic_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'passed_samples': count, 'checked_samples': count+(failure is not None),
            'clear_prefix_angle_deg': diagnostic['samples'][count-1]['angle_deg'] if count else 0.,
            'failure': failure, 'completed_input_check': failure is None}


class RotationMarginTests(unittest.TestCase):
    def test_joint_distances_preserve_both_bounds_and_unwrapped_angles(self):
        limits = np.array([[-2*np.pi, 2*np.pi]]*6)
        q = np.array([6., 0., 0., 0., 0., 0.])
        a = joint_margins(q, limits)
        b = joint_margins(q-np.array([.1, 0., 0., 0., 0., 0.]), limits)
        self.assertAlmostEqual(a['joint_upper_margin_rad'][0], 2*np.pi-6.)
        self.assertAlmostEqual(a['joint_lower_margin_rad'][0], 2*np.pi+6.)
        self.assertGreater(b['minimum_joint_margin_rad'], a['minimum_joint_margin_rad'])
        q[0] = 7.
        self.assertLess(joint_margins(q, limits)['minimum_joint_margin_rad'], 0.)
        for invalid in ([0.]*5, [float('nan')]*6):
            with self.assertRaises(ValueError):
                joint_margins(invalid, limits)

    def test_scan_grid_keeps_step_and_final_partial_increment(self):
        np.testing.assert_allclose(RotationScanSettings(2.25).angles(), [0., 1., 2., 2.25])
        np.testing.assert_allclose(RotationScanSettings(.1, .5).angles(), [0., .1])
        for maximum, step in ((0., 1.), (-1., 1.), (float('nan'), 1.), (1., 0.), (1., 2.), (True, 1.)):
            with self.assertRaises(ValueError):
                RotationScanSettings(maximum, step)

    def test_nonconvergence_saves_candidate_without_claiming_joint_limit(self):
        source = RotationDiagnostic(TRAJECTORY)
        q, error = source.q0.copy(), np.array([.001, 0., 0., 0., 0., .01])
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/'diagnostic.json'
            with patch('fuel_cap_task.rotation_diagnostic.solve_continuous', return_value=(q, error, False)) as solve:
                report = source.run(output, settings=RotationScanSettings(3.))
            self.assertEqual(solve.call_count, 1)
            self.assertEqual(report['failure']['kind'], 'continuous_ik_not_converged')
            self.assertEqual(report['failure']['arm_qpos_rad'], q.tolist())
            self.assertAlmostEqual(report['failure']['position_error_m'], .001)
            self.assertGreater(report['failure']['minimum_joint_margin_rad'], 0.)
            self.assertEqual(report['mujoco_prefix_angle_deg'], 0.)
            original = output.read_bytes()
            with self.assertRaisesRegex(ValueError, '不覆盖'):
                source.run(output)
            self.assertEqual(original, output.read_bytes())

    def test_out_of_bounds_candidate_is_distinguished_from_ik_failure(self):
        source = RotationDiagnostic(TRAJECTORY)
        q = source.q0.copy()
        q[0] = source.preview.limits[0, 1]+.001
        with tempfile.TemporaryDirectory() as tmp:
            with patch('fuel_cap_task.rotation_diagnostic.solve_continuous', return_value=(q, np.zeros(6), True)) as solve:
                report = source.run(Path(tmp)/'diagnostic.json', settings=RotationScanSettings(3.))
            self.assertEqual(solve.call_count, 1)
            self.assertEqual(report['failure']['kind'], 'joint_bounds')
            self.assertLess(report['failure']['minimum_joint_margin_rad'], 0.)

    def test_actual_short_scan_and_summary_use_only_common_fcl_prefix(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            path = folder/'diagnostic.json'
            report = RotationDiagnostic(TRAJECTORY).run(path, settings=RotationScanSettings(2.25))
            self.assertIsNone(report['failure'])
            self.assertEqual(report['samples'][-1]['angle_deg'], 2.25)
            fcl = fixture_audit(path, report, 2)
            (folder/'fcl_validation.json').write_text(json.dumps(fcl))
            summary = write_outputs(folder)
            self.assertEqual(summary['passed_samples'], 2)
            self.assertEqual(summary['checked_prefix_angle_deg'], report['samples'][1]['angle_deg'])
            self.assertEqual(summary['failure']['kind'], 'fcl_robot_collision')
            self.assertFalse(summary['scan_horizon_reached'])
            self.assertIsNone(summary['execution_stop_angle_deg'])
            self.assertEqual(len((folder/'joint_margins.csv').read_text().splitlines()), 13)
            with self.assertRaisesRegex(ValueError, '不覆盖'):
                write_outputs(folder)

    def test_reaching_horizon_does_not_produce_execution_threshold(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            path = folder/'diagnostic.json'
            report = RotationDiagnostic(TRAJECTORY).run(path, settings=RotationScanSettings(.5))
            fcl_path = folder/'fcl.json'
            fcl_path.write_text(json.dumps(fixture_audit(path, report)))
            summary, _ = summarize(path, fcl_path)
            self.assertTrue(summary['scan_horizon_reached'])
            self.assertIsNone(summary['execution_stopping_reserve_deg'])
            self.assertFalse(summary['physical_grasp_verified'])

    def test_start_rejected_by_full_coverage_has_no_usable_range(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            path = folder/'diagnostic.json'
            report = RotationDiagnostic(TRAJECTORY).run(path, settings=RotationScanSettings(.5))
            fcl_path = folder/'fcl.json'
            fcl_path.write_text(json.dumps(fixture_audit(path, report, 0)))
            summary, rows = summarize(path, fcl_path)
            self.assertIsNone(summary['checked_prefix_angle_deg'])
            self.assertIsNone(summary['joint_tables_deg']['last_checked'])
            self.assertEqual(rows, [])

    def test_tampered_counts_hashes_sources_and_margin_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            path, fcl_path = folder/'diagnostic.json', folder/'fcl.json'
            report = RotationDiagnostic(TRAJECTORY).run(path, settings=RotationScanSettings(.5))
            audit = fixture_audit(path, report)
            for key, value in (('checked_samples', 0), ('passed_samples', 999),
                               ('diagnostic_sha256', 'bad'), ('completed_input_check', False)):
                invalid = copy.deepcopy(audit)
                invalid[key] = value
                fcl_path.write_text(json.dumps(invalid))
                with self.assertRaises(ValueError):
                    summarize(path, fcl_path)
            for change in ('margin', 'source'):
                invalid = copy.deepcopy(report)
                if change == 'margin':
                    invalid['samples'][0]['joint_margin_rad'][0] += 1.
                else:
                    invalid['analysis_source_hashes']['src/fuel_cap_task/rotation_diagnostic.py'] = 'bad'
                path.write_text(json.dumps(invalid))
                fcl_path.write_text(json.dumps(fixture_audit(path, invalid)))
                with self.assertRaises(ValueError):
                    summarize(path, fcl_path)


if __name__ == '__main__':
    unittest.main()
