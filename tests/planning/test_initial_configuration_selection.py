"""初始构型筛选的合同边界检查；注入反馈只验证编排，真实 ROS 结果另存。"""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from fuel_cap_task.initial_configuration_selection import (
    InitialContext, InitialCandidateContext, generate_pool, prefilter, result_status,
    run, selected_request, validate_path, validate_pool, verify,
)
from fuel_cap_task.regrasp_margin import EndpointRotationDiagnostic
from fuel_cap_task.rotation_diagnostic import RotationDiagnostic, hand_target
from fuel_cap_task.segmented_rotation import digest, save
from fuel_cap_task.trajectory_replay import ROOT

TRAJECTORY = ROOT/'artifacts/planning/random_single/approach.json'


class InitialSelectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = InitialContext(TRAJECTORY)

    def test_zero_progress_is_observed_initial_cap_not_model_zero(self):
        source = RotationDiagnostic(TRAJECTORY)
        node = InitialCandidateContext(self.context, self.context.before, candidate_id='baseline', bindings={}, role='candidate')
        endpoint = EndpointRotationDiagnostic(node, 'after')
        self.assertEqual(node.angle, 0.)
        self.assertEqual(endpoint.origin['cumulative_angle_deg'], 0.)
        self.assertNotEqual(float(endpoint.start[endpoint.cap_address]), 0.)
        np.testing.assert_allclose(endpoint.cap0, source.cap0, atol=1e-12)
        np.testing.assert_array_equal(endpoint.q0, source.q0)
        self.assertEqual(endpoint.row(endpoint.q0, 0.)['cumulative_angle_deg'], 0.)

    def test_raw_winding_kept_at_initial_endpoint(self):
        q = self.context.before.copy()
        q[5] -= 2*np.pi
        node = InitialCandidateContext(self.context, q, candidate_id='winding', bindings={}, role='candidate')
        endpoint = EndpointRotationDiagnostic(node, 'after')
        np.testing.assert_array_equal(endpoint.q0, q)
        np.testing.assert_allclose(hand_target(endpoint.cap0, endpoint.relative, 0.),
                                   hand_target(RotationDiagnostic(TRAJECTORY).cap0, endpoint.relative, 0.), atol=1e-12)

    def test_finite_ik_budget_and_baseline_are_preserved(self):
        def fail(preview, target, seed):
            return np.asarray(seed), np.ones(6), False
        with patch('fuel_cap_task.initial_configuration_selection.solve_continuous', side_effect=fail) as solver:
            pool = generate_pool(self.context)
        self.assertEqual(solver.call_count, 24)
        self.assertEqual(len(pool['ik_attempts']), 24)
        self.assertTrue(all(not item['converged'] for item in pool['ik_attempts']))
        np.testing.assert_array_equal(pool['ik_attempts'][1]['seed_q_rad'], self.context.initial_arm)
        np.testing.assert_array_equal(pool['ik_attempts'][2]['seed_q_rad'], np.zeros(6))
        self.assertLessEqual(sum(row['scan_selected'] for row in pool['candidates']), 12)
        self.assertEqual(pool['candidates'][0]['labels'], ['baseline'])
        self.assertTrue(pool['candidates'][0]['scan_selected'])
        self.assertTrue(all(row['prefilter_reason'] in ('selected', 'initial_geometry_rejected', 'finite_scan_budget') for row in pool['candidates']))

    def test_prefilter_keeps_baseline_even_if_invalid_and_obeys_budget(self):
        rows = [{'id': 'baseline', 'labels': ['baseline'], 'initial_geometry_passed': False}]
        rows += [{'id': f'q{i}', 'labels': [], 'initial_geometry_passed': True,
                  'minimum_joint_margin_rad': i, 'distance_from_before_rad': i} for i in range(20)]
        selected = prefilter(rows)
        self.assertEqual(len(selected), 12)
        self.assertIn('baseline', selected)
        self.assertIn('q19', selected)
        self.assertNotIn('q0', selected)

    def test_selected_request_retains_initial_state_scene_and_pose(self):
        candidate = {'id': 'baseline', 'arm_qpos_rad': self.context.before.tolist()}
        request, axial = selected_request(self.context, candidate, {})
        self.assertTrue(axial['passed'])
        self.assertLessEqual(axial['target_refinement']['position_error_m'], .000001)
        self.assertLessEqual(axial['target_refinement']['orientation_error_rad'], .00002)
        self.assertNotIn('regrasp', request)
        actual = copy.deepcopy(request)
        selection = actual.pop('configuration_selection')
        self.assertEqual(actual, self.context.request)
        self.assertEqual(selection['role'], 'initial_approach')
        self.assertEqual(selection['grasp_q_rad'], candidate['arm_qpos_rad'])
        self.assertEqual(selection['endpoint_joint_tolerance_rad'], .01)
        self.assertAlmostEqual(axial['samples'][-1]['distance_m'], self.context.request['approach_distance_m'])

    def test_failed_pregrasp_refinement_rejects_candidate(self):
        candidate = {'id': 'baseline', 'arm_qpos_rad': self.context.before.tolist()}
        def fail(preview, target, seed):
            return seed, {'converged': False, 'position_error_m': .001, 'orientation_error_rad': .001}
        with patch('fuel_cap_task.initial_configuration_selection.refine_pregrasp', side_effect=fail) as refine:
            request, axial = selected_request(self.context, candidate, {})
        self.assertEqual(refine.call_count, 1)
        self.assertIsNone(request)
        self.assertFalse(axial['passed'])
        self.assertFalse(axial['target_refinement']['converged'])

    def test_no_improvement_keeps_baseline_without_path_search(self):
        baseline = {'id': 'candidate_000', 'labels': ['baseline'], 'arm_qpos_rad': self.context.before.tolist(),
                    'scan_selected': True, 'minimum_joint_margin_rad': 1., 'distance_from_before_rad': 0.}
        alternative = {**baseline, 'id': 'candidate_001', 'labels': [], 'distance_from_before_rad': 1.}
        pool = {'candidates': [baseline, alternative]}
        response = {'checked_additional_angle_deg': 189., 'passed_samples': 1, 'failure': None}
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as temp:
            folder = Path(temp)/'result'
            with patch('fuel_cap_task.initial_configuration_selection.InitialContext', return_value=self.context), \
                 patch('fuel_cap_task.initial_configuration_selection.source_bindings', return_value={}), \
                 patch('fuel_cap_task.initial_configuration_selection.generate_pool', return_value=pool), \
                 patch('fuel_cap_task.initial_configuration_selection.diagnostic', return_value=response) as scan, \
                 patch('fuel_cap_task.initial_configuration_selection.launch') as planner:
                summary = run(TRAJECTORY, folder)
            self.assertEqual(scan.call_count, 2)
            planner.assert_not_called()
            self.assertEqual(summary['status'], 'baseline_retained_no_verified_improvement')
            self.assertIsNone(summary['selected'])
            self.assertFalse(summary['physical_grasp_verified'])

    def test_runtime_audit_failure_stops_before_other_candidates(self):
        pool = {'candidates': [{'id': 'baseline', 'scan_selected': True,
                              'arm_qpos_rad': self.context.before.tolist()}]}
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as temp:
            with patch('fuel_cap_task.initial_configuration_selection.InitialContext', return_value=self.context), \
                 patch('fuel_cap_task.initial_configuration_selection.source_bindings', return_value={}), \
                 patch('fuel_cap_task.initial_configuration_selection.generate_pool', return_value=pool), \
                 patch('fuel_cap_task.initial_configuration_selection.diagnostic', side_effect=RuntimeError('ROS unavailable')) as scan, \
                 patch('fuel_cap_task.initial_configuration_selection.launch') as planner:
                with self.assertRaises(RuntimeError):
                    run(TRAJECTORY, Path(temp)/'result')
            self.assertEqual(scan.call_count, 1)
            planner.assert_not_called()
            self.assertFalse(json.loads((Path(temp)/'result/checkpoint.json').read_text())['analysis_finished'])

    def test_existing_output_is_rejected_before_loading(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as temp:
            with patch('fuel_cap_task.initial_configuration_selection.InitialContext') as context:
                with self.assertRaises(ValueError):
                    run(TRAJECTORY, temp)
                context.assert_not_called()

    def test_actual_trajectory_requires_selected_pregrasp_check(self):
        report = json.loads(TRAJECTORY.read_text())
        report['selected_configuration_check_passed'] = True
        candidate = {'id': 'test', 'arm_qpos_rad': self.context.before.tolist()}
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as temp:
            path = Path(temp)/'fixture.json'
            save(path, report)
            with self.assertRaises(ValueError):
                validate_path(path, self.context, self.context.request, candidate)

    def test_missing_or_changed_actual_trajectory_binding_is_rejected(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts') as temp:
            unbound = Path(temp)/'actual.json'
            unbound.write_bytes(TRAJECTORY.read_bytes())
            with self.assertRaises(ValueError):
                InitialCandidateContext(self.context, self.context.before, candidate_id='test', bindings={},
                                        role='actual_mtc_endpoint', trajectory=unbound)
        binding = {str(TRAJECTORY.relative_to(ROOT)): digest(TRAJECTORY)}
        node = InitialCandidateContext(self.context, self.context.before, candidate_id='test', bindings=binding,
                                       role='actual_mtc_endpoint', trajectory=TRAJECTORY)
        self.assertEqual(node.origin('after')['diagnostic_trajectory_path'], str(TRAJECTORY.relative_to(ROOT)))
        binding[str(TRAJECTORY.relative_to(ROOT))] = '0'*64
        with self.assertRaises(ValueError):
            InitialCandidateContext(self.context, self.context.before, candidate_id='test', bindings=binding, role='candidate')

    def test_status_keeps_unknown_baseline_distinct(self):
        self.assertEqual(result_status({'selected': None, 'baseline_angle_deg': None}), 'baseline_invalid_no_comparison')
        self.assertEqual(result_status({'selected': None, 'baseline_angle_deg': 189.}), 'baseline_retained_no_verified_improvement')

    def test_saved_pool_geometry_and_prefilter_rechecked(self):
        path = ROOT/'artifacts/planning/initial_selection_single_v2/candidate_pool.json'
        if not path.exists():
            self.skipTest('本轮真实候选池尚未生成')
        pool = json.loads(path.read_text())
        checked = validate_pool(self.context, pool)
        self.assertEqual(len(checked), len(pool['candidates']))
        changed = copy.deepcopy(pool)
        changed['candidates'][0]['arm_qpos_rad'][5] += 2*np.pi
        with self.assertRaises(ValueError):
            validate_pool(self.context, changed)
        changed = copy.deepcopy(pool)
        changed['candidates'][0]['scan_selected'] = False
        with self.assertRaises(ValueError):
            validate_pool(self.context, changed)
        changed = copy.deepcopy(pool)
        changed['ik_attempts'][1]['seed_q_rad'][0] += .1
        with self.assertRaises(ValueError):
            validate_pool(self.context, changed)

    def test_saved_real_initial_selection_when_available(self):
        folder = ROOT/'artifacts/planning/initial_selection_single_v2'
        if not (folder/'summary.json').exists():
            self.skipTest('本轮真实 ROS 结果尚未生成')
        summary = verify(folder)
        self.assertTrue(summary['analysis_finished'])
        if summary['selected']:
            self.assertGreater(summary['selected']['checked_range_change_deg'], 0.)
            self.assertLessEqual(summary['selected']['endpoint_joint_error_rad'], .01)
        self.assertFalse(summary['physical_grasp_verified'])


if __name__ == '__main__':
    unittest.main()
