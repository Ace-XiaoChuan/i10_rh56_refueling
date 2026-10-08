"""检查分段累计角、新观测同步以及失败停止边界。"""
import json
import copy
from pathlib import Path
import tempfile
import unittest
import sys
from unittest.mock import patch

import numpy as np

from fuel_cap_task.rotation_diagnostic import RotationDiagnostic
from fuel_cap_task.segmented_rotation import regrasp_request, rotate, run, validate_regrasp, pose_tolerances, CURRENT_POSE_TOLERANCES
from fuel_cap_task.planning_scene_snapshot import check_snapshot, scene_snapshot

ROOT = Path(__file__).resolve().parents[2]
TRAJECTORY = ROOT/'artifacts/planning/random_single/approach.json'


class SegmentedRotationTests(unittest.TestCase):
    def test_new_tolerance_accepts_error_rejected_by_historical_setting(self):
        folder = ROOT/'artifacts/planning/segmented_single'
        request = json.loads((folder/'regrasp_1_request.json').read_text())
        report = json.loads((folder/'regrasp_1_mtc.json').read_text())
        source = RotationDiagnostic(TRAJECTORY)
        # 注入 0.15 mm / 0.004 rad 的末态反馈，原阈值拒绝，新阈值接受。
        with patch('fuel_cap_task.segmented_rotation.pose_error', return_value=(.00015, .004)):
            with self.assertRaises(ValueError):
                validate_regrasp(source, request, report)
            updated = copy.deepcopy(request)
            updated['endpoint_tolerances'] = dict(CURRENT_POSE_TOLERANCES)
            result = copy.deepcopy(report)
            result['request'] = updated
            _, validation = validate_regrasp(source, updated, result)
            self.assertEqual(validation['endpoint_position_error_m'], .00015)
        self.assertEqual(pose_tolerances(request), {'position_m': .0001, 'orientation_rad': .002})
        for value in (0., float('nan'), -1.):
            with self.assertRaises(ValueError):
                pose_tolerances({'endpoint_tolerances': {'position_m': value, 'orientation_rad': .005}})

    def test_new_estimate_and_scene_use_accumulated_angle(self):
        source = RotationDiagnostic(TRAJECTORY)
        initial = source.replay.report['request']['cap_estimate']
        request = regrasp_request(source, source.q0, 90.)
        self.assertEqual(request['regrasp']['cumulative_angle_deg'], 90.)
        self.assertEqual(request['cap_estimate']['source'], 'simulation_ground_truth')
        self.assertNotEqual(initial['pose']['orientation_xyzw'], request['cap_estimate']['pose']['orientation_xyzw'])
        np.testing.assert_allclose(np.array(request['pregrasp_position'])-request['hand_position'],
                                   .1*np.array(request['outward_axis_world']), atol=1e-12)
        check_snapshot(request['planning_scene'], scene_snapshot(source.world, source.preview.model, source.preview.data))
        self.assertEqual(request['pregrasp_channels'], source.preview.candidate['pregrasp_channels'])

    def test_mid_segment_failure_preserves_cumulative_progress(self):
        source = RotationDiagnostic(TRAJECTORY)
        # 仅注入下一度 IK 不收敛，起始状态错误会先被位姿检查拦截。
        with tempfile.TemporaryDirectory() as tmp:
            first = rotate(source, source.q0, 0., 1., Path(tmp)/'first.json')
            self.assertIsNone(first['failure'])
            q = np.array(first['samples'][-1]['arm_qpos_rad'])
            with patch('fuel_cap_task.segmented_rotation.solve_continuous', return_value=(q, np.ones(6), False)) as solve:
                second = rotate(source, q, 1., 3., Path(tmp)/'second.json')
            self.assertEqual(solve.call_count, 1)
            self.assertEqual(second['samples'][0]['angle_deg'], 1.)
            self.assertEqual(second['failure']['angle_deg'], 2.)
            self.assertEqual(second['mujoco_prefix_angle_deg'], 1.)

    def test_planning_failure_stops_without_followup_segment(self):
        with tempfile.TemporaryDirectory() as tmp:
            with patch('fuel_cap_task.segmented_rotation.launch', side_effect=RuntimeError('injected failure')) as launch:
                result = run(TRAJECTORY, Path(tmp))
            self.assertEqual(launch.call_count, 1)
            self.assertEqual(result['completed_regrasp_paths'], 0)
            self.assertFalse(result['full_geometric_sequence_passed'])
            self.assertTrue(result['analysis_finished'])
            with self.assertRaises(ValueError):
                run(TRAJECTORY, Path(tmp))

    def test_wrong_motion_stage_or_unvalidated_result_is_rejected(self):
        source = RotationDiagnostic(TRAJECTORY)
        request = regrasp_request(source, source.q0, 90.)
        invalid = {'mode': 'mtc_geometric_approach_not_execution', 'dense_fcl_check_passed': True,
                   'request': request, 'stages': [{'name': 'axial approach', 'points': [source.q0.tolist()]}]}
        with self.assertRaises(ValueError):
            validate_regrasp(source, request, invalid)

    def test_saved_checked_prefix_and_tampered_summary(self):
        sys.path.insert(0, str(ROOT/'tools'))
        from preview_segmented_rotation import SegmentedReplay
        path = ROOT/'artifacts/planning/segmented_single/summary.json'
        replay = SegmentedReplay(path)
        self.assertEqual(replay.summary['cumulative_angle_deg'], 180.)
        self.assertFalse(replay.summary['full_geometric_sequence_passed'])
        mask = np.ones(replay.model.nq, dtype=bool)
        mask[replay.source.preview.addresses] = False
        mask[replay.source.cap_address] = False
        for fraction in (0., .5, 1.):
            replay.set_progress(fraction)
            np.testing.assert_array_equal(replay.data.qpos[mask], replay.source.start[mask])
        changed = json.loads(path.read_text())
        changed['samples'][2]['arm_qpos_rad'][0] += .01
        with tempfile.TemporaryDirectory() as tmp:
            bad = Path(tmp)/'summary.json'
            bad.write_text(json.dumps(changed))
            with self.assertRaises(ValueError):
                SegmentedReplay(bad)


if __name__ == '__main__':
    unittest.main()
