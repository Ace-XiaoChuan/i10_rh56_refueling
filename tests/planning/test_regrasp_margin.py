"""检查同姿态比较、累计角、绕数、输入绑定及失败停止；注入审计只验证软件边界。"""
import copy
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from fuel_cap_task.arm_ik import body_transform
from fuel_cap_task.regrasp_margin import RegraspContext, EndpointRotationDiagnostic, prepare, audit, write_outputs
from fuel_cap_task.rotation_diagnostic import RotationScanSettings, hand_target
from fuel_cap_task.segmented_rotation import digest, save
from fuel_cap_task.trajectory_replay import ROOT

SEGMENT = ROOT/'artifacts/planning/segmented_single_tol_020'


def fixture_audit(folder, count=None):
    """为汇总单测注入计数；不冒充真实 FCL 执行结果。"""
    path = folder/'diagnostic.json'
    report = json.loads(path.read_text())
    rows = report['samples']
    count = len(rows) if count is None else count
    failure = None if count == len(rows) else {'kind': 'fcl_robot_collision', 'angle_deg': rows[count]['angle_deg']}
    save(folder/'fcl_validation.json', {
        'mode': 'full_fcl_rotation_prefix_audit_not_execution', 'diagnostic_sha256': digest(path),
        'passed_samples': count, 'checked_samples': count+(failure is not None),
        'clear_prefix_angle_deg': rows[count-1]['angle_deg'] if count else 0.,
        'failure': failure, 'completed_input_check': failure is None})


def copy_segment(folder):
    """只复制所需小型记录到测试目录；原实验产物不改动。"""
    summary = json.loads((SEGMENT/'summary.json').read_text())
    names = ('rotation_1.json', 'rotation_1_fcl.json', 'regrasp_1_request.json',
             'regrasp_1_mtc.json', 'regrasp_1_validation.json')
    for name in names:
        shutil.copyfile(SEGMENT/name, folder/name)
        summary['artifacts_sha256'].pop(str((SEGMENT/name).relative_to(ROOT)))
        summary['artifacts_sha256'][str((folder/name).relative_to(ROOT))] = digest(folder/name)
    save(folder/'summary.json', summary)
    return summary


class RegraspMarginTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = RegraspContext(SEGMENT)

    def test_same_cap_and_hand_target_preserve_cumulative_angle_and_joint_winding(self):
        context = self.context
        before, after = (EndpointRotationDiagnostic(context, side) for side in ('before', 'after'))
        self.assertEqual(context.angle, 90.)
        np.testing.assert_array_equal(before.q0, context.before)
        np.testing.assert_array_equal(after.q0, context.after)
        np.testing.assert_allclose(before.cap0, after.cap0, atol=1e-12, rtol=0)
        np.testing.assert_allclose(hand_target(before.cap0, before.relative, .3),
                                   hand_target(after.cap0, after.relative, .3), atol=1e-12, rtol=0)
        self.assertGreater(np.max(np.abs(after.q0-before.q0)), 6.)
        # 只读取几何状态，不把保存的手姿态视为新抓稳证据。
        for source in (before, after):
            mask = np.ones(source.preview.model.nq, dtype=bool)
            mask[source.preview.addresses] = False
            mask[source.cap_address] = False
            original = source.start[mask].copy()
            row = source.row(source.q0, .5, detailed=True)
            self.assertEqual(row['cumulative_angle_deg'], 90.5)
            np.testing.assert_array_equal(source.preview.data.qpos[mask], original)
            np.testing.assert_allclose(body_transform(source.preview.data, 'cap_grip_center')[:3, 2],
                                       source.cap0[:3, 2], atol=1e-12, rtol=0)

    def test_short_scan_saves_relative_and_cumulative_failure_without_retry(self):
        source = EndpointRotationDiagnostic(self.context, 'after')
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'diagnostic.json'
            with patch('fuel_cap_task.rotation_diagnostic.solve_continuous',
                       return_value=(source.q0.copy(), np.ones(6), False)) as solve:
                report = source.run(path, settings=RotationScanSettings(2.))
            self.assertEqual(solve.call_count, 1)
            self.assertEqual(report['samples'][0]['angle_deg'], 0.)
            self.assertEqual(report['samples'][0]['cumulative_angle_deg'], 90.)
            self.assertEqual(report['failure']['angle_deg'], 1.)
            self.assertEqual(report['failure']['cumulative_angle_deg'], 91.)
            self.assertEqual(report['failure']['kind'], 'continuous_ik_not_converged')
            self.assertEqual(json.loads((Path(tmp)/'input.json').read_text())['origin'], report['origin'])

    def test_changed_input_hash_and_false_success_are_rejected(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts/planning') as tmp:
            folder = Path(tmp)
            summary = copy_segment(folder)
            path = folder/'regrasp_1_mtc.json'
            report = json.loads(path.read_text())
            report['dense_fcl_check_passed'] = False
            save(path, report)
            with self.assertRaisesRegex(ValueError, '来源已变更'):
                RegraspContext(folder)
            summary['artifacts_sha256'][str(path.relative_to(ROOT))] = digest(path)
            save(folder/'summary.json', summary)
            with self.assertRaisesRegex(ValueError, '不完整'):
                RegraspContext(folder)

    def test_rebound_wrong_cap_angle_or_target_still_rejected(self):
        for field in ('angle', 'target'):
            with self.subTest(field=field), tempfile.TemporaryDirectory(dir=ROOT/'artifacts/planning') as tmp:
                folder = Path(tmp)
                summary = copy_segment(folder)
                request = json.loads((folder/'regrasp_1_request.json').read_text())
                if field == 'angle':
                    request['regrasp']['cumulative_angle_deg'] += 1.
                else:
                    request['hand_position'][0] += .01
                mtc = json.loads((folder/'regrasp_1_mtc.json').read_text())
                mtc['request'] = request
                for name, value in (('regrasp_1_request.json', request), ('regrasp_1_mtc.json', mtc)):
                    save(folder/name, value)
                    summary['artifacts_sha256'][str((folder/name).relative_to(ROOT))] = digest(folder/name)
                save(folder/'summary.json', summary)
                with self.assertRaises(ValueError):
                    RegraspContext(folder)

    def test_summary_uses_common_prefix_and_never_sets_execution_threshold(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts/planning') as tmp:
            folder = Path(tmp)/'comparison'
            prepare(SEGMENT, folder, max_additional_angle_deg=.5)
            fixture_audit(folder/'before')
            fixture_audit(folder/'after', 1)
            with patch('fuel_cap_task.rotation_diagnostic.solve_continuous', side_effect=AssertionError('汇总不得求 IK')):
                result = write_outputs(folder)
            self.assertEqual(result['endpoints']['before']['checked_additional_angle_deg'], .5)
            self.assertEqual(result['endpoints']['after']['checked_cumulative_angle_deg'], 90.)
            self.assertEqual(result['checked_range_change_deg'], -.5)
            self.assertFalse(result['checked_range_increased'])
            self.assertIsNone(result['execution_stop_angle_deg'])
            self.assertFalse(result['physical_regrasp_verified'])
            self.assertGreater(len((folder/'joint_margins.csv').read_text().splitlines()), 1)
            with self.assertRaisesRegex(ValueError, '不覆盖'):
                write_outputs(folder)
            with self.assertRaisesRegex(ValueError, '独立输出'):
                prepare(SEGMENT, folder)

    def test_rejected_start_has_unknown_comparison_not_zero_range(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts/planning') as tmp:
            folder = Path(tmp)/'comparison'
            prepare(SEGMENT, folder, max_additional_angle_deg=.5)
            fixture_audit(folder/'before')
            fixture_audit(folder/'after', 0)
            result = write_outputs(folder)
            self.assertIsNone(result['endpoints']['after']['checked_additional_angle_deg'])
            self.assertIsNone(result['checked_range_increased'])
            self.assertIsNone(result['checked_range_change_deg'])

    def test_rebound_tampered_cumulative_angle_or_unequal_budget_rejected(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'artifacts/planning') as tmp:
            folder = Path(tmp)/'comparison'
            prepare(SEGMENT, folder, max_additional_angle_deg=.5)
            fixture_audit(folder/'before')
            fixture_audit(folder/'after')
            path = folder/'after/diagnostic.json'
            original = json.loads(path.read_text())
            for field in ('angle', 'budget'):
                invalid = copy.deepcopy(original)
                if field == 'angle':
                    invalid['samples'][0]['cumulative_angle_deg'] = 0.
                else:
                    invalid['scan_settings']['max_angle_deg'] = 1.
                    invalid['target_angle_deg'] = 1.
                save(path, invalid)
                fixture_audit(folder/'after')
                with self.assertRaises(ValueError):
                    write_outputs(folder)

    def test_audit_failure_does_not_continue_or_repeat(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            (folder/'before').mkdir()
            (folder/'after').mkdir()
            with patch('fuel_cap_task.regrasp_margin.launch', side_effect=RuntimeError('injected startup failure')) as launch:
                with self.assertRaises(RuntimeError):
                    audit(folder)
            self.assertEqual(launch.call_count, 1)


if __name__ == '__main__':
    unittest.main()
