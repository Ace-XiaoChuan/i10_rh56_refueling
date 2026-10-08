"""跨模型验证轨迹，并拒绝不安全或来源失效的回放输入。"""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from fuel_cap_task.trajectory_replay import ApproachReplay
ROOT=Path(__file__).resolve().parents[2]
REPORT=ROOT/'artifacts/planning/approach.json'


class ApproachReplayTests(unittest.TestCase):
    def test_mtc_path_and_non_arm_state_invariance(self):
        replay=ApproachReplay(REPORT)
        result=replay.validate()
        self.assertLess(result['endpoint_position_error_m'],1e-4)
        self.assertLess(result['endpoint_orientation_error_rad'],.002)
        self.assertEqual(result['maximum_robot_penetration_m'],0)
        mask=np.ones(replay.model.nq,dtype=bool);mask[replay.addresses]=False
        for fraction in (0,.15,.5,.9,1):
            replay.set_progress(fraction)
            np.testing.assert_array_equal(replay.data.qpos[mask],replay.initial[mask])
        np.testing.assert_allclose(replay.data.qpos[replay.addresses],replay.points[-1],rtol=0,atol=1e-12)

    def test_bad_and_stale_reports_are_rejected(self):
        source=json.loads(REPORT.read_text())
        for kind in ('failed','hash','order','hand','jump','nan','start'):
            with self.subTest(kind=kind),tempfile.TemporaryDirectory() as tmp:
                report=copy.deepcopy(source)
                if kind=='failed':report['mode']='planning_not_completed_no_replay'
                elif kind=='hash':report['request']['source_hashes']['assets/scenes/i10_rh56_fuel_cap_preview.xml']='0'*64
                elif kind=='order':report['request']['arm_joints'].reverse()
                elif kind=='hand':report['request']['initial_joint_positions']['index_proximal_joint']+=.1
                elif kind=='jump':report['stages'][0]['points'][2][0]+=1
                elif kind=='nan':report['stages'][0]['points'][2][0]=float('nan')
                elif kind=='start':report['stages'][0]['points'][0][0]+=.001
                path=Path(tmp)/'bad.json';path.write_text(json.dumps(report))
                with self.assertRaises(ValueError):ApproachReplay(path)

if __name__=='__main__':unittest.main()
