"""Endpoint verification on the current full robot, not trajectory acceptance."""
from pathlib import Path
import unittest

import mujoco
import numpy as np

from fuel_cap_task.arm_ik import ArmIKPreview, IKSettings, body_transform, inverse, rotation_error

CANDIDATE = Path(__file__).resolve().parents[2]/'artifacts/manual_grasp/user_selected_20260930.json'


class ArmIKTests(unittest.TestCase):
    def test_candidate_chain_and_arm_only_solution(self):
        p=ArmIKPreview(CANDIDATE,IKSettings(seeds=2))
        before=p.data.qpos.copy()
        report=p.solve()
        self.assertTrue(report['ik_found'])
        actual=body_transform(p.data,'base')
        np.testing.assert_allclose(actual[:3,3],p.world_from_hand[:3,3],atol=p.settings.position_tolerance_m)
        self.assertLess(np.linalg.norm(rotation_error(p.world_from_hand[:3,:3],actual[:3,:3])),p.settings.orientation_tolerance_rad)
        np.testing.assert_allclose(p.world_from_flange@p.flange_from_hand,p.world_from_hand,atol=1e-12)
        np.testing.assert_allclose(body_transform(p.data,'base_link')@p.base_from_hand,p.world_from_hand,atol=1e-12)
        others=np.setdiff1d(np.arange(p.model.nq),p.addresses)
        np.testing.assert_array_equal(p.data.qpos[others],before[others])
        q=p.data.qpos[p.addresses]
        self.assertTrue(np.all(q>=p.limits[:,0]) and np.all(q<=p.limits[:,1]))
        self.assertEqual(p.data.time,0)
        self.assertFalse(report['path_verified'])
        self.assertEqual(report['initial_preview_contacts'], [])
        # Recorded hand closure must preserve the solved arm and the root pose.
        p.show(report['solutions'][0],closed=True)
        np.testing.assert_array_equal(p.data.qpos[p.addresses],q)
        np.testing.assert_allclose(body_transform(p.data,'base'),actual,atol=1e-12)

    def test_fk_generated_target_recovered_by_ik(self):
        p=ArmIKPreview(CANDIDATE,IKSettings(seeds=2))
        p.forward(np.array([.3,-.6,1.1,.4,-.5,.2]))
        p.world_from_hand=body_transform(p.data,'base')
        p.world_from_flange=p.world_from_hand@inverse(p.flange_from_hand)
        p.forward(p.initial)
        result=p.solve()
        self.assertTrue(result['ik_found'])
        np.testing.assert_allclose(body_transform(p.data,'base')[:3,3],p.world_from_hand[:3,3],atol=p.settings.position_tolerance_m)

    def test_unreachable_target_returns_failure_without_moving_to_best_residual(self):
        p=ArmIKPreview(CANDIDATE,IKSettings(seeds=2,iterations=30))
        p.world_from_flange[:3,3]=[100,100,100]
        result=p.solve()
        self.assertFalse(result['ik_found'])
        np.testing.assert_array_equal(p.data.qpos[p.addresses],p.initial)


if __name__=='__main__': unittest.main()
