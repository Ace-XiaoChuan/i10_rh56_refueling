"""Bounded sampling, rigid fixture transforms and cap-relative target propagation."""
from pathlib import Path
import unittest
import numpy as np

from fuel_cap_task.arm_ik import body_transform,inverse
from fuel_cap_task.grasp_search import rotation
from fuel_cap_task.random_scenes import SceneSampler,SamplingSettings,classify

CANDIDATE=Path(__file__).resolve().parents[2]/'artifacts/manual_grasp/user_selected_20260930.json'


class RandomSceneTests(unittest.TestCase):
    def test_sampling_bounds_reproducibility_and_single_factor_groups(self):
        sampler=SceneSampler(CANDIDATE,SamplingSettings(per_group=20))
        samples=sampler.samples()
        self.assertEqual(samples,sampler.samples())
        self.assertEqual(len(samples),81)
        for row in samples:
            x,y,z=row['opening_base_m']
            self.assertTrue(-1e-12<=x<=.2 and -1<=y<=-.8 and abs(z-.4)<1e-12)
            self.assertLessEqual(np.hypot(x,y),1+1e-12)
            self.assertLessEqual(abs(row['tank_yaw_deg']),30)
            self.assertLessEqual(abs(row['cap_yaw_deg']),30)
            if row['group']=='position':self.assertEqual((row['tank_yaw_deg'],row['cap_yaw_deg']),(0,0))
            if row['group']=='tank_yaw':self.assertEqual(row['cap_yaw_deg'],0)
            if row['group']=='cap_yaw':self.assertEqual(row['tank_yaw_deg'],0)

    def test_rigid_tank_door_and_coaxial_cap(self):
        sampler=SceneSampler(CANDIDATE)
        a=sampler.reference
        sample={'opening_base_m':[.12,-.9,.4],'tank_yaw_deg':25.,'cap_yaw_deg':-20.}
        b=sampler.preview(sample)
        br=sampler.world_from_base[:3,:3]
        np.testing.assert_allclose(br.T@(b.data.body('tank_opening_center').xpos-sampler.world_from_base[:3,3]),sample['opening_base_m'],atol=1e-12)
        # Door stays fixed relative to tank despite whole-tank motion.
        np.testing.assert_allclose(inverse(body_transform(a.data,'fuel_tank_body'))@body_transform(a.data,'fuel_door'),
                                   inverse(body_transform(b.data,'fuel_tank_body'))@body_transform(b.data,'fuel_door'),atol=1e-12)
        delta=br@rotation(2,np.deg2rad(25))@br.T
        np.testing.assert_allclose(b.world_from_cap[:3,:3],delta@a.world_from_cap[:3,:3]@rotation(2,np.deg2rad(-20)),atol=1e-12)
        np.testing.assert_allclose(inverse(b.world_from_cap)@b.world_from_hand,inverse(a.world_from_cap)@a.world_from_hand,atol=1e-12)
        # Cap center offset from opening and axis remain tied to tank.
        np.testing.assert_allclose(b.world_from_cap[:3,3]-b.data.body('tank_opening_center').xpos,
                                   delta@(a.world_from_cap[:3,3]-a.data.body('tank_opening_center').xpos),atol=1e-12)
        self.assertEqual(b.data.time,0)

    def test_failure_classification_and_invalid_sampling(self):
        self.assertEqual(classify({'ik_found':False}),'ik_not_found')
        self.assertEqual(classify({'ik_found':True,'enabled_geometry_interference_free_solution_found':False}),'endpoint_interference')
        self.assertEqual(classify({'ik_found':True,'enabled_geometry_interference_free_solution_found':True}),'endpoint_clear')
        with self.assertRaises(ValueError):SamplingSettings(per_group=0)
        with self.assertRaises(ValueError):SamplingSettings(x_min_m=2,x_max_m=3)


if __name__=='__main__':unittest.main()
