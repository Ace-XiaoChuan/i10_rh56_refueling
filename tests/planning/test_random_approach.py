"""检查采样可复现性、完整场景同步以及无效输入的拒绝行为。"""
import copy
import json
from pathlib import Path
import unittest
import tempfile
import numpy as np
from fuel_cap_task.random_scenes import SceneSampler, SamplingSettings
from fuel_cap_task.planning_scene_snapshot import scene_snapshot, check_snapshot
from fuel_cap_task.trajectory_replay import ApproachReplay

ROOT = Path(__file__).resolve().parents[2]


class RandomApproachTests(unittest.TestCase):
    def setUp(self):
        self.sampler = SceneSampler(ROOT/'artifacts/manual_grasp/user_selected_20260930.json',
                                    SamplingSettings(random_seed=20261002))
        self.baseline = json.loads((ROOT/'src/i10_rh56_moveit_config/config/scene.json').read_text())

    def test_single_sample_reproducibility_and_bounds(self):
        sample = self.sampler.single_sample()
        self.assertEqual(sample, self.sampler.single_sample())
        x, y, z = sample['opening_base_m']
        self.assertTrue(0 <= x <= .2 and -1 <= y <= -.8)
        self.assertLessEqual(np.hypot(x, y), 1)
        self.assertEqual(z, self.sampler.base_opening[2])
        self.assertLessEqual(abs(sample['tank_yaw_deg']), 30)
        self.assertLessEqual(abs(sample['cap_yaw_deg']), 30)

    def test_every_world_shape_tracks_its_sampled_geom(self):
        reference = self.sampler.reference
        check_snapshot(self.baseline, scene_snapshot(self.baseline, reference.model, reference.data))
        preview = self.sampler.preview(self.sampler.single_sample())
        snapshot = scene_snapshot(self.baseline, preview.model, preview.data)
        moved = set()
        for before, after in zip(self.baseline['objects'], snapshot['objects']):
            geom = after['source_geom']
            np.testing.assert_allclose(after['position'], preview.data.geom_xpos[geom])
            np.testing.assert_allclose(after['rotation_matrix'], preview.data.geom_xmat[geom].reshape(3, 3))
            self.assertEqual(before['shape'], after['shape'])
            if not np.allclose(before['position'], after['position']):
                moved.add(after['source_body'])
        self.assertTrue({'fuel_tank_body', 'cap_body', 'fuel_door'}.issubset(moved))
        self.assertEqual(snapshot['allowed_world_contacts'], self.baseline['allowed_world_contacts'])
        for kind in ('pose', 'shape', 'missing', 'exclusion'):
            bad = copy.deepcopy(snapshot)
            if kind == 'pose': bad['objects'][1]['position'][0] += .01
            elif kind == 'shape': bad['objects'][1]['shape']['uri'] = 'wrong'
            elif kind == 'missing': bad['objects'].pop()
            else: bad['allowed_world_contacts'] = []
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                check_snapshot(bad, snapshot)

    def test_saved_random_path_and_rejection_of_mismatched_world(self):
        path = ROOT/'artifacts/planning/random_single/approach.json'
        replay = ApproachReplay(path)
        result = replay.validate()
        self.assertLess(result['endpoint_position_error_m'], 1e-4)
        self.assertEqual(result['maximum_robot_penetration_m'], 0)
        source = json.loads(path.read_text())
        for kind in ('seed', 'world', 'target', 'axis', 'orientation', 'candidate'):
            bad = copy.deepcopy(source)
            request = bad['request']
            if kind == 'seed': request['scene_variation']['random_seed'] += 1
            elif kind == 'world': request['planning_scene']['objects'][1]['position'][0] += .01
            elif kind == 'target': request['hand_position'][0] += .01
            elif kind == 'axis': request['outward_axis_world'][0] += .01
            elif kind == 'orientation': request['hand_orientation_xyzw'] = [0, 0, 0, 1]
            else: request['candidate_path'] = 'unbound.json'
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as tmp:
                bad_path = Path(tmp)/'bad.json'
                bad_path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):
                    ApproachReplay(bad_path)


if __name__ == '__main__':
    unittest.main()
