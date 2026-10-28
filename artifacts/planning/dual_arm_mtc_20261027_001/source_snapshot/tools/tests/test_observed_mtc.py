"""用可手算的旋转与平移检查实测目标绑定，并拒绝文件或手预型版本混用。"""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src'))
from fuel_cap_task.observed_mtc import observed_fields, sha256, validate_binding, model_directory


class ObservedMtcTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.fixed = self.root/'fixed.json'
        self.fixed.write_text(json.dumps({'pregrasp_channels': [0,0,650,650,900,0]}))
        geometry = self.root/'geometry.json'
        geometry.write_text('{}')
        self.candidates = self.root/'candidates.json'
        values = []
        for label, sign in [('direction_000',1),('direction_180',-1)]:
            hand = np.diag([sign,sign,1.,1.])
            hand[:3,3] = [sign*.1, sign*.2, .3]
            values.append({'candidate_id': label, 'camera_from_hand': hand.tolist(),
                           'camera_from_cap': np.diag([sign,sign,1.,1.]).tolist()})
        self.candidates.write_text(json.dumps({'candidates': values,
            'pregrasp_channels': [0,0,650,650,900,0], 'fixed_grasp_sha256': sha256(self.fixed),
            'geometry_result': str(geometry), 'geometry_result_sha256': sha256(geometry)}))
        original = self.root/'original.json'
        original.write_text(json.dumps({'sources': {str(self.candidates): sha256(self.candidates)}}))
        (self.root/'environment.bt').write_bytes(b'fixture only: hash validation')
        (self.root/'cap.json').write_text('{}')
        self.manifest = self.root/'scene.json'
        world = [[0,-1,0,1],[1,0,0,2],[0,0,1,3],[0,0,0,1]]
        self.manifest.write_text(json.dumps({'scene_representation': 'environment_octomap_and_cap_collision_object',
            'planning_frame': 'world', 'allowed_collision_pairs': [], 'valid_for_hardware': False,
            'map_path': 'environment.bt', 'map_sha256': sha256(self.root/'environment.bt'),
            'cap_object_path': 'cap.json', 'cap_object_sha256': sha256(self.root/'cap.json'),
            'world_from_camera': world, 'world_from_octomap': world, 'source_octomap_scene': str(original)}))
        self.settings = {'replace_source_bodies': ['fuel_tank_body','cap_body','fuel_door'],
                         'dense_joint_step_rad': .002, 'dense_shape_motion_m': .001}

    def tearDown(self):
        self.temporary.cleanup()

    def fields(self, direction='direction_000'):
        return observed_fields(self.manifest, self.candidates, self.fixed, self.settings, direction, .1)

    def test_two_targets_share_map_and_keep_independent_hand_directions(self):
        a, b = self.fields(), self.fields('direction_180')
        self.assertEqual(a['observed_scene'], b['observed_scene'])
        np.testing.assert_allclose(a['hand_position'], [.8,2.1,3.3], atol=1e-12)
        np.testing.assert_allclose(b['hand_position'], [1.2,1.9,3.3], atol=1e-12)
        np.testing.assert_allclose(a['pregrasp_position'], [.8,2.1,3.4], atol=1e-12)
        rotation = Rotation.from_quat(a['hand_orientation_xyzw']).as_matrix()
        np.testing.assert_allclose(rotation @ [1,0,0], [0,1,0], atol=1e-12)
        self.assertFalse(a['cap_estimate']['valid_for_hardware'])

    def test_geometry_change_is_rejected(self):
        binding = self.fields()['observed_scene']
        (self.root/'cap.json').write_text('{"changed":true}')
        with self.assertRaises(ValueError):
            validate_binding(binding)

    def test_fixed_grasp_version_change_is_rejected(self):
        self.fixed.write_text(json.dumps({'pregrasp_channels': [0,0,800,800,900,0]}))
        with self.assertRaises(ValueError):
            self.fields()

    def test_mirrored_map_transform_is_rejected(self):
        manifest = json.loads(self.manifest.read_text())
        manifest['world_from_camera'] = np.diag([1,-1,1,1]).tolist()
        self.manifest.write_text(json.dumps(manifest))
        with self.assertRaises(ValueError):
            self.fields()

    def test_dual_world_cannot_fall_back_to_single_model(self):
        request={'robot_model_profile':'single','observed_scene':{'robot_model_binding':{'profile':'dual_arm'}}}
        with self.assertRaises(ValueError):
            model_directory(self.root,request)

    def test_dual_model_file_versions_are_bound(self):
        folder=self.root/'dual_arm'
        folder.mkdir()
        files={}
        for name in ['robot.urdf','robot.srdf','kinematics.json','model.json']:
            (folder/name).write_text('fixture '+name)
            files[name]=sha256(folder/name)
        request={'robot_model_profile':'dual_arm','observed_scene':{'robot_model_binding':{'profile':'dual_arm','files_sha256':files}}}
        self.assertEqual(model_directory(self.root,request),folder)
        (folder/'robot.urdf').write_text('changed geometry')
        with self.assertRaises(ValueError):
            model_directory(self.root,request)


if __name__ == '__main__':
    unittest.main()
