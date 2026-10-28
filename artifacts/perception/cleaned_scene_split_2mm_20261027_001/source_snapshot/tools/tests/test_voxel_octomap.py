"""用带缺口、负坐标与非零原点的小网格检查原生文件及整图坐标变换。"""

import importlib.util
import itertools
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"src"))
from fuel_cap_task.voxel_octomap import (align_map, load_occupied_voxels,
                                        split_voxel_membership, cap_collision_object)

spec = importlib.util.spec_from_file_location("export_voxel_octomap", ROOT/"tools/export_voxel_octomap.py")
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)


class OctomapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="test_voxel_octomap_")
        cls.converter = exporter.compile_converter(cls.temporary.name)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def convert(self, rows, size=.002):
        """原生库实际保存再读取文件；每个样例使用独立的新目录。"""
        directory = Path(tempfile.mkdtemp(dir=self.temporary.name))
        source = directory/"input.json"
        source.write_text(json.dumps({"voxel_size_m": size, "indices_xyz": rows}))
        process = subprocess.run([str(self.converter), str(source), str(directory)], capture_output=True, text=True)
        return directory, process

    def test_pruned_cube_and_negative_cells_survive_disk_roundtrip(self):
        rows = list(itertools.product([0, 1], repeat=3))+[(-1, -2, -3), (10, 0, 0)]
        directory, process = self.convert(rows)
        self.assertEqual(process.returncode, 0, process.stderr)
        report = json.loads((directory/"octomap_validation.json").read_text())
        self.assertEqual(report["occupied_base_cell_count"], 10)
        self.assertEqual(report["occupied_leaf_count"], 3)
        self.assertEqual(report["free_leaf_count"], 0)
        self.assertTrue((directory/"scene.bt").read_bytes().startswith(b"# Octomap OcTree binary file"))

    def test_missing_corner_is_not_filled_by_pruning(self):
        rows = list(itertools.product([0, 1], repeat=3))[:-1]
        directory, process = self.convert(rows)
        self.assertEqual(process.returncode, 0, process.stderr)
        report = json.loads((directory/"octomap_validation.json").read_text())
        self.assertEqual(report["occupied_base_cell_count"], 7)
        self.assertEqual(report["occupied_leaf_count"], 7)

    def test_reject_duplicate_fractional_out_of_range_and_empty_cells(self):
        for rows in [[], [[0, 0, 0], [0, 0, 0]], [[.5, 0, 0]], [[32768, 0, 0]], [[-32769, 0, 0]]]:
            _, process = self.convert(rows)
            self.assertNotEqual(process.returncode, 0)

    def test_nonzero_grid_origin_and_rotated_placement_keep_cube_centers(self):
        target = [[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]]
        observed = np.eye(4)
        observed[:3, 3] = [.1, .2, .3]
        world_camera, camera_map, world_map = align_map(target, observed, [.003, -.001, .007])
        local_center = np.array([-.003, .001, .005, 1])
        camera_center = np.array([0, 0, .012, 1])
        np.testing.assert_allclose(camera_map @ local_center, camera_center, atol=1e-12)
        # 独立手算：先减去观测盖中心，再绕 Z 正转 90°，最后加目标盖中心。
        expected = [1.2, 1.9, 2.712, 1]
        np.testing.assert_allclose(world_map @ local_center, expected, atol=1e-12)
        np.testing.assert_allclose(world_camera @ observed, target, atol=1e-12)

    def test_invalid_rotation_is_rejected(self):
        mirrored = np.diag([1, -1, 1, 1])
        with self.assertRaises(ValueError):
            align_map(np.eye(4), mirrored, [0, 0, 0])

    def test_source_center_mismatch_is_rejected(self):
        path = Path(self.temporary.name)/"bad.npz"
        np.savez(path, indices_xyz=[[0, 0, 0]], centers_camera_m=[[.002, .001, .001]],
                 voxel_size_m=.002, grid_origin_camera_m=[0, 0, 0], cap_voxel_mask=[True])
        with self.assertRaisesRegex(ValueError, "中心"):
            load_occupied_voxels(path)

    def test_split_keeps_mixed_cell_in_environment_without_losing_observations(self):
        indices = np.array([[-1, 0, 0], [0, 0, 0], [1, 0, 0]])
        components, cap_counts = split_voxel_membership(indices, np.array([2, 2, 1]),
                                                        np.array([0, 0, 1, 1, 2]), np.array([0, 1, 2]))
        np.testing.assert_array_equal(components, [1, 2, 0])
        np.testing.assert_array_equal(cap_counts, [2, 1, 0])
        directory, process = self.convert(indices[components != 1].tolist())
        self.assertEqual(process.returncode, 0, process.stderr)
        report = json.loads((directory/"octomap_validation.json").read_text())
        self.assertEqual(report["occupied_base_cell_count"], 2)
        target = {tuple(p) for p in indices[components == 1]}
        environment = {tuple(p) for p in indices[components != 1]}
        self.assertFalse(target & environment)
        self.assertEqual(target | environment, {(-1, 0, 0), (0, 0, 0), (1, 0, 0)})

    def test_invalid_membership_and_duplicate_cap_rows_are_rejected(self):
        indices = np.array([[0, 0, 0], [1, 0, 0]])
        for counts, membership, rows in [([1, 2], [0, 1], [0]),
                                          ([1, 1], [0, 2], [0]),
                                          ([1, 1], [0, 1], [0, 0])]:
            with self.assertRaises(ValueError):
                split_voxel_membership(indices, np.array(counts), np.array(membership), np.array(rows))

    def test_cap_boxes_keep_rotated_grid_corners_in_world_frame(self):
        from scipy.spatial.transform import Rotation
        transform = np.array([[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]])
        obj = cap_collision_object([[.001, .003, .005]], .002, transform,
                                   object_id="fuel_cap", frame_id="world")
        pose = obj["primitive_poses"][0]
        center = np.array([pose["position"][a] for a in ("x", "y", "z")])
        rotation = Rotation.from_quat([pose["orientation"][a] for a in ("x", "y", "z", "w")]).as_matrix()
        np.testing.assert_allclose(center, [.997, 2.001, 3.005], atol=1e-12)
        np.testing.assert_allclose(rotation @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        corners = np.array(list(itertools.product([-.001, .001], repeat=3)))
        actual = corners @ rotation.T + center
        expected = np.array([[1-y, 2+x, 3+z] for x, y, z in
                             itertools.product([0, .002], [.002, .004], [.004, .006])])
        np.testing.assert_allclose(np.array(sorted(map(tuple, actual))),
                                   np.array(sorted(map(tuple, expected))), atol=1e-12)
        self.assertEqual(obj["primitives"][0]["dimensions"], [.002]*3)

    def test_cap_object_is_a_serializable_native_ros_message(self):
        from moveit_msgs.msg import CollisionObject
        from rosidl_runtime_py.set_message import set_message_fields
        from rclpy.serialization import serialize_message, deserialize_message
        obj = cap_collision_object([[.001, .001, .001], [.003, .001, .001]], .002, np.eye(4),
                                   object_id="fuel_cap", frame_id="world")
        msg = CollisionObject()
        set_message_fields(msg, obj)
        restored = deserialize_message(serialize_message(msg), CollisionObject)
        self.assertEqual(restored.id, "fuel_cap")
        self.assertEqual(restored.header.frame_id, "world")
        self.assertEqual(restored.operation, CollisionObject.ADD)
        self.assertEqual(len(restored.primitives), len(restored.primitive_poses))
        self.assertEqual(len(restored.primitives), 2)


if __name__ == "__main__":
    unittest.main()
