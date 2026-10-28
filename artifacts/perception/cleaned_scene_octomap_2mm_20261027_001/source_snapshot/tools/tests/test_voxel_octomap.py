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
from fuel_cap_task.voxel_octomap import align_map, load_occupied_voxels

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


if __name__ == "__main__":
    unittest.main()
