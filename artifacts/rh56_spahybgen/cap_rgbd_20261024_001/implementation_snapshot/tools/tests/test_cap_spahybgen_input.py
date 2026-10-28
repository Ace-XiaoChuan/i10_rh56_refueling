"""用已知平面和刚体变换检查体积坐标、物理尺度及未知区域处理。"""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from fuel_cap_task.cap_spahybgen_input import local_geometry, observed_free_grid, restrict_prediction, target_ellipse


class CapVolumeTests(unittest.TestCase):
    def setUp(self):
        self.depth = np.ones((61, 81), dtype=np.float32)
        self.valid = np.ones_like(self.depth, dtype=bool)
        self.k = np.array([[100., 0., 40.], [0., 100., 30.], [0., 0., 1.]])
        self.mask = target_ellipse(self.depth.shape, [34, 24, 46, 36])
        self.world = np.array([[0., -1., 0., .3], [1., 0., 0., -.2],
                               [0., 0., 1., .7], [0., 0., 0., 1.]])
        self.geometry = local_geometry(self.depth, self.valid, self.k, self.world, self.mask)

    def test_roundtrip_matches_independent_pixel_plane_coordinates(self):
        g = self.geometry
        u, v = g["pixels_uv"].T
        expected_camera = np.column_stack([(u - 40) / 100, (v - 30) / 100, np.ones(len(u))])
        actual_camera = g["points_volume_m"] @ g["camera_from_volume"][:3, :3].T + g["camera_from_volume"][:3, 3]
        np.testing.assert_allclose(actual_camera, expected_camera, atol=1e-12)
        actual_world = g["points_volume_m"] @ g["world_from_volume"][:3, :3].T + g["world_from_volume"][:3, 3]
        expected_world = np.column_stack([.3 - expected_camera[:, 1], expected_camera[:, 0] - .2,
                                          expected_camera[:, 2] + .7])
        np.testing.assert_allclose(actual_world, expected_world, atol=1e-12)

    def test_physical_size_and_camera_facing_axes(self):
        g = self.geometry
        np.testing.assert_allclose(g["camera_from_volume"][:3, :3], np.diag([1, -1, -1]))
        self.assertAlmostEqual(float(g["voxel_size_m"]), .005)
        p = g["points_volume_m"]
        pixels = g["pixels_uv"]
        left = p[np.all(pixels == [39, 30], axis=1)][0]
        right = p[np.all(pixels == [41, 30], axis=1)][0]
        self.assertAlmostEqual(float(np.linalg.norm(left - right)), .02)
        self.assertTrue(np.all(g["contact_region"] <= g["center_region"]))

    def test_invalid_depth_is_not_used_and_empty_target_is_rejected(self):
        damaged = self.depth.copy()
        damaged[self.mask] = np.nan
        with self.assertRaisesRegex(ValueError, "不足"):
            local_geometry(damaged, self.valid, self.k, self.world, self.mask)

    def test_mirror_transform_and_wrong_pixel_annotation_are_rejected(self):
        mirror = self.world.copy()
        mirror[:3, 0] *= -1
        with self.assertRaisesRegex(ValueError, "右手"):
            local_geometry(self.depth, self.valid, self.k, mirror, self.mask)
        with self.assertRaises(ValueError):
            target_ellipse(self.depth.shape, [-1, 0, 20, 30])

    def test_only_observed_front_of_plane_is_free(self):
        transform = self.geometry["camera_from_volume"]
        free = observed_free_grid(self.depth, self.valid, self.k, transform)
        self.assertFalse(free[40, 40, 20])
        self.assertTrue(free[40, 40, 60])
        no_depth = observed_free_grid(self.depth, np.zeros_like(self.valid), self.k, transform)
        self.assertFalse(no_depth.any())

    def test_target_filter_preserves_predictions_and_rejects_empty_evidence(self):
        shape = self.geometry["contact_region"].shape
        quality, centers = restrict_prediction(np.full(shape, .6), np.full(shape, .8), self.geometry)
        self.assertTrue(np.all(quality[self.geometry["contact_region"]] == np.float32(.6)))
        self.assertTrue(np.all(quality[~self.geometry["contact_region"]] == 0))
        self.assertTrue(np.all(centers[~self.geometry["center_region"]] == 0))
        with self.assertRaisesRegex(ValueError, "没有正"):
            restrict_prediction(np.zeros(shape), np.ones(shape), self.geometry)


if __name__ == "__main__":
    unittest.main()
