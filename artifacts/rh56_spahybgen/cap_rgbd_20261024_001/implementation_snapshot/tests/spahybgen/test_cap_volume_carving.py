"""用已知深度平面验证原雕刻体积的变换方向、实体含义与未知区域处理。"""

import unittest
import numpy as np

from spahybgen.observation import VoxelVolume, CameraIntrinsic
from spahybgen.utils.utils_trans_np import Transform
from fuel_cap_task.cap_spahybgen_input import observed_free_grid


class OriginalCarvingTests(unittest.TestCase):
    def setUp(self):
        self.depth = np.full((81, 81), .6, dtype=np.float32)
        self.k = np.array([[100., 0., 40.], [0., 100., 40.], [0., 0., 1.]])
        self.camera_from_volume = np.diag([1., -1., -1., 1.])
        self.camera_from_volume[:3, 3] = [-.2, .2, .8]
        self.intrinsic = CameraIntrinsic(81, 81, 100, 100, 40, 40)

    def carve(self, depth):
        volume = VoxelVolume(.4, 16)
        volume.integrate(depth, self.intrinsic, Transform.from_matrix(self.camera_from_volume))
        return volume.get_grid()[0]

    def test_known_plane_keeps_back_volume_and_removes_observed_front(self):
        grid = self.carve(self.depth)
        self.assertEqual(grid[8, 8, 4], 1)
        self.assertEqual(grid[8, 8, 12], 0)
        free = observed_free_grid(self.depth, np.ones_like(self.depth, dtype=bool),
                                  self.k, self.camera_from_volume, .4, 16)
        self.assertFalse(free[8, 8, 4])
        self.assertTrue(free[8, 8, 12])

    def test_unobserved_is_not_accepted_as_free_when_network_grid_is_zero(self):
        empty = np.zeros_like(self.depth)
        grid = self.carve(empty)
        free = observed_free_grid(empty, np.zeros_like(empty, dtype=bool),
                                  self.k, self.camera_from_volume, .4, 16)
        self.assertEqual(grid.sum(), 0)
        self.assertFalse(free.any())


if __name__ == "__main__":
    unittest.main()
