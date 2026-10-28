"""检查单个小目标的网络中心加权和少量接触特征处理。"""

import unittest
import numpy as np
import torch

from fuel_cap_task.cap_spahybgen_optimizer import CapRH56Optimization


class SingleTargetFeatures(unittest.TestCase):
    def optimizer(self):
        optimizer = object.__new__(CapRH56Optimization)
        optimizer.device = torch.device("cpu")
        optimizer.batch, optimizer.npts_cnt_pts = 2, 75
        optimizer.voxel_size, optimizer.focal_ratio = .005, .2
        optimizer.tip_downsample, optimizer.wrench_downsample = .7, .9
        optimizer.tip_score_full = torch.zeros(8, 8, 8)
        optimizer.scene_wrench_full = torch.zeros(8, 8, 8)
        return optimizer

    def test_two_contacts_survive_and_center_uses_network_weights(self):
        optimizer = self.optimizer()
        optimizer.tip_score_full[1, 2, 3] = .8
        optimizer.tip_score_full[2, 3, 4] = .9
        optimizer.scene_wrench_full[2, 2, 2] = .95
        optimizer.scene_wrench_full[4, 2, 2] = 1.
        optimizer.refine_contact_features_from_raw()
        self.assertEqual(optimizer.npts_tip_focal_points, 2)
        expected = np.array([(2 * .95 + 4) / 1.95, 2, 2]) * .005
        np.testing.assert_allclose(optimizer.wrench_focal_points[0].numpy(), expected, atol=1e-8)

    def test_no_network_contacts_stops_without_inventing_grasp_points(self):
        optimizer = self.optimizer()
        with self.assertRaisesRegex(ValueError, "不足"):
            optimizer.refine_contact_features_from_raw()


if __name__ == "__main__":
    unittest.main()
