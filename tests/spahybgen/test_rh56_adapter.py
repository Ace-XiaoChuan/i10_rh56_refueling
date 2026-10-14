"""核对真实 RH56 耦合 FK、梯度和硬约束；仅在独立优化环境运行。"""

import copy
import json
from pathlib import Path
import unittest

import numpy as np
import torch
from spahybgen.handmodel import HandModel
import spahybgen.optimization as upstream
from spahybgen.optimization import MeanScaleNormalizer

from fuel_cap_task.rh56_drive_map import RH56DriveMap
from fuel_cap_task.rh56_spahybgen import RH56Optimization, audit_hand_model

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "artifacts/rh56_spahybgen/model_20261008"


class MappingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = json.loads((MODEL / "model.json").read_text())["mapping"]
        cls.mapping = RH56DriveMap.from_dict(cls.data)

    def test_reject_driver_range_that_exceeds_follower_limit(self):
        data = copy.deepcopy(self.data)
        data["upper"][4] += .2
        with self.assertRaises(ValueError):
            RH56DriveMap.from_dict(data)

    def test_reject_nonfinite_mapping(self):
        data = copy.deepcopy(self.data)
        data["matrix"][0][0] = float("nan")
        with self.assertRaises(ValueError):
            RH56DriveMap.from_dict(data)

    def test_detect_broken_follower_even_inside_limits(self):
        mapping = self.mapping
        q = (mapping.lower + mapping.upper) / 2
        full = mapping.expand(q)
        follower = next(i for i, name in enumerate(mapping.joint_names) if name not in mapping.driver_names)
        full[follower] += .01
        self.assertFalse(mapping.validate(q, full))

    def test_out_of_range_input_is_not_silently_repaired(self):
        q = self.mapping.lower.copy()
        q[0] -= .01
        self.assertFalse(self.mapping.validate(q))
        self.assertLess(self.mapping.expand(q)[self.mapping.joint_names.index(self.mapping.driver_names[0])],
                        self.mapping.lower[0])


class AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(4)
        cls.optimizer = RH56Optimization(MODEL, batch=2)

    def test_full_hand_fk_and_contact_gradient_against_independent_reference(self):
        audit = audit_hand_model(MODEL)
        self.assertEqual(audit["reference_poses"], 11)
        self.assertEqual(audit["links"], 13)
        self.assertEqual(audit["sampled_contact_points"], 75)
        self.assertLess(audit["position_max_error_m"], 2e-6)
        self.assertLess(audit["contact_normal_max_element_error"], 2e-6)
        self.assertTrue(audit["gradient_all_six_drivers_nonzero"])

    def test_global_pose_transforms_points_and_rotates_normals(self):
        hand = self.optimizer.handmodel
        q = torch.zeros(2, 15)
        q[:, 3], q[:, 7] = 1, 1
        q[:, 9:] = (hand.joints_q_lower + hand.joints_q_upper) / 2
        p, n = hand.sample_contact_points_and_normal(q)
        moved = q.clone()
        moved[:, :3] = torch.tensor([.1, -.2, .3])
        moved[:, 3:9] = torch.tensor([0., 1., 0., -1., 0., 0.])
        pp, nn = hand.sample_contact_points_and_normal(moved)
        rotation = torch.tensor([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        torch.testing.assert_close(pp, p @ rotation.T + moved[:, None, :3], atol=1e-6, rtol=0)
        torch.testing.assert_close(nn, n @ rotation.T, atol=1e-6, rtol=0)

    def test_projection_changes_only_drivers(self):
        optimizer = self.optimizer
        q = torch.arange(30, dtype=torch.float32).reshape(2, 15)
        q[:, 9:] = optimizer.q_joint_lower - .2
        optimizer.q_current = q.clone().requires_grad_(True)
        optimizer.project_drivers()
        torch.testing.assert_close(optimizer.q_current[:, :9], q[:, :9])
        torch.testing.assert_close(optimizer.q_current[:, 9:], optimizer.q_joint_lower)

    def test_adam_cannot_leave_driver_box(self):
        optimizer = self.optimizer
        q = torch.zeros(2, 15)
        q[:, 3], q[:, 7] = 1, 1
        q[:, 9:] = optimizer.q_joint_upper - 1e-5
        optimizer.q_current = q.requires_grad_(True)
        optimizer.optimizer = torch.optim.Adam([optimizer.q_current], lr=.1)
        optimizer.global_step = 1000
        optimizer.loss_normalizer = MeanScaleNormalizer()
        # 用可解释的外推目标驱使 Adam 越界，检查更新后的真实投影结果。
        def outward_objective(check):
            loss = ((optimizer.q_current[:, 9:] - optimizer.q_joint_upper - 1) ** 2).sum(dim=1)
            values = {key: loss * 0 for key in ("WH", "PN", "FQH", "QH", "JR", "CK", "FC", "LD", "AB")}
            values["RH"] = loss
            return values
        previous = optimizer.compute_spacial_loss
        try:
            optimizer.compute_spacial_loss = outward_objective
            optimizer.step(False)
        finally:
            optimizer.compute_spacial_loss = previous
        torch.testing.assert_close(optimizer.q_current[:, 9:], optimizer.q_joint_upper)
        self.assertTrue(optimizer.handmodel.drives.validate(optimizer.q_current.detach().numpy()[:, 9:]).all())

    def test_factory_binding_restored_after_construction(self):
        self.assertIs(upstream.HandModel, HandModel)

    def test_final_loss_uses_current_pose_without_updating_running_mean(self):
        optimizer = self.optimizer
        q = torch.zeros(2, 15)
        q[:, 3], q[:, 7] = 1, 1
        q[:, 9:] = optimizer.q_joint_upper
        optimizer.q_current = q.requires_grad_(True)
        optimizer.loss_normalizer = MeanScaleNormalizer()
        optimizer.loss_normalizer.mean = [2., 3., 4., 5., 6., 7., 8.]
        before_mean = optimizer.loss_normalizer.mean.copy()
        before_q = optimizer.q_current.detach().clone()
        def current_pose_loss(check):
            loss = optimizer.q_current[:, 9:].sum(dim=1)
            values = {key: loss * 0 for key in ("WH", "PN", "FQH", "QH", "JR", "CK", "FC", "LD", "AB")}
            values["RH"] = loss
            return values
        previous = optimizer.compute_spacial_loss
        try:
            optimizer.compute_spacial_loss = current_pose_loss
            final = optimizer.evaluate_final()
        finally:
            optimizer.compute_spacial_loss = previous
        np.testing.assert_allclose(final["RH"], before_q[:, 9:].sum(dim=1).numpy())
        np.testing.assert_allclose(final["loss_opti"], final["RH"] / (2 + optimizer.loss_normalizer.eps) * 5)
        self.assertEqual(optimizer.loss_normalizer.mean, before_mean)
        torch.testing.assert_close(optimizer.q_current, before_q)


if __name__ == "__main__":
    unittest.main()
