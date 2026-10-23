"""用可手算的数值检查变化场景，并验证常见坐标错误确实能被检查器发现。"""

from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'tools'))

from check_grasp_scenarios import (
    check_prediction, geometry_fault_checks, predict_targets, scenario_samples,
)
from fuel_cap_task.fixed_grasp import DEFAULT_FIXED_GRASP
from fuel_cap_task.random_scenes import SceneSampler, SamplingSettings


class GraspScenarioTests(unittest.TestCase):
    """绝对坐标预期与模型刚体变化相互独立，能识别坐标轴、位置及来源误用。"""

    @classmethod
    def setUpClass(cls):
        """每个案例使用新派生模型，避免静态物体移动后碰撞缓存失效。"""
        cls.sampler = SceneSampler(DEFAULT_FIXED_GRASP, SamplingSettings(random_seed=20261002))
        cls.samples = {row['group']: row for row in scenario_samples(cls.sampler)}

    def prepared(self, name):
        """运行公共目标计算，并与提前形成的预测核对。"""
        sample = self.samples[name]
        predicted = predict_targets(self.sampler, sample, .1)
        preview = self.sampler.preview(sample)
        result = preview.preparation.with_approach(.1)
        check_prediction(result, predicted, self.sampler.world_from_base, preview.flange_from_hand)
        return result

    def test_translation_moves_grasp_and_approach_by_the_same_world_offset(self):
        """世界平移 (40, -60, 30) mm 后，手根及外退方向符合绝对数值预期。"""
        result = self.prepared('translation')
        np.testing.assert_allclose(result.targets.world_from_hand[:3, 3], [-.035, .787, .61], atol=1e-12)
        np.testing.assert_allclose(result.approach.world_from_pregrasp_hand[:3, 3], [-.035, .687, .61], atol=1e-12)
        np.testing.assert_allclose(result.approach.outward_axis_world, [0, -1, 0], atol=1e-12)

    def test_cap_spin_changes_the_hand_offset_but_keeps_the_approach_axis(self):
        """绕盖自身 Z 转 90°，手根为 (20, 847, 525) mm，外退仍沿世界 -Y。"""
        result = self.prepared('cap_spin_90')
        np.testing.assert_allclose(result.targets.world_from_hand[:3, 3], [.02, .847, .525], atol=1e-12)
        np.testing.assert_allclose(result.approach.world_from_pregrasp_hand[:3, 3], [.02, .747, .525], atol=1e-12)
        np.testing.assert_allclose(result.approach.outward_axis_world, [0, -1, 0], atol=1e-12)

    def test_tank_yaw_rotates_the_approach_axis(self):
        """油箱偏航 20° 后，外退向量为 (sin20°, -cos20°, 0) × 0.1 m。"""
        result = self.prepared('tank_yaw_20')
        axis = [np.sin(np.deg2rad(20)), -np.cos(np.deg2rad(20)), 0]
        np.testing.assert_allclose(result.approach.outward_axis_world, axis, atol=1e-12)
        np.testing.assert_allclose(result.approach.world_from_pregrasp_hand[:3, 3] - result.targets.world_from_hand[:3, 3],
                                   .1 * np.array(axis), atol=1e-12)

    def test_prediction_detects_three_independent_geometry_faults(self):
        """安装不取逆、固定世界退开轴和沿用旧目标都必须被拒绝。"""
        faults = geometry_fault_checks(self.sampler, .1)
        self.assertEqual(len(faults), 3)
        self.assertTrue(all(row['detected'] for row in faults))


if __name__ == '__main__':
    unittest.main()
