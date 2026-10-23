"""核对固定候选到预览、MTC 请求及执行参数的适配；不执行机器人运动。"""

import asyncio
import copy
from dataclasses import asdict
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))

from fuel_cap_task.arm_ik import ArmIKPreview
from fuel_cap_task.fixed_grasp import DEFAULT_FIXED_GRASP
from fuel_cap_task.grasp_preparation import check_mtc_fields
from fuel_cap_task.interfaces import TaskParameters
from fuel_cap_task.planning_scene_snapshot import check_snapshot, scene_snapshot
from fuel_cap_task.random_scenes import SceneSampler, SamplingSettings


class GraspPreparationTests(unittest.TestCase):
    """用当前模型及真实工具入口检查字段含义和误传的拒绝行为。"""

    @classmethod
    def setUpClass(cls):
        """只读取一次基准候选与盖体，后续检查保留这份输入。"""
        cls.preview = ArmIKPreview(DEFAULT_FIXED_GRASP)
        cls.prepared = cls.preview.preparation.with_approach(.1)

    def test_approach_reuses_the_same_candidate_observation_and_targets(self):
        """添加距离不会重新采样，序列化也不会改变原始输入。"""
        original = self.preview.preparation
        self.assertIs(self.prepared.grasp, original.grasp)
        self.assertIs(self.prepared.cap, original.cap)
        self.assertIs(self.prepared.targets, original.targets)
        self.assertIsNone(original.approach)
        self.assertEqual(self.prepared.approach.approach_distance_m, .1)
        self.assertEqual(self.prepared.mtc_fields()['cap_estimate'], asdict(original.cap))

    def test_mtc_uses_world_hand_target_and_separate_channel_commands(self):
        """数值预期独立于序列化实现，能识别手根/法兰或两组指令误传。"""
        request = self.prepared.mtc_fields()
        self.assertEqual((request['target_frame_id'], request['target_link']), ('world', 'base'))
        np.testing.assert_allclose(request['hand_position'], [-.075, .847, .58], atol=1e-12)
        np.testing.assert_allclose(request['pregrasp_position'], [-.075, .747, .58], atol=1e-12)
        self.assertEqual(request['pregrasp_channels'], [0, 0, 800, 800, 900, 0])
        self.assertEqual(request['grasp_channels'], [0, 0, 440, 390, 660, 0])
        check_mtc_fields(request, self.prepared)
        request['hand_position'] = self.prepared.targets.world_from_flange[:3, 3].tolist()
        with self.assertRaises(ValueError):
            check_mtc_fields(request, self.prepared)

    def test_task_parameters_preserve_existing_physical_values(self):
        """只替换抓姿及两组指令，不填写未知值，不改写调用方原对象。"""
        existing = TaskParameters(unlock_angle_deg=180., thread_lead_m_per_turn=.003,
                                  retreat_distance_m=.06)
        parameters = self.prepared.task_parameters(existing)
        self.assertEqual(parameters.approach_channels, (0, 0, 800, 800, 900, 0))
        self.assertEqual(parameters.grasp_channels, (0, 0, 440, 390, 660, 0))
        self.assertEqual(parameters.grasp_pose_in_cap, self.prepared.grasp.cap_from_hand)
        self.assertEqual(parameters.unlock_angle_deg, 180.)
        self.assertEqual(parameters.thread_lead_m_per_turn, .003)
        self.assertEqual(parameters.retreat_distance_m, .06)
        self.assertIsNone(parameters.hold_duration_s)
        self.assertIsNone(existing.approach_channels)
        self.assertIsNone(self.prepared.task_parameters().thread_lead_m_per_turn)

    def test_wrong_request_fields_are_rejected(self):
        """检查目标、参考系、方向、指令和来源的错误，不只做格式校验。"""
        original = self.prepared.mtc_fields()
        changes = {
            'target_link': 'wrist3_Link', 'target_frame_id': 'base_link',
            'hand_position': [0, 0, 0], 'pregrasp_position': [0, 0, 0],
            'outward_axis_world': [0, 1, 0], 'hand_orientation_xyzw': [0, 0, 0, 1],
            'pregrasp_channels': original['grasp_channels'],
            'grasp_channels': original['pregrasp_channels'], 'approach_distance_m': .2,
            'candidate_provenance': {},
            'cap_estimate': {**original['cap_estimate'], 'valid': False},
        }
        for field, value in changes.items():
            request = copy.deepcopy(original)
            request[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                check_mtc_fields(request, self.prepared)
        legacy = copy.deepcopy(original)
        for field in ('target_frame_id', 'target_link', 'grasp_channels', 'candidate_provenance'):
            legacy.pop(field)
        check_mtc_fields(legacy, self.prepared)

    def test_distance_is_required_and_invalid_distances_are_rejected(self):
        """单独查看抓取目标可省略距离，MTC 请求需要明确的有限正距离。"""
        with self.assertRaises(ValueError):
            self.preview.preparation.mtc_fields()
        for distance in (0, -.1, float('nan'), float('inf'), True):
            with self.subTest(distance=distance), self.assertRaises(ValueError):
                self.preview.preparation.with_approach(distance)

    def test_synchronous_preview_works_inside_an_async_caller(self):
        """场景有同步读取入口，适配器不在外层事件循环中再运行 asyncio.run。"""
        async def read():
            return ArmIKPreview(DEFAULT_FIXED_GRASP).estimate
        self.assertTrue(asyncio.run(read()).valid)

    def test_request_cli_matches_preparation_in_baseline_and_random_scene(self):
        """真实命令行同时验证起点、候选绑定及随机障碍快照，不调用规划器。"""
        for seed in (None, 20261002):
            with self.subTest(seed=seed), tempfile.TemporaryDirectory() as temporary:
                output = Path(temporary) / 'request.json'
                command = [sys.executable, str(ROOT / 'tools/prepare_cap_approach.py'),
                           '--start', '0', '-.5', '1.2', '0', '0', '0',
                           '--distance', '.1', '--output', str(output)]
                if seed is None:
                    preview = self.preview
                else:
                    command += ['--random-seed', str(seed)]
                    sampler = SceneSampler(DEFAULT_FIXED_GRASP, SamplingSettings(random_seed=seed))
                    preview = sampler.preview(sampler.single_sample())
                subprocess.run(command, check=True, capture_output=True, text=True)
                request = json.loads(output.read_text())
                check_mtc_fields(request, preview.preparation.with_approach(.1))
                self.assertEqual(request['candidate_path'], 'config/fixed_grasp.json')
                self.assertEqual(request['source_hashes'][request['candidate_path']],
                                 preview.fixed_grasp.provenance.candidate_sha256)
                self.assertEqual([request['initial_joint_positions'][name] for name in request['arm_joints']],
                                 [0., -.5, 1.2, 0., 0., 0.])
                if seed is not None:
                    baseline = json.loads((ROOT / 'src/i10_rh56_moveit_config/config/scene.json').read_text())
                    check_snapshot(request['planning_scene'], scene_snapshot(baseline, preview.model, preview.data))

    def test_inspection_cli_matches_the_same_pregrasp_target(self):
        """数值查看和 MTC 字段使用同一套准备及序列化接口。"""
        result = subprocess.run([sys.executable, str(ROOT / 'tools/read_grasp_targets.py'),
                                 '--distance', '.1'], check=True, capture_output=True, text=True)
        report = json.loads(result.stdout)
        self.assertEqual(report['approach_targets'], self.prepared.approach_record())
        self.assertEqual(report['candidate_provenance'], asdict(self.prepared.grasp.provenance))


if __name__ == '__main__':
    unittest.main()
