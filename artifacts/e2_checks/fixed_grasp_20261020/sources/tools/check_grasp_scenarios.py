#!/usr/bin/env python3
"""固定变化案例，先保存独立预测，再检查目标、端点 IK 和无效输入。

本工具不调用路径规划器；路径结果需用真实 MTC 请求和回放另行检查。
预测使用整组目标绕开孔/盖心的刚体运动，不重复调用被检查的目标计算函数。
"""

import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import sys
import tempfile

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from fuel_cap_task.arm_ik import body_transform
from fuel_cap_task.fixed_grasp import DEFAULT_FIXED_GRASP, load_fixed_grasp
from fuel_cap_task.grasp_preparation import check_mtc_fields, prepare_grasp
from fuel_cap_task.interfaces import TaskFailure
from fuel_cap_task.random_scenes import SceneSampler, SamplingSettings, classify, save_report
from fuel_cap_task.validation import finite_real


def axis_rotation(axis, angle_deg):
    """用 Rodrigues 公式表达预测的整体旋转；不使用目标计算中的四元数转换。"""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    angle = np.deg2rad(angle_deg)
    return np.eye(3) + np.sin(angle) * cross + (1 - np.cos(angle)) * (cross @ cross)


def scenario_samples(sampler):
    """在求解前冻结案例，包含单因素变化、固定种子混合变化和明显超出臂长的目标。"""
    opening = sampler.base_opening
    # 基座相对世界绕 Z 转了 180°；先把指定的世界平移转换到基座系。
    translated = opening + sampler.world_from_base[:3, :3].T @ [.04, -.06, .03]
    definitions = [
        ('baseline', opening, 0., 0.),
        ('translation', translated, 0., 0.),
        ('tank_yaw_20', opening, 20., 0.),
        ('cap_spin_90', opening, 0., 90.),
        ('unreachable', [3., -3., .6], 0., 0.),
    ]
    rows = [dict(id=i, group=name, opening_base_m=np.asarray(p).tolist(),
                 tank_yaw_deg=yaw, cap_yaw_deg=spin)
            for i, (name, p, yaw, spin) in enumerate(definitions)]
    mixed = sampler.single_sample()
    rows.append({**mixed, 'id': len(rows), 'group': 'mixed_seed_20261002'})
    return rows


def predict_targets(sampler, sample, distance):
    """围绕实物转轴移动基准目标，独立预测变化后的世界系目标及外退方向。"""
    reference = sampler.reference.preparation.targets
    base = sampler.world_from_base
    yaw = axis_rotation(base[:3, 2], sample['tank_yaw_deg'])
    spin = axis_rotation(reference.world_from_cap[:3, 2], sample['cap_yaw_deg'])
    cap_center = reference.world_from_cap[:3, 3]
    desired_opening = base[:3, 3] + base[:3, :3] @ sample['opening_base_m']
    predicted = {}
    for name in ('world_from_cap', 'world_from_hand', 'world_from_flange'):
        original = getattr(reference, name)
        target = np.eye(4)
        # 先绕原盖心转动，再随整个油箱绕开孔改变方向并平移到指定开孔位置。
        point = cap_center + spin @ (original[:3, 3] - cap_center)
        target[:3, 3] = desired_opening + yaw @ (point - sampler.opening)
        target[:3, :3] = yaw @ spin @ original[:3, :3]
        predicted[name] = target.tolist()
    axis = yaw @ reference.world_from_cap[:3, 2]
    predicted['outward_axis_world'] = axis.tolist()
    for name in ('hand', 'flange'):
        target = np.array(predicted[f'world_from_{name}'])
        target[:3, 3] += distance * axis
        predicted[f'world_from_pregrasp_{name}'] = target.tolist()
    return predicted


def check_prediction(prepared, predicted, world_from_base, mount):
    """比较独立预测与真实输出，并核对完整安装链和基座转换；误差为矩阵元素最大值。"""
    actual = {name: getattr(prepared.targets, name) for name in
              ('world_from_cap', 'world_from_hand', 'world_from_flange')}
    actual.update(asdict(prepared.approach))
    errors = {name: float(np.max(np.abs(np.asarray(actual[name]) - expected)))
              for name, expected in predicted.items()}
    t = prepared.targets
    errors.update(
        mount_chain=float(np.max(np.abs(t.world_from_flange @ mount - t.world_from_hand))),
        base_hand=float(np.max(np.abs(world_from_base @ t.base_from_hand - t.world_from_hand))),
        base_flange=float(np.max(np.abs(world_from_base @ t.base_from_flange - t.world_from_flange))),
    )
    if max(errors.values()) > 1e-10:
        raise ValueError(f'目标计算与独立预测不一致：{errors}')
    check_mtc_fields(prepared.mtc_fields(), prepared)
    return errors


def invalid_input_checks(reference, distance):
    """实际调用公共入口，保存各个错误被拒绝的位置和原始原因，不模拟成功回执。"""
    prepared = reference.preparation.with_approach(distance)
    rows = []

    def reject(name, stage, call, expected):
        """只有指定异常算预期拒绝；未拒绝或其他异常均使检查失败。"""
        try:
            call()
        except expected as error:
            rows.append(dict(case=name, stage=stage, rejected=True,
                             error_type=type(error).__name__, reason=str(error)))
        else:
            raise AssertionError(f'{name} 未拒绝错误输入')

    def prepare(cap=reference.estimate, mount=None, base=None):
        """保留其余有效输入，每次只改变一个被检查字段。"""
        return prepare_grasp(reference.fixed_grasp, cap,
                             flange_from_hand=reference.flange_from_hand if mount is None else mount,
                             world_from_base=body_transform(reference.data, 'base_link') if base is None else base)

    cap = reference.estimate
    cap_cases = {
        'invalid_observation': replace(cap, valid=False),
        'wrong_observation_frame': replace(cap, pose=replace(cap.pose, frame_id='base_link')),
        'nonfinite_position': replace(cap, pose=replace(cap.pose, position_m=(float('nan'), 0., 0.))),
        'zero_quaternion': replace(cap, pose=replace(cap.pose, orientation_xyzw=(0., 0., 0., 0.))),
        'nonfinite_timestamp': replace(cap, timestamp_s=float('inf')),
        'missing_source': replace(cap, source=''),
    }
    for name, value in cap_cases.items():
        reject(name, 'observation', lambda value=value: prepare(value), TaskFailure)
    for value in (0., -.1, float('nan'), float('inf'), True):
        reject(f'invalid_distance_{value}', 'approach',
               lambda value=value: prepared.with_approach(value), ValueError)
    reflection = np.eye(4)
    reflection[0, 0] = -1
    reject('invalid_mount_shape', 'transform', lambda: prepare(mount=np.eye(3)), ValueError)
    reject('reflected_mount', 'transform', lambda: prepare(mount=reflection), ValueError)
    reject('reflected_base', 'transform', lambda: prepare(base=reflection), ValueError)

    original = json.loads(reference.candidate_path.read_text())
    candidate_cases = {
        'wrong_scene_hash': {'source_scene_sha256': '0' * 64},
        'wrong_mujoco_version': {'mujoco_version': '0.0.0'},
        'wrong_channel_count': {'pregrasp_channels': [0] * 5},
        'channel_out_of_range': {'grasp_channels': [1001] * 6},
        'candidate_wrong_frame': {'cap_from_hand': {**original['cap_from_hand'], 'frame_id': 'world'}},
        'candidate_zero_quaternion': {'cap_from_hand': {**original['cap_from_hand'], 'orientation_xyzw': [0] * 4}},
    }
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / 'invalid.json'
        reject('missing_candidate', 'candidate', lambda: load_fixed_grasp(path), FileNotFoundError)
        for name, change in candidate_cases.items():
            path.write_text(json.dumps({**original, **change}))
            reject(name, 'candidate', lambda: load_fixed_grasp(
                path, source_scene_path=reference.scene_path, expected_mujoco_version=mujoco.__version__), ValueError)

    request = prepared.mtc_fields()
    changes = {
        'flange_sent_as_hand': {'hand_position': prepared.targets.world_from_flange[:3, 3].tolist()},
        'request_wrong_frame': {'target_frame_id': 'base_link'},
        'request_wrong_link': {'target_link': 'wrist3_Link'},
        'request_wrong_axis': {'outward_axis_world': (-prepared.approach.outward_axis_world).tolist()},
        'close_during_approach': {'pregrasp_channels': request['grasp_channels']},
        'request_wrong_close': {'grasp_channels': request['pregrasp_channels']},
        'request_wrong_observation': {'cap_estimate': {**request['cap_estimate'], 'valid': False}},
        'request_wrong_provenance': {'candidate_provenance': {}},
    }
    for name, change in changes.items():
        reject(name, 'mtc_fields', lambda change=change: check_mtc_fields({**request, **change}, prepared), ValueError)
    return rows


def geometry_fault_checks(sampler, distance):
    """主动注入三种坐标错误，确认独立预测能发现它们；不把注入错误算作实现故障。"""
    sample = next(row for row in scenario_samples(sampler) if row['group'] == 'tank_yaw_20')
    preview = sampler.preview(sample)
    prepared = preview.preparation.with_approach(distance)
    predicted = predict_targets(sampler, sample, distance)
    wrong_pregrasp = prepared.targets.world_from_hand.copy()
    wrong_pregrasp[:3, 3] += [0., -distance, 0.]
    faults = {
        'mount_used_without_inverse': replace(prepared, targets=replace(
            prepared.targets, world_from_flange=prepared.targets.world_from_hand @ preview.flange_from_hand)),
        'fixed_world_approach_axis': replace(prepared, approach=replace(
            prepared.approach, world_from_pregrasp_hand=wrong_pregrasp)),
        'stale_baseline_targets': sampler.reference.preparation.with_approach(distance),
    }
    rows = []
    for name, fault in faults.items():
        try:
            check_prediction(fault, predicted, sampler.world_from_base, preview.flange_from_hand)
        except ValueError as error:
            rows.append(dict(case=name, injected=True, detected=True, reason=str(error)))
        else:
            raise AssertionError(f'独立预测未发现注入的坐标错误：{name}')
    return rows


def run(candidate, output, distance):
    """保存预测、逐例结果和失败信息；IK 失败保留原场景，不换样或把最优残差当成功。"""
    sampler = SceneSampler(candidate, SamplingSettings(random_seed=20261002))
    reference = sampler.reference
    # 用约定中的绝对数值锚定基准，避免错误基准经整体运动后仍自洽。
    np.testing.assert_allclose(reference.world_from_hand[:3, 3], [-.075, .847, .58], atol=1e-12)
    np.testing.assert_allclose(reference.world_from_flange[:3, 3],
                               [-.10832293327567549, .8190387389786354, .58], atol=1e-12)
    np.testing.assert_allclose(reference.world_from_cap[:3, :3],
                               [[0, -1, 0], [0, 0, -1], [1, 0, 0]], atol=1e-12)
    samples = scenario_samples(sampler)
    predictions = [dict(sample=sample, expected=predict_targets(sampler, sample, distance))
                   for sample in samples]
    save_report(output.with_suffix('.predictions.json'), predictions)
    report = dict(schema_version=1, mode='fixed_grasp_geometry_and_endpoint_checks',
                  complete=False, approach_distance_m=distance, mujoco_version=mujoco.__version__,
                  candidate_provenance=asdict(reference.fixed_grasp.provenance),
                  sampling_settings=asdict(sampler.settings), planned_samples=samples,
                  checker_sources={str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                   for path in [Path(__file__), ROOT / 'src/fuel_cap_task/grasp_targets.py',
                                                ROOT / 'src/fuel_cap_task/grasp_preparation.py',
                                                ROOT / 'src/fuel_cap_task/arm_ik.py',
                                                ROOT / 'src/fuel_cap_task/random_scenes.py']},
                  results=[], invalid_inputs=[], geometry_faults=[],
                  path_verified=False, physical_grasp_verified=False)
    save_report(output, report)
    for planned in predictions:
        sample = planned['sample']
        p = sampler.preview(sample)
        prepared = p.preparation.with_approach(distance)
        errors = check_prediction(prepared, planned['expected'], sampler.world_from_base, p.flange_from_hand)
        ik = p.solve()
        # 所有臂关节均为转动关节；固定局部平移的长度之和给出保守可达半径上界。
        link = p.flange
        base_id = p.model.body('base_link').id
        reach_bound = 0.
        while link != base_id:
            if link == 0:
                raise ValueError('法兰不属于 base_link 的子树')
            first = int(p.model.body_jntadr[link])
            count = int(p.model.body_jntnum[link])
            if any(p.model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE
                   for j in range(first, first + count)):
                raise ValueError('当前臂长上界只适用于全部采用转动关节的臂链')
            reach_bound += float(np.linalg.norm(p.model.body_pos[link]))
            link = int(p.model.body_parentid[link])
        distance_from_base = float(np.linalg.norm(p.base_from_flange[:3, 3]))
        if sample['group'] == 'unreachable':
            if ik['ik_found'] or distance_from_base <= reach_bound:
                raise AssertionError('远距案例应超出保守臂长上界，并且不能返回 IK 成功')
            np.testing.assert_array_equal(p.data.qpos[p.addresses], p.initial)
        row = dict(sample=sample, geometry_passed=True, matrix_max_errors=errors,
                   predicted=planned['expected'], approach=prepared.approach_record(),
                   request_fields=prepared.mtc_fields(),
                   variation_source_sha256=p.variation_source_sha256,
                   status=classify(ik), ik=ik,
                   reach_check=dict(flange_distance_from_base_m=distance_from_base,
                                    conservative_reach_bound_m=reach_bound,
                                    beyond_reach_bound=distance_from_base > reach_bound))
        report['results'].append(row)
        save_report(output, report)
        print(f"{sample['group']}: geometry OK; {row['status']}; {len(ik['solutions'])} IK solutions", flush=True)
    report['invalid_inputs'] = invalid_input_checks(reference, distance)
    report['geometry_faults'] = geometry_fault_checks(sampler, distance)
    report['complete'] = True
    save_report(output, report)
    print(f"Rejected {len(report['invalid_inputs'])} invalid inputs; saved {output}", flush=True)
    return sampler, report


def main():
    """距离必须显式提供；输出必须另存，不覆盖输入候选。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, default=DEFAULT_FIXED_GRASP)
    parser.add_argument('--distance', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--render-dir', type=Path)
    args = parser.parse_args()
    if not finite_real(args.distance) or args.distance <= 0:
        parser.error('distance 必须是以米为单位的有限正数')
    if args.output.resolve() == args.candidate.resolve() or args.output.with_suffix('.predictions.json').resolve() == args.candidate.resolve():
        parser.error('输出不能覆盖输入候选')
    sampler, report = run(args.candidate, args.output, args.distance)
    if args.render_dir:
        from sample_cap_scenes import render
        render(sampler, report, args.render_dir)


if __name__ == '__main__':
    main()
