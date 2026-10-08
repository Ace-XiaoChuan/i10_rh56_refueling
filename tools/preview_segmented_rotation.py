#!/usr/bin/env python3
"""核对并显示分段几何实验的已通过部分；不显示被拒绝的路径。"""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
import numpy as np
from fuel_cap_task.arm_ik import body_transform
from fuel_cap_task.planning_scene_snapshot import check_snapshot
from fuel_cap_task.rotation_diagnostic import RotationDiagnostic, hand_target
from fuel_cap_task.segmented_rotation import digest, pose_error, pose_tolerances
from fuel_cap_task.trajectory_replay import ROOT
from replay_cap_approach import render, view


class SegmentedReplay:
    """回放仅使用有完整碰撞复核记录的路点，盖角按阶段保持或推进。"""
    def __init__(self, path):
        self.summary = json.loads(Path(path).read_text())
        summary = self.summary
        limits = pose_tolerances(summary)
        if summary.get('mode') != 'segmented_fixed_depth_preform_geometry_not_execution' or not summary.get('analysis_finished'):
            raise ValueError('分段报告尚未完成')
        source = ROOT/summary['trajectory_path']
        if digest(source) != summary['trajectory_sha256']:
            raise ValueError('原接近轨迹已变更')
        for relative, expected in summary['artifacts_sha256'].items():
            path = (ROOT/relative).resolve()
            if not path.is_relative_to(ROOT) or digest(path) != expected:
                raise ValueError('分段阶段记录已变更')
        self.source = RotationDiagnostic(source)
        self.report = self.source.replay.report
        self.model, self.data = self.source.preview.model, self.source.preview.data
        self.rows = summary['samples']
        if not self.rows:
            raise ValueError('没有可回放的通过前缀')
        q = np.array([r['arm_qpos_rad'] for r in self.rows])
        angles = np.array([r['angle_deg'] for r in self.rows])
        if (q.shape != (len(self.rows), 6) or not np.isfinite(q).all() or not np.isfinite(angles).all()
                or abs(angles[0]) > 1e-12 or np.any(np.diff(angles) < -1e-12)
                or angles[-1] > 225. or abs(angles[-1]-summary['cumulative_angle_deg']) > 1e-12
                or not np.allclose(q[0], self.source.q0, atol=1e-12, rtol=0)
                or np.max(np.abs(np.diff(q, axis=0))) > .02000001):
            raise ValueError('分段路点不连续或盖角无效')
        # 从阶段文件重建通过前缀，防止摘要加入没有 FCL 证据的状态。
        expected_rows = []
        for i in range(1, 4):
            matches = [ROOT/p for p in summary['artifacts_sha256'] if p.endswith(f'/rotation_{i}.json')]
            if not matches:
                break
            rotation_path = matches[0]
            folder = rotation_path.parent
            diagnostic = json.loads(rotation_path.read_text())
            fcl = json.loads((folder/f'rotation_{i}_fcl.json').read_text())
            if not fcl['completed_input_check']:
                break
            if (fcl['diagnostic_sha256'] != digest(rotation_path)
                    or fcl['passed_samples'] != len(diagnostic['samples'])):
                raise ValueError('旋转 FCL 记录不完整')
            if pose_tolerances(diagnostic) != limits:
                raise ValueError('阶段与摘要容差不一致')
            expected_rows.extend(diagnostic['samples'])
            if i <= summary['completed_regrasp_paths']:
                from fuel_cap_task.segmented_rotation import validate_regrasp
                request = json.loads((folder/f'regrasp_{i}_request.json').read_text())
                if pose_tolerances(request) != limits:
                    raise ValueError('重新接近容差与摘要不一致')
                mtc = json.loads((folder/f'regrasp_{i}_mtc.json').read_text())
                rows, _ = validate_regrasp(self.source, request, mtc)
                expected_rows.extend(rows)
        if expected_rows != self.rows:
            raise ValueError('摘要与通过阶段路点不一致')
        for row in self.rows:
            actual_row = self.source.row(np.array(row['arm_qpos_rad']), row['angle_deg'])
            check_snapshot({'objects': row['cap_objects']}, {'objects': actual_row['cap_objects']})
            if actual_row['minimum_joint_margin_rad'] < -1e-8 or any(c['penetration_m'] > .0001 for c in self.source.preview.contacts()):
                raise ValueError('分段路点 MuJoCo 复核失败')
            if row['phase'] == 'rotation':
                expected = hand_target(self.source.cap0, self.source.relative, np.deg2rad(row['angle_deg']))
                p, r = pose_error(body_transform(self.data, 'base'), expected)
                if p > limits['position_m'] or r > limits['orientation_rad']:
                    raise ValueError('旋转目标位姿失效')
        # 显示按记录路点取样，不生成未经阶段检查的新连接轨迹。
        self.set_progress(0.)

    def set_progress(self, fraction):
        index = int(round(np.clip(fraction, 0, 1)*(len(self.rows)-1)))
        row = self.rows[index]
        self.source.set_state(np.array(row['arm_qpos_rad']), row['angle_deg'])
        return f'{row["phase"]}；累计 {row["angle_deg"]:.1f}°；仅预型几何'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--summary', type=Path, default=Path('artifacts/planning/segmented_single/summary.json'))
    parser.add_argument('--render-dir', type=Path)
    parser.add_argument('--view', action='store_true')
    parser.add_argument('--auto-close', action='store_true')
    args = parser.parse_args()
    replay = SegmentedReplay(args.summary)
    print(json.dumps({k: v for k, v in replay.summary.items() if k not in ('samples', 'artifacts_sha256')}, ensure_ascii=False, indent=2))
    if args.render_dir:
        render(replay, args.render_dir, frames=180)
        (args.render_dir/'approach.png').rename(args.render_dir/'segmented.png')
        (args.render_dir/'index.html').write_text('<meta charset="utf-8"><title>分段旋拧几何实验</title>'
            '<h2>分段旋拧几何实验：已检查前缀</h2>'
            f'<p>累计 {replay.summary["cumulative_angle_deg"]}°；完整几何流程通过：{replay.summary["full_geometric_sequence_passed"]}。'
            '手保持接近预型，未执行物理松手或闭合，未模拟螺纹轴向运动。</p>'
            '<p>失败阶段和原因见上级 summary.json；动画节奏不代表执行时间。</p>'
            '<img src="segmented.png" width="900">')
    if args.view:
        view(replay, 12., args.auto_close)


if __name__ == '__main__':
    main()
