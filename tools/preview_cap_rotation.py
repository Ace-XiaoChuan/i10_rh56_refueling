#!/usr/bin/env python3
"""显示旋转诊断已检查前缀；失败标记保留，不伪装成完整旋拧。"""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from fuel_cap_task.rotation_diagnostic import RotationReplay
from replay_cap_approach import render, view


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--diagnostic', type=Path, default=Path('artifacts/planning/rotation_single/diagnostic.json'))
    parser.add_argument('--fcl', type=Path, default=Path('artifacts/planning/rotation_single/fcl_validation.json'))
    parser.add_argument('--render-dir', type=Path)
    parser.add_argument('--view', action='store_true')
    parser.add_argument('--auto-close', action='store_true')
    args = parser.parse_args()
    replay = RotationReplay(args.diagnostic, args.fcl)
    summary = {'mode': 'fixed_depth_preform_rotation_prefix_not_execution',
               'target_angle_deg': replay.diagnostic['target_angle_deg'],
               'checked_prefix_angle_deg': float(replay.angles[-1]),
               'fcl_failure': replay.fcl['failure'], 'mujoco_failure': replay.diagnostic['failure'],
               'full_rotation_passed': (replay.diagnostic['failure'] is None and replay.fcl['failure'] is None
                                        and abs(replay.angles[-1]-replay.diagnostic['target_angle_deg']) < 1e-12),
               'physical_grasp_verified': False, 'thread_motion_verified': False,
               'diagnostic_sha256': replay.fcl['diagnostic_sha256']}
    args.diagnostic.with_name('summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
    if args.render_dir:
        render(replay, args.render_dir)
        (args.render_dir/'approach.png').rename(args.render_dir/'rotation.png')
        (args.render_dir/'index.html').write_text(
            '<meta charset="utf-8"><title>固定深度旋转初筛</title><h2>固定深度旋转初筛</h2>'
            f'<p>只显示已检查前缀 0～{replay.angles[-1]:.1f}°；目标 {summary["target_angle_deg"]}°。'
            '手保持接近预型；没有闭合或螺纹轴向运动。</p>'
            f'<p>完整转角通过：{summary["full_rotation_passed"]}；失败原因见同目录上级 summary.json。</p>'
            '<img src="rotation.png" width="900">')
    if args.view:
        view(replay, duration=8., auto_close=args.auto_close)


if __name__ == '__main__':
    main()
