#!/usr/bin/env python3
"""计算当前接近终点的旋转余量；独立留档，失败不改构型、不重试。"""
import argparse
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from fuel_cap_task.interfaces import TaskParameters
from fuel_cap_task.rotation_diagnostic import RotationDiagnostic, RotationScanSettings
from fuel_cap_task.rotation_margin import write_outputs
from fuel_cap_task.segmented_rotation import launch
from fuel_cap_task.trajectory_replay import ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory', type=Path, default=ROOT/'artifacts/planning/random_single/approach.json')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'artifacts/planning/rotation_margin_single')
    parser.add_argument('--max-angle-deg', type=float, default=TaskParameters().unlock_angle_deg)
    parser.add_argument('--angle-step-deg', type=float, default=1.)
    parser.add_argument('--summarize-only', action='store_true', help='只汇总已有诊断及 FCL 审计，不重新计算或规划')
    args = parser.parse_args()
    folder = args.output_dir.resolve()
    if not args.summarize_only:
        settings = RotationScanSettings(args.max_angle_deg, args.angle_step_deg)
        if folder.exists():
            raise ValueError('输出目录已存在，不覆盖或续跑；请使用独立目录')
        source = RotationDiagnostic(args.trajectory.resolve())
        folder.mkdir(parents=True)
        source.run(folder/'diagnostic.json', settings=settings)
        # 完整覆盖仅审计已计算的前缀；不调用 MTC 重新搜索接近或旋转路径。
        os.environ.setdefault('ROS_LOG_DIR', str(folder/'ros_logs'))
        launch('rotation_audit.launch.py', {'diagnostic_path': folder/'diagnostic.json',
               'output_path': folder/'fcl_validation.json'}, folder/'fcl_audit.log')
    summary = write_outputs(folder)
    print((folder/'report.md').read_text())
    print(json.dumps({'checked_prefix_angle_deg': summary['checked_prefix_angle_deg'],
                      'failure': summary['failure'], 'report_path': str(folder/'report.md')},
                     indent=2, ensure_ascii=False))
    # 发现几何边界也属于完成诊断；运行故障、无效来源和非法输入才返回非零。
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
