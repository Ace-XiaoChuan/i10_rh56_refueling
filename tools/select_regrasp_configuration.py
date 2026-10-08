#!/usr/bin/env python3
"""在同一成功节点离线筛选构型，验证 MTC 调整路径和实际末态旋转余量。"""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from fuel_cap_task.configuration_selection import run, verify
from fuel_cap_task.trajectory_replay import ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--segment-dir', type=Path, default=ROOT/'artifacts/planning/segmented_single_tol_020')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'artifacts/planning/regrasp_selection_single')
    parser.add_argument('--verify-only', action='store_true', help='只复核已有结果，不重新扫描或规划')
    args = parser.parse_args()
    summary = verify(args.output_dir) if args.verify_only else run(args.segment_dir, args.output_dir)
    print((args.output_dir/'report.md').read_text())
    return 0 if summary['analysis_finished'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
