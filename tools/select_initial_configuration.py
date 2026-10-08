#!/usr/bin/env python3
"""筛选当前单场景的初始到达构型，验证真实接近路径与实际末态余量。"""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from fuel_cap_task.initial_configuration_selection import run, verify
from fuel_cap_task.trajectory_replay import ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory', type=Path, default=ROOT/'artifacts/planning/random_single/approach.json')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'artifacts/planning/initial_selection_single_v2')
    parser.add_argument('--verify-only', action='store_true', help='只复核已有来源、路径及范围，不重新求解或规划')
    args = parser.parse_args()
    summary = verify(args.output_dir) if args.verify_only else run(args.trajectory, args.output_dir)
    print((args.output_dir/'report.md').read_text())
    return 0 if summary['analysis_finished'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
