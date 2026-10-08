#!/usr/bin/env python3
"""复用第一次成功重新接近节点，比较同一盖姿态下的前后可转范围。"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from fuel_cap_task.regrasp_margin import prepare, audit, write_outputs
from fuel_cap_task.trajectory_replay import ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--segment-dir', type=Path, default=ROOT/'artifacts/planning/segmented_single_tol_020')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'artifacts/planning/regrasp_margin_single')
    parser.add_argument('--max-additional-angle-deg', type=float)
    parser.add_argument('--angle-step-deg', type=float, default=1.)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--prepare-only', action='store_true', help='只计算两侧诊断，完整 FCL 审计另行进行')
    mode.add_argument('--summarize-only', action='store_true', help='只复核汇总已有诊断及审计，不重新求解或规划')
    args = parser.parse_args()
    if not args.summarize_only:
        prepare(args.segment_dir, args.output_dir, max_additional_angle_deg=args.max_additional_angle_deg,
                angle_step_deg=args.angle_step_deg)
        if args.prepare_only:
            print(f'两侧诊断已保存：{args.output_dir.resolve()}；完整覆盖审计尚未完成。')
            return 0
        audit(args.output_dir)
    summary = write_outputs(args.output_dir)
    print((args.output_dir/'report.md').read_text())
    print(json.dumps({'checked_range_change_deg': summary['checked_range_change_deg'],
                      'checked_range_increased': summary['checked_range_increased']}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
