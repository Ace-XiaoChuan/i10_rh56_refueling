#!/usr/bin/env python3
"""运行已确认分段旋拧的几何实验；失败停止，无物理执行。"""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from fuel_cap_task.segmented_rotation import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory', type=Path, default=Path('artifacts/planning/random_single/approach.json'))
    parser.add_argument('--output-dir', type=Path, default=Path('artifacts/planning/segmented_single'))
    parser.add_argument('--resume-validation', action='store_true', help='仅修复已记录格式复核误报，复用原轨迹')
    args = parser.parse_args()
    result = run(args.trajectory, args.output_dir, resume_validation=args.resume_validation)
    print(json.dumps({k: v for k, v in result.items() if k not in ('samples', 'artifacts_sha256')}, ensure_ascii=False, indent=2))
    return 1 if result['failure'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
