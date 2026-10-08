#!/usr/bin/env python3
"""从已验证接近终点进行固定深度旋转初筛；失败保留检查点，不自动重试。"""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from fuel_cap_task.rotation_diagnostic import RotationDiagnostic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory', type=Path, default=Path('artifacts/planning/random_single/approach.json'))
    parser.add_argument('--output', type=Path, default=Path('artifacts/planning/rotation_single/diagnostic.json'))
    args = parser.parse_args()
    result = RotationDiagnostic(args.trajectory).run(args.output)
    print(json.dumps({k: result[k] for k in ('target_angle_deg', 'mujoco_prefix_angle_deg', 'failure')}, ensure_ascii=False, indent=2))
    return 1 if result['failure'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
