#!/usr/bin/env python3
"""直接读取当前仿真场景的盖体位姿，以 JSON 展示 CapEstimate 数据流。"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from fuel_cap_task.interfaces import TaskFailure
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE, MujocoCapScene
from fuel_cap_task.validation import validate_estimate


def main() -> int:
    """加载预览场景、读取一次位姿、检查有效性，再打印统一接口数据。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE,
                        help="完整臂手盖体场景 XML；默认使用项目预览场景")
    args = parser.parse_args()

    # 场景持有 model 和 data；读取器与场景共用状态，不另外初始化盖体。
    scene = MujocoCapScene(args.scene)
    cap = scene.estimator.read()
    try:
        # 后续计算目标前也须先检查有效性；此处复用任务层已有校验。
        validate_estimate(cap)
    except TaskFailure as exc:
        parser.exit(1, f"盖体观测无效：{exc}\n")
    print(json.dumps(asdict(cap), ensure_ascii=False, allow_nan=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
