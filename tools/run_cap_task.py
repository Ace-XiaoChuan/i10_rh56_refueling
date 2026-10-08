#!/usr/bin/env python3
"""用合成反馈运行取盖编排，不执行机器人运动或物理判据。"""

import argparse
import asyncio
from dataclasses import asdict
import json
from math import isfinite
from pathlib import Path
import signal
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from fuel_cap_task import CapTask, State
from fuel_cap_task.execution_contract import RunLimits
from fuel_cap_task.mock import MOCK_SCENARIOS, MockEstimator, scenario_executor


async def run_demo(args) -> int:
    executor = scenario_executor(args.scenario, fail_at=args.fail_at, delay_s=args.delay)
    # 默认段数只对应所选有限演示脚本，不作为硬件动作预算。
    segments = args.max_segments if args.max_segments is not None else (2 if args.scenario == "regrasp" else 1)
    task = CapTask(MockEstimator(), executor, limits=RunLimits(segments, args.total_timeout),
                   step_timeout_s=args.timeout, stop_timeout_s=args.timeout)
    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGINT, task.cancel)
    loop.add_signal_handler(signal.SIGTERM, task.cancel)
    timer = loop.call_later(args.cancel_after, task.cancel) if args.cancel_after is not None else None
    try:
        result = await task.run()
    finally:
        if timer is not None:
            timer.cancel()
        loop.remove_signal_handler(signal.SIGINT)
        loop.remove_signal_handler(signal.SIGTERM)
    records = [asdict(event) for event in task.events]
    if args.events:
        args.events.write_text("".join(json.dumps(event, ensure_ascii=False) + "\n" for event in records))
    print(json.dumps({"result": asdict(result), "events": records, "calls": executor.calls,
                      "progress": asdict(task.progress)}, ensure_ascii=False, indent=2))
    return 0 if result.state == State.COMPLETED else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=MOCK_SCENARIOS, default="default")
    parser.add_argument("--fail-at", choices=["check_ready", "approach", "grasp", "twist",
                                              "check_support", "release", "withdraw", "retreat", "hold"])
    parser.add_argument("--max-segments", type=int, help="maximum software twist stages")
    parser.add_argument("--total-timeout", type=float, default=30.0, help="total software wait budget, seconds")
    parser.add_argument("--cancel-after", type=float, help="request cancellation after N seconds")
    parser.add_argument("--delay", type=float, default=0.01, help="mock delay per operation, seconds")
    parser.add_argument("--timeout", type=float, default=5.0, help="demo operation/stop timeout, seconds")
    parser.add_argument("--events", type=Path, help="write JSONL events to this file")
    args = parser.parse_args()
    if (not all(isfinite(v) for v in (args.timeout, args.delay, args.total_timeout))
            or args.timeout <= 0 or args.total_timeout <= 0 or args.delay < 0
            or (args.max_segments is not None and args.max_segments <= 0)
            or (args.cancel_after is not None
                and (not isfinite(args.cancel_after) or args.cancel_after < 0))):
        parser.error("budgets must be finite and positive; delay and cancel-after must be finite and nonnegative")
    return asyncio.run(run_demo(args))


if __name__ == "__main__":
    raise SystemExit(main())
