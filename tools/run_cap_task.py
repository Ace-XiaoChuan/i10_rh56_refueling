#!/usr/bin/env python3
"""Run the fuel-cap state machine with synthetic feedback; no robot motion."""

import argparse
import asyncio
from dataclasses import asdict
import json
from pathlib import Path
import signal
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from fuel_cap_task import CapTask, State
from fuel_cap_task.mock import MockEstimator, MockExecutor


async def run_demo(args) -> int:
    executor = MockExecutor(fail_at=args.fail_at, delay_s=args.delay)
    task = CapTask(MockEstimator(), executor, step_timeout_s=args.timeout, stop_timeout_s=args.timeout)
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
    print(json.dumps({"result": asdict(result), "events": records}, ensure_ascii=False, indent=2))
    return 0 if result.state == State.COMPLETED else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fail-at", choices=["check_ready", "approach", "grasp", "twist", "retreat", "hold"])
    parser.add_argument("--cancel-after", type=float, help="request cancellation after N seconds")
    parser.add_argument("--delay", type=float, default=0.01, help="mock delay per operation, seconds")
    parser.add_argument("--timeout", type=float, default=5.0, help="demo operation/stop timeout, seconds")
    parser.add_argument("--events", type=Path, help="write JSONL events to this file")
    args = parser.parse_args()
    if args.timeout <= 0 or args.delay < 0 or (args.cancel_after is not None and args.cancel_after < 0):
        parser.error("timeout must be positive; delay and cancel-after must be nonnegative")
    return asyncio.run(run_demo(args))


if __name__ == "__main__":
    raise SystemExit(main())
