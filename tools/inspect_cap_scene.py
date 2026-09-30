#!/usr/bin/env python3
"""Inspect the cap scene and provisional RH56 grasp; no dynamics or hardware."""

import argparse
import asyncio
import json
from pathlib import Path
from queue import SimpleQueue
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import mujoco

from fuel_cap_task.interfaces import TaskParameters
from fuel_cap_task.mujoco_scene import MujocoCapScene


def configure_view(scene, camera, options, focus):
    options.geomgroup[0] = 0
    options.geomgroup[3] = 0
    options.frame = mujoco.mjtFrame.mjFRAME_NONE
    camera.azimuth = -110
    camera.elevation = -20
    if focus == "hand":
        camera.lookat[:] = (scene.data.site("right_thumb_tip").xpos + scene.data.site("right_index_tip").xpos) / 2
        camera.distance = 0.45
    elif focus == "cap":
        camera.lookat[:] = scene.data.site("cap_grip_center").xpos
        camera.distance = 0.3
        camera.azimuth = -90
    else:
        camera.lookat[:] = [0, 0.5, 0.55]
        camera.distance = 3.2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="print model, mapping and pose without a window")
    parser.add_argument("--focus", choices=["hand", "scene", "cap"], default="hand")
    parser.add_argument("--channels", nargs=6, type=int, default=TaskParameters().grasp_channels,
                        metavar="N", help="six preview commands in documented channel order")
    args = parser.parse_args()
    if any(value < 0 or value > 1000 for value in args.channels):
        parser.error("channel values must be in 0..1000")
    scene = MujocoCapScene()
    channels = args.channels
    scene.hand.apply(scene.data, channels)

    def report():
        print(json.dumps(asyncio.run(scene.report(channels)), ensure_ascii=False, indent=2), flush=True)

    report()
    if args.check:
        return

    import mujoco.viewer

    keys = SimpleQueue()
    print("G: requested grasp | O: model open endpoints | P: current pose report; hand preset stays active.")
    with mujoco.viewer.launch_passive(scene.model, scene.data, key_callback=keys.put) as viewer:
        with viewer.lock():
            configure_view(scene, viewer.cam, viewer.opt, args.focus)
        while viewer.is_running():
            with viewer.lock():
                print_report = False
                while not keys.empty():
                    key = keys.get()
                    if key == ord("G"):
                        channels = args.channels
                    elif key == ord("O"):
                        channels = [1000] * 6
                    elif key == ord("P"):
                        print_report = True
                scene.hand.apply(scene.data, channels)
                if print_report:
                    report()
            viewer.sync()
            time.sleep(1 / 60)


if __name__ == "__main__":
    main()
