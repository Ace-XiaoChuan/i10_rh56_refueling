#!/usr/bin/env python3
"""在 MuJoCo 窗口查看已生成的实测体素场景，只更新几何，不推进物理时间。"""

import argparse
import json
from pathlib import Path
import sys
import threading
import time

import mujoco
import mujoco.viewer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from fuel_cap_task.voxel_mujoco import check_compiled_voxels


def main():
    """读取米制相机系模型；F/G/D 切换完整场景、盖体区域和格子细节。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-dir", type=Path, required=True)
    parser.add_argument("--focus", choices=("full_scene", "cap_region", "voxel_detail"), default="cap_region")
    parser.add_argument("--check", action="store_true", help="无窗口核对模型和可视化容量")
    parser.add_argument("--max-seconds", type=float, help="仅用于窗口启动检查，超时后关闭")
    args = parser.parse_args()
    record = json.loads((args.scene_dir/"result.json").read_text())
    # 同版本优先读取已核对的编译缓存，避免每次打开重新编译数万个格子。
    binary = args.scene_dir/"scene.mjb"
    if binary.exists() and record["mujoco_version"] == mujoco.__version__:
        model = mujoco.MjModel.from_binary_path(str(binary))
    else:
        model = mujoco.MjModel.from_xml_path(str(args.scene_dir/"scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    import numpy as np
    with np.load(args.scene_dir/"voxels.npz", allow_pickle=False) as arrays:
        check_compiled_voxels(model, data, arrays["centers_camera_m"], float(arrays["voxel_size_m"]))
    # 标准查看器的场景容量不可动态扩大；数量过多时明确拒绝，不悄悄丢弃格子。
    capacity = mujoco.viewer._Simulate.MAX_GEOM
    if model.ngeom+10 > capacity:
        raise ValueError(f"体素数量 {model.ngeom} 超出当前 MuJoCo 查看器容量 {capacity}")
    if args.check:
        print(json.dumps({"model_geoms": model.ngeom, "viewer_capacity": capacity,
                          "all_voxels_fit": True, "simulation_time_s": data.time}))
        return
    selected = [args.focus]

    def on_key(key):
        """只修改取景请求；模型与关节状态保持不变。"""
        if key in (ord("F"), ord("G"), ord("D")):
            selected[0] = {ord("F"):"full_scene", ord("G"):"cap_region", ord("D"):"voxel_detail"}[key]

    existing_threads = set(threading.enumerate())
    with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
        # MuJoCo 3.13 的 passive Handle.close() 只发退出请求，不等待 UI 线程。
        # 在模型仍被引用时捕获该线程，窗口关闭后等待实际资源释放完成。
        render_threads = [thread for thread in threading.enumerate()
                          if thread not in existing_threads
                          and getattr(thread, "_target", None) is mujoco.viewer._launch_internal]
        def set_focus(name):
            """自由相机朝向近似 SDK 正视图，初始图像上方为相机 -Y；鼠标可继续旋转。"""
            view = record["camera_views"][name]
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.lookat[:] = view["target_m"]
            viewer.cam.distance = view["distance_m"]
            viewer.cam.azimuth, viewer.cam.elevation = -90, -89.9

        with viewer.lock():
            set_focus(selected[0])
            viewer.opt.sitegroup[:] = 0
            viewer.opt.geomgroup[:2] = 1
            viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        previous = selected[0]
        started = time.monotonic()
        print(f"MuJoCo ready: {model.ngeom} boxes; F full / G cap / D detail; no physics steps", flush=True)
        while viewer.is_running():
            if args.max_seconds is not None and time.monotonic()-started >= args.max_seconds:
                break
            with viewer.lock():
                if selected[0] != previous:
                    set_focus(selected[0])
                    previous = selected[0]
            viewer.sync()
            time.sleep(1/30)
    for thread in render_threads:
        thread.join()
    print("MuJoCo viewer closed cleanly; simulation time remains 0", flush=True)


if __name__ == "__main__":
    main()
