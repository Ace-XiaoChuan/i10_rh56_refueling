#!/usr/bin/env python3
"""保存固定虚拟相机的 RGB-D 观测；盖体真值只供独立评价。"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

os.environ.setdefault("MUJOCO_GL", "egl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import mujoco
import numpy as np
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE, MujocoCapScene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    scene = MujocoCapScene()
    m, d = scene.model, scene.data
    width, height = 640, 480
    m.vis.global_.offwidth, m.vis.global_.offheight = width, height
    cam, opt = mujoco.MjvCamera(), mujoco.MjvOption()
    cam.lookat[:] = [0, .86, .42]
    cam.distance, cam.azimuth, cam.elevation = .65, 90, -20
    opt.geomgroup[0] = opt.geomgroup[3] = 0
    opt.sitegroup[:] = 0
    # 不加坐标箭头、接触标记或 GUI 文字，输入只含场景渲染。
    with mujoco.Renderer(m, height=height, width=width) as renderer:
        renderer.update_scene(d, camera=cam, scene_option=opt)
        rgb = renderer.render().copy()
        gl = renderer.scene.camera
        position = (np.array(gl[0].pos) + gl[1].pos) / 2
        forward = np.array(gl[0].forward, dtype=float)
        up = np.array(gl[0].up, dtype=float)
        forward /= np.linalg.norm(forward)
        up /= np.linalg.norm(up)
        right = np.cross(forward, up)
        world_from_optical = np.eye(4)
        world_from_optical[:3, :3] = np.column_stack((right, -up, forward))
        world_from_optical[:3, 3] = position
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
        renderer.disable_depth_rendering()
        renderer.enable_segmentation_rendering()
        segmentation = renderer.render().copy()
    # 已注册的深度为光轴 Z（米），不把无命中/远裁剪区域当成自由空间。
    near = m.vis.map.znear * m.stat.extent
    far = m.vis.map.zfar * m.stat.extent
    valid = np.isfinite(depth) & (depth > near) & (depth < far * .999)
    fy = height / (2 * np.tan(np.deg2rad(m.vis.global_.fovy) / 2))
    intrinsics = [[fy, 0, (width - 1) / 2], [0, fy, (height - 1) / 2], [0, 0, 1]]
    from search_cap_grasp import write_png
    write_png(output / "rgb.png", rgb)
    np.savez_compressed(output / "depth.npz", depth_m=depth, valid=valid)
    # 真值不与推理输入打包；分割 ID 只提取可见盖体表面，用于 IoU 评价。
    cap_id = m.body("cap_body").id
    cap_geoms = np.flatnonzero(m.geom_bodyid == cap_id)
    target_mask = np.isin(segmentation[..., 0], cap_geoms) & (
        segmentation[..., 1] == int(mujoco.mjtObj.mjOBJ_GEOM))
    np.savez_compressed(output / "evaluation_only.npz", target_mask=target_mask)
    metadata = {
        "source": "mujoco_rendered_rgbd", "timestamp_s": float(d.time),
        "timestamp_clock": "simulation", "frame_id": "virtual_camera_optical",
        "image_size_wh": [width, height], "camera_matrix": intrinsics,
        "world_from_optical": world_from_optical.tolist(), "depth_unit": "m",
        "depth_convention": "optical_z", "mujoco_version": mujoco.__version__,
        "camera_configuration_source": "assembly_contract/N2",
        "capture_code_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "scene_xml_sha256": hashlib.sha256(DEFAULT_SCENE.read_bytes()).hexdigest(),
        "visible_cap_pixels_evaluation_only": int(target_mask.sum()),
        "files": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in output.iterdir() if p.is_file()},
        "limitations": "仿真外观与虚拟标定；无物理执行、真实相机或不可见补全",
    }
    (output / "observation.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(output), "cap_pixels": int(target_mask.sum())}))


if __name__ == "__main__":
    main()
