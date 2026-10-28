#!/usr/bin/env python3
"""按当前配置将 SDK 清理点云体素化，生成完整 MuJoCo box 场景及真实渲染图。"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))

import numpy as np
from scipy.spatial import cKDTree

from fuel_cap_task.pcd_point_cloud import load_pcd
from fuel_cap_task.voxelization import voxelize_points
from fuel_cap_task.voxel_mujoco import voxel_scene_xml, check_compiled_voxels


def digest(path):
    """记录本次实际输入和输出，历史实验不随当前配置变化而改写。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_centers(path, centers_m, colors):
    """导出毫米制体素中心 PLY，便于与原始 PCD 同时加载；中心点不表示格子体积。"""
    header = ("ply\nformat ascii 1.0\ncomment camera_sdk_native; length_unit mm\n"
              f"element vertex {len(centers_m)}\nproperty double x\nproperty double y\nproperty double z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
    with path.open("w") as stream:
        stream.write(header)
        np.savetxt(stream, np.column_stack([centers_m*1000, colors]), fmt=["%.9f"]*3+["%d"]*3)


def render_scene(output, model, data, *, voxel_size_m=None):
    """用 MuJoCo 渲染完整模型；容量按全部体素分配，不为截图抽样或删格子。"""
    import mujoco
    from PIL import Image, ImageDraw

    options = mujoco.MjvOption()
    options.sitegroup[:] = 0
    panels = []
    rendered = {}
    with mujoco.Renderer(model, height=900, width=1280, max_geom=model.ngeom+100) as renderer:
        for name in ("full_scene", "cap_region", "voxel_detail"):
            renderer.update_scene(data, camera=name, scene_option=options)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            if renderer.scene.ngeom != model.ngeom:
                raise ValueError("渲染未包含完整体素模型，停止保存不完整预览")
            frame = Image.fromarray(renderer.render().copy())
            frame.save(output/f"{name}.png")
            panels.append(frame.resize((640, 450)))
            rendered[name] = int(renderer.scene.ngeom)
    canvas = Image.new("RGB", (1920, 500), "#f5f7fa")
    draw = ImageDraw.Draw(canvas)
    edge_mm = float(2*model.geom_size[0, 0]*1000 if voxel_size_m is None else voxel_size_m*1000)
    for column, (name, image) in enumerate(zip(("Full measured scene", "Cap and recess", f"Actual {edge_mm:g} mm boxes"), panels)):
        canvas.paste(image, (column*640, 50))
        draw.text((column*640+18, 12), name, fill="#26384a", font_size=23)
    canvas.save(output/"preview.png")
    return rendered


def main():
    """校验点云来源、分格、标记盖体并编译模型，保存可复核的独立归档。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-config", type=Path, default=ROOT/"config/active_point_cloud_scene.json")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--no-render", action="store_true")
    parser.add_argument("--compiled-scene", type=Path, help="复用同一 XML 的已编译场景，避免重复编译；仍逐格校验")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists():
        parser.error("输出目录已存在，请指定新目录以保留历史结果")
    config = json.loads(args.scene_config.read_text())
    source = ROOT/config["point_cloud"]["path"]
    if config["point_cloud"]["frame_id"] != "camera_sdk_native" or config["point_cloud"]["length_unit"] != "mm":
        parser.error("输入须为已注明来源的 SDK 原生相机系毫米点云")
    if digest(source) != config["point_cloud"]["sha256"]:
        parser.error("清理点云摘要与配置不符")
    settings = config["surface_voxelization"]
    if settings["occupancy_policy"] != "observed_surface_only" or settings["mujoco_geom_type"] != "box":
        parser.error("当前入口仅支持观测表面逐格 box，不隐式填充")
    size = settings["voxel_size_m"]
    origin = np.asarray(settings["grid_origin_camera_m"], dtype=float)
    cloud = load_pcd(source)
    points = cloud.xyz_raw/1000
    if len(points) != config["point_cloud"]["point_count"]:
        parser.error("点数与配置不符")
    # 此模型的 world 定义为相机系，因此可直接复用固定网格算法，不做基座换算。
    voxels = voxelize_points(points, np.zeros(points.shape, dtype=np.uint8),
                             size_m=size, origin_world_m=origin)
    crop = load_pcd(ROOT/config["cap_crop"])
    distances, cap_rows = cKDTree(cloud.xyz_raw).query(crop.xyz_raw)
    if float(distances.max()) > .0002 or len(np.unique(cap_rows)) != len(cap_rows):
        parser.error("盖体点未完整保留，不能沿用已有盖体标记")
    cap_mask = np.zeros(len(voxels.indices_xyz), dtype=bool)
    cap_mask[voxels.point_to_voxel[cap_rows]] = True
    candidates = json.loads((ROOT/config["grasp_candidates"]).read_text())
    focus = np.asarray(candidates["candidates"][0]["camera_from_cap"])[:3, 3]
    xml, colors, views = voxel_scene_xml(voxels.centers_world_m, size, cap_mask, focus)

    # 检查所有点都落在所记录的格子中；负坐标必须向下取整，不能截断。
    assigned = voxels.indices_xyz[voxels.point_to_voxel]
    np.testing.assert_array_equal(assigned, np.floor((points-origin)/size).astype(np.int64))
    lower = origin+assigned*size
    assert np.all(points >= lower-1e-12) and np.all(points <= lower+size+1e-12)
    assert int(voxels.point_counts.sum()) == len(points)
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    started = time.monotonic()
    print(f"{len(points)} points -> {len(voxels.indices_xyz)} voxels; edge {size*1000:g} mm; loading MuJoCo", flush=True)
    cache = None
    if args.compiled_scene is not None:
        cache = args.compiled_scene.resolve()
        if (cache/"scene.xml").read_text() != xml+"\n":
            parser.error("编译缓存的 XML 与当前体素场景不同，不能复用")
        model = mujoco.MjModel.from_binary_path(str(cache/"scene.mjb"))
    else:
        model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    validation = check_compiled_voxels(model, data, voxels.centers_world_m, size)
    validation.update({"input_point_count": len(points), "voxel_count": len(voxels.indices_xyz),
                       "every_point_retained_in_membership": True, "every_point_inside_assigned_box": True,
                       "point_counts_sum": int(voxels.point_counts.sum()), "cap_points_retained": len(cap_rows),
                       "cap_voxel_count": int(cap_mask.sum()), "grid_origin_camera_m": origin.tolist(),
                       "unobserved_space_filled": False, "robot_connections": 0,
                       "model_load_and_forward_seconds": time.monotonic()-started})
    output.mkdir(parents=True)
    (output/"scene.xml").write_text(xml+"\n")
    mujoco.mj_saveModel(model, str(output/"scene.mjb"))
    np.savez_compressed(output/"voxels.npz", indices_xyz=voxels.indices_xyz,
                        centers_camera_m=voxels.centers_world_m, point_counts=voxels.point_counts,
                        point_to_voxel=voxels.point_to_voxel, cap_point_rows=cap_rows,
                        cap_voxel_mask=cap_mask, display_rgb=colors,
                        voxel_size_m=size, grid_origin_camera_m=origin)
    write_centers(output/"voxel_centers_camera_mm.ply", voxels.centers_world_m, colors)
    record = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
              "mode": "sdk_cleaned_surface_voxels_mujoco", "source_point_cloud": str(source),
              "source_point_cloud_sha256": digest(source), "scene_config_sha256": digest(args.scene_config),
              "frame_id": "camera_sdk_native", "mujoco_world_frame": "camera_sdk_native",
              "length_unit": "m", "voxel_size_m": float(size), "grid_origin_camera_m": origin.tolist(),
              "occupancy_policy": "observed_surface_only", "color_mode": "depth_pseudocolor_and_cap_red",
              "mujoco_version": mujoco.__version__, "camera_views": views,
              "cap_focus_camera_m": focus.tolist(), "validation": validation}
    if cache is not None:
        record["compiled_cache"] = {"source":str(cache), "xml_exact_match":True,
                                    "mjb_sha256":digest(cache/"scene.mjb")}
    # 先保存已完成的几何核对，图形驱动不可用时仍可复用模型而无需重新编译。
    (output/"result.json").write_text(json.dumps(record, ensure_ascii=False, indent=2)+"\n")
    (output/"validation.json").write_text(json.dumps(validation, ensure_ascii=False, indent=2)+"\n")
    for name in ("tools/voxelize_cleaned_scene.py", "tools/view_point_cloud_voxels.py",
                 "src/fuel_cap_task/voxel_mujoco.py", "src/fuel_cap_task/voxelization.py",
                 "src/fuel_cap_task/pcd_point_cloud.py", "config/active_point_cloud_scene.json"):
        target = output/"source_snapshot"/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/name, target)
    if not args.no_render:
        print("MuJoCo model ready; rendering full scene, cap region and voxel detail", flush=True)
        validation["rendered_geom_counts"] = render_scene(output, model, data)
        (output/"result.json").write_text(json.dumps(record, ensure_ascii=False, indent=2)+"\n")
        (output/"validation.json").write_text(json.dumps(validation, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({"output_dir": str(output), "validation": validation}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
