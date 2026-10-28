#!/usr/bin/env python3
"""从同一份实测体素导出环境 OctoMap 与独立盖体 CollisionObject。"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from fuel_cap_task.pcd_point_cloud import load_pcd
from fuel_cap_task.voxel_mujoco import FACE_NORMALS, exposed_voxel_faces
from fuel_cap_task.voxel_octomap import cap_collision_object, load_occupied_voxels, split_voxel_membership
from export_voxel_octomap import compile_converter, digest, write_json


def write_display(output, indices, size, origin, components, source_digest):
    """将同一占据并集的外露格面按组件着色；内部公共面仍省略，不改变碰撞几何。"""
    display = output/"split_display"
    display.mkdir()
    rows, directions, corners = exposed_voxel_faces(indices)
    root = ET.Element("mujoco", model="split_voxel_components_display")
    ET.SubElement(root, "compiler", angle="radian")
    assets = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    groups = [(0, "environment", ".12 .55 .70 1"), (1, "cap", ".95 .10 .10 1"),
              (2, "mixed", "1 .75 .05 1")]
    count = 0
    for code, name, rgba in groups:
        quads = corners[components[rows] == code]
        if not len(quads):
            continue
        vertices, inverse = np.unique(quads.reshape(-1, 3), axis=0, return_inverse=True)
        faces = inverse.reshape(-1, 4)[:, [[0, 1, 2], [0, 2, 3]]].reshape(-1, 3)
        mesh_name = "surface_"+name
        with (display/(mesh_name+".obj")).open("w") as stream:
            np.savetxt(stream, origin+vertices*size, fmt="v %.17g %.17g %.17g")
            np.savetxt(stream, FACE_NORMALS, fmt="vn %d %d %d")
            normals = np.repeat(directions[components[rows] == code]+1, 2)
            for face, normal in zip(faces+1, normals):
                stream.write(f"f {face[0]}//{normal} {face[1]}//{normal} {face[2]}//{normal}\n")
        ET.SubElement(assets, "mesh", name=mesh_name, file=mesh_name+".obj", inertia="shell")
        ET.SubElement(world, "geom", name=mesh_name, type="mesh", mesh=mesh_name,
                      rgba=rgba, contype="0", conaffinity="0", group="1")
        count += 1
    ET.indent(root)
    (display/"scene.xml").write_text(ET.tostring(root, encoding="unicode")+"\n")
    record = {"source_voxels_sha256": source_digest,
              "validation": {"display_geom_count": count, "exposed_square_faces": len(rows)},
              "files_sha256": {p.name: digest(p) for p in display.iterdir() if p.is_file()}}
    write_json(display/"result.json", record)


def mixed_cell_report(indices, components, membership, cap_rows, points, camera_from_cap):
    """保留混合格所有原始点及最近盖体点距离；距离只作诊断，不推断语义。"""
    tree = cKDTree(points[cap_rows])
    point_cap = np.zeros(len(points), dtype=bool)
    point_cap[cap_rows] = True
    transform = np.asarray(camera_from_cap)
    records = []
    for cell in np.flatnonzero(components == 2):
        rows = np.flatnonzero(membership == cell)
        entries = []
        for row in rows:
            distance, neighbor = tree.query(points[row])
            entries.append({"point_row": int(row), "in_cap_crop": bool(point_cap[row]),
                            "camera_m": points[row].tolist(),
                            "cap_local_m": ((points[row]-transform[:3, 3]) @ transform[:3, :3]).tolist(),
                            "nearest_cap_distance_m": float(distance),
                            "nearest_cap_point_row": int(cap_rows[neighbor])})
        records.append({"source_voxel_row": int(cell), "index_xyz": indices[cell].tolist(),
                        "assigned_to": "environment", "semantic_status": "unresolved", "points": entries})
    return records


def main():
    """准备可分别加载的组件；只写新结果目录，原完整地图和 MTC 场景保持原版本。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-config", type=Path, default=ROOT/"config/active_point_cloud_scene.json")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--converter", type=Path)
    args = parser.parse_args()
    config = json.loads(args.scene_config.read_text())
    settings = config["collision_scene"]
    if settings["mixed_voxel_policy"] != "keep_in_environment":
        parser.error("当前分离入口只支持混合格保留在环境")
    output = (args.output_dir or ROOT/settings["output_dir"]).resolve()
    if output.exists():
        parser.error("输出目录已存在，请指定新目录")
    source_manifest = ROOT/settings["source_octomap_scene"]
    original = json.loads(source_manifest.read_text())
    source_dir = source_manifest.parent
    for name, expected in [(original["map_path"], original["map_sha256"]),
                           ("placement_config.json", original["placement_config_sha256"]),
                           ("scene_config.json", original["scene_config_sha256"])]:
        if digest(source_dir/name) != expected:
            parser.error("原地图或配置副本摘要已改变："+name)
    source = ROOT/config["octomap"]["source_voxels"]
    if original["sources"].get(str(source.resolve())) != digest(source):
        parser.error("当前体素与完整地图来源不同")
    indices, centers, size, origin, original_cap = load_occupied_voxels(source)
    if size != original["resolution_m"]:
        parser.error("原地图分辨率与体素不同")
    with np.load(source, allow_pickle=False) as arrays:
        counts, membership, cap_rows = (arrays[key].copy() for key in
                                       ("point_counts", "point_to_voxel", "cap_point_rows"))
    components, cap_counts = split_voxel_membership(indices, counts, membership, cap_rows)
    if not np.array_equal(original_cap, cap_counts > 0):
        parser.error("原盖体标记与点归属不符")
    cloud_path = ROOT/config["point_cloud"]["path"]
    if digest(cloud_path) != config["point_cloud"]["sha256"]:
        parser.error("清理点云摘要已改变")
    # 复用生成原体素时的 /1000 运算，避免边界点因浮点乘法舍入而换格。
    points = load_pcd(cloud_path).xyz_raw/1000
    if (config["point_cloud"]["frame_id"] != original["camera_frame"]
            or config["point_cloud"]["length_unit"] != "mm" or len(points) != len(membership)
            or not np.array_equal(np.floor((points-origin)/size).astype(np.int64), indices[membership])):
        parser.error("原始点云坐标或逐点体素归属不一致")
    crop_path = ROOT/config["cap_crop"]
    crop = load_pcd(crop_path).xyz_raw/1000
    distances, crop_rows = cKDTree(points).query(crop)
    if (distances.max() > .0000002 or not np.array_equal(np.sort(crop_rows), np.sort(cap_rows))):
        parser.error("当前盖体裁剪与原始标记不同")
    cap = components == 1
    environment = ~cap
    world_camera = np.asarray(original["world_from_camera"])
    collision_object = cap_collision_object(centers[cap], size, world_camera,
                                            object_id=settings["cap_object_id"], frame_id=original["planning_frame"])
    # 通过世界系 box 中心反算原相机格号，独立核对变换和原几何一致。
    world_centers = np.array([[p["position"][a] for a in ("x", "y", "z")]
                              for p in collision_object["primitive_poses"]])
    reconstructed = (world_centers-world_camera[:3, 3]) @ world_camera[:3, :3]
    np.testing.assert_allclose(reconstructed, centers[cap], atol=1e-12, rtol=0)
    camera_from_cap = np.linalg.inv(world_camera) @ np.asarray(original["world_from_cap"])
    mixed = mixed_cell_report(indices, components, membership, cap_rows, points, camera_from_cap)

    with tempfile.TemporaryDirectory(prefix="split_voxel_octomap_") as temporary:
        converter = args.converter.resolve() if args.converter else compile_converter(temporary)
        spec = Path(temporary)/"environment_grid.json"
        write_json(spec, {"voxel_size_m": size, "indices_xyz": indices[environment].tolist()})
        output.mkdir(parents=True)
        subprocess.run([str(converter), str(spec), str(output)], check=True)
    (output/"scene.bt").rename(output/"environment.bt")
    write_json(output/"cap_collision_object.json", collision_object)
    write_json(output/"mixed_voxels.json", {"policy": settings["mixed_voxel_policy"], "cells": mixed})
    np.savez_compressed(output/"voxel_components.npz", indices_xyz=indices, components=components,
                        point_to_voxel=membership, cap_point_rows=cap_rows,
                        cap_point_counts=cap_counts, point_counts=counts)
    shutil.copyfile(source_dir/"placement_config.json", output/"placement_config.json")
    write_json(output/"scene_config.json", config)
    validation = {**json.loads((output/"octomap_validation.json").read_text()),
                  "source_voxel_count": len(indices), "environment_voxel_count": int(environment.sum()),
                  "cap_object_voxel_count": int(cap.sum()), "mixed_voxel_count": int((components == 2).sum()),
                  "cap_object_id": collision_object["id"], "components_disjoint": True,
                  "union_matches_source": bool(np.all(environment | cap) and not np.any(environment & cap)),
                  "point_count_preserved": int(counts.sum()), "cap_crop_point_count": len(cap_rows),
                  "cap_points_in_independent_object": int(cap_counts[cap].sum()),
                  "cap_points_in_environment_mixed_cells": int(cap_counts[components == 2].sum()),
                  "cap_box_max_center_error_m": float(np.max(np.abs(reconstructed-centers[cap]))),
                  "cap_box_full_edge_m": size, "grid_not_resampled": True,
                  "contact_exemptions_added": False, "moveit_scene_loaded": False}
    write_json(output/"validation.json", validation)
    write_display(output, indices, size, origin, components, digest(source))
    manifest = {**original, "schema_version": 2,
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "scene_representation": "environment_octomap_and_cap_collision_object",
                "map_path": "environment.bt", "map_sha256": digest(output/"environment.bt"),
                "cap_object_path": "cap_collision_object.json", "cap_object_sha256": digest(output/"cap_collision_object.json"),
                "cap_object_id": collision_object["id"], "cap_included_in_map": bool((components == 2).any()),
                "independent_cap_cells_in_map": False, "mixed_voxel_policy": settings["mixed_voxel_policy"],
                "voxel_components_path": "voxel_components.npz", "voxel_components_sha256": digest(output/"voxel_components.npz"),
                "allowed_collision_pairs": [], "display_scene_dir": "split_display",
                "display_record_sha256": digest(output/"split_display/result.json"),
                "scene_config_sha256": digest(output/"scene_config.json"),
                "source_octomap_scene": str(source_manifest),
                "sources": {str(p.resolve()): digest(p) for p in
                            [source_manifest, source, cloud_path, crop_path, args.scene_config]},
                "implementation_sha256": {name: digest(ROOT/name) for name in
                                          ["tools/split_voxel_collision_scene.py", "tools/voxel_octomap_export.cpp",
                                           "src/fuel_cap_task/voxel_octomap.py"]}, "validation": validation}
    # 完整地图的语义标记仍在原归档；本结果使用包含混合格类别的组件标记。
    manifest.pop("voxel_labels_path", None)
    manifest.pop("voxel_labels_sha256", None)
    write_json(output/"octomap_scene.json", manifest)
    print(json.dumps({"output_dir": str(output), "validation": validation}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
