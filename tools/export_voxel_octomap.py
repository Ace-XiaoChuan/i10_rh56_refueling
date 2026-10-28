#!/usr/bin/env python3
"""把现有 NPZ 占据格导出为原生 OctoMap，并保存独立的仿真地图摆放清单。"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from fuel_cap_task.voxel_octomap import align_map, load_occupied_voxels, rigid_matrix


def digest(path):
    """对实际读取的文件保留摘要，避免把不同版本的点云、抓姿或地图混用。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, data):
    """结果使用可读 JSON，禁止保存 NaN 或无穷值。"""
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+"\n")


def compile_converter(directory):
    """直接链接本机原生 OctoMap；编译文件只写入临时目录，不安装系统依赖。"""
    executable = Path(directory)/"voxel_octomap_export"
    flags = shlex.split(subprocess.check_output(["pkg-config", "--cflags", "--libs", "octomap"], text=True))
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic",
                    str(ROOT/"tools/voxel_octomap_export.cpp"), "-o", str(executable), *flags], check=True)
    return executable


def main():
    """先校验输入来源，再导出和复读；地图坐标通过清单传递，不烘焙进体素中心。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-config", type=Path, default=ROOT/"config/active_point_cloud_scene.json")
    parser.add_argument("--output-dir", type=Path, help="默认使用场景配置中的新结果目录")
    parser.add_argument("--converter", type=Path, help="复用已编译的原生转换器；默认临时编译")
    args = parser.parse_args()
    config = json.loads(args.scene_config.read_text())
    settings = config["octomap"]
    source = ROOT/settings["source_voxels"]
    output = (args.output_dir or ROOT/settings["output_dir"]).resolve()
    if output.exists():
        parser.error("输出目录已存在，请指定新目录以保留历史结果")
    placement_path = ROOT/settings["placement_config"]
    placement = json.loads(placement_path.read_text())
    if (placement["mode"] != "simulation_cap_alignment" or placement["valid_for_hardware"] is not False
            or placement["right_base_from_left_base"] is not None):
        parser.error("此入口只处理显式标注的临时仿真摆放")
    source_record = json.loads(source.with_name("result.json").read_text())
    if (source_record["frame_id"] != placement["camera_frame"]
            or source_record["length_unit"] != "m"
            or source_record["occupancy_policy"] != "observed_surface_only"
            or source_record["source_point_cloud_sha256"] != config["point_cloud"]["sha256"]
            or digest(ROOT/config["point_cloud"]["path"]) != config["point_cloud"]["sha256"]):
        parser.error("体素与当前相机系清理点云不匹配")
    indices, centers, size, origin, cap_mask = load_occupied_voxels(source)
    if (size != config["surface_voxelization"]["voxel_size_m"]
            or not np.array_equal(origin, config["surface_voxelization"]["grid_origin_camera_m"])):
        parser.error("体素网格与当前设置不同")
    candidates_path = ROOT/config["grasp_candidates"]
    candidates = json.loads(candidates_path.read_text())
    if digest(candidates["geometry_result"]) != candidates["geometry_result_sha256"]:
        parser.error("盖体估计来源已改变")
    candidate = next(c for c in candidates["candidates"] if c["candidate_id"] == placement["candidate_id"])
    observed = rigid_matrix(candidate["camera_from_cap"])
    target = rigid_matrix(placement["world_from_cap"])
    anchor = ROOT/placement["anchor_source"]["path"]
    if digest(anchor) != placement["anchor_source"]["sha256"]:
        parser.error("仿真参考模型已改变，请重新核对摆放参数")
    world_camera, camera_map, world_map = align_map(target, observed, origin)
    world_centers = centers @ world_camera[:3, :3].T + world_camera[:3, 3]
    alignment_error = float(np.max(np.abs(world_camera @ observed-target)))
    if alignment_error > 1e-10:
        raise ValueError("盖体参考系未对齐")

    with tempfile.TemporaryDirectory(prefix="voxel_octomap_") as temporary:
        converter = args.converter.resolve() if args.converter else compile_converter(temporary)
        input_path = Path(temporary)/"occupied_grid.json"
        write_json(input_path, {"voxel_size_m": size, "indices_xyz": indices.tolist()})
        output.mkdir(parents=True)
        subprocess.run([str(converter), str(input_path), str(output)], check=True)
    validation = json.loads((output/"octomap_validation.json").read_text())
    validation.update({"source_voxel_count": len(indices), "cap_voxel_count": int(cap_mask.sum()),
                       "cap_alignment_max_matrix_error": alignment_error,
                       "grid_not_resampled": True, "hardware_valid": False})
    write_json(output/"validation.json", validation)
    # OctoMap 本身没有语义标签；为后续分离盖体保留同一格号下的原始标记。
    np.savez_compressed(output/"voxel_labels.npz", indices_xyz=indices, cap_voxel_mask=cap_mask)
    write_json(output/"placement_config.json", placement)
    write_json(output/"scene_config.json", config)
    transformed_candidates = []
    for item in candidates["candidates"]:
        camera_cap = rigid_matrix(item["camera_from_cap"])
        transformed_candidates.append({"candidate_id": item["candidate_id"],
                                       "world_from_cap": (world_camera @ camera_cap).tolist()})
    manifest = {
        "schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "mode": placement["mode"], "planning_frame": placement["planning_frame"],
        "camera_frame": placement["camera_frame"], "octomap_frame": "observed_voxel_grid",
        "length_unit": "m", "map_path": "scene.bt", "map_sha256": digest(output/"scene.bt"),
        "resolution_m": size, "occupancy_policy": "observed_surface_only",
        "world_from_camera": world_camera.tolist(), "camera_from_octomap": camera_map.tolist(),
        "world_from_octomap": world_map.tolist(), "world_from_cap": target.tolist(),
        "alignment_candidate_id": placement["candidate_id"], "cap_candidates_world": transformed_candidates,
        "world_center_bounds_m": [world_centers.min(0).tolist(), world_centers.max(0).tolist()],
        "voxel_labels_path": "voxel_labels.npz", "voxel_labels_sha256": digest(output/"voxel_labels.npz"),
        "cap_included_in_map": True, "valid_for_hardware": False,
        "right_base_from_left_base": None, "moveit_scene_loaded": False,
        "placement_config_sha256": digest(output/"placement_config.json"),
        "scene_config_sha256": digest(output/"scene_config.json"),
        "implementation_sha256": {name: digest(ROOT/name) for name in
                                  ["tools/export_voxel_octomap.py", "tools/voxel_octomap_export.cpp",
                                   "src/fuel_cap_task/voxel_octomap.py"]},
        "sources": {str(p.resolve()): digest(p) for p in
                    [source, source.with_name("result.json"), args.scene_config, placement_path,
                     candidates_path, Path(candidates["geometry_result"]), anchor]},
        "validation": validation,
    }
    write_json(output/"octomap_scene.json", manifest)
    print(json.dumps({"output_dir": str(output), "world_from_camera": world_camera.tolist(),
                      "validation": validation}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
