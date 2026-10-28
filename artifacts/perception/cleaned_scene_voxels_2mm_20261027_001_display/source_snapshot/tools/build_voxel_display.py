#!/usr/bin/env python3
"""把完整体素的外露面按原材质合批，生成可交互的 MuJoCo 显示模型。"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from fuel_cap_task.voxel_mujoco import (
    FACE_NORMALS, canonical_triangles, check_compiled_voxels, exposed_voxel_faces, numbers)
from voxelize_cleaned_scene import render_scene


def digest(path):
    """绑定源体素及模型，避免显示缓存对应到其他格子版本。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    """只改变绘制组织；原始占据格与碰撞模型保留，显示 mesh 不参与碰撞。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--no-render", action="store_true")
    args = parser.parse_args()
    source, output = args.scene_dir.resolve(), args.output_dir.resolve()
    if output.exists():
        parser.error("输出目录已存在，请使用新目录")
    record = json.loads((source/"result.json").read_text())
    with np.load(source/"voxels.npz", allow_pickle=False) as arrays:
        indices, centers = arrays["indices_xyz"], arrays["centers_camera_m"]
        size, origin = float(arrays["voxel_size_m"]), arrays["grid_origin_camera_m"]
    original = mujoco.MjModel.from_binary_path(str(source/"scene.mjb"))
    original_data = mujoco.MjData(original)
    mujoco.mj_forward(original, original_data)
    check_compiled_voxels(original, original_data, centers, size)
    cell_rows, directions, corners = exposed_voxel_faces(indices)
    print(f"{len(indices)} voxels -> {len(cell_rows)} exposed square faces", flush=True)
    root = ET.parse(source/"scene.xml").getroot()
    root.set("model", "measured_voxel_surface_display")
    world = root.find("worldbody")
    for item in list(world):
        if item.tag == "geom":
            world.remove(item)
    defaults = root.find("default/geom")
    defaults.set("type", "mesh")
    defaults.attrib.pop("size", None)
    defaults.set("contype", "0")
    defaults.set("conaffinity", "0")
    assets = root.find("asset")
    material_ids = original.geom_matid[cell_rows]
    output.mkdir(parents=True)
    (output/"meshes").mkdir()
    expected = []
    for material in np.unique(material_ids):
        selected = material_ids == material
        quads = corners[selected]
        vertices, inverse = np.unique(quads.reshape(-1,3), axis=0, return_inverse=True)
        local_quads = inverse.reshape(-1,4)
        faces = local_quads[:,[[0,1,2],[0,2,3]]].reshape(-1,3)
        normals = np.repeat(directions[selected]+1, 2)
        name = f"surface_{material}"
        with (output/"meshes"/f"{name}.obj").open("w") as stream:
            np.savetxt(stream, origin+vertices*size, fmt="v %.15g %.15g %.15g")
            np.savetxt(stream, FACE_NORMALS, fmt="vn %d %d %d")
            for face, normal in zip(faces+1, normals):
                stream.write(f"f {face[0]}//{normal} {face[1]}//{normal} {face[2]}//{normal}\n")
        ET.SubElement(assets, "mesh", name=name, file=f"meshes/{name}.obj", inertia="shell")
        rows = cell_rows[selected]
        ET.SubElement(world, "geom", name=name, mesh=name,
                      material=original.mat(int(material)).name,
                      group=str(int(original.geom_group[rows[0]])))
        expected.append(vertices[faces])
    ET.indent(root)
    (output/"scene.xml").write_text(ET.tostring(root, encoding="unicode")+"\n")
    started = time.monotonic()
    model = mujoco.MjModel.from_xml_path(str(output/"scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    # MuJoCo 会对 mesh 居中、旋转；用编译后完整变换反算每个三角形的原始格角。
    # 比较整数格角的三角形集合，防止合批时丢面、移位或填掉凹槽。
    max_error = 0.0
    for geom, wanted in enumerate(expected):
        mesh = int(model.geom_dataid[geom])
        va, vn = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
        fa, fn = int(model.mesh_faceadr[mesh]), int(model.mesh_facenum[mesh])
        vertices = model.mesh_vert[va:va+vn].astype(float) @ data.geom_xmat[geom].reshape(3,3).T + data.geom_xpos[geom]
        triangles = vertices[model.mesh_face[fa:fa+fn]]
        lattice = np.rint((triangles-origin)/size).astype(np.int64)
        error = float(np.max(np.abs(triangles-(origin+lattice*size))))
        max_error = max(max_error,error)
        assert error < 1e-6
        np.testing.assert_array_equal(canonical_triangles(lattice), canonical_triangles(wanted))
    assert np.all(model.geom_contype == 0) and np.all(model.geom_conaffinity == 0)
    mujoco.mj_saveModel(model, str(output/"scene.mjb"))
    shutil.copyfile(source/"voxels.npz", output/"voxels.npz")
    validation = {"voxel_count":len(indices), "voxel_size_m":size,
                  "source_voxel_data_byte_identical":True, "exposed_square_faces":len(cell_rows),
                  "display_geom_count":int(model.ngeom), "compiled_surface_triangles_match_grid":True,
                  "compiled_vertex_max_error_m":max_error, "display_mesh_collision_enabled":False,
                  "source_box_collision_scene_preserved":True, "physics_steps":0,
                  "model_compile_and_validation_seconds":time.monotonic()-started}
    result = {**record, "mode":"sdk_voxel_batched_surface_display",
              "created_utc":datetime.now(timezone.utc).isoformat(),
              "source_box_scene":str(source), "source_voxels_sha256":digest(source/"voxels.npz"),
              "source_box_scene_sha256":digest(source/"scene.xml"),
              "source_box_validation":record["validation"], "validation":validation}
    result.pop("compiled_cache",None)
    (output/"result.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n")
    (output/"validation.json").write_text(json.dumps(validation,ensure_ascii=False,indent=2)+"\n")
    for name in ("tools/build_voxel_display.py", "tools/view_point_cloud_voxels.py",
                 "tools/voxelize_cleaned_scene.py", "src/fuel_cap_task/voxel_mujoco.py"):
        target = output/"source_snapshot"/name
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,target)
    if not args.no_render:
        validation["rendered_geom_counts"] = render_scene(output,model,data,voxel_size_m=size)
        (output/"result.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n")
        (output/"validation.json").write_text(json.dumps(validation,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps(validation,ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
