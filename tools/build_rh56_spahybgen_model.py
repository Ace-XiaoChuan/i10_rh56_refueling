#!/usr/bin/env python3
"""派生独立 RH56 手模型、指面和 MuJoCo FK 参考；不改源模型或积分。"""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from fuel_cap_task.mujoco_scene import MujocoCapScene
from fuel_cap_task.rh56_drive_map import RH56DriveMap

PACKAGE = ROOT / "src/i10_rh56_moveit_config"


def digest(path):
    """绑定当前实际来源，不由模型名称推断一致性。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, data):
    """写入本次独立派生目录。"""
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def body_matrix(data, name):
    """取得世界到 body 的齐次变换，单位米。"""
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = data.body(name).xmat.reshape(3, 3), data.body(name).xpos
    return t


def build(output):
    """从现有派生 URDF 子树取几何，从编译模型提取驱动与参考 FK。"""
    output.mkdir(parents=True, exist_ok=False)
    (output / "meshes").mkdir()
    source_provenance = json.loads((PACKAGE / "config/provenance.json").read_text())
    sources = dict(source_provenance["sources"])
    for name, expected in sources.items():
        if digest(ROOT / name) != expected:
            raise ValueError(f"planning model source changed: {name}")
    source = ET.parse(PACKAGE / "config/robot.urdf").getroot()
    links = {"base"}
    selected_joints = []
    # 遍历子树，排除机械臂、安装件及世界安装变换。
    remaining = list(source.findall("joint"))
    while True:
        added = [j for j in remaining if j.find("parent").get("link") in links]
        if not added:
            break
        for j in added:
            selected_joints.append(copy.deepcopy(j))
            links.add(j.find("child").get("link"))
            remaining.remove(j)
    tree = ET.Element("robot", name="rh56dfx_right_spahybgen")
    meshes = {}
    for original in source.findall("link"):
        name = original.get("name")
        if name not in links:
            continue
        link = ET.SubElement(tree, "link", name=name)
        parts = []
        for visual in original.findall("visual"):
            mesh_element = visual.find("geometry/mesh")
            if mesh_element is None:
                raise ValueError("hand visual primitive requires explicit mesh conversion")
            uri = mesh_element.get("filename")
            prefix = "package://i10_rh56_moveit_config/"
            if not uri.startswith(prefix):
                raise ValueError("unexpected mesh source")
            path = PACKAGE / uri[len(prefix):]
            sources[str(path.relative_to(ROOT))] = digest(path)
            mesh = trimesh.load(path, force="mesh", process=False)
            if mesh_element.get("scale"):
                mesh.vertices *= np.fromstring(mesh_element.get("scale"), sep=" ")
            origin = visual.find("origin")
            if origin is not None:
                xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                transform = trimesh.transformations.euler_matrix(*rpy)
                transform[:3, 3] = xyz
                mesh.apply_transform(transform)
            parts.append(mesh)
        if not parts:
            continue
        mesh = trimesh.util.concatenate(parts)
        filename = f"meshes/{name}.obj"
        mesh.export(output / filename)
        # 与上游 process=False 的读取方法一致，接触索引绑定实际导出文件。
        meshes[name] = trimesh.load(output / filename, force="mesh", process=False)
        visual = ET.SubElement(link, "visual")
        ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", filename=filename)
    tree.extend(selected_joints)
    ET.indent(tree)
    ET.ElementTree(tree).write(output / "hand.urdf", encoding="utf-8", xml_declaration=True)

    # 以模型执行器顺序定义六驱动，完整顺序显式保存，不能用 URDF 遍历顺序猜测。
    scene = MujocoCapScene()
    m, d = scene.model, scene.data
    channels = scene.hand.channels
    names = [j.get("name") for j in selected_joints if j.get("type") == "revolute"]
    driver_names = [m.joint(c.joint_id).name for c in channels]
    matrix = np.zeros((len(names), 6))
    offset = np.zeros(len(names))
    for i, name in enumerate(driver_names):
        matrix[names.index(name), i] = 1
    for c in scene.hand.couplings:
        row = names.index(m.joint(c.follower_id).name)
        column = driver_names.index(m.joint(c.driver_id).name)
        matrix[row, column], offset[row] = c.slope, c.offset
    mapping = {"driver_names": driver_names, "joint_names": names, "matrix": matrix.tolist(),
               "offset": offset.tolist(), "lower": [c.q_open for c in channels],
               "upper": [c.q_closed for c in channels],
               "full_lower": [float(m.joint(name).range[0]) for name in names],
               "full_upper": [float(m.joint(name).range[1]) for name in names]}
    drives = RH56DriveMap.from_dict(mapping)
    def place(q):
        for name, value in zip(names, drives.expand(q)):
            d.joint(name).qpos[0] = value
        mujoco.mj_kinematics(m, d)
    middle = (drives.lower + drives.upper) / 2
    place(middle)
    contacts, records = {}, []
    fingers = [("thumb", "thumb_distal", 4), ("index", "index_intermediate", 3),
               ("middle", "middle_intermediate", 2), ("ring", "ring_intermediate", 1),
               ("pinky", "pinky_intermediate", 0)]
    for finger, link, driver in fingers:
        site = m.site(f"right_{finger}_tip")
        site_local = m.site_pos[site.id].copy()
        rotation = d.body(link).xmat.reshape(3, 3).copy()
        delta = np.zeros(6)
        delta[driver] = 1e-4
        place(middle + delta)
        plus = d.site(site.id).xpos.copy()
        place(middle - delta)
        minus = d.site(site.id).xpos.copy()
        place(middle)
        direction = rotation.T @ (plus - minus)
        direction /= np.linalg.norm(direction)
        mesh = meshes[link]
        distances = np.linalg.norm(mesh.triangles_center - site_local, axis=1)
        alignment = mesh.face_normals @ direction
        available = np.flatnonzero((distances <= .015) & (alignment >= .5) & (mesh.area_faces > 1e-10))
        order = sorted(available, key=lambda i: (-float(mesh.area_faces[i]), int(i)))[:3]
        if not order:
            raise ValueError(f"no fingertip contact face found: {finger}")
        bases = []
        for face in order:
            vertices = mesh.faces[face].tolist()
            bases.append(vertices + [vertices[0]])
            records.append({"finger": finger, "link": link, "face_index": int(face),
                            "vertex_indices": bases[-1], "vertices_link_m": mesh.vertices[bases[-1]].tolist(),
                            "normal_link": mesh.face_normals[face].tolist(),
                            "tip_site_link_m": site_local.tolist(), "closing_direction_link": direction.tolist(),
                            "distance_to_tip_m": float(distances[face]), "normal_alignment": float(alignment[face])})
        contacts[link] = bases
    dump(output / "contacts.json", contacts)

    # 参考 FK 由项目 MuJoCo 3.13 生成，优化环境不必安装或混入这套解释器。
    rng = np.random.default_rng(20261008)
    samples = np.vstack([drives.lower, middle, drives.upper,
                         rng.uniform(drives.lower, drives.upper, size=(8, 6))])
    ordered_links = [l.get("name") for l in tree.findall("link")]
    frames = []
    for q in samples:
        place(q)
        root_from_world = np.linalg.inv(body_matrix(d, "base"))
        frames.append([root_from_world @ body_matrix(d, link) for link in ordered_links])
    np.savez_compressed(output / "fk_reference.npz", drivers=samples, base_from_links=np.asarray(frames))
    for path in (PACKAGE / "config/robot.urdf", PACKAGE / "config/provenance.json",
                 ROOT / "tools/build_rh56_spahybgen_model.py", ROOT / "src/fuel_cap_task/rh56_drive_map.py"):
        sources[str(path.relative_to(ROOT))] = digest(path)
    metadata = {"schema_version": 1, "hand": "rh56dfx_right", "root_frame": "base", "unit": "m/rad",
                "source_mujoco_version": mujoco.__version__, "mapping": mapping,
                "link_names": ordered_links, "contact_regions": records,
                "hand_approaching_matrix": [[0, 1, 0], [0, 0, 1], [1, 0, 0]],
                "hand_position_offset_m": .05, "hardware_calibrated": False,
                "inputs": sources, "outputs": {str(p.relative_to(output)): digest(p)
                    for p in output.rglob("*") if p.is_file()},
                "limitations": "仅独立手模型、几何接触候选和运动学；不含接触动力学、驱动力或实物通道标定"}
    dump(output / "model.json", metadata)
    print(json.dumps({"output": str(output), "links": len(ordered_links), "joints": len(names),
                      "drivers": 6, "contact_regions": len(records), "fk_reference_poses": len(samples)}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    build(args.output_dir.resolve())


if __name__ == "__main__":
    main()
