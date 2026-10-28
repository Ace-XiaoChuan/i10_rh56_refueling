#!/usr/bin/env python3
"""以单臂模型和原始体素显示网格预览地图摆放；不运行规划或物理步进。"""

import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from fuel_cap_task.voxel_octomap import rigid_matrix


def digest(path):
    """核对显示来源，防止地图与机器人预览使用不同实验结果。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def make_preview(directory, manifest, placement):
    """仅保留模型中的臂手和参考地面；实测地图代替原油箱、盖体和门板。"""
    import mujoco
    from fuel_cap_task.mujoco_scene import ARM_JOINTS, HandPreviewMapping

    scene_config = json.loads((directory/"scene_config.json").read_text())
    source = ROOT/placement["anchor_source"]["path"]
    voxel_path = ROOT/scene_config["octomap"]["source_voxels"]
    display = (directory/manifest["display_scene_dir"] if "display_scene_dir" in manifest else
               voxel_path.parent.with_name(voxel_path.parent.name+"_display"))
    packed = json.loads((display/"result.json").read_text())
    if "display_record_sha256" in manifest:
        if digest(display/"result.json") != manifest["display_record_sha256"]:
            raise ValueError("组件显示清单摘要改变")
        for name, expected in packed["files_sha256"].items():
            if digest(display/name) != expected:
                raise ValueError("组件显示几何摘要改变："+name)
    if (digest(source) != placement["anchor_source"]["sha256"]
            or packed["source_voxels_sha256"] != digest(voxel_path)
            or manifest["sources"].get(str(voxel_path.resolve())) != digest(voxel_path)):
        raise ValueError("显示模型与地图来源不一致")
    model = mujoco.MjModel.from_xml_path(str(source))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    site = data.site(placement["anchor_source"]["site"])
    anchor = np.eye(4)
    anchor[:3, :3], anchor[:3, 3] = site.xmat.reshape(3, 3), site.xpos
    np.testing.assert_allclose(anchor, manifest["world_from_cap"], atol=1e-10, rtol=0)

    root = ET.parse(source).getroot()
    root.set("model", "observed_octomap_placement_preview")
    world = root.find("worldbody")
    for child in list(world):
        if child.tag == "body" and child.find(".//body[@name='base_link']") is None and child.get("name") != "base_link":
            world.remove(child)
    assets = root.find("asset")
    needed = {g.get("mesh") for g in world.iter("geom") if g.get("mesh")}
    for asset in list(assets):
        if asset.tag == "mesh" and asset.get("name") not in needed:
            assets.remove(asset)
        elif asset.get("file"):
            asset.set("file", str((source.parent/asset.get("file")).resolve()))
    observed = ET.parse(display/"scene.xml").getroot()
    existing = {a.get("name") for a in assets}
    for asset in observed.find("asset"):
        item = deepcopy(asset)
        if item.get("name") in existing:
            raise ValueError("地图与机器人资产重名")
        if item.get("file"):
            item.set("file", str((display/item.get("file")).resolve()))
        assets.append(item)
    transform = rigid_matrix(manifest["world_from_camera"])
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, np.ascontiguousarray(transform[:3, :3]).ravel())
    numbers = lambda values: " ".join(f"{value:.17g}" for value in values)
    carrier = ET.SubElement(world, "body", name="observed_camera_map",
                            pos=numbers(transform[:3, 3]), quat=numbers(quaternion))
    for geom in observed.find("worldbody").findall("geom"):
        item = deepcopy(geom)
        item.attrib.update(type="mesh", contype="0", conaffinity="0", group="1")
        carrier.append(item)
    # 坐标参考只用于显示，地图实际形状仍由完整占据格的外露格面提供。
    center = np.asarray(manifest["world_from_cap"])[:3, 3]
    axis = np.asarray(manifest["world_from_cap"])[:3, 2]
    ET.SubElement(world, "geom", name="cap_outward_axis", type="capsule", size=".003",
                  fromto=numbers(np.r_[center, center+.12*axis]), rgba="1 .8 .1 1",
                  contype="0", conaffinity="0", group="1")
    for direction, color in zip(np.eye(3), ["1 .1 .1 1", ".1 .8 .1 1", ".1 .3 1 1"]):
        ET.SubElement(world, "geom", type="capsule", size=".003", fromto=numbers(np.r_[np.zeros(3), .18*direction]),
                      rgba=color, contype="0", conaffinity="0", group="1")
    visual = root.find("visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="900")
    ET.SubElement(visual, "map", znear=".001", zfar="8")
    ET.indent(root)
    xml = ET.tostring(root, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    for name, value in zip(ARM_JOINTS, placement["preview_arm_joints_rad"]):
        data.joint(name).qpos[0] = value
    candidates = json.loads((ROOT/scene_config["grasp_candidates"]).read_text())
    HandPreviewMapping(model).apply(data, candidates["pregrasp_channels"])
    mujoco.mj_forward(model, data)
    count = sum(model.geom(i).name.startswith("surface_") for i in range(model.ngeom) if model.geom(i).name)
    if count != packed["validation"]["display_geom_count"]:
        raise ValueError("完整地图未装入预览")
    # 复核地图载体确实使用整图变换，排除导出预览时额外旋转或漏应用平移。
    body = data.body("observed_camera_map")
    np.testing.assert_allclose(body.xpos, transform[:3, 3], atol=1e-10, rtol=0)
    np.testing.assert_allclose(body.xmat.reshape(3, 3), transform[:3, :3], atol=1e-10, rtol=0)
    report = {"map_display_mesh_count": count, "map_voxel_count": manifest["validation"]["source_voxel_count"],
              "placeholder_tank_removed": True, "map_carrier_transform_verified": True,
              "simulation_time_s": data.time, "collision_checks_performed": False,
              "anchor_site_matches_placement": True, "mujoco_version": mujoco.__version__,
              "display_xml_sha256": digest(display/"scene.xml")}
    return xml, model, data, report


def main():
    """保存全景与局部渲染，或启动只读几何窗口；关节始终使用配置中的预览值。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-dir", type=Path, required=True)
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--view", action="store_true")
    parser.add_argument("--max-seconds", type=float, help="仅用于窗口启动检查")
    args = parser.parse_args()
    if not args.view:
        os.environ.setdefault("MUJOCO_GL", "glfw" if os.environ.get("DISPLAY") else "egl")
    import mujoco
    directory = args.scene_dir.resolve()
    manifest = json.loads((directory/"octomap_scene.json").read_text())
    placement = json.loads((directory/"placement_config.json").read_text())
    if digest(directory/manifest["map_path"]) != manifest["map_sha256"]:
        raise ValueError("OctoMap 文件摘要改变")
    for name in ("placement_config", "scene_config"):
        if digest(directory/(name+".json")) != manifest[name+"_sha256"]:
            raise ValueError("地图配置副本摘要改变")
    if "cap_object_sha256" in manifest:
        if digest(directory/manifest["cap_object_path"]) != manifest["cap_object_sha256"]:
            raise ValueError("独立盖体对象摘要改变")
    xml, model, data, report = make_preview(directory, manifest, placement)
    options = mujoco.MjvOption()
    options.geomgroup[:] = 0
    options.geomgroup[:3] = 1
    options.sitegroup[:] = 0
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [.15, .9, .65]
    camera.distance, camera.azimuth, camera.elevation = 3.4, -25, -22
    if args.render:
        from PIL import Image, ImageDraw
        panels = []
        with mujoco.Renderer(model, height=900, width=1280, max_geom=model.ngeom+100) as renderer:
            for title, lookat, distance, azimuth, elevation in [
                    ("Robot and measured map", [.15, .9, .65], 3.4, -25, -22),
                    ("Cap aligned at (0, 1.007, 0.600) m", [0, 1.007, .6], .65, -50, -15)]:
                camera.lookat[:] = lookat
                camera.distance, camera.azimuth, camera.elevation = distance, azimuth, elevation
                renderer.update_scene(data, camera=camera, scene_option=options)
                panels.append((title, Image.fromarray(renderer.render().copy()).resize((960, 675))))
        image = Image.new("RGB", (1920, 750), "#f5f7fa")
        draw = ImageDraw.Draw(image)
        for i, (title, frame) in enumerate(panels):
            image.paste(frame, (i*960, 75))
            draw.text((i*960+18, 12), title, fill="#26384a", font_size=26)
        legend = ("Simulation placement | 2 mm cells | cyan: environment | red: cap | yellow: mixed cells / axis"
                  if "cap_object_path" in manifest else
                  "Simulation placement | 2 mm occupied cells | red: cap | yellow: outward axis")
        draw.text((18, 47), legend, fill="#26384a", font_size=20)
        image.save(directory/"placement_preview.png")
        (directory/"placement_preview.xml").write_text(xml+"\n")
        (directory/"preview_validation.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    if args.view:
        import mujoco.viewer
        import threading
        before = set(threading.enumerate())
        with mujoco.viewer.launch_passive(model, data) as viewer:
            render_threads = [t for t in threading.enumerate() if t not in before
                              and getattr(t, "_target", None) is mujoco.viewer._launch_internal]
            with viewer.lock():
                viewer.opt.geomgroup[:] = options.geomgroup
                viewer.opt.sitegroup[:] = 0
                viewer.cam.lookat[:] = [.15, .9, .65]
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 3.4, -25, -22
            started = time.monotonic()
            while viewer.is_running():
                if args.max_seconds is not None and time.monotonic()-started >= args.max_seconds:
                    break
                viewer.sync()
                time.sleep(1/30)
        for thread in render_threads:
            thread.join()


if __name__ == "__main__":
    main()
