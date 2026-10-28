#!/usr/bin/env python3
"""在现有右臂 URDF 上组合左臂和 T 型架，保留原有臂手内部定义。"""

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/i10_rh56_moveit_config"
CONFIG = ROOT / "config/dual_arm_mount.json"
OUTPUT = PACKAGE / "dual_arm"
ARM_LINKS = (
    "base_link", "shoulder_Link", "upperArm_Link", "foreArm_Link",
    "wrist1_Link", "wrist2_Link", "wrist3_Link",
)
ARM_JOINTS = (
    "shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint",
)


def numbers(values):
    """使用足够有效数字保存米、弧度及无量纲方向。"""
    return " ".join(format(float(value), ".17g") for value in values)


def rotation(rpy):
    """固定轴 XYZ：将子系向量旋转到父系，R = Rz(yaw) Ry(pitch) Rx(roll)。"""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def rpy(matrix):
    """将旋转矩阵写为 URDF 的固定轴 XYZ；奇异处固定 yaw 为零。"""
    c = math.hypot(matrix[0, 0], matrix[1, 0])
    pitch = math.atan2(-matrix[2, 0], c)
    if c < 1e-12:
        return np.array([math.atan2(-matrix[1, 2], matrix[1, 1]), pitch, 0.0])
    return np.array([math.atan2(matrix[2, 1], matrix[2, 2]), pitch,
                     math.atan2(matrix[1, 0], matrix[0, 0])])


def unit(value):
    """归一化有限三维向量，拒绝零重力或退化安装轴。"""
    value = np.asarray(value, dtype=float)
    if value.shape != (3,) or not np.all(np.isfinite(value)) or np.linalg.norm(value) < 1e-12:
        raise ValueError("安装方向必须是非零有限三维向量")
    return value / np.linalg.norm(value)


def matrix_pose(position, orientation):
    """构造 A_from_B 齐次变换，平移单位为米。"""
    value = np.eye(4)
    value[:3, :3], value[:3, 3] = orientation, position
    return value


def derive(config):
    """由两基座位姿、左臂重力及有限圆柱支撑范围推算世界安装位姿。"""
    mount, stand = config["right_base_from_left_base"], config["stand"]
    if (mount["rotation_convention"] != "fixed_axis_XYZ_RzRyRx"
            or mount["base_origins_at_end_face_centers"] is not True
            or stand["post_attachment"] != "crossbar_midpoint"
            or stand["post_top"] != "crossbar_centerline"):
        raise ValueError("安装定义与当前生成器支持的端面中心、立柱连接约定不符")
    position = np.asarray(mount["translation_m"], dtype=float)
    angles = np.asarray(mount["rpy_rad"], dtype=float)
    if position.shape != (3,) or angles.shape != (3,) or not np.isfinite(np.r_[position, angles]).all():
        raise ValueError("两基座位姿必须包含有限的三维平移与 RPY")
    radius, post_radius, height = (float(stand[key]) for key in
                                  ("crossbar_radius_m", "post_radius_m", "floor_to_crossbar_top_m"))
    if not all(math.isfinite(v) and v > 0 for v in (radius, post_radius, height)):
        raise ValueError("圆柱半径和总高度必须为正有限值")

    # 左臂的控制器配置是重力方向依据，不使用全零的 getMountingPose。
    source = config["vertical_source"]
    if source["kind"] != "left_controller_gravity_configuration":
        raise ValueError("不支持的竖直方向来源")
    gravities = []
    for name in source["configuration_paths"]:
        entry = json.loads((ROOT / name).read_text())[source["query"]]
        if entry["ok"] is not True:
            raise ValueError("左臂重力查询失败：" + name)
        gravity = np.asarray(entry["value"], dtype=float)
        unit(gravity)
        gravities.append(gravity)
    if not gravities or any(not np.allclose(g, gravities[0], rtol=0, atol=1e-10) for g in gravities):
        raise ValueError("缺少重力配置或两批左臂重力配置不一致")
    right_from_left = matrix_pose(position, rotation(angles))
    up = -unit(right_from_left[:3, :3] @ gravities[0])
    axis = unit(position)

    # 世界 +X 取横梁由右向左的水平投影，+Z 朝上；水平朝向是显示约定。
    world_x_in_right = unit(axis - up * np.dot(up, axis))
    world_y_in_right = unit(np.cross(up, world_x_in_right))
    world_rotation = np.stack([world_x_in_right, world_y_in_right, up])
    world_axis = world_rotation @ axis
    length = float(np.linalg.norm(position))
    # 有限圆柱的最高点包含轴线倾斜和端面半径的竖直投影，不能直接减去半径。
    top_offset = length/2 * abs(world_axis[2]) + radius * math.sqrt(max(0.0, 1-world_axis[2]**2))
    center_height = height - top_offset
    if center_height <= 0:
        raise ValueError("总高度不足以容纳横梁")
    center = np.array([0.0, 0.0, center_height])
    world_from_right = matrix_pose(center - world_rotation @ (position/2), world_rotation)
    # 圆柱沿自身 +Z 建模。截面绕轴角度不影响几何。
    reference = np.array([0., 0., 1.]) if abs(world_axis[2]) < .9 else np.array([1., 0., 0.])
    bar_x = unit(reference-world_axis*np.dot(reference, world_axis))
    bar_rotation = np.column_stack([bar_x, np.cross(world_axis, bar_x), world_axis])
    return {
        "right_base_from_left_base": right_from_left.tolist(),
        "world_from_right_base": world_from_right.tolist(),
        "world_from_left_base": (world_from_right @ right_from_left).tolist(),
        "right_base_up_direction": up.tolist(),
        "left_base_gravity_m_s2": gravities[0].tolist(),
        "world_from_crossbar": matrix_pose(center, bar_rotation).tolist(),
        "crossbar_length_m": length, "crossbar_radius_m": radius,
        "crossbar_tilt_deg": math.degrees(math.asin(float(world_axis[2]))),
        "crossbar_center_height_m": center_height,
        "post_length_m": center_height, "post_radius_m": post_radius,
        "stand_height_m": height,
        "physical_vertical_verified": source["physical_vertical_verified"],
    }


def origin(element, pose):
    """设置子系在父系中的 URDF 安装位姿。"""
    pose = np.asarray(pose)
    node = element.find("origin")
    if node is None:
        node = ET.SubElement(element, "origin")
    node.attrib.update(xyz=numbers(pose[:3, 3]), rpy=numbers(rpy(pose[:3, :3])))


def add_cylinder(link, name, pose, radius, length):
    """为固定架添加相同的显示与碰撞圆柱，不虚构质量惯量。"""
    for role in ("visual", "collision"):
        element = ET.SubElement(link, role, name=name+"_"+role)
        origin(element, pose)
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(geometry, "cylinder", radius=str(radius), length=str(length))
        if role == "visual":
            material = ET.SubElement(element, "material", name="t_frame_steel")
            ET.SubElement(material, "color", rgba="0.35 0.40 0.46 1")


def xml_bytes(element):
    """固定缩进和编码，便于来源核对和稳定重建。"""
    ET.indent(element)
    return ET.tostring(element, encoding="utf-8", xml_declaration=True) + b"\n"


def json_bytes(value):
    """保存可读 JSON，禁止 NaN 和 Infinity。"""
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+"\n").encode()


def mesh_path(uri):
    """双臂复用当前 MoveIt 包的网格；不复制或重建模型资产。"""
    prefix = "package://i10_rh56_moveit_config/"
    if not uri.startswith(prefix):
        raise ValueError("未知网格 URI：" + uri)
    path = PACKAGE / uri[len(prefix):]
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def validate_tree(robot):
    """检查唯一命名、完整单根树和全部网格路径。"""
    links = [e.get("name") for e in robot.findall("link")]
    joints = robot.findall("joint")
    names = [j.get("name") for j in joints]
    children = [j.find("child").get("link") for j in joints]
    if len(set(links)) != len(links) or len(set(names)) != len(names) or len(set(children)) != len(children):
        raise ValueError("URDF 存在重复 link、joint 或子节点")
    reachable = set(links)-set(children)
    if reachable != {"world"}:
        raise ValueError("双臂模型必须以 world 为唯一根节点")
    if any(j.find("parent").get("link") not in links or j.find("child").get("link") not in links for j in joints):
        raise ValueError("URDF 父子节点不存在")
    while True:
        expanded = reachable | {j.find("child").get("link") for j in joints if j.find("parent").get("link") in reachable}
        if expanded == reachable:
            break
        reachable = expanded
    if reachable != set(links):
        raise ValueError("URDF 包含不连通节点或环")
    meshes = {mesh_path(e.get("filename")) for e in robot.iter("mesh")}
    return {"links": len(links), "joints": len(joints), "mesh_files": len(meshes),
            "independent_joints": sum(j.get("type") != "fixed" and j.find("mimic") is None for j in joints)}


def build(config_path=CONFIG):
    """组合双臂模型；右臂只改基座安装关节，左臂复制七段臂体和六轴关节。"""
    config = json.loads(config_path.read_text())
    geometry = derive(config)
    source = ET.parse(ROOT / config["source_robot"]).getroot()
    robot = deepcopy(source)
    robot.set("name", "i10_dual_arm_rh56")
    robot.insert(0, ET.Comment(" 由 tools/build_dual_arm_model.py 生成；双臂与支架几何模型。 "))
    frame = ET.SubElement(robot, "link", name="t_frame")
    add_cylinder(frame, "t_crossbar", geometry["world_from_crossbar"],
                 geometry["crossbar_radius_m"], geometry["crossbar_length_m"])
    post_pose = matrix_pose([0, 0, geometry["post_length_m"]/2], np.eye(3))
    add_cylinder(frame, "t_post", post_pose, geometry["post_radius_m"], geometry["post_length_m"])
    fixed = ET.SubElement(robot, "joint", name="t_frame_fixed", type="fixed")
    ET.SubElement(fixed, "parent", link="world")
    ET.SubElement(fixed, "child", link="t_frame")
    origin(fixed, np.eye(4))
    right_mount = robot.find("joint[@name='base_link_fixed']")
    if right_mount is None or right_mount.find("parent").get("link") != "world":
        raise ValueError("右臂来源缺少预期的世界安装关节")
    right_mount.find("parent").set("link", "t_frame")
    origin(right_mount, geometry["world_from_right_base"])

    for name in ARM_LINKS:
        original = source.find(f"link[@name='{name}']")
        if original is None:
            raise ValueError("右臂来源缺少臂体：" + name)
        link = deepcopy(original)
        for node in link.iter():
            if "name" in node.attrib:
                node.set("name", "left_"+node.get("name"))
        robot.append(link)
    for name in ARM_JOINTS:
        original = source.find(f"joint[@name='{name}']")
        if original is None:
            raise ValueError("右臂来源缺少关节：" + name)
        joint = deepcopy(original)
        joint.set("name", "left_"+name)
        for tag in ("parent", "child"):
            node = joint.find(tag)
            node.set("link", "left_"+node.get("link"))
        robot.append(joint)
    left_mount = ET.SubElement(robot, "joint", name="left_base_link_fixed", type="fixed")
    ET.SubElement(left_mount, "parent", link="t_frame")
    ET.SubElement(left_mount, "child", link="left_base_link")
    origin(left_mount, geometry["world_from_left_base"])

    srdf = ET.parse(ROOT / config["source_srdf"]).getroot()
    srdf.set("name", robot.get("name"))
    for entry in list(srdf.findall("disable_collisions")):
        if all(entry.get(key) in ARM_LINKS for key in ("link1", "link2")):
            clone = deepcopy(entry)
            for key in ("link1", "link2"):
                clone.set(key, "left_"+clone.get(key))
            srdf.append(clone)
    for name, base, tip in (("right_arm", "base_link", "base"),
                            ("left_arm", "left_base_link", "left_wrist3_Link")):
        group = ET.SubElement(srdf, "group", name=name)
        ET.SubElement(group, "chain", base_link=base, tip_link=tip)
    group = ET.SubElement(srdf, "group", name="dual_arms")
    for name in ("right_arm", "left_arm"):
        ET.SubElement(group, "group", name=name)
    for base in ("base_link", "left_base_link"):
        ET.SubElement(srdf, "disable_collisions", link1="t_frame", link2=base,
                      reason="Fixed base mounting interface")
    kinematics = json.loads((ROOT / config["source_kinematics"]).read_text())
    for name in ("left_arm", "right_arm"):
        kinematics[name] = deepcopy(kinematics["arm"])

    inputs = {config_path.resolve(), Path(__file__).resolve()}
    inputs.update(ROOT / config[key] for key in ("source_robot", "source_srdf", "source_kinematics"))
    inputs.update(ROOT / name for name in config["vertical_source"]["configuration_paths"])
    inputs.update(mesh_path(e.get("filename")) for e in robot.iter("mesh"))
    files = {"robot.urdf": xml_bytes(robot), "robot.srdf": xml_bytes(srdf), "kinematics.json": json_bytes(kinematics)}
    metadata = {
        "mode": "dual_arm_geometry_model", "config": str(config_path.resolve().relative_to(ROOT)),
        "geometry": geometry, "validation": validate_tree(robot),
        "right_base_link": "base_link", "left_base_link": "left_base_link",
        "sources_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(inputs)},
        "generated_sha256": {name: hashlib.sha256(data).hexdigest() for name, data in files.items()},
        "limitations": ["vertical_from_left_controller_configuration_not_independent_measurement",
                        "stand_is_two_intersecting_cylinders_without_measured_inertia",
                        "left_camera_geometry_not_added", "existing_single_arm_planners_keep_original_world_frame"],
    }
    files["model.json"] = json_bytes(metadata)
    return files, metadata


def check(directory=OUTPUT):
    """按保存的摘要核对来源与输出，避免不同 NumPy 环境的末位舍入误报过期。"""
    report = json.loads((directory/"model.json").read_text())
    for base, records in ((ROOT, report["sources_sha256"]), (directory, report["generated_sha256"])):
        for name, expected in records.items():
            path = base/name
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                raise ValueError("双臂来源或输出已改变，请重新生成："+str(path))
    actual = validate_tree(ET.parse(directory/"robot.urdf").getroot())
    if actual != report["validation"]:
        raise ValueError("双臂模型结构与生成记录不符")
    return report


def main():
    """生成装配文件，或只核对配置、来源和生成结果是否一致。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="核对来源及双臂生成文件，不写入")
    args = parser.parse_args()
    if args.check:
        report = check()
    else:
        files, report = build()
        OUTPUT.mkdir(parents=True, exist_ok=True)
        for name, data in files.items():
            (OUTPUT / name).write_bytes(data)
    dimensions = {name: report["geometry"][name] for name in
                  ("crossbar_length_m", "crossbar_radius_m", "post_length_m", "post_radius_m", "stand_height_m")}
    print(json.dumps({"directory": str(OUTPUT), "check_only": args.check,
                      "dimensions": dimensions, "validation": report["validation"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
