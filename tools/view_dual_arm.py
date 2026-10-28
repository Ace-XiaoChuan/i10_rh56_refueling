#!/usr/bin/env python3
"""检查双臂 URDF、用软件绘图生成预览，或在 RViz 中调整显示关节。"""

import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from build_dual_arm_model import ARM_JOINTS, CONFIG, OUTPUT, ROOT, check, mesh_path


def description():
    """核对生成文件来源，并将包网格解析为绝对路径供直接预览。"""
    report = check()
    robot = ET.parse(OUTPUT/"robot.urdf").getroot()
    for mesh in robot.iter("mesh"):
        mesh.set("filename", str(mesh_path(mesh.get("filename"))))
    return robot, report


def rviz_description(robot):
    """为 ROS 资源加载器生成 file:// URI，保留几何预览所需的本地路径。"""
    result = deepcopy(robot)
    for mesh in result.iter("mesh"):
        # 裸绝对路径可供 MuJoCo 使用，但 RViz 的 resource_retriever 要求 URI。
        mesh.set("filename", Path(mesh.get("filename")).as_uri())
    return ET.tostring(result, encoding="unicode")


def preview_positions():
    """左臂使用历史静止采集角，右臂读取当前离线起点；均仅用于显示。"""
    reference = ROOT / "src/i10_rh56_moveit_config/config/audit_reference.json"
    positions = json.loads(reference.read_text())["cases"][0]["joint_positions"]
    scene = json.loads((ROOT / "config/active_point_cloud_scene.json").read_text())
    placement = json.loads((ROOT / scene["octomap"]["placement_config"]).read_text())
    right_values = np.asarray(placement["preview_arm_joints_rad"], dtype=float)
    if right_values.shape != (6,) or not np.isfinite(right_values).all():
        raise ValueError("右臂离线初始关节角须为六个有限弧度值")
    positions.update({name: float(value) for name, value in zip(ARM_JOINTS, right_values)})
    config = json.loads(CONFIG.read_text())
    capture = ROOT / config["vertical_source"]["configuration_paths"][-1]
    sample = json.loads(capture.read_text())["RobotState.getJointPositions"]
    values = np.asarray(sample["value"], dtype=float)
    if not sample["ok"] or values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError("历史左臂关节快照无效")
    positions.update({"left_"+name: float(value) for name, value in zip(ARM_JOINTS, values)})
    return positions


def mujoco_model(robot):
    """仅导入几何和运动学；导入器的推算惯量不用于物理执行。"""
    import mujoco
    extension = ET.SubElement(robot, "mujoco")
    ET.SubElement(extension, "compiler", discardvisual="false", fusestatic="false",
                  strippath="false", inertiafromgeom="true")
    model = mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding="unicode"))
    data = mujoco.MjData(model)
    positions = preview_positions()
    for joint in robot.findall("joint"):
        mimic = joint.find("mimic")
        if mimic is not None:
            positions[joint.get("name")] = (positions[mimic.get("joint")]*float(mimic.get("multiplier", "1"))
                                             + float(mimic.get("offset", "0")))
    for name, value in positions.items():
        data.joint(name).qpos[0] = value
    mujoco.mj_forward(model, data)
    return model, data


def render(robot, destination):
    """将 MuJoCo 解析后的真实视觉网格交给 Matplotlib，避免依赖 OpenGL。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import mujoco

    model, data = mujoco_model(robot)
    surfaces = []
    for g in range(model.ngeom):
        if model.geom_group[g] != 1:
            continue
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = model.geom_dataid[g]
            va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
            fa, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
            triangles = model.mesh_vert[va:va+vn][model.mesh_face[fa:fa+fn]]
        elif model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER:
            radius, half = model.geom_size[g, :2]
            angles = np.linspace(0, 2*np.pi, 49)
            ring = np.column_stack([radius*np.cos(angles), radius*np.sin(angles), np.zeros_like(angles)])
            lower, upper = ring+[0, 0, -half], ring+[0, 0, half]
            triangles = np.array([face for i in range(48) for face in
                                  ([lower[i], lower[i+1], upper[i]], [upper[i], lower[i+1], upper[i+1]],
                                   [[0, 0, -half], lower[i+1], lower[i]], [[0, 0, half], upper[i], upper[i+1]])])
        else:
            raise ValueError("预览不支持的视觉几何："+str(model.geom_type[g]))
        triangles = triangles @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
        # 按面法线作固定光照，完整网格保留，显示轮廓更容易辨认。
        normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-15)
        light = .55+.45*np.abs(normals @ np.array([.3, -.4, .8660254]))
        colors = np.tile(model.geom_rgba[g], (len(triangles), 1))
        colors[:, :3] *= light[:, None]
        surfaces.append((triangles, colors, model.body(model.geom_bodyid[g]).name))

    fig = plt.figure(figsize=(15, 8), facecolor="#f4f6f8")
    for panel, title, elevation, azimuth in [(1, "Dual AUBO i10 + RH56", 18, -65),
                                             (2, "T frame and base mounts", 12, -88)]:
        ax = fig.add_subplot(1, 2, panel, projection="3d")
        ax.set_facecolor("#f4f6f8")
        for triangles, colors, body in surfaces:
            if panel == 2 and body not in {"t_frame", "base_link", "left_base_link"}:
                continue
            ax.add_collection3d(Poly3DCollection(triangles, facecolors=colors, linewidths=0, rasterized=True))
        if panel == 1:
            points = np.concatenate([s[0].reshape(-1, 3) for s in surfaces])
            lo, hi = points.min(axis=0), points.max(axis=0)
            lo[2] = min(0., lo[2])
            center = (lo+hi)/2
            span = max(hi-lo)*.58
            ax.set(xlim=(center[0]-span, center[0]+span), ylim=(center[1]-span, center[1]+span),
                   zlim=(center[2]-span, center[2]+span))
        else:
            ax.set(xlim=(-.25, .25), ylim=(-.25, .25), zlim=(0, 1.35))
            ax.text(.12, 0, 1.3, "303.42 mm | R = 75 mm", fontsize=10)
            ax.text(.1, 0, .5, "1270 mm overall", fontsize=10, rotation=90)
        ax.set_box_aspect([np.ptp(ax.get_xlim()), np.ptp(ax.get_ylim()), np.ptp(ax.get_zlim())])
        ax.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)", title=title)
        ax.view_init(elev=elevation, azim=azimuth)
    fig.suptitle("Vertical from saved left-arm gravity configuration | geometry preview", fontsize=14)
    fig.text(.5, .025, "Left: recorded static pose. Right: current configured initial pose. Camera geometry omitted.", ha="center", fontsize=10)
    fig.subplots_adjust(left=.02, right=.97, bottom=.10, top=.90, wspace=.12)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destination, dpi=160)
    plt.close(fig)
    print("双臂预览：", destination)


def view(robot):
    """用独立本地域启动 RViz 和关节滑块，仅发布模型显示状态。"""
    from launch import LaunchDescription, LaunchService
    from launch.actions import EmitEvent, RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch_ros.actions import Node

    os.environ.setdefault("ROS_LOG_DIR", "/tmp/i10_dual_arm_rviz")
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    os.environ.setdefault("ROS_DOMAIN_ID", "57")
    xml = rviz_description(robot)
    rviz = Node(package="rviz2", executable="rviz2", arguments=["-d", str(OUTPUT/"view.rviz")], output="screen")
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": xml}], output="screen"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             parameters=[{"robot_description": xml, **{"zeros."+k: v for k, v in preview_positions().items()}}], output="screen"),
        rviz,
        RegisterEventHandler(OnProcessExit(target_action=rviz, on_exit=[EmitEvent(event=Shutdown(reason="RViz closed"))])),
    ]))
    return service.run()


def main():
    """默认检查模型；渲染与交互查看分别通过显式参数启动。"""
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--render", action="store_true", help="用项目 .venv 生成预览 PNG")
    action.add_argument("--view", action="store_true", help="用 ROS 系统 Python 打开 RViz 和关节滑块")
    parser.add_argument("--output", type=Path, default=OUTPUT/"preview.png")
    args = parser.parse_args()
    robot, report = description()
    print(json.dumps(report["validation"], ensure_ascii=False))
    if args.render:
        render(robot, args.output)
    elif args.view:
        return view(robot)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
