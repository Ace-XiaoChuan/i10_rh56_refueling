#!/usr/bin/env python3
"""将 RH56 按固定抓姿放到观测盖体上，导出 Windows CloudCompare 叠加文件。"""

import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys

os.environ.setdefault("MUJOCO_GL", "egl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from fuel_cap_task.cap_hand_overlay import place_fixed_hand, write_hand_ply
from fuel_cap_task.fixed_grasp import DEFAULT_FIXED_GRASP, load_fixed_grasp
from fuel_cap_task.grasp_search import GraspSearchScene, pose_arrays
from fuel_cap_task.grasp_targets import inverse, transform
from fuel_cap_task.pcd_point_cloud import load_pcd


def digest(path):
    """绑定实际读取的观测、候选、实现与网格来源。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def render_views(output, scene, overlay, points, colors):
    """仅渲染手模型与真实点云，二维快照对应导出的同一份相机系几何。"""
    model, data = scene.model, scene.data
    model.vis.global_.offwidth, model.vis.global_.offheight = 1100, 800
    options = mujoco.MjvOption()
    options.geomgroup[:] = 0
    options.geomgroup[2] = 1  # 当前 RH56 visual；不显示源场景的油箱、盖体或机械臂。
    options.sitegroup[:] = 0
    combined = np.vstack((overlay.vertices_camera_m, points))
    center = (combined.min(axis=0) + combined.max(axis=0)) * .5
    span = float(np.linalg.norm(np.ptp(combined, axis=0)))
    views = [("overview", center, span * 1.35, 35., -25.),
             ("grip_detail", overlay.camera_from_cap[:3, 3], .19, 130., -20.)]
    frames = []
    with mujoco.Renderer(model, height=800, width=1100, max_geom=len(points)+1000) as renderer:
        for name, lookat, distance, azimuth, elevation in views:
            camera = mujoco.MjvCamera()
            camera.lookat[:] = lookat
            camera.distance, camera.azimuth, camera.elevation = distance, azimuth, elevation
            renderer.update_scene(data, camera=camera, scene_option=options)
            # 统一浅灰手色，观测点保留蓝/橙/紫分组色；形状与导出网格完全一致。
            for geom in renderer.scene.geoms[:renderer.scene.ngeom]:
                geom.rgba[:] = [.69, .72, .77, 1.]
            for point, color in zip(points, colors):
                geom = renderer.scene.geoms[renderer.scene.ngeom]
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                                   np.full(3, .00045), point, np.eye(3).ravel(),
                                   np.r_[color / 255., 1.].astype(np.float32))
                renderer.scene.ngeom += 1
            for axis, color in zip(range(3), ([.8,.18,.16,1], [.15,.6,.28,1], [.15,.32,.85,1])):
                geom = renderer.scene.geoms[renderer.scene.ngeom]
                origin = overlay.camera_from_cap[:3, 3]
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3),
                                   origin, np.eye(3).ravel(), np.array(color,dtype=np.float32))
                mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, .0008,
                                    origin, origin + overlay.camera_from_cap[:3, axis] * .025)
                renderer.scene.ngeom += 1
            image = Image.fromarray(renderer.render().copy())
            image.save(output / f"{name}.png")
            frames.append(image)
    canvas = Image.new('RGB', (2200, 875), '#f6f7f9')
    draw = ImageDraw.Draw(canvas)
    for column, (name, image) in enumerate(zip(("Fixed grasp root + approach finger shape", "Observed cap and fingertips"),frames)):
        canvas.paste(image,(column*1100,75))
        draw.text((column*1100+20,17),name,fill='#243447',font_size=22)
        draw.text((column*1100+20,47),'Gray: RH56 | Blue: cap face | Orange: grip | XYZ axes: red / green / blue',
                  fill='#46566a',font_size=16)
    canvas.save(output / 'preview.png')


def main():
    """显式读取相机系候选，不把未知采集状态变为有效运动目标。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry-dir', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, default=DEFAULT_FIXED_GRASP)
    parser.add_argument('--pose-index', type=int, default=0)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--no-render', action='store_true')
    args = parser.parse_args()
    folder, output = args.geometry_dir.resolve(), args.output_dir.resolve()
    if output.exists():
        parser.error('输出目录已存在，请指定新目录')
    report_path = folder / 'result.json'
    geometry = json.loads(report_path.read_text())
    if (geometry.get('mode') != 'cropped_cap_geometry' or geometry.get('frame_id') != 'camera'
            or geometry.get('length_unit') != 'm' or not geometry.get('geometric_checks_passed')):
        parser.error('需要已通过几何检查的米制相机系盖体结果')
    candidates = geometry['pose_candidates_camera_model_based']
    if args.pose_index not in range(len(candidates)):
        parser.error('姿态候选索引超出范围')
    pose = candidates[args.pose_index]
    if pose.get('frame_id') != 'camera':
        parser.error('所选姿态必须显式在 camera 中表达')
    camera_from_cap = transform(*pose_arrays(pose))
    # 核对导出点云仍是该份观测的同一批点；文件颜色只表达分组，不表示真实 RGB。
    cloud = load_pcd(folder / 'classified_cap.pcd')
    source = Path(geometry['input_path'])
    original = load_pcd(source)
    if original.sha256 != geometry['input_sha256']:
        raise ValueError('几何结果对应的原始裁剪输入已变化')
    exported_scale = {'mm': .001, 'm': 1.}[geometry['pcd_exports_length_unit']]
    input_scale = {'mm': .001, 'm': 1.}[geometry['input_length_unit']]
    points = cloud.xyz_raw * exported_scale
    if points.shape != original.xyz_raw.shape or not np.allclose(
            points, original.xyz_raw * input_scale, rtol=0, atol=1e-10):
        raise ValueError('着色点云坐标或点顺序与源观测不一致')
    grasp = load_fixed_grasp(args.candidate, expected_mujoco_version=mujoco.__version__)
    scene = GraspSearchScene(grasp.provenance.source_scene_path, fixture=True)
    overlay = place_fixed_hand(camera_from_cap, grasp, scene=scene)
    local = inverse(camera_from_cap) @ overlay.camera_from_hand
    expected_local = transform(*pose_arrays(asdict(grasp.cap_from_hand)))
    if not np.allclose(local, expected_local, atol=1e-12, rtol=0):
        raise ValueError('叠加后手盖相对位姿改变')
    output.mkdir(parents=True)
    write_hand_ply(output / 'rh56_fixed_pregrasp.ply', overlay.vertices_camera_m, overlay.faces,
                   length_unit=geometry['pcd_exports_length_unit'])
    shutil.copyfile(folder / 'classified_cap.pcd', output / 'cap_cloud.pcd')
    if not args.no_render:
        render_views(output, scene, overlay, points, cloud.rgb)
    mesh_sources = {record['source_mesh']:digest(record['source_mesh']) for record in overlay.geometries
                    if record['source_mesh'] is not None}
    record = {
        'schema_version':1, 'mode':'fixed_hand_on_observed_cap', 'frame_id':'camera',
        'length_unit':'m', 'export_length_unit':geometry['pcd_exports_length_unit'],
        'hand_root_pose_kind':'fixed_grasp', 'finger_shape_kind':'pregrasp_channels',
        'finger_channels':list(grasp.pregrasp_channels), 'geometry_pose_index':args.pose_index,
        'ambiguity_yaw_deg':pose['ambiguity_yaw_deg'], 'orientation_disambiguated':False,
        'camera_from_cap':overlay.camera_from_cap.tolist(),
        'camera_from_hand':overlay.camera_from_hand.tolist(),
        'cap_from_hand':local.tolist(), 'fixed_grasp':asdict(grasp),
        'relative_position_error_m':float(np.linalg.norm(local[:3,3]-expected_local[:3,3])),
        'relative_rotation_max_abs_error':float(np.max(abs(local[:3,:3]-expected_local[:3,:3]))),
        'hand_mesh_vertex_count':len(overlay.vertices_camera_m), 'hand_mesh_face_count':len(overlay.faces),
        'hand_meshes':overlay.geometries, 'hand_joint_positions_rad':overlay.joint_positions_rad,
        'fingertip_sites_camera_m':{name:value.tolist() for name,value in overlay.fingertip_sites_camera_m.items()},
        'coupling_residual_max_rad':overlay.coupling_residual_max_rad,
        'source_sha256':{'geometry_result':digest(report_path),'cap_cloud':cloud.sha256,
                         'crop_input':original.sha256,'overlay_module':digest(ROOT/'src/fuel_cap_task/cap_hand_overlay.py'),
                         'preview_tool':digest(Path(__file__)),'source_meshes':mesh_sources},
        'source_geometry_motion_target_valid':geometry['motion_target_valid'],
        'capture_timestamp_s':geometry['capture_timestamp_s'],
        'model_reference_prior':geometry['model_prior'],
        'physics_steps':0,'simulation_time_s':float(scene.data.time),
        'arm_ik_computed':False,'physical_grasp_verified':False,
        'note':f'手指预型用于观察，不声明已闭合抓稳；点云保持原始坐标。当前使用{pose["ambiguity_yaw_deg"]}度候选，尚未消除180度歧义。',
    }
    record['outputs_sha256']={p.name:digest(p) for p in output.iterdir() if p.is_file()}
    (output/'overlay.json').write_text(json.dumps(record,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output_dir':str(output),'vertices':len(overlay.vertices_camera_m),
                      'triangles':len(overlay.faces),'relative_position_error_m':record['relative_position_error_m'],
                      'relative_rotation_max_abs_error':record['relative_rotation_max_abs_error']},indent=2))


if __name__=='__main__':
    main()
