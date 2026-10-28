#!/usr/bin/env python3
"""把已有两方向手网格放入当前清理点云，导出离线三维网页和CloudCompare文件。"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sys

import numpy as np
from scipy.spatial import cKDTree
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from fuel_cap_task.pcd_point_cloud import load_pcd


PAGE = '''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>清理点云中的双方向抓取位</title><style>
*{box-sizing:border-box}body{margin:0;color:#20364a;background:#fff;font:15px/1.6 system-ui,sans-serif}
#app{display:flex;flex-direction:column;height:100vh;height:100dvh}header{padding:14px 22px;border-bottom:1px solid #dce4ed;display:flex;gap:18px;align-items:center;justify-content:space-between;flex-wrap:wrap}
h1{font-size:21px;margin:0}button,select{font:inherit;padding:6px 10px;border:1px solid #bdcbd9;border-radius:5px;background:white;color:inherit}button{cursor:pointer}.controls{display:flex;gap:9px;align-items:center;flex-wrap:wrap}.note{padding:7px 22px;background:#f1f5f9;font-size:13px}#cloud{flex:1;min-height:0}footer{padding:8px 22px;border-top:1px solid #dce4ed;font-size:13px;color:#526477}
</style></head><body><div id="app"><header><h1>清理点云中的双方向抓取位</h1><div class="controls">
<label>手模型 <select id="direction"><option value="0">0° 方向</option><option value="180">180° 方向</option><option value="both">两个方向同时显示</option><option value="none">只看清理点云</option></select></label>
<button id="focus">抓取区域</button><button id="full">完整场景</button><button id="reset">重置视角</button><button id="fullscreen">全屏</button>
</div></header><div class="note">当前接近预型：__PREGRASP__。红色为盖体观测点，其余按 Z 着色。手根位于已有抓取位。拖动旋转 · 滚轮缩放 · 右键拖动平移。</div><div id="cloud"></div><footer id="status">加载中…</footer></div>
<script id="scene-data" type="application/json">__DATA__</script><script>/*__PLOTLY__*/</script><script>
"use strict";
const data=JSON.parse(document.getElementById('scene-data').textContent);
const plot=document.getElementById('cloud');
const select=document.getElementById('direction');
const status=document.getElementById('status');
const initialCamera={eye:{x:1.25,y:1.25,z:-1.1},up:{x:0,y:-1,z:0},center:{x:0,y:0,z:0}};
let activeBounds=data.focus_bounds_mm;
function ranges(bounds){return {'scene.xaxis.range':[bounds[0][0],bounds[1][0]],'scene.yaxis.range':[bounds[0][1],bounds[1][1]],'scene.zaxis.range':[bounds[0][2],bounds[1][2]],'scene.camera':structuredClone(initialCamera)};}
function showDirection(){const value=select.value;const visible=[value==='0'||value==='both',value==='180'||value==='both'];Plotly.restyle(plot,{visible:visible},[2,3]);status.textContent=`清理点云 ${data.point_count.toLocaleString()} 点 · 相机系毫米 · ${select.options[select.selectedIndex].textContent}`;}
const bounds=activeBounds;
const layout={scene:{xaxis:{title:'X / mm',range:[bounds[0][0],bounds[1][0]]},yaxis:{title:'Y / mm',range:[bounds[0][1],bounds[1][1]]},zaxis:{title:'Z / mm',range:[bounds[0][2],bounds[1][2]]},aspectmode:'data',camera:structuredClone(initialCamera)},margin:{l:0,r:0,t:10,b:0},legend:{x:0,y:1},uirevision:'cleaned-scene'};
Plotly.newPlot(plot,data.traces,layout,{responsive:true,displaylogo:false}).then(showDirection).catch(error=>{status.textContent='加载失败，请使用支持WebGL的浏览器。';console.error(error);});
select.addEventListener('change',showDirection);
document.getElementById('focus').onclick=()=>{activeBounds=data.focus_bounds_mm;Plotly.relayout(plot,ranges(activeBounds));};
document.getElementById('full').onclick=()=>{activeBounds=data.full_bounds_mm;Plotly.relayout(plot,ranges(activeBounds));};
document.getElementById('reset').onclick=()=>Plotly.relayout(plot,ranges(activeBounds));
const fullscreen=document.getElementById('fullscreen');fullscreen.hidden=!document.fullscreenEnabled;
fullscreen.onclick=async()=>{if(document.fullscreenElement)await document.exitFullscreen();else await document.getElementById('app').requestFullscreen();};
document.addEventListener('fullscreenchange',()=>{fullscreen.textContent=document.fullscreenElement?'退出全屏':'全屏';if(plot._fullLayout)Plotly.Plots.resize(plot);});
</script></body></html>'''


def digest(path):
    """摘要绑定本次实际使用的点云、模型与实现。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def point_trace(points, rows, cap=False):
    """保留清理点云的每个坐标和行号，只为显示附加颜色。"""
    xyz = points[rows]
    marker = {'size': 2.5 if cap else 1.3, 'opacity': 1}
    marker.update({'color': '#df4839'} if cap else {
        'color': xyz[:, 2].tolist(), 'colorscale': 'Viridis', 'colorbar': {'title': 'Z / mm'}})
    return {'type': 'scatter3d', 'mode': 'markers', 'name': '盖体观测点' if cap else '清理后环境',
        'x': xyz[:, 0].tolist(), 'y': xyz[:, 1].tolist(), 'z': xyz[:, 2].tolist(),
        'customdata': rows.tolist(), 'marker': marker,
        'hovertemplate': '清理点云行号 %{customdata}<br>X %{x:.3f} mm<br>Y %{y:.3f} mm<br>Z %{z:.3f} mm<extra>%{fullData.name}</extra>'}


def mesh_trace(mesh, angle):
    """直接显示已处在相机系毫米坐标中的手网格，不再次施加位姿变换。"""
    vertices, faces = np.asarray(mesh.vertices), np.asarray(mesh.faces)
    return {'type': 'mesh3d', 'name': f'RH56 {angle}°',
        'x': vertices[:, 0].tolist(), 'y': vertices[:, 1].tolist(), 'z': vertices[:, 2].tolist(),
        'i': faces[:, 0].tolist(), 'j': faces[:, 1].tolist(), 'k': faces[:, 2].tolist(),
        'color': '#b0bac7' if angle == 0 else '#46a1ad', 'opacity': 1,
        'visible': angle == 0, 'flatshading': False,
        'lighting': {'ambient': .65, 'diffuse': .8, 'specular': .15, 'roughness': .7},
        'hovertemplate': f'RH56 {angle}°<br>X %{{x:.3f}} mm<br>Y %{{y:.3f}} mm<br>Z %{{z:.3f}} mm<extra></extra>'}


def write_preview(output, points, cap_mask, meshes, center):
    """用同一取景绘制两方向，背景仅在静态预览抽样，完整点和手网格保留在网页。"""
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/cleaned-scene-matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    window = np.all(np.abs(points-center) <= 300, axis=1)
    rows = np.flatnonzero(window & ~cap_mask)
    rows = rows[np.linspace(0, len(rows)-1, min(20000, len(rows)), dtype=int)]
    sample, cap = points[rows], points[cap_mask]
    figure = plt.figure(figsize=(15, 7), constrained_layout=True)
    for column, (angle, mesh) in enumerate(zip([0, 180], meshes)):
        ax = figure.add_subplot(1, 2, column+1, projection='3d')
        ax.scatter(*sample.T, c=sample[:, 2], cmap='viridis', s=.35, alpha=.6)
        ax.scatter(*cap.T, c='#df4839', s=.8)
        triangles = mesh.vertices[mesh.faces]
        ax.add_collection3d(Poly3DCollection(triangles, facecolors='#b0bac7' if angle == 0 else '#46a1ad',
                                           edgecolor='none', alpha=1, shade=True,
                                           lightsource=matplotlib.colors.LightSource(azdeg=130, altdeg=45)))
        for axis, value in zip(['x', 'y', 'z'], center):
            getattr(ax, f'set_{axis}lim')(value-300, value+300)
        ax.set_box_aspect([1, 1, 1])
        ax.view_init(elev=-35, azim=-70)
        ax.set(xlabel='X / mm', ylabel='Y / mm', zlabel='Z / mm', title=f'{angle} deg grasp | cleaned point cloud')
    figure.suptitle('Same camera XYZ coordinates | red: cap observations | scene_cleaned.pcd', fontsize=13)
    figure.savefig(output/'comparison.png', dpi=140)
    plt.close(figure)


def main():
    """验证清理只改变选点，再复用已有手网格生成新的场景叠加归档。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-config', type=Path, default=ROOT/'config/active_point_cloud_scene.json')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--no-preview', action='store_true')
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists():
        parser.error('输出目录已存在，请指定新目录，保留已有实验')
    config = json.loads(args.scene_config.read_text())
    source = ROOT/config['point_cloud']['path']
    if config['point_cloud']['length_unit'] != 'mm' or config['point_cloud']['frame_id'] != 'camera_sdk_native':
        parser.error('当前叠加要求相机系毫米坐标，不隐式换轴或缩放')
    if digest(source) != config['point_cloud']['sha256']:
        parser.error('当前点云内容与配置摘要不同，请先核对场景版本')
    cloud = load_pcd(source)
    points = cloud.xyz_raw
    if len(points) != config['point_cloud']['point_count'] or not np.isfinite(points).all():
        parser.error('点云为空或存在非有限值')
    original_path = ROOT/config['original_scan']
    original = load_pcd(original_path)
    distances, _ = cKDTree(original.xyz_raw).query(points)
    # 0.0002 mm仅用于ASCII保存精度核对，不能作为实物到位或碰撞容差。
    if float(distances.max()) > .0002:
        parser.error('清理点云与原始坐标不一致，不能直接沿用已有手根位姿')
    crop = load_pcd(ROOT/config['cap_crop'])
    cap_distances, cap_rows = cKDTree(points).query(crop.xyz_raw)
    if float(cap_distances.max()) > .0002 or len(np.unique(cap_rows)) != len(cap_rows):
        parser.error('盖体观测点未完整保留，需要先核对清理范围')
    cap_mask = np.zeros(len(points), dtype=bool)
    cap_mask[cap_rows] = True
    candidate_path = ROOT/config['grasp_candidates']
    candidate_data = json.loads(candidate_path.read_text())
    candidates = candidate_data['candidates']
    if [item['ambiguity_yaw_deg'] for item in candidates] != [0, 180]:
        parser.error('需要已保存的0度与180度候选')
    meshes = []
    preshape = None
    for candidate in candidates:
        mesh_path = candidate_path.parent/candidate['hand_mesh']
        record_path = candidate_path.parent/candidate['overlay_record']
        record = json.loads(record_path.read_text())
        if digest(mesh_path) != record['outputs_sha256'][mesh_path.name]:
            parser.error('已保存的手网格摘要发生变化')
        if record['export_length_unit'] != 'mm' or record['frame_id'] != 'camera':
            parser.error('手网格必须使用相机系毫米坐标')
        if preshape is None:
            preshape = record['finger_channels']
        elif preshape != record['finger_channels']:
            parser.error('两个方向的接近预型不同，不能作为同一组预型进行对比')
        meshes.append(trimesh.load(mesh_path, force='mesh', process=False))
    center = np.asarray(candidates[0]['camera_from_cap'])[:3, 3]*1000
    all_bounds = np.vstack([points, *[mesh.vertices for mesh in meshes]])
    payload = {'point_count': len(points), 'focus_bounds_mm': [(center-300).tolist(), (center+300).tolist()],
        'full_bounds_mm': [all_bounds.min(0).tolist(), all_bounds.max(0).tolist()],
        'traces': [point_trace(points, np.flatnonzero(~cap_mask)), point_trace(points, cap_rows, True),
                   *[mesh_trace(mesh, angle) for mesh, angle in zip(meshes, [0, 180])]]}
    embedded = json.dumps(payload, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
    for char in ['<', '>', '&', '\u2028', '\u2029']:
        embedded = embedded.replace(char, f'\\u{ord(char):04x}')
    library_path = ROOT/'artifacts/perception/neisai_pcd_20261022/plotly.min.js'
    library = re.sub(r'</script', r'<\\/script', library_path.read_text(), flags=re.I)
    page = PAGE.replace('__DATA__', embedded, 1).replace('/*__PLOTLY__*/', library, 1)
    page = page.replace('__PREGRASP__', json.dumps(preshape), 1)
    output.mkdir(parents=True)
    (output/'index.html').write_text(page)
    shutil.copyfile(source, output/'scene_cleaned.pcd')
    for angle, candidate in zip([0, 180], candidates):
        shutil.copyfile(candidate_path.parent/candidate['hand_mesh'], output/f'hand_{angle:03d}.ply')
    if not args.no_preview:
        write_preview(output, points, cap_mask, meshes, center)
    # 从导出的网页反读坐标和三角面，验证没有在显示时二次变换或缩放。
    restored = json.loads(re.search(r'<script id="scene-data" type="application/json">(.*?)</script>', page, re.S)[1])
    rows_seen = []
    for item in restored['traces'][:2]:
        rows = np.asarray(item['customdata'], dtype=int)
        np.testing.assert_array_equal(np.column_stack([item[key] for key in ['x', 'y', 'z']]), points[rows])
        rows_seen.extend(rows.tolist())
    np.testing.assert_array_equal(np.sort(rows_seen), np.arange(len(points)))
    for item, mesh in zip(restored['traces'][2:], meshes):
        np.testing.assert_array_equal(np.column_stack([item[key] for key in ['x', 'y', 'z']]), mesh.vertices)
        np.testing.assert_array_equal(np.column_stack([item[key] for key in ['i', 'j', 'k']]), mesh.faces)
    validation = {'scene_point_count': len(points), 'cap_points_retained': len(cap_rows),
        'original_scan_point_count': len(original.xyz_raw),
        'max_scene_to_original_distance_mm': float(distances.max()),
        'max_cap_to_cleaned_distance_mm': float(cap_distances.max()),
        'every_cleaned_point_displayed_once': True, 'hand_vertices_and_faces_unchanged': True,
        'hand_grasp_poses_unchanged': True, 'coordinate_transform_applied': False,
        'collision_check_performed': False, 'approach_offset_applied': False, 'robot_connections': 0}
    exported_candidates = []
    for candidate in candidates:
        exported = {**candidate,
            'source_hand_mesh': str(candidate_path.parent/candidate['hand_mesh']),
            'source_overlay_record': str(candidate_path.parent/candidate['overlay_record']),
            'pregrasp_channels': preshape,
            'hand_mesh': f"hand_{candidate['ambiguity_yaw_deg']:03d}.ply"}
        exported.pop('overlay_record')
        exported.pop('cap_cloud')
        exported_candidates.append(exported)
    record = {'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
        'mode': 'two_grasps_on_user_cleaned_scene', 'scene_source': str(source), 'scene_sha256': digest(source),
        'scene_config_sha256': digest(args.scene_config), 'frame_id': 'camera', 'length_unit': 'mm',
        'pose_translation_unit': 'm',
        'pregrasp_channels': preshape,
        'grasp_candidates_source': str(candidate_path), 'grasp_candidates_sha256': digest(candidate_path),
        'candidates': exported_candidates, 'validation': validation}
    (output/'result.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
    (output/'validation.json').write_text(json.dumps(validation, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'output_dir': str(output), 'validation': validation,
                      'html_size_bytes': (output/'index.html').stat().st_size}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
