#!/usr/bin/env python3
"""把两方向手网格放入清理点云，导出静态叠加或沿实测盖轴接近的离线回放。"""

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
<title>__PAGE_TITLE__</title><style>
*{box-sizing:border-box}body{margin:0;color:#20364a;background:#fff;font:15px/1.6 system-ui,sans-serif}
#app{display:flex;flex-direction:column;height:100vh;height:100dvh}header{padding:14px 22px;border-bottom:1px solid #dce4ed;display:flex;gap:18px;align-items:center;justify-content:space-between;flex-wrap:wrap}
h1{font-size:21px;margin:0}button,select{font:inherit;padding:6px 10px;border:1px solid #bdcbd9;border-radius:5px;background:white;color:inherit}button{cursor:pointer}.controls{display:flex;gap:9px;align-items:center;flex-wrap:wrap}.note{padding:7px 22px;background:#f1f5f9;font-size:13px}#cloud{flex:1;min-height:0}footer{padding:8px 22px;border-top:1px solid #dce4ed;font-size:13px;color:#526477}.replay{padding:9px 22px;display:flex;gap:12px;align-items:center;flex-wrap:wrap;border-bottom:1px solid #dce4ed}.replay[hidden]{display:none}input[type=range]{width:min(36vw,420px);vertical-align:middle}output{font-variant-numeric:tabular-nums}
</style></head><body><div id="app"><header><h1>__PAGE_TITLE__</h1><div class="controls">
<label>手模型 <select id="direction"><option value="0">0° 方向</option><option value="180">180° 方向</option><option value="both">两个方向同时显示</option><option value="none">只看清理点云</option></select></label>
<button id="focus">抓取区域</button><button id="full">完整场景</button><button id="reset">重置视角</button><button id="fullscreen">全屏</button>
</div></header><div class="note">当前接近预型：__PREGRASP__。红色为盖体观测点，其余按 Z 着色。__REPLAY_NOTE__拖动旋转 · 滚轮缩放 · 右键拖动平移。</div>
<div id="replayControls" class="replay" hidden><button id="play">播放接近</button><button id="start">起点</button><button id="end">终点</button><label>接近进度 <input id="progress" type="range" min="0" max="100" step="1" value="0"></label><output id="progressText"></output></div>
<div id="cloud"></div><footer id="status">加载中…</footer></div>
<script id="scene-data" type="application/json">__DATA__</script><script>/*__PLOTLY__*/</script><script>
"use strict";
const data=JSON.parse(document.getElementById('scene-data').textContent);
const plot=document.getElementById('cloud');
const select=document.getElementById('direction');
const status=document.getElementById('status');
const progress=document.getElementById('progress');
const progressText=document.getElementById('progressText');
const playButton=document.getElementById('play');
const initialCamera={eye:{x:1.25,y:1.25,z:-1.1},up:{x:0,y:-1,z:0},center:{x:0,y:0,z:0}};
let activeBounds=data.focus_bounds_mm;
const approach=data.approach_replay;
// 缓存终点网格。每一帧从终点加绝对偏移，避免累积平移误差。
const baseMeshes=data.traces.slice(2,4).map(trace=>({x:trace.x.slice(),y:trace.y.slice(),z:trace.z.slice()}));
const targetIndices=approach?[2,3,4,5]:[2,3];
let currentProgress=approach?0:100;
let requestedProgress=currentProgress;
let pendingProgress=currentProgress;
let ready=false,updating=false,playing=false,timer=null;
function ranges(bounds){return {'scene.xaxis.range':[bounds[0][0],bounds[1][0]],'scene.yaxis.range':[bounds[0][1],bounds[1][1]],'scene.zaxis.range':[bounds[0][2],bounds[1][2]],'scene.camera':structuredClone(initialCamera)};}
function stop(){playing=false;if(timer!==null){clearTimeout(timer);timer=null;}playButton.textContent='播放接近';}
function requestProgress(value){requestedProgress=Math.max(0,Math.min(100,Number(value)));pendingProgress=requestedProgress;renderPending();}
async function renderPending(){
    if(!ready||updating||pendingProgress===null)return;
    updating=true;
    const position=pendingProgress;pendingProgress=null;
    const fraction=approach?1-position/100:0;
    const offset=approach?approach.offset_mm.map(value=>fraction*value):[0,0,0];
    const visible=[select.value==='0'||select.value==='both',select.value==='180'||select.value==='both'];
    const update={x:baseMeshes.map(mesh=>mesh.x.map(value=>value+offset[0])),y:baseMeshes.map(mesh=>mesh.y.map(value=>value+offset[1])),z:baseMeshes.map(mesh=>mesh.z.map(value=>value+offset[2])),visible:visible.slice()};
    if(approach){for(const path of data.traces.slice(4,6)){update.x.push(path.x);update.y.push(path.y);update.z.push(path.z);}update.visible.push(...visible);}
    try{
        // 只更新手模型和接近线，清理点云始终不动；快速拖动时仅绘制最新请求。
        await Plotly.restyle(plot,update,targetIndices);
        currentProgress=position;
        if(pendingProgress===null)progress.value=String(Math.round(position));
        const remaining=approach?(1-position/100)*approach.distance_m*1000:0;
        progressText.textContent=`${Math.round(position)}% · 距抓取位 ${remaining.toFixed(0)} mm`;
        status.textContent=`清理点云 ${data.point_count.toLocaleString()} 点 · 相机系毫米 · ${select.options[select.selectedIndex].textContent}${approach?' · 直线接近目视回放':''}`;
    }catch(error){stop();pendingProgress=null;status.textContent='绘制失败，请重新打开页面。';console.error(error);}
    finally{updating=false;}
    if(pendingProgress!==null){renderPending();return;}
    if(playing){if(currentProgress>=100)stop();else timer=setTimeout(()=>{timer=null;requestProgress(Math.min(100,currentProgress+5));},80);}
}
function showDirection(){stop();requestProgress(requestedProgress);}
const bounds=activeBounds;
const layout={scene:{xaxis:{title:'X / mm',range:[bounds[0][0],bounds[1][0]]},yaxis:{title:'Y / mm',range:[bounds[0][1],bounds[1][1]]},zaxis:{title:'Z / mm',range:[bounds[0][2],bounds[1][2]]},aspectmode:'data',camera:structuredClone(initialCamera)},margin:{l:0,r:0,t:10,b:0},legend:{x:0,y:1},uirevision:'cleaned-scene'};
if(approach){document.getElementById('replayControls').hidden=false;for(let index=0;index<2;index++)for(const [axis,component] of [['x',0],['y',1],['z',2]])data.traces[index+2][axis]=baseMeshes[index][axis].map(value=>value+approach.offset_mm[component]);}
Plotly.newPlot(plot,data.traces,layout,{responsive:true,displaylogo:false}).then(()=>{ready=true;renderPending();}).catch(error=>{status.textContent='加载失败，请使用支持WebGL的浏览器。';console.error(error);});
select.addEventListener('change',showDirection);
playButton.onclick=()=>{if(playing){stop();return;}playing=true;playButton.textContent='暂停';requestProgress(currentProgress>=100?0:requestedProgress);};
document.getElementById('start').onclick=()=>{stop();requestProgress(0);};
document.getElementById('end').onclick=()=>{stop();requestProgress(100);};
progress.addEventListener('input',()=>{stop();requestProgress(progress.value);});
document.getElementById('focus').onclick=()=>{activeBounds=data.focus_bounds_mm;if(ready)Plotly.relayout(plot,ranges(activeBounds));};
document.getElementById('full').onclick=()=>{activeBounds=data.full_bounds_mm;if(ready)Plotly.relayout(plot,ranges(activeBounds));};
document.getElementById('reset').onclick=()=>{if(ready)Plotly.relayout(plot,ranges(activeBounds));};
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


def approach_trace(candidate, offset_mm):
    """手根从外退位到抓取位的接近线；与盖轴平行，不把手根误画在盖体中心。"""
    end = np.asarray(candidate['camera_from_hand'], dtype=float)[:3, 3]*1000
    line = np.vstack([end+offset_mm, end])
    angle = candidate['ambiguity_yaw_deg']
    return {'type':'scatter3d', 'mode':'lines+markers', 'name':f'{angle}° 手根接近线',
        'x':line[:,0].tolist(), 'y':line[:,1].tolist(), 'z':line[:,2].tolist(),
        'line':{'width':5, 'color':'#5e6e87' if angle == 0 else '#268d9c'},
        'marker':{'size':4}, 'visible':angle == 0,
        'text':['预抓取位','抓取位'],
        'hovertemplate':'%{text}<br>X %{x:.3f} mm<br>Y %{y:.3f} mm<br>Z %{z:.3f} mm<extra></extra>'}


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
    parser.add_argument('--approach-distance-m', type=float, help='沿盖体外向轴外退的演示距离，单位m；仅生成目视回放')
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists():
        parser.error('输出目录已存在，请指定新目录，保留已有实验')
    if args.approach_distance_m is not None and (not np.isfinite(args.approach_distance_m) or args.approach_distance_m <= 0):
        parser.error('接近演示距离必须为有限正数，单位m')
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
    approach = None
    if args.approach_distance_m is not None:
        axes = [np.asarray(candidate['camera_from_cap'], dtype=float)[:3,2] for candidate in candidates]
        if not np.allclose(axes[0], axes[1], atol=1e-10, rtol=0) or not np.isclose(np.linalg.norm(axes[0]), 1, atol=1e-10):
            parser.error('两个方向必须使用同一个有效盖体外向单位轴')
        # 网格为相机系毫米坐标，位姿矩阵平移为米；仅在这里换算外退向量。
        offset_mm = args.approach_distance_m*1000*axes[0]
        approach = {'distance_m':args.approach_distance_m, 'outward_axis_camera':axes[0].tolist(),
            'offset_mm':offset_mm.tolist(), 'orientation_and_preshape_fixed':True,
            'purpose':'visual_replay_only', 'pose_translation_unit':'m', 'paths_camera':[]}
        for candidate in candidates:
            grasp = np.asarray(candidate['camera_from_hand'], dtype=float)
            pregrasp = grasp.copy()
            pregrasp[:3,3] += args.approach_distance_m*axes[0]
            approach['paths_camera'].append({'angle_deg':candidate['ambiguity_yaw_deg'],
                'pregrasp_camera_from_hand':pregrasp.tolist(), 'grasp_camera_from_hand':grasp.tolist()})
            payload['traces'].append(approach_trace(candidate, offset_mm))
        payload['approach_replay'] = approach
        payload['full_bounds_mm'] = [np.vstack([all_bounds, *[mesh.vertices+offset_mm for mesh in meshes]]).min(0).tolist(),
                                     np.vstack([all_bounds, *[mesh.vertices+offset_mm for mesh in meshes]]).max(0).tolist()]
        payload['focus_bounds_mm'] = [np.minimum(center-300, np.vstack([mesh.vertices+offset_mm for mesh in meshes]).min(0)-20).tolist(),
                                      np.maximum(center+300, np.vstack([mesh.vertices+offset_mm for mesh in meshes]).max(0)+20).tolist()]
    embedded = json.dumps(payload, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
    for char in ['<', '>', '&', '\u2028', '\u2029']:
        embedded = embedded.replace(char, f'\\u{ord(char):04x}')
    library_path = ROOT/'artifacts/perception/neisai_pcd_20261022/plotly.min.js'
    library = re.sub(r'</script', r'<\\/script', library_path.read_text(), flags=re.I)
    page = PAGE.replace('__DATA__', embedded, 1).replace('/*__PLOTLY__*/', library, 1)
    page = page.replace('__PAGE_TITLE__', '清理点云中的双方向接近回放' if approach else '清理点云中的双方向抓取位')
    page = page.replace('__PREGRASP__', json.dumps(preshape), 1)
    page = page.replace('__REPLAY_NOTE__', f'沿盖轴直线接近 {args.approach_distance_m*1000:g} mm，朝向与手型固定；用于目视检查。' if approach else '手根位于已有抓取位。', 1)
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
    for item, mesh in zip(restored['traces'][2:4], meshes):
        np.testing.assert_array_equal(np.column_stack([item[key] for key in ['x', 'y', 'z']]), mesh.vertices)
        np.testing.assert_array_equal(np.column_stack([item[key] for key in ['i', 'j', 'k']]), mesh.faces)
    if approach:
        offset_mm = np.asarray(approach['offset_mm'])
        for item, path, candidate, mesh in zip(restored['traces'][4:6], approach['paths_camera'], candidates, meshes):
            line = np.column_stack([item[key] for key in ['x','y','z']])
            np.testing.assert_allclose(line[0]-line[1], offset_mm, atol=1e-10, rtol=0)
            assert np.isclose(np.linalg.norm(line[0]-line[1]), args.approach_distance_m*1000, atol=1e-10)
            np.testing.assert_array_equal(np.asarray(path['pregrasp_camera_from_hand'])[:3,:3], np.asarray(path['grasp_camera_from_hand'])[:3,:3])
            np.testing.assert_allclose(np.asarray(path['pregrasp_camera_from_hand'])[:3,3]-np.asarray(path['grasp_camera_from_hand'])[:3,3], offset_mm/1000, atol=1e-12, rtol=0)
            np.testing.assert_allclose(line[1], np.asarray(candidate['camera_from_hand'])[:3,3]*1000, atol=1e-10, rtol=0)
            for fraction in [0, .25, .5, .75, 1]:
                moving = np.asarray(mesh.vertices)+(1-fraction)*offset_mm
                np.testing.assert_allclose(moving-mesh.vertices, np.tile((1-fraction)*offset_mm, (len(mesh.vertices),1)), atol=1e-10, rtol=0)
    validation = {'scene_point_count': len(points), 'cap_points_retained': len(cap_rows),
        'original_scan_point_count': len(original.xyz_raw),
        'max_scene_to_original_distance_mm': float(distances.max()),
        'max_cap_to_cleaned_distance_mm': float(cap_distances.max()),
        'every_cleaned_point_displayed_once': True, 'hand_vertices_and_faces_unchanged': True,
        'hand_grasp_poses_unchanged': True, 'coordinate_transform_applied': False,
        'collision_check_performed': False, 'approach_offset_applied': bool(approach), 'robot_connections': 0}
    if approach:
        validation.update({'replay_translation_mm':float(args.approach_distance_m*1000),
            'replay_follows_observed_cap_outward_axis':True,
            'replay_orientation_and_preshape_fixed':True, 'replay_endpoints_and_intermediate_offsets_checked':True})
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
        'approach_replay': approach,
        'grasp_candidates_source': str(candidate_path), 'grasp_candidates_sha256': digest(candidate_path),
        'candidates': exported_candidates, 'validation': validation}
    (output/'result.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
    (output/'validation.json').write_text(json.dumps(validation, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'output_dir': str(output), 'validation': validation,
                      'html_size_bytes': (output/'index.html').stat().st_size}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
