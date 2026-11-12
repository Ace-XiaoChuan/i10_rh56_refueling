"""把保存的局部几何结果导出为自包含HTML，不重新求抓姿或积分。"""
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import trimesh
from threadpoolctl import threadpool_limits

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from fuel_cap_task.fixed_contact_kinematics import NativeHandKinematics
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE
from fuel_cap_task.observed_hand_geometry import primitive_inside_depth
from fuel_cap_task.saved_sdk_input import digest_file, save_json

NAMES = dict(thumb='拇指', index='食指', middle='中指', ring='无名指', pinky='小指')
COLORS = dict(thumb='#ed9229', index='#398bd2', middle='#35a778', ring='#9b78be', pinky='#cd7799')


def transform(points, matrix):
    """按行保存的点；输入、输出均为米。"""
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def xyz(points, digits=3):
    """HTML统一使用毫米；显示舍入不参与源几何检查。"""
    return {k: np.round(points[:, i] * 1000, digits).tolist() for i, k in enumerate('xyz')}


def scatter(points, name, layer, color, size=3, text=None, **kwargs):
    """带图层标签的散点，候选分数与侵入来源可悬停查看。"""
    return dict(type='scatter3d', mode='markers', name=name, meta=dict(layer=layer),
                marker=dict(color=color, size=size), **xyz(points), text=text,
                hovertemplate='%{text}<br>X %{x:.2f} · Y %{y:.2f} · Z %{z:.2f} mm<extra>%{fullData.name}</extra>', **kwargs)


def main():
    """读取几何前后与实际闭合末态，不扩展试验预算。"""
    row = json.loads((OUT / 'final_result.json').read_text())
    manifest = json.loads((OUT / 'manifest.json').read_text())
    assert all(digest_file(Path(p)) == sha for p, sha in manifest['input_sha256'].items())
    run = Path(manifest['source_run'])
    vol, geo, pred = [dict(np.load(run / n)) for n in ['network_volume.npz', 'geometry_1mm.npz', 'contacts.npz']]
    confirmed = dict(np.load(OUT.parent / 'cap_confirmed_sides_20261112_001/confirmed_geometry.npz'))
    frame = confirmed['volume_from_manual'].copy()
    frame[:3, :3] = Rotation.from_matrix(frame[:3, :3]).as_matrix()
    inverse = np.linalg.inv(frame)
    points = transform(geo['points_volume_m'], np.linalg.inv(vol['reference_from_volume']) @ geo['reference_from_volume'])
    local_scene = transform(points, inverse)
    ref = json.loads((OUT.parent / 'cap_single_reference_20261111_001/result.json').read_text())
    kin = NativeHandKinematics(ref['model_dir'], DEFAULT_SCENE)
    static = []
    region = (abs(local_scene[:, 0]) < .12) & (abs(local_scene[:, 1]) < .14) & (local_scene[:, 2] > -.065) & (local_scene[:, 2] < .20)
    environment_ids = np.flatnonzero(region & ~geo['target_point_mask'])
    shown_ids = environment_ids[::max(1, int(np.ceil(len(environment_ids) / 35000))) ]
    static.append(scatter(local_scene[shown_ids], '周边实测点云（显示抽样）', 'environment', '#8796a5', .9, opacity=.28,
                          text=[f'原场景点 {i}' for i in shown_ids]))
    observed_ids = np.flatnonzero(geo['target_point_mask'][:634610])
    observed_ids = observed_ids[::max(1, int(np.ceil(len(observed_ids) / 5000)))]
    static.append(scatter(local_scene[observed_ids], '实测握梁点', 'observed', '#b38931', 1.5, text=['实测目标表面'] * len(observed_ids)))
    vertices = confirmed['vertices_manual_m']
    hull = ConvexHull(vertices)
    static.append(dict(type='mesh3d', name='人工确认握梁参考', meta=dict(layer='grip'), **xyz(vertices),
                       i=hull.simplices[:, 0].tolist(), j=hull.simplices[:, 1].tolist(), k=hull.simplices[:, 2].tolist(),
                       color='#e3be61', opacity=.45, flatshading=True, showlegend=True,
                       hovertemplate='人工确认侧面封成的参考棱柱<extra></extra>'))
    ids = np.argwhere(pred['quality'] >= .1)
    raw_local = transform(ids * .001, inverse)
    valid = (abs(raw_local[:, 0]) <= .03) & (raw_local[:, 2] >= -.017) & (raw_local[:, 2] <= 0)
    for side, label, color in [(-1, '对侧网络候选', '#c19a15'), (1, '拇指侧网络候选', '#bd7134')]:
        chosen = np.flatnonzero(valid & (raw_local[:, 1] * side > 0))
        projected = raw_local[chosen].copy()
        projected[:, 1] = side * (.0085 - projected[:, 2] / 17)
        static.append(scatter(projected, label + '（侧面投影）', 'candidates', color, 2.8,
                              text=[f"原网络格 {ids[i].tolist()}<br>分数 {float(pred['quality'][tuple(ids[i])]):.4f}" for i in chosen]))
    anchor = np.array(row['anchor_manual_m'])[None]
    static.append(scatter(anchor, '换端后拇指锚点', 'anchor', '#182c41', 7, text=['本轮唯一拇指目标'],
                          marker_symbol='diamond'))
    # plotly的symbol属于marker；单独写入保持兼容既有本地Plotly版本。
    static[-1].pop('marker_symbol')
    static[-1]['marker']['symbol'] = 'diamond'
    static[-1]['marker']['line'] = dict(color='white', width=2)
    normal = np.array([0., 1., 1 / 17]); normal /= np.linalg.norm(normal)
    hold = anchor + normal * .0013
    static.append(scatter(hold, '带间隙的拇指预型点', 'anchor', '#f08b23', 5, text=['表面外留1.3 mm；几何求解中保持位置'],
                          opacity=.9))
    # 只显示几何前后和实际闭合末态，避免重复保存中间网格。
    states, keys, steps = [], {}, []
    display_trace = [dict(row['optimization_trace'][0], label='几何第1步'), dict(row['optimization_trace'][-1], label='几何第24步：无超差预型')]
    saved = np.load(OUT / 'states.npz')
    physical_model = mujoco.MjModel.from_xml_path(str(OUT / 'candidate.xml'))
    physical_data = mujoco.MjData(physical_model)
    physical_data.qpos[:] = saved['qpos'][-1]
    physical_angles = [float(physical_data.joint(n).qpos[0]) for n in kin.drives.joint_names]
    display_trace.append(dict(row['optimization_trace'][-1], iteration=24, label='实际闭合：保持对握末态',
                             fingers=[], actual_joint_positions=physical_angles))
    for step in display_trace:
        key = json.dumps([step['volume_from_hand'], step['drivers'], step.get('actual_joint_positions')], separators=(',', ':'))
        if key not in keys:
            state_traces = []
            pose = np.asarray(step['volume_from_hand'])
            kin.set_state(pose, step['drivers'])
            if 'actual_joint_positions' in step:
                kin.hand.place(kin.drives.joint_names, step['actual_joint_positions'])
            manual_from_hand = inverse @ pose
            first = set()
            for geom in kin.hand.geoms:
                kind, planes, low, high = kin.hand.shapes[geom]
                model, data = kin.hand.model, kin.hand.data
                size = model.geom_size[geom]
                if kind == int(mujoco.mjtGeom.mjGEOM_MESH):
                    mid = int(model.geom_dataid[geom]); a, n = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
                    v = np.asarray(model.mesh_vert[a:a + n], dtype=float)
                    f = ConvexHull(v).simplices
                elif kind == int(mujoco.mjtGeom.mjGEOM_BOX):
                    mesh = trimesh.creation.box(extents=2 * size); v, f = mesh.vertices, mesh.faces
                elif kind == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
                    mesh = trimesh.creation.capsule(height=2 * size[1], radius=size[0], count=[12, 16]); v, f = mesh.vertices, mesh.faces
                elif kind == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
                    mesh = trimesh.creation.cylinder(height=2 * size[1], radius=size[0], sections=32); v, f = mesh.vertices, mesh.faces
                elif kind == int(mujoco.mjtGeom.mjGEOM_SPHERE):
                    mesh = trimesh.creation.icosphere(subdivisions=2, radius=size[0]); v, f = mesh.vertices, mesh.faces
                else:
                    raise ValueError('未覆盖的显示形状：' + str(kind))
                v = v @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
                v = transform(v, manual_from_hand)
                body = model.body(int(model.geom_bodyid[geom])).name
                finger = next((name for name in NAMES if body.startswith(name + '_')), 'base')
                name = NAMES.get(finger, '手掌')
                state_traces.append(dict(type='mesh3d', name=name, legendgroup=finger, showlegend=finger not in first,
                    meta=dict(layer='hand'), color=COLORS.get(finger, '#9da9b5'), opacity=.67, flatshading=True,
                    **xyz(v), i=f[:, 0].tolist(), j=f[:, 1].tolist(), k=f[:, 2].tolist(),
                    hovertemplate=name + '<br>碰撞实体：' + body + '<extra></extra>'))
                first.add(finger)
            hits, occupied = kin.hand.surface_penetrations(transform(points, np.linalg.inv(pose)), geo['target_point_mask'], np.arange(len(points)), .0001)
            collision_ids = np.flatnonzero(occupied)
            evidence = local_scene[collision_ids]
            hand_points = transform(points[collision_ids], np.linalg.inv(pose))
            labels = [[] for _ in collision_ids]
            for geom in kin.hand.geoms:
                kind, planes, low, high = kin.hand.shapes[geom]
                local = (hand_points - kin.hand.data.geom_xpos[geom]) @ kin.hand.data.geom_xmat[geom].reshape(3, 3)
                depth = primitive_inside_depth(kind, kin.hand.model.geom_size[geom], local, planes)
                body = kin.hand.model.body(int(kin.hand.model.geom_bodyid[geom])).name
                for i in np.flatnonzero(depth > .0001):
                    labels[i].append(f'{body}：侵入 {depth[i] * 1000:.3f} mm')
            state_traces.append(scatter(evidence, '超出容差的碰撞点', 'collision', '#ef293e', 5,
                                       text=[f'原场景点 {i}<br>' + '<br>'.join(label) for i, label in zip(collision_ids, labels)]))
            if 'actual_joint_positions' in step:
                for contact in row['final_forces']['contacts']:
                    name = NAMES[contact['finger']]
                    label = f"{name}：法向 {contact['normal_force_n']:.2f} N"
                    state_traces.append(scatter(np.array([contact['point_manual_m']]), label, 'contacts',
                                               COLORS[contact['finger']], 7, text=[label + '<br>实际积分接触；未标定仿真']))
            for finger in step['fingers']:
                if not finger['active']:
                    continue
                name = NAMES[finger['finger']]
                ends = np.array([finger['pad_point_manual_m'], finger['projected_target_manual_m']])
                state_traces.append(dict(type='scatter3d', mode='lines+markers', **xyz(ends), name=name + '尝试目标',
                    meta=dict(layer='targets'), marker=dict(color=COLORS[finger['finger']], size=[4, 6], symbol=['circle', 'diamond']),
                    line=dict(color=COLORS[finger['finger']], width=5, dash='dash'),
                    text=[name + '指面样本', name + '对侧网络目标'], hovertemplate='%{text}<extra></extra>'))
            keys[key] = len(states)
            states.append(dict(traces=state_traces, collision_count=len(collision_ids),
                               max_inside_mm=max([h['maximum_inside_depth_m'] - .0001 for h in hits] + [0.]) * 1000,
                               active_fingers=[NAMES[f['finger']] for f in step['fingers'] if f['active']], physics_state='actual_joint_positions' in step))
        steps.append(dict(state=keys[key], accepted=step['accepted'], number=step['iteration'] + 1, label=step['label']))
    payload = dict(static=static, states=states, steps=steps, display_environment_points=len(shown_ids), scene_check_points=len(points))
    js_path = ROOT / '.venv-spahybgen/lib/python3.8/site-packages/plotly/package_data/plotly.min.js'
    plotly_js = js_path.read_text()
    page = TEMPLATE.replace('__PLOTLY__', plotly_js).replace('__DATA__', json.dumps(payload, ensure_ascii=False, separators=(',', ':'), allow_nan=False))
    (OUT / 'index.html').write_text(page)
    report = dict(html='index.html', html_bytes=(OUT / 'index.html').stat().st_size,
                  source_result_sha256=digest_file(OUT / 'final_result.json'), source_verification_sha256=digest_file(OUT / 'verification.json'),
                  exporter_sha256=digest_file(Path(__file__)), plotly_js_sha256=digest_file(js_path),
                  display_frames=len(steps), source_geometric_steps=len(row['optimization_trace']), unique_poses=len(states), scene_check_points=len(points),
                  displayed_environment_points=len(shown_ids), all_collision_points_displayed=True,
                  final_collision_points=states[-1]['collision_count'], embedded_library=True, external_requests_required=False,
                  new_candidates=0, export_new_physics_steps=0, source_physics_steps=row['physics_steps'])
    assert report['final_collision_points'] == 0
    assert all(digest_file(Path(p)) == sha for p, sha in manifest['input_sha256'].items())
    save_json(OUT / 'viewer_report.json', report)
    print(json.dumps(report, ensure_ascii=False))


TEMPLATE = r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>拇指换端后 · 三维小试查看</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#f3f5f8;color:#203044;font:14px/1.65 system-ui,-apple-system,"Microsoft YaHei",sans-serif}header{padding:20px 28px 12px;background:#fff;border-bottom:1px solid #dce3eb}h1{font-size:23px;margin:0 0 5px}header p{margin:0;color:#627084}.badge{display:inline-block;font-size:12px;background:#fff0ed;color:#aa3b2b;padding:3px 10px;border-radius:20px;vertical-align:middle;margin-left:12px}main{display:grid;grid-template-columns:290px minmax(0,1fr);gap:14px;padding:16px;max-width:1900px;margin:auto}.panel,.viewport{background:#fff;border:1px solid #dce3eb;border-radius:10px;overflow:hidden}.panel{padding:18px}h2{font-size:15px;margin:0 0 10px}.section{padding:0 0 16px;margin:0 0 16px;border-bottom:1px solid #e5eaf0}.section:last-child{border:0;padding:0;margin:0}label{display:flex;align-items:center;gap:8px;margin:8px 0;cursor:pointer}input[type=checkbox]{accent-color:#3979bd}input[type=range]{width:100%;accent-color:#3979bd}button{font:inherit;cursor:pointer;border:1px solid #cfd9e5;border-radius:6px;background:#fff;color:#29465e;padding:6px 10px}button:hover{background:#edf4fb}.buttons{display:flex;flex-wrap:wrap;gap:7px}.hint{font-size:12px;color:#6d7a88}.readout{padding:10px 12px;border-radius:7px;background:#f5f7fa;font-size:13px}.readout strong{color:#a93a30}#plot{width:100%;height:calc(100vh - 190px);min-height:570px}.caption{padding:10px 18px;border-top:1px solid #e5eaf0;color:#667687;font-size:12px}.dot{height:9px;width:9px;display:inline-block;border-radius:50%}.legend-note{display:flex;gap:12px;flex-wrap:wrap;margin-top:9px;font-size:12px;color:#667687}details{font-size:12px;color:#667687}summary{cursor:pointer}a{color:#2c6da7}#error{display:none;background:#fff0ed;color:#aa3b2b;padding:12px;margin:12px}@media(max-width:850px){main{grid-template-columns:1fr}.panel{display:grid;grid-template-columns:1fr 1fr;gap:15px}.section{border:0;margin:0;padding:0}#plot{height:70vh;min-height:480px}}
</style><script>__PLOTLY__</script></head>
<body><header><h1>拇指换端后 · 三维查看<span class="badge">固定握梁对握已形成</span></h1><p>拇指换到握梁长轴另一端，实际闭合形成拇指与中指对握，保持0.20秒。</p></header>
<div id="error"></div><main><aside class="panel">
<div class="section"><h2>查看角度</h2><div class="buttons"><button data-camera="oblique">斜视</button><button data-camera="top">俯视</button><button data-camera="side">侧视</button><button data-camera="front">正视</button></div><div class="buttons" style="margin-top:8px"><button id="near">握梁近景</button><button id="whole">查看整手</button></div><p class="hint">拖动旋转，滚轮缩放，右键拖动平移。悬停红点可查看碰撞部位和侵入量。</p></div>
<div class="section"><h2>保存的求解尝试</h2><input id="step" type="range" min="0" max="2" value="2" step="1"><div id="step-label"></div><div id="status" class="readout"></div><p class="hint">依次查看几何第1步、无超差预型、实际闭合保持末态。</p></div>
<div class="section"><h2>显示图层</h2>
<label><input type="checkbox" data-layer="hand" checked>手部碰撞外形</label>
<label><input type="checkbox" data-layer="grip" checked>人工确认握梁</label>
<label><input type="checkbox" data-layer="observed" checked>实测握梁点</label>
<label><input type="checkbox" data-layer="environment" checked>周边实测点云</label>
<label><input type="checkbox" data-layer="candidates" checked>两侧网络候选</label>
<label><input type="checkbox" data-layer="anchor" checked>拇指目标与预型点</label>
<label><input type="checkbox" data-layer="targets" checked>四指尝试目标</label>
<label><input type="checkbox" data-layer="contacts" checked>实际承力接触点</label>
<label><input type="checkbox" data-layer="collision" checked><span class="dot" style="background:#ef293e"></span>全部超差碰撞点</label>
<label for="opacity">手部透明度 <span id="opacity-label">67%</span></label><input id="opacity" type="range" min="10" max="100" value="67"></div>
<div class="section"><details><summary>数据与结果说明</summary><p>单位为mm。X沿握梁长轴，Y跨两侧，Z朝外。换端后目标锚点X=+22.55，握梁两端为±30。</p><p>橙色预型点在目标表面外1.3mm；它在几何求解中保持位置，闭合时依靠真实驱动建立接触，没有手盖黏接。</p><p>周边点云仅在显示时抽样。碰撞采用完整667868点，原容差0.1mm。网格展示实际使用的凸碰撞外形。</p><p>网络候选是原分数≥0.1的格点在确认侧面上的投影，悬停可查看原格点与分数。</p><p><a href="README.md">实验说明</a> · <a href="final_result.json">原始结果</a> · <a href="verification.json">独立复核</a></p></details></div>
</aside><section class="viewport"><div id="plot"></div><div class="caption">黑色菱形：换端后的拇指目标　橙色点：带间隙预型　虚线：手指向对侧目标的引导　红色点：实测周边结构侵入证据。<br>前两帧是几何求解，末帧为实际闭合积分状态。接触力来自未标定仿真，尚未验证旋拧或持盖。图例可单击隐藏某根手指，双击单独查看。</div></section></main>
<script id="viewer-data" type="application/json">__DATA__</script>
<script>
'use strict';
const DATA=JSON.parse(document.getElementById('viewer-data').textContent);
const el=id=>document.getElementById(id), plot=el('plot');
let ready=false;
const views={oblique:{eye:{x:1.4,y:1.6,z:1.15},up:{x:0,y:0,z:1}},top:{eye:{x:0,y:0,z:2.4},up:{x:0,y:1,z:0}},side:{eye:{x:0,y:-2.5,z:.01},up:{x:0,y:0,z:1}},front:{eye:{x:2.5,y:0,z:.01},up:{x:0,y:0,z:1}}};
const layout={margin:{l:0,r:0,t:0,b:0},paper_bgcolor:'white',uirevision:'saved-grasp',showlegend:true,legend:{x:0,y:1,font:{size:11},bgcolor:'rgba(255,255,255,.75)',groupclick:'togglegroup'},scene:{aspectmode:'data',camera:{...views.oblique,projection:{type:'orthographic'}},xaxis:{title:'X 长轴 / mm',range:[-70,45],backgroundcolor:'#f4f7fa',gridcolor:'#dce4ec'},yaxis:{title:'Y 两侧 / mm',range:[-50,70],backgroundcolor:'#f4f7fa',gridcolor:'#dce4ec'},zaxis:{title:'Z 朝外 / mm',range:[-40,85],backgroundcolor:'#f4f7fa',gridcolor:'#dce4ec'}}};
function tracesForStep(){const step=DATA.steps[Number(el('step').value)], state=DATA.states[step.state]; const layers={};document.querySelectorAll('[data-layer]').forEach(c=>layers[c.dataset.layer]=c.checked); return DATA.static.concat(state.traces).map(t=>{const copy={...t,visible:layers[t.meta.layer]};if(t.meta.layer==='hand')copy.opacity=Number(el('opacity').value)/100;return copy;});}
function status(){const step=DATA.steps[Number(el('step').value)], state=DATA.states[step.state];el('step-label').textContent=`第 ${Number(el('step').value)+1} / ${DATA.steps.length} 帧 · ${step.label} · ${step.accepted?'接受更新':'未接受更新'}`;el('status').innerHTML=`${state.collision_count ? '<strong>仍有环境侵入</strong>' : '<strong style="color:#16754b">无超差场景侵入</strong>'}<br>超差点：${state.collision_count} 个<br>最大超差量：${state.max_inside_mm.toFixed(6)} mm<br>允许容差：0.100 mm<br>参与目标引导：${state.active_fingers.join('、')||'见几何预型'}<br>当前状态：${state.physics_state?'实际闭合并保持对握0.20秒<br>拇指 8.37 N · 中指 4.15 N':'几何求解'}`;el('opacity-label').textContent=el('opacity').value+'%';}
function fail(error){el('error').style.display='block';el('error').textContent='三维显示失败：'+String(error)+'。可先查看同目录的 result.png。';}
async function update(){if(!ready)return;try{status();await Plotly.react(plot,tracesForStep(),plot.layout,{responsive:true,displaylogo:false,scrollZoom:true});}catch(error){fail(error);}}
async function init(){try{status();await Plotly.newPlot(plot,tracesForStep(),layout,{responsive:true,displaylogo:false,scrollZoom:true,toImageButtonOptions:{format:'png',filename:'thumb_anchor_spike',scale:2}});ready=true;window.__viewerReady=true;plot.on('plotly_afterplot',()=>{window.__viewerRenderCount=(window.__viewerRenderCount||0)+1;});}catch(error){fail(error);}}
el('step').addEventListener('input',update);el('opacity').addEventListener('input',update);document.querySelectorAll('[data-layer]').forEach(c=>c.addEventListener('change',update));
document.querySelectorAll('[data-camera]').forEach(b=>b.addEventListener('click',()=>{if(ready)Plotly.relayout(plot,{'scene.camera':{...views[b.dataset.camera],projection:{type:'orthographic'}}});}));
function rangeView(near){if(ready)Plotly.relayout(plot,{'scene.xaxis.range':near?[-43,33]:[-105,65],'scene.yaxis.range':near?[-34,40]:[-65,135],'scene.zaxis.range':near?[-28,38]:[-55,175]});}
el('near').addEventListener('click',()=>rangeView(true));el('whole').addEventListener('click',()=>rangeView(false));init();
</script></body></html>'''


if __name__ == '__main__':
    with threadpool_limits(limits=1):
        main()
