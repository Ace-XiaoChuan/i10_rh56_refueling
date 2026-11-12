"""导出自动锚点流程的离线HTML，显示保存的实际关节和接触反馈。"""
import html
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial import ConvexHull
import trimesh

from fuel_cap_task.thumb_anchor import transform

NAMES = dict(thumb='拇指', index='食指', middle='中指', ring='无名指', pinky='小指', base='手掌')
COLORS = dict(thumb='#ec952f', index='#398bd2', middle='#35a778', ring='#9b78be', pinky='#cd7799', base='#9da9b5')


def xyz(points):
    """仅显示换成毫米，计算和源数据仍用米。"""
    points = np.asarray(points).reshape(-1, 3)
    return {name: np.round(points[:, i] * 1000, 3).tolist() for i, name in enumerate('xyz')}


def scatter(points, name, color, size=3):
    """点云和候选点的统一显示样式。"""
    return dict(type='scatter3d', mode='markers', name=name, **xyz(points), marker=dict(color=color, size=size))


def hand_meshes(trial, pose, drivers, actual=None):
    """实际物理帧使用12关节状态，显示与碰撞检查相同的实体。"""
    kin = trial.kin
    kin.set_state(pose, drivers)
    if actual is not None:
        kin.hand.place(kin.drives.joint_names, actual)
    model, data = kin.hand.model, kin.hand.data
    output, seen = [], set()
    for geom in kin.hand.geoms:
        kind, _, _, _ = kin.hand.shapes[geom]
        size = model.geom_size[geom]
        if kind == int(mujoco.mjtGeom.mjGEOM_MESH):
            mesh = int(model.geom_dataid[geom])
            first, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
            vertices = model.mesh_vert[first:first + count].astype(float)
            faces = ConvexHull(vertices).simplices
        else:
            if kind == int(mujoco.mjtGeom.mjGEOM_BOX):
                mesh = trimesh.creation.box(extents=2 * size)
            elif kind == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
                mesh = trimesh.creation.capsule(height=2 * size[1], radius=size[0], count=[12, 16])
            elif kind == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
                mesh = trimesh.creation.cylinder(height=2 * size[1], radius=size[0], sections=24)
            elif kind == int(mujoco.mjtGeom.mjGEOM_SPHERE):
                mesh = trimesh.creation.icosphere(subdivisions=1, radius=size[0])
            else:
                raise ValueError('未覆盖的显示形状')
            vertices, faces = mesh.vertices, mesh.faces
        vertices = vertices @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
        vertices = transform(vertices, trial.inverse @ pose)
        body = model.body(int(model.geom_bodyid[geom])).name
        finger = next((f for f in NAMES if body.startswith(f + '_')), 'base')
        output.append(dict(type='mesh3d', **xyz(vertices), i=faces[:, 0].tolist(), j=faces[:, 1].tolist(), k=faces[:, 2].tolist(),
                           name=NAMES[finger], color=COLORS[finger], opacity=.72, flatshading=True,
                           legendgroup=finger, showlegend=finger not in seen))
        seen.add(finger)
    return output


def write_viewer(trial, result, output):
    """无需网络的三维页面，阶段切换只读取已保存状态。"""
    root = Path(__file__).resolve().parents[1]
    scene = transform(trial.points, trial.inverse)
    near = (np.max(abs(scene[:, :2]), axis=1) < .15) & (scene[:, 2] > -.08) & (scene[:, 2] < .20)
    points = scene[near]
    points = points[::max(1, int(np.ceil(len(points) / 26000)))]
    base = [scatter(points, '完整检查场景的显示抽样', '#8997a4', .8),
            scatter(trial.pool_local, '另一侧网络候选（侧面投影）', '#b99a31', 2)]
    vertices = trial.shape.vertices()
    faces = ConvexHull(vertices).simplices
    base.append(dict(type='mesh3d', **xyz(vertices), i=faces[:, 0].tolist(), j=faces[:, 1].tolist(), k=faces[:, 2].tolist(),
                     color='#e3be61', opacity=.45, name='握梁几何参考'))
    base.append(scatter([r['anchor_grip_m'] for r in trial.anchors], '程序自动筛出的锚点', '#263849', 5))
    frames = []
    for candidate_id, row in enumerate(result['results']):
        last = row['optimization_trace'][-1]
        pose, drivers = np.array(last['volume_from_hand']), np.array(last['drivers'])
        static = base + [scatter([row['anchor_grip_m']], '本候选拇指锚点', '#f47b25', 8)]
        frames.append(dict(label=f'候选{candidate_id}：几何末态', data=static + hand_meshes(trial, pose, drivers)))
        states_file = output / ('candidate_%02d' % candidate_id) / 'states.npz'
        if not states_file.exists():
            continue
        saved = np.load(states_file)
        model = mujoco.MjModel.from_xml_path(str(states_file.parent / 'candidate.xml'))
        data = mujoco.MjData(model)
        # 显示拇指确认、首次对握和末态；每帧均来自真实积分。
        trace = row['contact_trace']
        chosen = {0, len(trace) - 1}
        for phase in ['close_fingers', 'hold']:
            index = next((i for i, item in enumerate(trace) if item['phase'] == phase), None)
            if index is not None:
                chosen.add(index)
        for index in sorted(chosen):
            if index < 0:
                continue
            data.qpos[:] = saved['qpos'][index]
            mujoco.mj_forward(model, data)
            actual = np.array([data.joint(n).qpos[0] for n in trial.kin.drives.joint_names])
            meshes = hand_meshes(trial, pose, drivers, actual)
            for contact in trace[index]['contacts']:
                if contact['normal_force_n'] > .01:
                    label = NAMES[contact['finger']] + f" {contact['normal_force_n']:.2f} N"
                    meshes.append(scatter([contact['point_grip_m']], label, COLORS[contact['finger']], 7))
            item = trace[index]
            frames.append(dict(label=f"实际积分 {item['time_s']:.3f}s · {item['phase']}", data=static + meshes))
    if not frames:
        frames.append(dict(label='没有可选锚点：当前观测与几何', data=base))
    payload = dict(frames=frames, scene_check_points=len(trial.points), result=result)
    # 已安装的Plotly脚本只作为静态资源读取，不引入另一个Python运行环境。
    js_path = root / '.venv-spahybgen/lib/python3.8/site-packages/plotly/package_data/plotly.min.js'
    source = '人工确认握梁定位' if trial.geometry_source == 'human_confirmed_geometry' else '相机几何估计＋已知尺寸'
    summary = f"{result['status']} · 自动检查 {result['candidates_tried']} 个锚点 · {result['elapsed_seconds']:.1f}s"
    page = TEMPLATE.replace('__PLOTLY__', js_path.read_text()).replace('__DATA__', json.dumps(payload, ensure_ascii=False, allow_nan=False).replace('</', '<\\/'))
    page = page.replace('__SUMMARY__', html.escape(summary)).replace('__SOURCE__', html.escape(source))
    (output / 'index.html').write_text(page)
    (output / 'viewer_data.json').write_text(json.dumps(payload, ensure_ascii=False, allow_nan=False))


TEMPLATE = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>自动拇指锚点</title>
<style>body{margin:0;background:#f5f3ed;color:#253344;font:16px system-ui}header{padding:18px 24px}h1{font-size:22px;margin:0 0 8px}p{margin:7px 0}#plot{height:72vh}select{padding:7px;min-width:320px}details{margin:15px 24px}pre{white-space:pre-wrap}</style>
<header><h1>自动选锚点 → 拇指先接触 → 四指闭合</h1><p>__SUMMARY__</p>
<p>定位来源：__SOURCE__。固定握梁仿真，力值未标定；机械臂路径与真机接触反馈待接。</p>
<select id="frame"></select></header><div id="plot"></div>
<details><summary>四指展开、闭合与受阻证据</summary><pre id="evidence"></pre></details>
<script>__PLOTLY__</script><script>const payload=__DATA__;
const select=document.getElementById('frame');payload.frames.forEach((f,i)=>{const o=document.createElement('option');o.value=i;o.textContent=f.label;select.appendChild(o)});
function draw(){const f=payload.frames[Number(select.value)];if(!f)return;Plotly.react('plot',f.data,{margin:{l:0,r:0,t:0,b:0},uirevision:'grip',scene:{aspectmode:'data',xaxis:{title:'长轴 X / mm'},yaxis:{title:'两侧 Y / mm'},zaxis:{title:'朝外 Z / mm'},camera:{eye:{x:1.1,y:1.8,z:1.1}}},legend:{orientation:'h'}},{responsive:true})}
select.value=payload.frames.length-1;select.onchange=draw;draw();document.getElementById('evidence').textContent=JSON.stringify(payload.result.results.map(r=>({opening:r.finger_opening_attempts,participation:r.finger_participation})),null,2);
</script></html>'''
