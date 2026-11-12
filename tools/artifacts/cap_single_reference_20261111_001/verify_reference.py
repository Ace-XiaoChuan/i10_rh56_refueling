"""用原独立几何检查器和原始数组重算单组结果，再验证页面的真实回调。"""
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from scipy.spatial import cKDTree
from threadpoolctl import threadpool_limits
import trimesh

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
sys.path.insert(0, str(ROOT/'src'))
from fuel_cap_task.observed_hand_geometry import ObservedHandGeometry
from fuel_cap_task.rh56_drive_map import RH56DriveMap
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE
from fuel_cap_task.saved_sdk_input import digest_file, save_json


def main():
    """不用局部优化器、Jacobian或KD广相复算碰撞；只读保存的实际构型。"""
    report = json.loads((OUT/'result.json').read_text())
    for name, expected in report['sources_sha256'].items():
        assert digest_file(name) == expected, name
    source = Path(report['source_run'])
    geometry, volume = np.load(source/'geometry_1mm.npz'), np.load(source/'network_volume.npz')
    mapping = np.linalg.inv(volume['reference_from_volume'])@geometry['reference_from_volume']
    points = geometry['points_volume_m']@mapping[:3,:3].T+mapping[:3,3]
    labels = geometry['target_point_mask']
    assert len(points) == report['scene_point_count'] == 634610
    assert labels.sum() == report['target_point_count'] == 3724
    manual = np.load(report['manual_file'])
    index = np.floor(manual['points_volume_m']/.001).astype(int)
    valid = np.all((index >= 0) & (index < 160), axis=1)
    accepted = np.zeros(len(index), bool)
    accepted[valid] = manual['accepted_unknown_surface_grid'][tuple(index[valid].T)]
    assumed = manual['points_volume_m'][accepted]
    assert len(assumed) == report['assumed_sample_count'] == 27879
    frame = np.asarray(report['volume_from_manual'])
    normals = []
    for contact in report['contacts']:
        mesh = trimesh.load(Path(report['model_dir'])/'meshes'/(contact['link']+'.obj'), force='mesh', process=False)
        p = np.asarray(contact['barycentric'])@mesh.triangles[contact['face_id']]
        np.testing.assert_allclose(p, contact['point_link_m'], atol=1e-12, rtol=0)
        np.testing.assert_allclose(mesh.face_normals[contact['face_id']], contact['normal_link'], atol=1e-12, rtol=0)
        distance, row = cKDTree(assumed).query(contact['target_m'])
        assert distance < 1e-12 and contact['network_feature'] is False
        target = (np.asarray(contact['target_m'])-frame[:3,3])@frame[:3,:3]
        assert target[1]*contact['side'] > 0 and -.01600001 <= target[2] <= -.00099999
        normal = frame[:3,:3]@np.array([0.,contact['side'],1/17])
        normal /= np.linalg.norm(normal)
        np.testing.assert_allclose(normal, contact['target_normal'], atol=1e-12, rtol=0)
        normals.append(normal)
    assert [c['finger'] for c in report['contacts']] == ['thumb','index','middle']
    assert [c['side'] for c in report['contacts']] == [1,-1,-1]
    metadata = json.loads((Path(report['model_dir'])/'model.json').read_text())
    drives = RH56DriveMap.from_dict(metadata['mapping'])
    hand = ObservedHandGeometry(DEFAULT_SCENE)
    rows = []
    for state in [report['preform']]+report['states']:
        q = np.asarray(state['drivers']);assert drives.validate(q)
        hand.place(drives.joint_names, drives.expand(q))
        pose = np.asarray(state['pose'])
        assert np.max(abs(pose[:3,:3].T@pose[:3,:3]-np.eye(3))) < 1e-12
        maxima, counts = {}, {}
        for name, cloud in [('environment',points[~labels]),('target',points[labels]),('assumed_side',assumed)]:
            local = (cloud-pose[:3,3])@pose[:3,:3]
            penetration_rows, inside = hand.surface_penetrations(local, np.ones(len(cloud),bool),
                np.arange(len(cloud)), .0001)
            maxima[name] = max([max(0.,r['maximum_inside_depth_m']-.0001) for r in penetration_rows]+[0.])
            counts[name] = sum(r['target_points'] for r in penetration_rows)
        self_rows = hand.self_penetrations(.0001)
        maxima['self'] = max([max(0.,-r['distance_m']-.0001) for r in self_rows]+[0.])
        counts['self'] = len(self_rows)
        for name in maxima:
            assert abs(maxima[name]-state['collision'][name]) < 1e-11, (name,maxima,state['collision'])
            assert counts[name] == state['counts'][name]
        metrics = []
        for i, (contact, saved) in enumerate(zip(report['contacts'],state['metrics'])):
            body = hand.data.body(contact['link'])
            p = pose[:3,:3]@(body.xmat.reshape(3,3)@np.asarray(contact['point_link_m'])+body.xpos)+pose[:3,3]
            n = pose[:3,:3]@body.xmat.reshape(3,3)@np.asarray(contact['normal_link']);n /= np.linalg.norm(n)
            distance = float(np.linalg.norm(p-contact['target_m']))
            angle = float(np.rad2deg(np.arccos(np.clip(-n@normals[i],-1,1))))
            assert abs(distance-saved['distance_m']) < 1e-12
            assert abs(angle-saved['angle_deg']) < 1e-6
            metrics.append(dict(finger=contact['finger'],distance_mm=distance*1000,angle_deg=angle))
        rows.append(dict(label=state['label'],maximum_excess_m=maxima,counts=counts,metrics=metrics))
    final = rows[-1]
    actual_pass = all(v == 0 for v in final['maximum_excess_m'].values()) and all(
        m['distance_mm'] <= 1 and m['angle_deg'] <= 45 for m in final['metrics'])
    assert actual_pass == report['passes'] is True
    origin, last = np.asarray(report['states'][0]['pose']),np.asarray(report['final']['pose'])
    assert np.linalg.norm(last[:3,3]-origin[:3,3]) < .005*np.sqrt(3)
    for state in report['states']:
        assert state['geometry_pass']
    # 单独输出静态候选；不写回固定执行配置，也不标为有效机器人运动目标。
    base_from_hand = np.asarray(report['reference_from_volume'])@last
    candidate = dict(mode='offline_single_reference_static_candidate',frame_id='left_base',
        left_base_from_hand=base_from_hand.tolist(),volume_from_hand=last.tolist(),
        driver_names=list(drives.driver_names),driver_angles_rad=report['final']['drivers'],
        joint_names=list(drives.joint_names),joint_angles_rad=drives.expand(np.asarray(report['final']['drivers'])).tolist(),
        source_result_sha256=digest_file(OUT/'result.json'),geometry_pass=True,
        contact_geometry_source='hypothetical_manual_side',physical_grasp_verified=False,motion_target_valid=False)
    save_json(OUT/'candidate.json',candidate)
    ui = verify_viewer()
    output = dict(passes=True,checked_state_count=len(rows),source_hash_count=len(report['sources_sha256']),
        scene_point_count=len(points),target_point_count=int(labels.sum()),assumed_sample_count=len(assumed),
        states=rows,viewer=ui,real_browser_checked=False,static_png_rendered_by='Matplotlib Agg visual-mesh projection',
        artifact_sha256={name:digest_file(OUT/name) for name in ['result.json','candidate.json','fit_reference.py',
            'render_reference.py','verify_reference.py','index.html','viewer.js','scene_data.js','single_grasp.png']})
    save_json(OUT/'verification.json',output)
    print(json.dumps(dict(passes=True,checked_states=len(rows),source_hashes=output['source_hash_count'],
        viewer=ui,final=final),ensure_ascii=False),flush=True)


def verify_viewer():
    """实际执行页面JS与全部控件回调；此处不声称真实浏览器WebGL检查。"""
    nodes = list(Path('/home/ace/.vscode-server/bin').glob('*/node'))
    assert nodes, '缺少页面回调检查使用的Node'
    script = r'''
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const elements={};['state','hand','env','assumed','status','metrics','overview','close','reverse'].forEach(id=>elements[id]={value:id==='state'?'2':'',checked:true,textContent:'',innerHTML:''});
let traces=[],layout=null,calls=0;
const context={document:{getElementById:id=>elements[id]},Plotly:{react:(id,tr,la)=>{assert.equal(id,'plot');traces=tr;layout=la;calls++}}};vm.createContext(context);
vm.runInContext(fs.readFileSync('scene_data.js','utf8'),context);vm.runInContext(fs.readFileSync('viewer.js','utf8'),context);
assert(elements.status.textContent.includes('满足当前静态几何条件'));assert(elements.metrics.innerHTML.includes('中指'));
for(const state of ['0','1','2']){elements.state.value=state;elements.state.onchange();
assert.equal(traces.filter(t=>t.name&&t.name.endsWith('参考目标')).length,3);
assert.equal(traces.filter(t=>t.name&&t.name.endsWith('指面')).length,3);
for(const tr of traces)for(const axis of ['x','y','z'])if(tr[axis])assert(tr[axis].every(Number.isFinite));}
elements.state.value='0';elements.state.onchange();assert(elements.status.textContent.includes('存在侵入'));
elements.state.value='2';elements.state.onchange();assert(elements.status.textContent.includes('未检出超容差侵入'));
for(const id of ['hand','env','assumed']){elements[id].checked=false;elements[id].onchange();elements[id].checked=true;elements[id].onchange();}
elements.hand.checked=false;elements.hand.onchange();assert.equal(traces.filter(t=>t.type==='mesh3d').length,0);
elements.close.onclick();assert.equal(layout.scene.xaxis.range[0],-40);elements.reverse.onclick();assert(layout.scene.camera.eye.y<0);elements.overview.onclick();assert.equal(layout.scene.zaxis.range[1],200);
process.stdout.write(JSON.stringify({passes:true,callback_draws:calls,targets:3,model_points:3,configurations:3}));
'''
    completed = subprocess.run([str(nodes[0]),'-e',script],cwd=OUT,text=True,capture_output=True,check=True)
    return json.loads(completed.stdout)


if __name__ == '__main__':
    with threadpool_limits(limits=1):
        main()
