"""把单组参考与修正结果画成可交互页面和显示网格投影图。"""
import json
import os
from pathlib import Path
import sys

os.environ.setdefault('MPLCONFIGDIR', '/tmp/cap-single-reference-matplotlib')
import numpy as np
import trimesh
from threadpoolctl import threadpool_limits

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
sys.path.insert(0, str(ROOT/'src'))
from fuel_cap_task.fixed_contact_kinematics import NativeHandKinematics
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE


def transform(points, matrix):
    """米制刚体坐标变换，显示时另乘1000。"""
    return np.asarray(points)@matrix[:3, :3].T+matrix[:3, 3]


def representatives(points, spacing):
    """显示用格代表点，碰撞数据不变。"""
    _, rows = np.unique(np.floor(points/spacing).astype(int), axis=0, return_index=True)
    return points[rows]


def main():
    """三种静态构型分别显示；页面不插值播放为闭合或接近路径。"""
    result = json.loads((OUT/'result.json').read_text())
    source = Path(result['source_run'])
    volume, geometry = np.load(source/'network_volume.npz'), np.load(source/'geometry_1mm.npz')
    manual = np.load(result['manual_file'])
    frame = np.asarray(result['volume_from_manual'])
    local_from_volume = np.linalg.inv(frame)
    scene = transform(geometry['points_volume_m'],
        local_from_volume@np.linalg.inv(volume['reference_from_volume'])@geometry['reference_from_volume'])
    labels = geometry['target_point_mask']
    crop = (np.all(abs(scene[:, :2]) < .14, axis=1) & (scene[:, 2] > -.055) & (scene[:, 2] < .21))
    environment = representatives(scene[crop & ~labels], .001)
    index = np.floor(manual['points_volume_m']/.001).astype(int)
    valid = np.all((index >= 0) & (index < 160), axis=1)
    accepted = np.zeros(len(index), bool)
    accepted[valid] = manual['accepted_unknown_surface_grid'][tuple(index[valid].T)]
    assumed = representatives(transform(manual['points_volume_m'][accepted], local_from_volume), .0005)
    kin = NativeHandKinematics(result['model_dir'], DEFAULT_SCENE)
    meshes = []
    for link in kin.metadata['link_names']:
        mesh = trimesh.load(Path(result['model_dir'])/'meshes'/(link+'.obj'), force='mesh', process=False)
        meshes.append(dict(link=link, vertices=np.round(mesh.vertices*1000, 6).tolist(), faces=mesh.faces.tolist()))
    frames = []
    chosen = [result['preform'], result['states'][0], result['final']]
    titles = ['650预张开（静态对照）', '闭合参考（耦合换算）', '本组最终抓姿']
    for title, state in zip(titles, chosen):
        kin.set_state(state['pose'], state['drivers'])
        poses = []
        for mesh in meshes:
            body = kin.hand.data.body(mesh['link'])
            body_pose = np.eye(4)
            body_pose[:3, :3] = body.xmat.reshape(3, 3)
            body_pose[:3, 3] = body.xpos
            pose = local_from_volume@kin.pose@body_pose
            pose[:3, 3] *= 1000
            poses.append(pose.tolist())
        points = transform([m['point_m'] for m in state['metrics']], local_from_volume)*1000
        frames.append(dict(title=title, poses=poses, points=points.tolist(),
            distances_mm=[m['distance_m']*1000 for m in state['metrics']],
            angles_deg=[m['angle_deg'] for m in state['metrics']], geometry_pass=state['geometry_pass'],
            maximum_excess_mm=max(state['collision'].values())*1000,
            environment_penetration_pairs=state['counts']['environment']))
    payload = dict(meshes=meshes, frames=frames, environment=np.round(environment*1000, 4).tolist(),
        target=np.round(scene[labels]*1000, 5).tolist(), assumed=np.round(assumed*1000, 5).tolist(),
        targets=(transform([c['target_m'] for c in result['contacts']], local_from_volume)*1000).tolist(),
        passes=result['passes'], scene_count=result['scene_point_count'])
    (OUT/'scene_data.js').write_text('const DATA='+json.dumps(payload, ensure_ascii=False, separators=(',', ':'))+';\n')
    render_static(result, payload)
    print(json.dumps(dict(frames=len(frames), mesh_count=len(meshes), environment_displayed=len(environment),
        target_displayed=int(labels.sum()), assumed_displayed=len(assumed)), ensure_ascii=False))


def render_static(result, payload):
    """用全部原显示三角面作正交投影；彩点覆盖其上，便于识别逐指对应。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.font_manager import FontProperties, fontManager
    fontManager.addfont('/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf')
    font = FontProperties(family=['DejaVu Sans', 'Droid Sans Fallback'])
    final = payload['frames'][2]
    triangles = []
    for mesh, pose in zip(payload['meshes'],final['poses']):
        matrix = np.asarray(pose)
        vertices = np.asarray(mesh['vertices'])@matrix[:3,:3].T+matrix[:3,3]
        triangles.append(vertices[np.asarray(mesh['faces'])])
    triangles = np.concatenate(triangles)
    area_normals = np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    area_normals /= np.maximum(np.linalg.norm(area_normals,axis=1)[:,None],1e-20)
    light = np.array([.3,.5,1.]);light /= np.linalg.norm(light)
    shade = .50+.50*np.maximum(area_normals@light,0)
    face_colors = np.array([.48,.59,.68])[None,:]*shade[:,None]
    points,targets = np.asarray(final['points']),np.asarray(payload['targets'])
    colors = ['#ef681b','#2074d4','#1aa25c']
    views = [('整手显示网格投影',[1.2,1.8,1.1],None),
             ('沿握梁长轴看两侧',[1,0,0],([-28,28],[-24,17])),
             ('沿横向看三指',[0,-1,0],([-38,38],[-24,17]))]
    fig, axes = plt.subplots(1,3,figsize=(16,6.3))
    for ax,(title,direction,limits) in zip(axes,views):
        direction = np.asarray(direction,dtype=float);direction /= np.linalg.norm(direction)
        right = np.cross([0.,0.,1.],direction);right /= np.linalg.norm(right)
        up = np.cross(direction,right)
        basis = np.column_stack([right,up])
        depth = triangles.mean(axis=1)@direction
        order = np.argsort(depth)
        ax.add_collection(PolyCollection(triangles[order]@basis,facecolors=face_colors[order],edgecolors='none',rasterized=True))
        for key,color,size in [('target','#d5a63e',1),('assumed','#a66bc7',1)]:
            cloud = np.asarray(payload[key])@basis
            ax.scatter(cloud[:,0],cloud[:,1],s=size,c=color,alpha=.35,rasterized=True)
        for j,(point,target) in enumerate(zip(points@basis,targets@basis)):
            ax.plot([point[0],target[0]],[point[1],target[1]],'--',color=colors[j],linewidth=1.5)
            ax.scatter(*point,s=42,c=colors[j],zorder=4)
            ax.scatter(*target,s=16,c='white',edgecolor=colors[j],marker='D',zorder=5)
            ax.annotate(['拇指','食指','中指'][j],point,xytext=(6,8+j*5),textcoords='offset points',
                fontproperties=font,color=colors[j],fontsize=11,zorder=6)
        ax.autoscale();ax.set_aspect('equal');ax.grid(alpha=.15)
        ax.set_title(title,fontproperties=font)
        ax.set_xlabel('投影横向 / mm',fontproperties=font);ax.set_ylabel('投影高度 / mm',fontproperties=font)
        if limits:
            ax.set_xlim(*limits[0]);ax.set_ylim(*limits[1])
    distances=' / '.join('%.2f'%v for v in final['distances_mm'])
    fig.suptitle('一组参考抓姿：三指距离 '+distances+' mm',fontproperties=font,fontsize=17)
    fig.text(.04,.055,'彩色大点：实际指面位置；同色空心菱形：参考目标；紫色侧面：人工假设',fontproperties=font,fontsize=12)
    fig.text(.04,.015,'全部634610实测点检查无超容差侵入。仅验证静态几何，实际接触和抓稳尚待验证。',fontproperties=font,fontsize=12)
    fig.tight_layout(rect=[0,.1,1,.94]);fig.savefig(OUT/'single_grasp.png',dpi=150);plt.close(fig)
    (OUT/'rendering.json').write_text(json.dumps(dict(backend='Matplotlib Agg',
        geometry='all_exported_visual_mesh_triangles',triangle_count=len(triangles),
        projection_markers_overlaid=True,opengl_available=False,
        fallback_reason='EGL device absent and X11 display unavailable in this session'),indent=2)+'\n')


if __name__ == '__main__':
    with threadpool_limits(limits=1):
        main()
