"""复读单点阴性结果，用独立逐点检查复核阻挡并画出实际几何。"""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
from threadpoolctl import threadpool_limits

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from fuel_cap_task.fixed_contact_kinematics import NativeHandKinematics
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE
from fuel_cap_task.saved_sdk_input import digest_file, save_json


def transform(points, matrix):
    """按行保存的米制刚体变换。"""
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def main():
    """不重新求抓姿或运行闭合，只核对原网络、锚点与真实碰撞。"""
    row = json.loads((OUT / 'result.json').read_text())
    manifest = json.loads((OUT / 'manifest.json').read_text())
    assert all(digest_file(Path(p)) == sha for p, sha in manifest['input_sha256'].items())
    assert row['status'] == 'no_safe_geometric_pose' and row['physics_steps'] == 0
    assert not row['sustained_opposed_contact']
    trace = row['optimization_trace']
    assert len(trace) <= 24 and sum(t['accepted'] for t in trace) == 2
    run = Path(manifest['source_run'])
    pred, vol, geo = [dict(np.load(run / f)) for f in ['contacts.npz', 'network_volume.npz', 'geometry_1mm.npz']]
    confirmed = dict(np.load(OUT.parent / 'cap_confirmed_sides_20261112_001/confirmed_geometry.npz'))
    frame = confirmed['volume_from_manual'].copy()
    frame[:3, :3] = Rotation.from_matrix(frame[:3, :3]).as_matrix()
    assert float(pred['quality'][tuple(row['feature_index_xyz'])]) == row['raw_network_score'] >= .1
    assert row['anchor_manual_m'][0] < -.015
    for t in trace:
        for f in t['fingers']:
            assert float(pred['quality'][tuple(f['feature_index_xyz'])]) == f['raw_network_score'] >= .1
            assert f['projected_target_manual_m'][1] < 0
            assert f['active'] == (f['distance_m'] < .020)
    ref = json.loads((OUT.parent / 'cap_single_reference_20261111_001/result.json').read_text())
    kin = NativeHandKinematics(ref['model_dir'], DEFAULT_SCENE)
    points = transform(geo['points_volume_m'], np.linalg.inv(vol['reference_from_volume']) @ geo['reference_from_volume'])
    independent = []
    for t in [trace[0], trace[-1]]:
        pose, drivers = np.asarray(t['volume_from_hand']), np.asarray(t['drivers'])
        kin.set_state(pose, drivers)
        pad = ref['contacts'][0]
        p = kin.point(pad['link'], pad['point_link_m'], pad['normal_link'])[0]
        normal = frame[:3, :3] @ np.array([0., 1., 1 / 17])
        expected = transform(np.array(row['anchor_manual_m'])[None], frame)[0] + .0013 * normal / np.linalg.norm(normal)
        assert np.linalg.norm(p - expected) < 1e-12
        hits, occupied = kin.hand.surface_penetrations(transform(points, np.linalg.inv(pose)), geo['target_point_mask'], np.arange(len(points)), .0001)
        self_hits = kin.hand.self_penetrations(.0001)
        assert occupied.any() and not self_hits
        max_depth = max(h['maximum_inside_depth_m'] for h in hits)
        assert abs(max_depth - .0001 - t['collision']['environment']) < 1e-10
        independent.append(dict(iteration=t['iteration'], checked_scene_points=len(points),
                                occupied_scene_points=int(occupied.sum()), max_inside_depth_m=max_depth,
                                excess_over_tolerance_m=max_depth - .0001, self_collisions=len(self_hits), hits=hits))
    report = dict(passes=True, result_is_negative=True, candidate_count=1, network_forwards=0,
                  independent_scene_checks=independent, accepted_updates=2, attempted_updates=len(trace),
                  fingers_outside_local_attraction_band=[f['finger'] for f in trace[-1]['fingers'] if not f['active']],
                  physics_executed=False, force_claim_permitted=False,
                  input_hashes_checked=len(manifest['input_sha256']), original_network_scores_unchanged=True,
                  thumb_material_point_fixed_during_geometry=True, physical_grasp_verified=False, motion_target_valid=False)
    save_json(OUT / 'verification.json', report)

    font_manager.fontManager.addfont('/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf')
    plt.rcParams['font.family'] = ['DejaVu Sans', 'Droid Sans Fallback']
    fig, axes = plt.subplots(1, 2, figsize=(11, 5), constrained_layout=True)
    pose = np.asarray(trace[-1]['volume_from_hand'])
    manual_from_hand = np.linalg.inv(frame) @ pose
    model, data = kin.hand.model, kin.hand.data
    colors = dict(thumb='#ed8b24', index='#438bc9', middle='#43a36f', ring='#9a79ad', pinky='#ba7782')
    ids = np.argwhere(pred['quality'] >= .1)
    local = transform(ids * .001, np.linalg.inv(frame))
    pool = local[(local[:, 1] < 0) & (abs(local[:, 0]) <= .03) & (local[:, 2] >= -.017) & (local[:, 2] <= 0)].copy()
    pool[:, 1] = -(.0085 - pool[:, 2] / 17)
    environment = transform(points[~geo['target_point_mask']], np.linalg.inv(frame))
    environment = environment[(abs(environment[:, 0]) < .07) & (abs(environment[:, 1]) < .06) & (abs(environment[:, 2]) < .05)]
    environment = environment[::max(1, len(environment) // 4000)] * 1000
    final_hits = independent[-1]['hits']
    evidence_ids = sorted({i for h in final_hits for i in h['evidence_source_indices']})
    evidence = transform(points[evidence_ids], np.linalg.inv(frame)) * 1000
    vertices = confirmed['vertices_manual_m'] * 1000
    faces = np.array([[0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6], [0, 3, 7], [0, 7, 4], [1, 5, 6], [1, 6, 2], [3, 2, 6], [3, 6, 7]])
    for ax, dims, title in zip(axes, [[0, 1], [0, 2]], ['俯视：单拇指锚点与对侧候选', '侧视：红点为完整实测场景侵入证据']):
        ax.scatter(environment[:, dims[0]], environment[:, dims[1]], s=1, c='#a8adb0', alpha=.15)
        ax.add_collection(PolyCollection(vertices[faces][:, :, dims], color='#dcb34b', alpha=.3))
        ax.scatter(pool[:, dims[0]] * 1000, pool[:, dims[1]] * 1000, s=5, c='#d5a526', alpha=.4, label='对侧网络候选投影')
        for g in kin.hand.geoms:
            kind, size = int(model.geom_type[g]), model.geom_size[g]
            if kind == int(mujoco.mjtGeom.mjGEOM_MESH):
                mesh = int(model.geom_dataid[g]); start, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
                v = model.mesh_vert[start:start + count]
                a, n = int(model.mesh_faceadr[mesh]), int(model.mesh_facenum[mesh])
                f = model.mesh_face[a:a + n]
            elif kind == int(mujoco.mjtGeom.mjGEOM_BOX):
                mesh = trimesh.creation.box(extents=2 * size); v, f = mesh.vertices, mesh.faces
            elif kind == int(mujoco.mjtGeom.mjGEOM_CAPSULE):
                mesh = trimesh.creation.capsule(height=2 * size[1], radius=size[0], count=[6, 8]); v, f = mesh.vertices, mesh.faces
            else:
                continue
            v = v @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
            v = transform(v, manual_from_hand) * 1000
            body = model.body(int(model.geom_bodyid[g])).name
            color = next((c for name, c in colors.items() if body.startswith(name + '_')), '#8c969e')
            ax.add_collection(PolyCollection(v[f][:, :, dims], color=color, edgecolor='none', alpha=.12))
        for finger in trace[-1]['fingers']:
            a, b = np.array(finger['pad_point_manual_m']) * 1000, np.array(finger['projected_target_manual_m']) * 1000
            if finger['active']:
                ax.plot([a[dims[0]], b[dims[0]]], [a[dims[1]], b[dims[1]]], '--', color=colors[finger['finger']], linewidth=1.3)
        anchor = np.array(row['anchor_manual_m']) * 1000
        ax.scatter(anchor[dims[0]], anchor[dims[1]], s=70, marker='x', color='black', label='靠端拇指目标')
        ax.scatter(evidence[:, dims[0]], evidence[:, dims[1]], s=18, c='#d52d32', label='侵入证据')
        ax.set(xlim=(-75, 40), ylim=((-40, 60) if dims[1] == 1 else (-40, 70)), xlabel='握梁长轴 X / mm',
               ylabel=('两侧方向 Y / mm' if dims[1] == 1 else '朝外方向 Z / mm'), title=title)
        ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend(fontsize=8, loc='upper right')
    fig.suptitle('单点小试未找到无侵入摆放：停在几何阶段，未产生受力结果', fontsize=14)
    fig.savefig(OUT / 'result.png', dpi=160)
    print(json.dumps({k: v for k, v in report.items() if k != 'independent_scene_checks'}, ensure_ascii=False))
    print(json.dumps(independent[-1], ensure_ascii=False))


if __name__ == '__main__':
    with threadpool_limits(limits=1):
        main()
