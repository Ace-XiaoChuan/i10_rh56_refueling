"""采用用户确认的握梁双侧占据，读取旧分数并绘图；不运行网络或手部求解。"""
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager
from matplotlib.colors import LogNorm
import matplotlib.pyplot as plt
import numpy as np
from plotly.offline import get_plotlyjs

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
SOURCE = ROOT / 'tools/artifacts/cap_grip_scores_20261111_001'
THRESHOLD = .01


def digest(path):
    """绑定旧输入和本轮产物，避免覆盖旧实验后仍引用旧分数。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def transform(points, pose):
    """对行向量三维坐标应用刚体变换；输入输出同为米。"""
    return points @ pose[:3, :3].T + pose[:3, 3]


def main():
    """仅覆盖双侧表面格的有效分类，原始观测和网络数组分别保留来源。"""
    paths = {
        'manual': SOURCE / 'manual_sides.npz',
        'volume': SOURCE / 'cases/network/run_candidates/network_volume.npz',
        'contacts': SOURCE / 'cases/network/run_candidates/contacts.npz',
        'masks': SOURCE / 'target_masks.npz',
    }
    hashes = {str(p.relative_to(ROOT)): digest(p) for p in paths.values()}
    source = {k: dict(np.load(p, allow_pickle=False)) for k, p in paths.items()}
    manual, volume = source['manual'], source['volume']
    raw = source['contacts']['quality_raw']
    spacing = float(volume['voxel_size_m'])
    assert spacing == .001 and raw.shape == (160, 160, 160)
    confirmed = manual['assumed_surface_grid'].copy()
    original_free = volume['observed_free_grid']
    overridden = confirmed & original_free
    effective_surface = volume['surface_grid'] | confirmed
    effective_free = volume['free_grid'] & ~confirmed
    effective_unknown = volume['unknown_grid'] & ~confirmed
    effective_binary = volume['grid'].copy()
    effective_binary[confirmed] = 1
    contact_region = volume['contact_measured_top_grid'] | confirmed
    assert int(confirmed.sum()) == 3134 and int(overridden.sum()) == 521
    assert not np.any(effective_surface & effective_free)
    assert not np.any(effective_surface & effective_unknown)
    assert np.all(effective_surface | effective_free | effective_unknown)
    np.savez_compressed(OUT / 'confirmed_geometry.npz',
        confirmed_occupied_surface_grid=confirmed,
        effective_surface_grid=effective_surface,
        effective_free_grid=effective_free,
        effective_unknown_grid=effective_unknown,
        effective_binary_grid=effective_binary,
        effective_contact_region=contact_region,
        original_observed_surface_grid=volume['surface_grid'],
        original_observed_free_grid=original_free,
        overridden_free_grid=overridden,
        previous_accepted_unknown_surface_grid=manual['accepted_unknown_surface_grid'],
        points_volume_m=manual['points_volume_m'],
        points_manual_m=manual['points_manual_m'],
        normals_volume=manual['normals_volume'], side_id=manual['side_id'],
        vertices_manual_m=manual['vertices_manual_m'], faces=manual['faces'],
        volume_from_manual=manual['volume_from_manual'],
        reference_from_volume=volume['reference_from_volume'],
        voxel_size_m=np.array(spacing),
        occupancy_source=np.array('user_visual_confirmation'),
        occupancy_confirmed_date=np.array('2026-11-12'))

    indices = np.argwhere(confirmed)
    points = indices * spacing
    local_mm = transform(points, np.linalg.inv(manual['volume_from_manual'])) * 1000
    scores = raw[tuple(indices.T)]
    side = np.where(local_mm[:, 1] < 0, -1, 1).astype(np.int8)
    was_free = original_free[tuple(indices.T)]
    previously_used = manual['accepted_unknown_surface_grid'][tuple(indices.T)]
    high = scores > THRESHOLD
    assert np.isfinite(scores).all() and not np.any(local_mm[:, 1] == 0)
    np.savez_compressed(OUT / 'side_features.npz',
        indices_xyz=indices, points_volume_m=points, points_manual_mm=local_mm,
        scores_raw=scores, side_id=side, original_free_evidence=was_free,
        previously_accepted_unknown=previously_used, above_threshold=high,
        threshold=np.array(THRESHOLD), comparison=np.array('greater_than'))
    with (OUT / 'high_scores.csv').open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.writer(stream)
        writer.writerow(['side_y', 'index_x', 'index_y', 'index_z', 'manual_x_mm',
                         'manual_y_mm', 'manual_z_mm', 'score_raw', 'original_free_evidence'])
        for j in np.flatnonzero(high):
            writer.writerow([int(side[j]), *indices[j].tolist(), *local_mm[j].tolist(),
                             float(scores[j]), bool(was_free[j])])

    groups = {}
    for sign in [-1, 1]:
        selected = side == sign
        accepted = selected & high
        groups[str(sign)] = dict(surface_cells=int(selected.sum()),
            high_score_cells=int(accepted.sum()),
            previous_high_score_cells=int((accepted & previously_used).sum()),
            restored_from_free_high_score_cells=int((accepted & was_free).sum()),
            maximum_raw_score=float(scores[selected].max()),
            thresholds={str(t): int((selected & (scores > t)).sum()) for t in [.1, .01, .001, .0001]},
            high_x_min_max_mm=[float(local_mm[accepted, 0].min()), float(local_mm[accepted, 0].max())])
    assert [groups[str(s)]['high_score_cells'] for s in [-1, 1]] == [278, 263]
    assert [groups[str(s)]['previous_high_score_cells'] for s in [-1, 1]] == [15, 8]
    report = dict(threshold=THRESHOLD, comparison='greater_than',
        scores_exactly_equal_threshold=int((scores == THRESHOLD).sum()),
        occupancy_source='user_visual_confirmation',
        confirmation='用户肉眼确认握梁双侧存在实体，未扫出点云来自观测角度；本次双侧按确认占据采用。',
        confirmed_surface_cells=int(confirmed.sum()), overridden_free_cells=int(overridden.sum()),
        overlap_measured_surface_cells=int((confirmed & volume['surface_grid']).sum()),
        grid_spacing_m=spacing, grid_position_convention='voxel_index_times_voxel_size',
        network_scores_source='saved quality_raw before this occupancy update',
        network_forwards=0, network_training_steps=0, grasp_optimization_steps=0,
        generated_grasp_candidates=0, source_sha256=hashes, groups=groups)
    (OUT / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')

    # 两侧分开投影到同一X/Z坐标范围，颜色仅编码原始得分。
    font_path = Path('/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf')
    if font_path.exists():
        font_manager.fontManager.addfont(str(font_path))
        plt.rcParams['font.family'] = ['DejaVu Sans', font_manager.FontProperties(fname=font_path).get_name()]
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.8))
    fig.subplots_adjust(left=.06, right=.9, top=.77, bottom=.18, wspace=.17)
    norm = LogNorm(vmin=THRESHOLD, vmax=1)
    for ax, sign, name in zip(axes, [-1, 1], ['Y<0侧（左图）', 'Y>0侧（右图）']):
        selected = side == sign
        chosen = np.flatnonzero(selected & high)
        chosen = chosen[np.argsort(scores[chosen], kind='stable')]
        ax.scatter(local_mm[selected, 0], local_mm[selected, 2], s=7, c='#e1e5ea', marker='s', linewidths=0)
        ax.plot([-30, 30, 30, -30, -30], [-17, -17, 0, 0, -17], color='#687582', lw=.9)
        plot = ax.scatter(local_mm[chosen, 0], local_mm[chosen, 2], s=22,
            c=scores[chosen], cmap='viridis', norm=norm, linewidths=0)
        g = groups[str(sign)]
        ax.set_title(f"{name}：{g['high_score_cells']}格\n原采用{g['previous_high_score_cells']}格；本次恢复{g['restored_from_free_high_score_cells']}个原空闲分类高分格", fontsize=12)
        ax.set_xlim(-33, 33); ax.set_ylim(-20, 3)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('握梁长轴 X / mm'); ax.set_ylabel('相对顶面的高度 Z / mm')
        ax.grid(alpha=.15)
    fig.colorbar(plot, cax=fig.add_axes([.93, .27, .015, .4]), label='原始网络接触分数（对数色标）')
    fig.suptitle('人工确认占据的握梁两侧：原始接触分数 > 0.01', fontsize=17, y=.94)
    fig.text(.5, .84, '两侧共541格；彩色点为高分格，浅灰为完整侧面支持格，线框为侧面范围', ha='center', fontsize=11)
    fig.text(.5, .07, '沿用同一次1 mm网格和已保存的网络分数；本图只展示接触位置。\n左右指本图排列，实际侧别以握梁坐标Y的正负标识。', ha='center', fontsize=10, color='#435365')
    fig.savefig(OUT / 'sides_above_001.png', dpi=180, facecolor='white')
    plt.close(fig)

    # 背景直接采用实测表面格，保持坐标变换与高分格相同。
    measured_top_i = np.argwhere(volume['contact_measured_top_grid'])
    measured_top_mm = transform(measured_top_i * spacing, np.linalg.inv(manual['volume_from_manual'])) * 1000
    payload = dict(points=local_mm.tolist(), indices=indices.tolist(), scores=scores.tolist(),
        sides=side.tolist(), was_free=was_free.tolist(), previously_used=previously_used.tolist(),
        vertices=(manual['vertices_manual_m'] * 1000).tolist(), faces=manual['faces'].tolist(),
        top=measured_top_mm.tolist(), report=report)
    template = (OUT / 'viewer_template.html').read_text()
    page = template.replace('__PLOTLY__', get_plotlyjs()).replace('__DATA__', json.dumps(payload, ensure_ascii=False, separators=(',', ':')))
    (OUT / 'index.html').write_text(page, encoding='utf-8')
    assert hashes == {str(p.relative_to(ROOT)): digest(p) for p in paths.values()}
    verification = dict(source_hashes_unchanged=True,
        full_confirmed_sides_occupied=True, effective_classification_disjoint_and_complete=True,
        high_scores_exactly_from_saved_quality_raw=True, expected_side_counts_match=True,
        network_forwards=0, grasp_optimization_steps=0, generated_grasp_candidates=0)
    (OUT / 'verification.json').write_text(json.dumps(verification, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(output=str(OUT), sides=groups, source_hashes_unchanged=True), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
