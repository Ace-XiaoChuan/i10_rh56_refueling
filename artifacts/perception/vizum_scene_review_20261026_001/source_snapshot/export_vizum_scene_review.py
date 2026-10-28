"""从已归档的26号原始点云生成全量、周边窗口和整盖裁剪离线查看页。"""

from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import re
import shutil
import sys

import numpy as np

ROOT = Path('/home/ace/i10_rh56_refueling_ws')
OUT = Path('/tmp/vizum_scene_review_20261026_001')
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'tools'))
from fuel_cap_task.pcd_point_cloud import load_pcd
from export_pcd_viewer import PAGE


def digest(path):
    """用实际文件内容核对原始点云和导出结果。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def trace(points, rows, cap=False):
    """坐标和原始行号不变，红色仅用于标记已有整盖选区。"""
    xyz = points[rows]
    marker = {'size': 2.5 if cap else 1.5, 'opacity': 1}
    marker.update({'color': '#df4839'} if cap else {
        'color': xyz[:, 2].tolist(), 'colorscale': 'Viridis',
        'colorbar': {'title': 'SDK Z / mm'}})
    return {'type': 'scatter3d', 'mode': 'markers',
            'name': '定位用整盖观测点' if cap else '其余实测点（Z伪彩色）',
            'x': xyz[:, 0].tolist(), 'y': xyz[:, 1].tolist(), 'z': xyz[:, 2].tolist(),
            'customdata': rows.tolist(), 'marker': marker,
            'hovertemplate': '原始点行号 %{customdata}<br>X %{x:.4f} mm<br>Y %{y:.4f} mm<br>Z %{z:.4f} mm<extra>%{fullData.name}</extra>'}


def main():
    OUT.mkdir(exist_ok=False)
    source_path = ROOT / 'artifacts/hardware/vizum_i10_capture_20261026T055151Z/cloud_sdk_native.pcd'
    crop_path = ROOT / 'artifacts/perception/cap_observation_20261026_001/crop/cap_crop.pcd'
    indices_path = crop_path.parent / 'source_indices.npz'
    geometry_path = ROOT / 'artifacts/perception/cap_observation_20261026_002/geometry/result.json'
    source, crop = load_pcd(source_path), load_pcd(crop_path)
    points = source.xyz_raw
    assert len(points) == 268852 and np.isfinite(points).all()
    with np.load(indices_path) as saved:
        cap_rows = saved['source_indices']
    assert len(cap_rows) == 5742
    np.testing.assert_allclose(points[cap_rows], crop.xyz_raw, atol=1e-7, rtol=0)
    cap_mask = np.zeros(len(points), dtype=bool)
    cap_mask[cap_rows] = True
    geometry = json.loads(geometry_path.read_text())
    center = np.asarray(geometry['cap_grip_center_model_based_m']) * 1000
    half_width = np.array([300., 300., 250.])
    window = np.all(np.abs(points - center) <= half_width, axis=1)
    assert window[cap_rows].all()
    views = [('原始扫描：全部 268,852 点', np.arange(len(points))),
             ('盖体周围：600×600×500 mm 观察窗口', np.flatnonzero(window)),
             ('定位所用：整盖裁剪 5,742 点', cap_rows)]
    axes = {key: {'title': f'{axis} / mm'} for key, axis in
            [('xaxis', 'X'), ('yaxis', 'Y'), ('zaxis', 'Z')]}
    samples = []
    for label, selected in views:
        other = selected[~cap_mask[selected]]
        cap = selected[cap_mask[selected]]
        traces = ([trace(points, other)] if len(other) else []) + [trace(points, cap, True)]
        samples.append({'label': label, 'length_unit': 'mm', 'frame_id': 'camera',
            'point_count': len(selected), 'source_sha256': source.sha256, 'traces': traces,
            'layout': {'scene': {**axes, 'aspectmode': 'data'},
                       'legend': {'x': .01, 'y': .98}, 'updatemenus': []}})
    payload = json.dumps(samples, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
    for char in ('<', '>', '&', '\u2028', '\u2029'):
        payload = payload.replace(char, f'\\u{ord(char):04x}')
    library_path = ROOT / 'artifacts/perception/neisai_pcd_20261022/plotly.min.js'
    library = re.sub(r'</script', r'<\\/script', library_path.read_text(), flags=re.I)
    page = PAGE.replace('__POINT_CLOUD_DATA__', payload, 1).replace('/*__PLOTLY__*/', library, 1)
    page = page.replace('油箱盖三维点云', '10月26日原始扫描与整盖裁剪')
    note = '<div style="padding:8px 24px;background:#f2f6fa;color:#34485b;font-size:14px">红色为定位用的整盖观测点；其余按 Z 着色。原始扫描视图显示本次全部返回点，不表示现场几何已完整覆盖。局部窗口只改变显示范围，未修改源点云。</div>'
    page = page.replace('</header>', '</header>' + note, 1)
    (OUT / 'point_cloud_viewer.html').write_text(page)
    shutil.copyfile(source_path, OUT / 'scene_raw.pcd')
    shutil.copyfile(crop_path, OUT / 'cap_crop.pcd')

    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    figure, panels = plt.subplots(2, 3, figsize=(18, 11), constrained_layout=True)
    for row, (_, selected) in enumerate(views[:2]):
        display = selected[np.linspace(0, len(selected)-1, min(40000, len(selected)), dtype=int)]
        sample, cap = points[display], points[selected[cap_mask[selected]]]
        label = 'All returned scan points' if row == 0 else '600 x 600 x 500 mm window around cap'
        for panel, (a, b) in zip(panels[row], [(0, 1), (0, 2), (1, 2)]):
            panel.scatter(sample[:, a], sample[:, b], c=sample[:, 2], cmap='viridis', s=.35)
            panel.scatter(cap[:, a], cap[:, b], c='#df4839', s=.8, label='Measured cap crop')
            panel.set(xlabel=f'{"XYZ"[a]} / mm', ylabel=f'{"XYZ"[b]} / mm', aspect='equal')
            panel.set_title(f'{label} | {len(selected):,} points', fontsize=10)
            panel.grid(alpha=.2)
            panel.legend(fontsize=8, loc='best')
    figure.suptitle('2026-10-26 SDK scan | native camera XYZ, mm | red: cap observations; other colors: Z', fontsize=13)
    figure.savefig(OUT / 'preview.png', dpi=140)
    plt.close(figure)

    report = {'created_utc': datetime.now(timezone.utc).isoformat(),
        'source_path': str(source_path), 'source_sha256': source.sha256,
        'source_point_count': len(points), 'finite_point_count': len(points),
        'frame_id': 'camera_sdk_native', 'length_unit': 'mm',
        'bounds_min_mm': points.min(0).tolist(), 'bounds_max_mm': points.max(0).tolist(),
        'extent_mm': np.ptp(points, axis=0).tolist(), 'cap_crop_point_count': len(cap_rows),
        'cap_crop_source_indices_sha256': digest(indices_path),
        'local_window_point_count': int(window.sum()), 'local_window_center_mm': center.tolist(),
        'local_window_bounds_min_mm': (center-half_width).tolist(),
        'local_window_bounds_max_mm': (center+half_width).tolist(),
        'local_window_dimensions_mm': (half_width*2).tolist(),
        'all_raw_points_in_first_view': True, 'coordinate_transform_applied': False,
        'extra_points_generated': False,
        'color_meaning': 'red=measured cap crop; others=Z pseudocolor; no true RGB',
        'coverage_status': 'single-view visible surfaces; tank opening and door completeness not verified',
        'robot_connections': 0, 'new_captures': 0}
    (OUT / 'coverage.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')

    # 独立解析嵌入数据，逐点核对坐标与原始行号；全量视图每行恰好出现一次。
    embedded = json.loads(re.search(r'<script id="cloud-data" type="application/json">(.*?)</script>', page, re.S)[1])
    for sample, (_, selected) in zip(embedded, views):
        restored = []
        for values in sample['traces']:
            rows = np.asarray(values['customdata'], dtype=int)
            np.testing.assert_array_equal(np.column_stack([values[key] for key in ['x', 'y', 'z']]), points[rows])
            restored.extend(rows.tolist())
        np.testing.assert_array_equal(np.sort(restored), np.sort(selected))
    assert digest(source_path) == digest(OUT / 'scene_raw.pcd')
    assert not re.search(r'<script\b[^>]*\bsrc\s*=', page, re.I)
    assert not re.search(r'<link\b[^>]*rel=["\']stylesheet', page, re.I)
    validation = {'raw_point_count': len(points), 'view_point_counts': [view['point_count'] for view in embedded],
        'every_displayed_coordinate_matches_original_source_row': True,
        'first_view_contains_every_source_row_exactly_once': True,
        'cap_highlight_uses_original_points_only': True, 'pcd_copy_sha256_matches_source': True,
        'external_script_or_stylesheet_dependencies': False, 'browser_interaction_tested': False}
    (OUT / 'validation.json').write_text(json.dumps(validation, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'output': str(OUT), 'html_size_bytes': (OUT/'point_cloud_viewer.html').stat().st_size,
                      'validation': validation}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
