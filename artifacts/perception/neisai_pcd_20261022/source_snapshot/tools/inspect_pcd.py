#!/usr/bin/env python3
"""检查真实 PCD，保存原始数组、来源摘要和独立视图；不假定 RGB 已配准。"""

import argparse
from datetime import datetime, timezone
import html
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np
from PIL import Image

from fuel_cap_task.pcd_point_cloud import load_pcd
from fuel_cap_task.rgbd_point_cloud import digest
from reconstruct_cap_point_cloud import plotly_asset


def save_json(path, value):
    """拒绝非标准 JSON 数值，保证摘要能由其他工具读取。"""
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8")


def write_views(folder, points, colors, title, unit, max_points, frame_id="unknown"):
    """在文件原坐标中显示有效点；抽样只用于显示，颜色模式明确区分 Z 和 RGB。"""
    indices = np.linspace(0, len(points) - 1, min(max_points, len(points)), dtype=int)
    sample = points[indices]
    marker = {"size": 2, "color": sample[:, 2].tolist(), "colorscale": "Viridis",
              "colorbar": {"title": f"原坐标 Z / {unit}"}}
    trace = {"type": "scatter3d", "mode": "markers", "x": sample[:, 0].tolist(),
             "y": sample[:, 1].tolist(), "z": sample[:, 2].tolist(), "marker": marker,
             "customdata": indices.tolist(),
             "hovertemplate": ("有效点行号 %{customdata}"
                               f"<br>X %{{x:.4f}} {unit}<br>Y %{{y:.4f}} {unit}"
                               f"<br>Z %{{z:.4f}} {unit}<extra></extra>")}
    axes = {name: {"title": f"{axis} / {unit}"}
            for name, axis in (("xaxis", "X"), ("yaxis", "Y"), ("zaxis", "Z"))}
    menus = []
    if colors is not None:
        packed_colors = [f"rgb({r},{g},{b})" for r, g, b in colors[indices]]
        menus = [{"buttons": [
            {"label": "原坐标 Z 伪彩色", "method": "restyle",
             "args": [{"marker.color": [sample[:, 2].tolist()], "marker.showscale": True}]},
            {"label": "按 PCL 解出的 RGB", "method": "restyle",
             "args": [{"marker.color": [packed_colors], "marker.showscale": False}]},
        ], "direction": "down"}]
    layout = {"scene": {**axes, "aspectmode": "data"}, "margin": {"l": 0, "r": 0, "t": 40, "b": 0},
              "updatemenus": menus}
    page = f"""<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>{html.escape(title)} 点云</title><style>body{{font-family:system-ui;margin:24px}}#cloud{{height:75vh}}</style>
<h1>{html.escape(title)}</h1><p>文件原坐标；坐标系：{html.escape(frame_id)}；单位：{unit}；坐标轴物理方向待核对。
显示 {len(sample):,} / {len(points):,} 个有效点。拖动旋转、滚轮缩放；下拉菜单切换颜色。</p>
<p>Z 伪彩色仅用于观察几何；RGB 模式保留字段解码结果，不等于已确认真实彩色。
未执行坐标配准或图像映射。</p><p><a href="../index.html">返回</a> · <a href="result.json">检查报告</a> ·
<a href="point_cloud_raw.npz">完整原始数组</a></p><div id="cloud"></div>
<script src="../plotly.min.js"></script><script>
Plotly.newPlot('cloud', {json.dumps([trace])}, {json.dumps(layout)}, {{responsive:true}});
</script></html>"""
    (folder / "index.html").write_text(page, encoding="utf-8")

    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    figure, panels = plt.subplots(1, 3, figsize=(15, 5), constrained_layout=True)
    for panel, (a, b) in zip(panels, ((0, 1), (0, 2), (1, 2))):
        panel.scatter(sample[:, a], sample[:, b], c=sample[:, 2], cmap="viridis", s=1)
        panel.set(xlabel=f"{'XYZ'[a]} / {unit}", ylabel=f"{'XYZ'[b]} / {unit}")
        panel.set_aspect("equal", adjustable="box")
        panel.grid(alpha=.2)
    figure.suptitle(f"{title} | original coordinates | Z pseudocolor")
    figure.savefig(folder / "preview.png", dpi=150)
    plt.close(figure)
    return len(sample)


def inspect_one(path, output, args):
    """统计完整点云，显式排除非有限点用于显示，保留原行号用于以后追溯。"""
    cloud = load_pcd(path, byte_order=args.byte_order)
    valid = np.isfinite(cloud.xyz_raw).all(axis=1)
    points = cloud.xyz_raw[valid]
    if not len(points):
        raise ValueError(f"没有有限坐标点：{path}")
    colors = None if cloud.rgb is None else cloud.rgb[valid]
    output.mkdir()
    arrays = {"xyz_raw": cloud.xyz_raw, "finite_mask": valid, "records": cloud.records,
              "finite_source_indices": np.flatnonzero(valid)}
    if cloud.rgb is not None:
        arrays["rgb_pcl_decoded"] = cloud.rgb
    # 只有调用者明确给出单位才生成米制坐标；仍然是文件坐标系，不命名为 world。
    scale = {"m": 1., "mm": .001}.get(args.length_unit)
    if scale is not None:
        arrays["xyz_m"] = cloud.xyz_raw * scale
    np.savez_compressed(output / "point_cloud_raw.npz", **arrays)
    shown = write_views(output, points, colors, path.name, args.length_unit,
                        args.max_view_points, args.frame_id)
    binary = cloud.header["DATA"] == ["binary"]
    report = {"mode": "pcd_inspection", "schema_version": 1, "input_path": str(path),
              "input_sha256": cloud.sha256, "header": cloud.header,
              "data_encoding": cloud.header["DATA"][0],
              "byte_order": cloud.byte_order if binary else None,
              "byte_order_source": "cli_argument" if binary else "not_applicable_ascii",
              "length_unit": args.length_unit,
              "unit_source": "unconfirmed" if scale is None else "cli_argument",
              "frame_id": args.frame_id, "capture_timestamp_s": None,
              "organized_layout": int(cloud.header["HEIGHT"][0]) > 1,
              "pixel_mapping_verified": False, "camera_calibration_loaded": False,
              "point_count": len(cloud.xyz_raw), "finite_point_count": len(points),
              "nonfinite_point_count": int((~valid).sum()),
              "zero_xyz_count": int(np.all(points == 0, axis=1).sum()),
              "duplicate_xyz_count": len(points) - len(np.unique(points, axis=0)),
              "bounds_min_raw": points.min(axis=0).tolist(),
              "bounds_max_raw": points.max(axis=0).tolist(),
              "extent_raw": np.ptp(points, axis=0).tolist(), "view_point_count": shown,
              "rgb_decoding": "PCL_0xRRGGBB" if colors is not None else None,
              "rgb_channel_min": None if colors is None else colors.min(axis=0).tolist(),
              "rgb_channel_max": None if colors is None else colors.max(axis=0).tolist(),
              "constant_rgb_channels": [] if colors is None else
                  [name for i, name in enumerate("RGB") if np.ptp(colors[:, i]) == 0],
              "geometry_filtering": "nonfinite_for_display_only", "coordinate_transform_applied": False,
              "cap_pose_estimated": False,
              "outputs_sha256": {p.name: digest(p) for p in output.iterdir() if p.is_file()}}
    save_json(output / "result.json", report)
    return report


def main():
    """处理一批 PCD，参考图片单独列出，不从文件顺序推断配对或采集时间。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", type=Path, action="append", required=True)
    parser.add_argument("--reference-image", type=Path, action="append", default=[])
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--length-unit", choices=("unknown", "mm", "m"), default="unknown")
    parser.add_argument("--frame-id", default="unknown")
    parser.add_argument("--byte-order", choices=("little", "big"), default="little")
    parser.add_argument("--max-view-points", type=int, default=30000)
    parser.add_argument("--plotly-js", type=Path)
    args = parser.parse_args()
    if args.max_view_points <= 0:
        parser.error("--max-view-points 须为正整数")
    output = args.output_dir.resolve()
    javascript = plotly_asset(args.plotly_js)
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(javascript, output / "plotly.min.js")
    reports, references, sections = [], [], []
    for i, path in enumerate(args.pcd):
        folder = output / f"cloud_{i:03d}"
        report = inspect_one(path.resolve(), folder, args)
        reports.append(report)
        sections.append(f'<h2>{html.escape(path.name)}</h2><p>{report["finite_point_count"]:,} 个有限点；'
                        f'单位：{args.length_unit}；坐标系：{html.escape(args.frame_id)}；像素映射未核实。</p>'
                        f'<a href="{folder.name}/index.html">打开三维点云</a> · '
                        f'<a href="{folder.name}/result.json">检查报告</a><br>'
                        f'<img src="{folder.name}/preview.png" style="max-width:100%">')
    for i, path in enumerate(args.reference_image):
        path = path.resolve()
        target = output / f"reference_{i:03d}{path.suffix.lower()}"
        shutil.copyfile(path, target)
        with Image.open(target) as image:
            size = list(image.size)
        references.append({"input_path": str(path), "input_sha256": digest(path),
                           "image_size_wh": size, "copy": target.name, "paired_cloud": None})
        sections.append(f'<h2>参考图片 {i+1}：{html.escape(path.name)}</h2>'
                        '<p>与 PCD 的配对和像素对应尚未核实。</p>'
                        f'<img src="{target.name}" style="max-width:500px;width:100%">')
    (output / "index.html").write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8">'
        '<title>真实 PCD 数据检查</title><body style="font-family:system-ui;margin:24px">'
        '<h1>真实 PCD 数据检查</h1><p>各份点云独立展示，保留原始坐标；未执行配准或位姿估计。</p>'
        + "\n".join(sections) + '</body></html>', encoding="utf-8")
    report = {"mode": "pcd_inspection_batch", "schema_version": 1,
              "inspection_time_utc": datetime.now(timezone.utc).isoformat(),
              "clouds": reports, "reference_images": references,
              "implementation_sha256": {str(p.relative_to(ROOT)): digest(p) for p in
                  (Path(__file__).resolve(), ROOT / "src/fuel_cap_task/pcd_point_cloud.py")},
              "plotly_sha256": digest(output / "plotly.min.js")}
    save_json(output / "result.json", report)
    print(json.dumps({"output": str(output), "point_counts": [r["point_count"] for r in reports],
                      "viewer": str(output / "index.html")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
