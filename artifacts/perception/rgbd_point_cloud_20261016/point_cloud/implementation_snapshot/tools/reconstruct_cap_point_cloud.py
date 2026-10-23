#!/usr/bin/env python3
"""从已保存 RGB-D 生成米制彩色点云，以及可离线旋转的查看页面。"""

import argparse
import html
import importlib.util
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np
from fuel_cap_task.rgbd_point_cloud import digest, load_observation, write_ply


def plotly_asset(explicit):
    """复用已有环境的离线绘图库，不要求 RGB-D 环境安装另一份 Plotly。"""
    if explicit is not None:
        path = explicit.resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        return path
    spec = importlib.util.find_spec("plotly")
    if spec is not None and spec.origin:
        path = Path(spec.origin).parent / "package_data/plotly.min.js"
        if path.is_file():
            return path
    candidates = sorted((ROOT / ".venv-spahybgen/lib").glob(
        "python*/site-packages/plotly/package_data/plotly.min.js"))
    if candidates:
        return candidates[0]
    raise FileNotFoundError("未找到离线 Plotly 脚本；请用 --plotly-js 指定 plotly.min.js")


def write_view(output, cloud, max_points, javascript):
    """只抽样显示，完整几何另存；页面内嵌脚本和 RGB，可直接离线打开。"""
    # 均匀选取观测点，不用场景对象 ID 或盖体真值选择区域。
    indices = np.linspace(0, len(cloud.world_xyz_m) - 1,
                          min(max_points, len(cloud.world_xyz_m)), dtype=int)
    points, colors = cloud.world_xyz_m[indices], cloud.rgb[indices]
    camera = np.asarray(cloud.metadata["world_from_optical"])[:3, 3]
    traces = [{"type": "scatter3d", "mode": "markers", "name": "可见表面",
               "x": points[:, 0].tolist(), "y": points[:, 1].tolist(), "z": points[:, 2].tolist(),
               "marker": {"size": 2, "color": [f"rgb({r},{g},{b})" for r, g, b in colors]},
               "hovertemplate": "X %{x:.4f} m<br>Y %{y:.4f} m<br>Z %{z:.4f} m<extra></extra>"},
              {"type": "scatter3d", "mode": "markers+text", "name": "虚拟相机",
               "x": [float(camera[0])], "y": [float(camera[1])], "z": [float(camera[2])],
               "text": ["相机"], "textfont": {"color": "#ffcf70"},
               "marker": {"size": 5, "color": "#ffcf70"}}]
    axes = {name: {"title": label, "color": "#e0e7ef", "gridcolor": "#344858"}
            for name, label in (("xaxis", "X / m"), ("yaxis", "Y / m"), ("zaxis", "Z / m"))}
    layout = {"margin": {"l": 0, "r": 0, "t": 0, "b": 0}, "showlegend": False,
              "scene": {**axes, "aspectmode": "data", "bgcolor": "#152433",
                        "camera": {"eye": {"x": 1.2, "y": -1.8, "z": 1.0},
                                   "up": {"x": 0, "y": 0, "z": 1}}}}
    source = html.escape(cloud.metadata["source"])
    page = f"""<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RGB-D 点云</title><style>
body{{margin:0;font-family:system-ui,sans-serif;color:#233344;background:#f4f6f8}}
header{{padding:16px 24px}} h1{{font-size:22px;margin:0 0 8px}} p{{margin:6px 0}}
#cloud{{height:72vh;min-height:420px}} img{{max-width:480px;width:100%}} a{{color:#165dba}}
</style><header><h1>虚拟 RGB-D 重建点云</h1>
<p>拖动旋转，滚轮缩放，右键拖动平移。坐标为世界坐标，单位为米。</p>
<p>共 {len(cloud.world_xyz_m):,} 个有效点，页面显示 {len(points):,} 个；下载文件保存全部点。
仅包含相机可见表面，无点区域仍可能被遮挡。</p>
<p><a href="cloud_world.ply">世界系 PLY</a> · <a href="cloud_camera.ply">相机系 PLY</a> ·
<a href="point_cloud.npz">完整数组与像素索引</a></p></header><div id="cloud"></div>
<header><p>原始 RGB，观测来源：{source}</p><img src="rgb.png" alt="原始 RGB"></header>
<script>{javascript.read_text()}</script><script>
Plotly.newPlot('cloud', {json.dumps(traces)}, {json.dumps(layout)}, {{responsive:true}});
</script></html>"""
    (output / "index.html").write_text(page)
    return indices


def write_preview(path, cloud, indices):
    """生成方便在对话中查看的静态侧视图，交互查看使用 HTML。"""
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    sample = indices[::max(1, len(indices) // 15000)]
    points = cloud.world_xyz_m[sample]
    figure = plt.figure(figsize=(9, 7), facecolor="#152433")
    axis = figure.add_subplot(111, projection="3d", facecolor="#152433")
    axis.scatter(*points.T, c=cloud.rgb[sample] / 255., s=1, depthshade=False)
    axis.view_init(elev=22, azim=-65)
    extent = np.ptp(cloud.world_xyz_m, axis=0)
    axis.set_box_aspect(np.maximum(extent, .001))
    for name in ("x", "y", "z"):
        getattr(axis, f"set_{name}label")(f"{name.upper()} / m", color="white")
        getattr(axis, f"{name}axis").set_pane_color((.08, .14, .19, 1))
        axis.tick_params(axis=name, colors="white", labelsize=8)
    axis.set_title("Visible RGB-D point cloud | world frame | metres", color="white")
    figure.savefig(path, dpi=140, bbox_inches="tight", facecolor=figure.get_facecolor())
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--observation-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-view-points", type=int, default=30000,
                        help="只限制网页显示点数，不减少导出的点")
    parser.add_argument("--plotly-js", type=Path, help="可选的本地 plotly.min.js 路径")
    args = parser.parse_args()
    if args.max_view_points <= 0:
        parser.error("--max-view-points 须为正整数")
    source, output = args.observation_dir.resolve(), args.output_dir.resolve()
    if output.exists():
        raise FileExistsError("输出目录已存在，请使用新目录")
    cloud = load_observation(source)
    javascript = plotly_asset(args.plotly_js)
    output.mkdir(parents=True, exist_ok=False)

    # 数组保存 float64 和原始像素对应；PLY 用于外部三维软件查看。
    np.savez_compressed(output / "point_cloud.npz", camera_xyz_m=cloud.camera_xyz_m,
                        world_xyz_m=cloud.world_xyz_m, rgb=cloud.rgb, pixel_uv=cloud.pixel_uv)
    write_ply(output / "cloud_camera.ply", cloud.camera_xyz_m, cloud.rgb)
    write_ply(output / "cloud_world.ply", cloud.world_xyz_m, cloud.rgb)
    shutil.copyfile(source / "rgb.png", output / "rgb.png")
    indices = write_view(output, cloud, args.max_view_points, javascript)
    write_preview(output / "preview.png", cloud, indices)
    report = {"schema_version": 1, "mode": "observed_rgbd_point_cloud",
              "observation_dir": str(source), "source": cloud.metadata["source"],
              "timestamp_s": cloud.metadata["timestamp_s"],
              "timestamp_clock": cloud.metadata["timestamp_clock"],
              "camera_frame_id": cloud.metadata["frame_id"], "world_frame_id": "world",
              "length_unit": "m", "point_count": len(cloud.camera_xyz_m),
              "view_point_count": len(indices), "image_size_wh": cloud.metadata["image_size_wh"],
              "world_from_optical": cloud.metadata["world_from_optical"],
              "world_bounds_min_m": cloud.world_xyz_m.min(axis=0).tolist(),
              "world_bounds_max_m": cloud.world_xyz_m.max(axis=0).tolist(),
              "input_sha256": cloud.input_sha256,
              "implementation_sha256": {str(path.relative_to(ROOT)): digest(path) for path in
                  (Path(__file__).resolve(), ROOT / "src/fuel_cap_task/rgbd_point_cloud.py")},
              "observed_surface_only": True, "ground_truth_used": False,
              "outputs_sha256": {path.name: digest(path) for path in output.iterdir() if path.is_file()}}
    (output / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(output), "point_count": len(cloud.camera_xyz_m),
                      "viewer": str(output / "index.html")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
