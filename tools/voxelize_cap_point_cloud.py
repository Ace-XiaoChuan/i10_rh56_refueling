#!/usr/bin/env python3
"""将已保存的世界系点云转为表面体素，并提供全景与局部立方格比较。"""

import argparse
import base64
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np
from fuel_cap_task.rgbd_point_cloud import digest, write_ply
from fuel_cap_task.voxelization import pixel_to_voxel_lookup, voxelize_points
from reconstruct_cap_point_cloud import plotly_asset


def load_point_cloud(folder):
    """读取已重建的世界系点云，核对来源、单位、点数和文件摘要。

    参数：
        folder：pathlib.Path，点云输出目录，包含 result.json、point_cloud.npz
            和 rgb.png，通常由 reconstruct_cap_point_cloud.py 生成。

    返回：
        (points, rgb, pixels, report) 元组：
        points：形状 (N, 3) 的世界系坐标数组，单位米；
        rgb：形状 (N, 3) 的 uint8 RGB 数组，通道顺序为红、绿、蓝；
        pixels：形状 (N, 2) 的整数数组，每行为对应原像素 (u, v)，即列、行；
        report：result.json 解析后的字典，含来源、时间和图像尺寸等信息。
        三个逐点数组的第 i 行须对应同一个观测点。

    异常：
        ValueError：模式、坐标系、单位、文件摘要或点数与约定不符；
        缺失文件、字段和解析错误向上传递。具体点云几何由 voxelize_points()
        继续检查。此处不读取 MuJoCo 场景几何或物体真值。
    """
    report = json.loads((folder / "result.json").read_text())
    path = folder / "point_cloud.npz"
    if (report.get("mode") != "observed_rgbd_point_cloud"
            or report.get("length_unit") != "m" or report.get("world_frame_id") != "world"):
        raise ValueError("输入须为已记录来源的世界系米制观测点云")
    for name in ("point_cloud.npz", "rgb.png"):
        if digest(folder/name) != report.get("outputs_sha256", {}).get(name):
            raise ValueError(f"观测与其结果报告的摘要不匹配：{name}")
    with np.load(path, allow_pickle=False) as arrays:
        points, rgb, pixels = arrays["world_xyz_m"], arrays["rgb"], arrays["pixel_uv"]
    if len(points) != report.get("point_count"):
        raise ValueError("点数与来源记录不一致")
    return points, rgb, pixels, report


def write_viewer(output, points, rgb, pixels, voxels, lookup, javascript, radius_m, max_cubes):
    """生成点云与体素交互页，支持从原图像素精确定位所属格子。

    参数：
        output：pathlib.Path，已创建的输出目录，写入 index.html；同目录须有
            调用方提供的 rgb.png，供页面显示和点击选取。
        points：形状 (N, 3) 的世界系点云数组，单位米，用于原始点云显示。
        rgb：形状 (N, 3) 的 uint8 RGB 数组，与 points 逐行对应。
        pixels：形状 (N, 2) 的整数数组，每行为对应点的原像素 (u, v)。
        voxels：SurfaceVoxels，必须由这同一批 points、rgb 生成；页面使用
            完整体素列表做局部选取，全景最多抽样显示 30000 个中心。
        lookup：形状 (H, W)、小端 int32 的像素到体素列表行号查表，
            lookup[v, u] 为对应体素行号，无有效深度处为 -1。
        javascript：pathlib.Path，本地 plotly.min.js 文件，内容内嵌到页面。
        radius_m：有限正数，初始局部窗口沿各世界坐标轴的半宽，单位米；
            这是立方观察范围的半宽，不是球形半径或格子数，页面中可调整。
        max_cubes：正整数，局部最多绘制的立方格数；超过时显示最近的格子
            并提示，不从完整存储数据中删除任何格子。

    返回：
        (initial, displayed) 元组：initial 为初始选中体素的列表行号，取自
        最接近原图中心的有效像素；displayed 为形状 (S,) 的整数数组，记录
        全景显示的体素列表行号，S=min(30000, M)，M 为完整体素数。

    调用方负责保证数组对应、参数有效和输出目录为新目录。本函数只生成页面，
    不重新估计深度、不识别门板类别，也不按物体真值选择初始位置。
    """
    displayed = np.linspace(0, len(voxels.indices_xyz)-1, min(30000, len(voxels.indices_xyz)), dtype=int)
    raw = np.linspace(0, len(points)-1, min(30000, len(points)), dtype=int)
    height, width = lookup.shape
    # 初始选择原图中心附近的有效像素，不用物体真值猜测初始位置。
    center_pixel = np.array([(width-1)/2, (height-1)/2])
    initial = int(voxels.point_to_voxel[np.argmin(np.sum((pixels-center_pixel)**2, axis=1))])
    data = {"point_count": len(points), "size_m": voxels.size_m,
            "centers": np.round(voxels.centers_world_m, 9).tolist(),
            "indices": voxels.indices_xyz.tolist(), "counts": voxels.point_counts.tolist(),
            "colors": voxels.mean_rgb.tolist(), "display_indices": displayed.tolist(),
            "raw_points": np.round(points[raw], 9).tolist(), "raw_colors": rgb[raw].tolist(),
            "raw_voxel_ids": voxels.point_to_voxel[raw].tolist(), "initial_index": initial,
            "radius_m": radius_m, "max_local_cubes": max_cubes,
            "image_width": width, "image_height": height,
            "pixel_voxel_base64": base64.b64encode(lookup.tobytes()).decode("ascii")}
    template = Path(__file__).with_name("voxel_viewer.html").read_text()
    page = template.replace("__VOXEL_DATA__", json.dumps(data))
    page = page.replace("__PLOTLY_JS__", javascript.read_text())
    (output / "index.html").write_text(page)
    return initial, displayed


def write_preview(path, voxels, initial, displayed, radius_m, max_cubes):
    """生成静态对照图：左侧为全景中心点，右侧为初始局部的真实大小格子。

    参数：
        path：str 或 pathlib.Path，待写入图片的路径，通常为 preview.png；
            父目录须存在，调用方须避免覆盖已有结果。
        voxels：非空的 SurfaceVoxels，使用其世界系中心（米）、边长及 RGB。
        initial：整数，初始观察中心在 voxels 列表中的行号，范围为 [0, M)。
        displayed：形状 (S,) 的整数数组，全景显示体素在完整列表中的行号；
            通常来自 write_viewer()，静态图还可能进一步抽样。
        radius_m：有限正数，局部立方观察范围沿各世界轴的半宽，单位米。
        max_cubes：正整数，局部绘制格子数上限；超出时按离观察中心的距离
            保留最近的格子，仅影响图片，不影响 voxels 中的完整数据。

    返回：
        None。使用无窗口绘图后端保存图片，并关闭图对象。
    """
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    focus = voxels.centers_world_m[initial]
    delta = voxels.centers_world_m - focus
    local = np.flatnonzero(np.max(np.abs(delta), axis=1) <= radius_m + 1e-12)
    local = local[np.argsort(np.sum(delta[local]**2, axis=1))[:max_cubes]]
    corners = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]])
    faces = np.array([[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]])
    vertices = voxels.centers_world_m[local, None, :] + (corners-.5)*voxels.size_m
    polygons = vertices[:, faces, :].reshape(-1, 4, 3)
    colors = np.repeat(voxels.mean_rgb[local]/255., 6, axis=0)
    figure = plt.figure(figsize=(13, 6), facecolor="#152433")
    axes = [figure.add_subplot(1, 2, i, projection="3d", facecolor="#152433") for i in (1, 2)]
    sample = displayed[::max(1, len(displayed)//15000)]
    axes[0].scatter(*voxels.centers_world_m[sample].T, c=voxels.mean_rgb[sample]/255.,
                    s=1, depthshade=False)
    axes[0].set_box_aspect(np.maximum(np.ptp(voxels.centers_world_m, axis=0), .001))
    axes[1].add_collection3d(Poly3DCollection(polygons, facecolors=colors,
                                            edgecolors="#40576c", linewidths=.35))
    lower, upper = vertices.min(axis=(0, 1)), vertices.max(axis=(0, 1))
    for k, name in enumerate(("x", "y", "z")):
        getattr(axes[1], f"set_{name}lim")(lower[k], upper[k])
    axes[1].set_box_aspect(np.maximum(upper-lower, .001))
    for axis, title in zip(axes, ("Observed voxel centers | world / m", "Local cubes | side length 5 mm")):
        axis.view_init(elev=22, azim=-65)
        axis.set_title(title.replace("5 mm", f"{voxels.size_m*1000:g} mm"), color="white", fontsize=11)
        for name in ("x", "y", "z"):
            getattr(axis, f"set_{name}label")(f"{name.upper()} / m", color="white")
            getattr(axis, f"{name}axis").set_pane_color((.08,.14,.19,1))
            axis.tick_params(axis=name, colors="white", labelsize=7)
    figure.savefig(path, dpi=140, bbox_inches="tight", facecolor=figure.get_facecolor())
    plt.close(figure)


def main():
    """读取命令行参数，执行表面体素化并保存完整数据和查看文件。

    形参：
        无 Python 形参；参数由 argparse 从命令行读取：
        --point-cloud-dir：必填，已保存的世界系米制点云目录。
        --output-dir：必填，新的输出目录；若已存在则拒绝写入。
        --voxel-size-m：必填，立方格边长，单位米，须为有限正数。
        --origin-world-m：可选，网格原点的世界系 X、Y、Z，单位米；
            省略时各轴取 -voxel_size_m/2，将世界原点置于格子中心。
        --local-radius-m：局部显示窗口沿各轴的半宽，单位米，默认 0.040。
        --max-local-cubes：局部显示格子数上限，默认 8000。
        --plotly-js：可选的本地 plotly.min.js 路径，省略时自动查找。

    返回：
        None。保存体素 NPZ、中心 PLY、RGB、HTML、静态预览和 result.json；
        在标准输出打印输出目录、输入点数、体素数及边长。

    流程：
        核对点云来源 → 固定原点分格 → 建立像素查表 → 保存数据与查看文件
        → 记录点数守恒、输入输出及实现摘要。未命中格子保持未知，不作实心填充。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--point-cloud-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--voxel-size-m", type=float, required=True)
    parser.add_argument("--origin-world-m", type=float, nargs=3,
                        help="世界系网格原点；省略时各轴为 -voxel_size/2")
    parser.add_argument("--local-radius-m", type=float, default=.040)
    parser.add_argument("--max-local-cubes", type=int, default=8000)
    parser.add_argument("--plotly-js", type=Path)
    args = parser.parse_args()
    if not np.isfinite(args.local_radius_m) or args.local_radius_m <= 0 or args.max_local_cubes <= 0:
        parser.error("局部显示半宽须为有限正数，显示格子数须为正整数")
    source, output = args.point_cloud_dir.resolve(), args.output_dir.resolve()
    if output.exists():
        raise FileExistsError("输出目录已存在，请使用新目录")
    points, rgb, pixels, source_report = load_point_cloud(source)
    # 不显式指定原点时，将每轴网格原点移至负半格；只移动网格划分，不移动点。
    origin = args.origin_world_m if args.origin_world_m is not None else [-args.voxel_size_m/2]*3
    voxels = voxelize_points(points, rgb, size_m=args.voxel_size_m,
                              origin_world_m=origin)
    lookup = pixel_to_voxel_lookup(pixels, voxels.point_to_voxel, source_report["image_size_wh"])
    javascript = plotly_asset(args.plotly_js)
    output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(output / "voxels.npz", indices_xyz=voxels.indices_xyz,
        centers_world_m=voxels.centers_world_m, point_counts=voxels.point_counts,
        mean_rgb=voxels.mean_rgb, point_to_voxel=voxels.point_to_voxel,
        origin_world_m=voxels.origin_world_m, voxel_size_m=voxels.size_m)
    write_ply(output / "voxel_centers.ply", voxels.centers_world_m, voxels.mean_rgb)
    shutil.copyfile(source/"rgb.png", output/"rgb.png")
    initial, displayed = write_viewer(output, points, rgb, pixels, voxels, lookup, javascript,
                                     args.local_radius_m, args.max_local_cubes)
    write_preview(output / "preview.png", voxels, initial, displayed,
                  args.local_radius_m, args.max_local_cubes)
    displacements = np.linalg.norm(points-voxels.centers_world_m[voxels.point_to_voxel], axis=1)
    report = {"schema_version": 1, "mode": "observed_surface_voxels",
        "point_cloud_dir": str(source), "source": source_report["source"],
        "timestamp_s": source_report["timestamp_s"], "timestamp_clock": source_report["timestamp_clock"],
        "frame_id": "world", "length_unit": "m", "voxel_size_m": voxels.size_m,
        "origin_world_m": voxels.origin_world_m.tolist(), "index_axis_order": ["x","y","z"],
        "input_point_count": len(points), "voxel_count": len(voxels.indices_xyz),
        "points_per_voxel_mean": float(voxels.point_counts.mean()),
        "points_per_voxel_max": int(voxels.point_counts.max()),
        "point_count_conserved": int(voxels.point_counts.sum()) == len(points),
        "global_view_point_count": len(displayed), "local_radius_m": args.local_radius_m,
        "max_local_display_cubes": args.max_local_cubes, "photo_pixel_lookup": True,
        "max_point_to_center_distance_m": float(displacements.max()),
        "cell_half_diagonal_m": float(np.sqrt(3)*voxels.size_m/2),
        "observed_surface_only": True, "unlisted_cell_state": "unknown",
        "ground_truth_used": False, "network_input_prepared": False,
        "input_sha256": {name:digest(source/name) for name in ("result.json","point_cloud.npz","rgb.png")},
        "implementation_sha256": {str(path.relative_to(ROOT)):digest(path) for path in (
            Path(__file__).resolve(), Path(__file__).with_name("voxel_viewer.html"),
            ROOT/"src/fuel_cap_task/voxelization.py", ROOT/"src/fuel_cap_task/rgbd_point_cloud.py",
            ROOT/"tools/reconstruct_cap_point_cloud.py")},
        "plotly_javascript_sha256": digest(javascript),
        "outputs_sha256": {path.name:digest(path) for path in output.iterdir() if path.is_file()}}
    (output/"result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({"output":str(output),"input_point_count":len(points),
                      "voxel_count":len(voxels.indices_xyz),"voxel_size_m":voxels.size_m},ensure_ascii=False))


if __name__ == "__main__":
    main()
