#!/usr/bin/env python3
"""为油箱盖离线抓姿生成可独立打开的接触图、候选网页和静态对照图。"""

import argparse
import html
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "dependencies/SpaHybGen/src"))
os.environ.setdefault("MPLCONFIGDIR", "/tmp/cap-spahybgen-matplotlib")

import numpy as np
import torch
import plotly.graph_objects as go
from fuel_cap_task.rh56_spahybgen import RH56HandModel
from fuel_cap_task.rgbd_point_cloud import digest


REASONS = {"hand_environment_penetration": "与周边环境干涉", "hand_target_penetration": "穿入盖体",
           "hand_self_penetration": "手指之间干涉", "too_few_fingers_near_target": "靠近目标的手指数不足",
           "invalid_joint_or_pose": "关节或位姿无效", "insufficient_network_evidence": "网络预测证据不足"}


def scatter(points, **kwargs):
    """仅在显示时将局部米坐标转毫米，原始候选保持米和弧度。"""
    points = np.asarray(points) * 1000
    return go.Scatter3d(x=points[:, 0], y=points[:, 1], z=points[:, 2], mode="markers", **kwargs)


def page(path, figure, description):
    """内嵌 Plotly 及全部三维数据，主体交互无需服务器或外部脚本。"""
    fragment = figure.to_html(full_html=False, include_plotlyjs=True,
                              config={"responsive": True, "displaylogo": False})
    path.write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8">'
                    '<meta name="viewport" content="width=device-width,initial-scale=1">'
                    '<title>油箱盖自动抓姿</title><style>body{font-family:system-ui;margin:20px;color:#243447}'
                    'p{line-height:1.6}.plotly-graph-div{min-height:75vh}</style>'
                    '<h1>油箱盖自动抓取姿态生成</h1><p>' + description + '</p>' + fragment + '</html>', encoding="utf-8")


def render(args):
    """显示所有候选中的前五个，几何通过者优先；每个结果明确注明筛选状态。"""
    run, output = args.result_dir.resolve(), args.output_dir.resolve()
    geometry_folder = args.geometry_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = json.loads((run / "result.json").read_text())
    candidates = json.loads((run / "candidates.json").read_text())
    checks = json.loads((geometry_folder / "geometry.json").read_text())
    if checks["generation_result_sha256"] != digest(run / "result.json"):
        raise ValueError("几何评价与当前生成结果不匹配")
    for name in ("scene_input.npz", "inference.npz", "optimization.npz", "candidates.json"):
        if digest(run / name) != report["outputs_sha256"][name]:
            raise ValueError("生成数据已变化：" + name)
    with np.load(run / "scene_input.npz") as data:
        points, mask = data["points_volume_m"], data["target_point_mask"]
        voxel_size = float(data["voxel_size_m"])
    with np.load(run / "inference.npz") as data:
        quality = data["quality_raw"]
        target_quality = data["quality"]
    with np.load(run / "optimization.npz") as data:
        q = torch.as_tensor(data["q_final"], dtype=torch.float32)
    target = points[mask]
    feature_indices = np.argwhere(target_quality >= report["settings"]["minimum_contact_score"])
    if not len(feature_indices):
        raise ValueError("没有达到展示门槛的目标接触预测")
    feature_indices = feature_indices[np.linspace(0, len(feature_indices) - 1,
                                                 min(4000, len(feature_indices)), dtype=int)]
    feature_points = feature_indices * voxel_size
    scores = target_quality[tuple(feature_indices.T)]
    context = points[np.linspace(0, len(points) - 1, min(12000, len(points)), dtype=int)]
    center = np.median(target, axis=0) * 1000
    scene_layout = {name: {"title": axis + " / mm", "range": [float(value - 165), float(value + 165)]}
                    for name, axis, value in zip(("xaxis", "yaxis", "zaxis"), "XYZ", center)}
    scene_layout.update({"aspectmode": "data", "camera": {"eye": {"x": 1.2, "y": -1.4, "z": 1.5}}})
    figure = go.Figure()
    figure.add_trace(scatter(context, name="周边观测", marker={"size": 1.5, "color": "#9aa6b2", "opacity": .3}))
    figure.add_trace(scatter(target[::3], name="盖体可见表面", marker={"size": 1.5, "color": "#707982", "opacity": .5}))
    figure.add_trace(scatter(feature_points, name="盖体邻域接触预测", marker={"size": 3, "color": scores,
                             "colorscale": "Turbo", "cmin": 0, "cmax": 1,
                             "colorbar": {"title": "预测接触得分"}},
                             hovertemplate="X %{x:.2f} mm<br>Y %{y:.2f} mm<br>Z %{z:.2f} mm<extra></extra>"))
    figure.update_layout(scene=scene_layout, margin={"l": 0, "r": 0, "b": 0, "t": 50},
                          title="整盖观测范围内的接触预测")
    base = len(figure.data)
    page(output / "contact_map.html", figure,
         "整盖范围由人工在 RGB 图中标注。灰色为可见表面，彩色为整盖邻域内的网络预测接触候选；它们尚未经过实际接触确认。")
    rows = {row["id"]: row for row in checks["candidates"]}
    rank = candidates["rank"]
    chosen = ([i for i in rank if rows[i]["geometrically_clear_and_near_target"]]
              + [i for i in rank if not rows[i]["geometrically_clear_and_near_target"]])[:5]
    hand = RH56HandModel((run / candidates["model_dir"]).resolve(), batch_size=len(q))
    hand.update_kinematics(q)
    groups, meshes_by_id, labels = [], {}, []
    for selection, index in enumerate(chosen):
        begin = len(figure.data)
        meshes = hand.get_meshes_from_q(i=index)
        meshes_by_id[index] = meshes
        valid = rows[index]["geometrically_clear_and_near_target"]
        reason = "几何检查通过" if valid else "；".join(REASONS.get(item, item) for item in rows[index]["rejection_reasons"])
        label = f"候选 {index}：{reason}"
        labels.append(label)
        for mesh in meshes:
            vertices, faces = mesh.vertices * 1000, mesh.faces
            figure.add_trace(go.Mesh3d(x=vertices[:, 0], y=vertices[:, 1], z=vertices[:, 2],
                                       i=faces[:, 0], j=faces[:, 1], k=faces[:, 2],
                                       color="#547a9e", opacity=.88, visible=selection == 0,
                                       name=f"RH56 候选 {index}", showlegend=False))
        groups.append((begin, len(figure.data)))
    buttons = []
    for label, (begin, end) in zip(labels, groups):
        visible = [i < base or begin <= i < end for i in range(len(figure.data))]
        buttons.append({"label": label, "method": "update", "args": [{"visible": visible}, {"title": label}]})
    figure.update_layout(title=labels[0], updatemenus=[{"buttons": buttons, "x": 0, "y": 1.12}])
    count = checks["geometrically_clear_count"]
    summary = f'已生成 {len(q)} 个候选，独立手几何检查通过 {count} 个。'
    if count == 0:
        summary += "当前候选用于观察和诊断，尚无通过几何筛选的抓姿。"
    else:
        summary += "抓稳、承载及旋拧能力仍待验证。"
    animation = html.escape(os.path.relpath(geometry_folder / "optimization.gif", output), quote=True)
    page(output / "index.html", figure, summary +
         ' 可在下拉框切换候选。<a href="contact_map.html">只看接触预测</a> · '
         f'<a href="{animation}">查看优化过程动画</a>。动画展示数值求解过程。')

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    plt.rcParams["font.family"] = ["Droid Sans Fallback", "DejaVu Sans"]
    first = chosen[:3]
    overview = plt.figure(figsize=(15, 5))
    for column, index in enumerate(first, 1):
        axis = overview.add_subplot(1, len(first), column, projection="3d")
        sample = context[::4] * 1000
        axis.scatter(*sample.T, c="#adb5bd", s=.5, alpha=.15)
        axis.scatter(*(target[::3] * 1000).T, c="#707982", s=1, alpha=.3)
        axis.scatter(*(feature_points * 1000).T, c=scores, cmap="turbo", vmin=0, vmax=1, s=2)
        for mesh in meshes_by_id[index]:
            axis.add_collection3d(Poly3DCollection(mesh.vertices[mesh.faces] * 1000,
                                                   facecolor="#547a9e", alpha=.75))
        axis.set(xlim=(center[0] - 150, center[0] + 150), ylim=(center[1] - 150, center[1] + 150),
                 zlim=(center[2] - 150, center[2] + 150), xlabel="X / mm", ylabel="Y / mm", zlabel="Z / mm")
        axis.set_box_aspect((1, 1, 1))
        axis.view_init(elev=25, azim=-60)
        axis.set_title(f"候选 {index} · " + ("几何通过" if rows[index]["geometrically_clear_and_near_target"] else "几何未通过"))
    overview.suptitle("RH56 油箱盖自动抓姿候选 · 仿真 RGB-D 输入")
    overview.tight_layout()
    overview.savefig(output / "candidate_comparison.png", dpi=140)
    plt.close(overview)
    contact = plt.figure(figsize=(8, 6))
    axis = contact.add_subplot(111, projection="3d")
    axis.scatter(*(context[::3] * 1000).T, c="#adb5bd", s=.8, alpha=.12)
    axis.scatter(*(target[::3] * 1000).T, c="#707982", s=1, alpha=.3)
    colored = axis.scatter(*(feature_points * 1000).T, c=scores, cmap="turbo", vmin=0, vmax=1, s=3)
    axis.set(xlim=(center[0] - 110, center[0] + 110), ylim=(center[1] - 110, center[1] + 110),
             zlim=(center[2] - 110, center[2] + 110), xlabel="X / mm", ylabel="Y / mm", zlabel="Z / mm")
    axis.set_box_aspect((1, 1, 1))
    axis.view_init(elev=40, azim=-60)
    contact.colorbar(colored, ax=axis, shrink=.6, label="预测接触得分")
    axis.set_title("油箱盖接触分布预测")
    contact.savefig(output / "contact_map.png", dpi=150, bbox_inches="tight")
    plt.close(contact)
    manifest = {"selected_candidate_ids": chosen, "candidate_count": len(q), "geometry_pass_count": count,
                "scene_coordinates_displayed_in": "mm", "generation_result_sha256": digest(run / "result.json"),
                "geometry_sha256": digest(geometry_folder / "geometry.json"),
                "renderer_sha256": digest(Path(__file__)),
                "outputs_sha256": {p.name: digest(p) for p in output.iterdir() if p.is_file()}}
    (output / "views.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"viewer": str(output / "index.html"), "shown_candidates": chosen,
                      "geometry_pass_count": count}, ensure_ascii=False))


def main():
    """分别指定候选、几何评价及新展示目录。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result-dir", type=Path, required=True)
    parser.add_argument("--geometry-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    render(parser.parse_args())


if __name__ == "__main__":
    main()
