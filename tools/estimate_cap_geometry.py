#!/usr/bin/env python3
"""检查裁剪盖体的几何方向，输出相机系诊断、分组索引及 CloudCompare 可读点云。"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from fuel_cap_task.cap_geometry import estimate_cap_geometry, read_grip_model_reference
from fuel_cap_task.pcd_point_cloud import load_pcd


def digest(path):
    """记录实际输入与实现摘要，不使用文件修改时间代替采集时间。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def direction_angle(first, second, undirected=False):
    """比较单位方向的夹角；握梁长轴允许正负等价。"""
    cosine = first @ second
    return float(np.rad2deg(np.arccos(np.clip(abs(cosine) if undirected else cosine, -1, 1))))


def subsampling_check(points, baseline, count):
    """固定种子，每次保留 80% 点，测量选点密度变化的敏感性，不估计实物精度。"""
    rng = np.random.default_rng(25)
    records = []
    for _ in range(count):
        sample = points[rng.choice(len(points), int(len(points) * .8), replace=False)]
        try:
            result = estimate_cap_geometry(sample)
            records.append({
                "valid": True,
                "boundary_center_delta_m": float(np.linalg.norm(
                    result.boundary_center_m - baseline.boundary_center_m)),
                "surface_reference_delta_m": float(np.linalg.norm(
                    result.surface_reference_m - baseline.surface_reference_m)),
                "normal_angle_deg": direction_angle(result.rotation[:, 2], baseline.rotation[:, 2]),
                "bridge_axis_angle_deg": direction_angle(
                    result.rotation[:, 0], baseline.rotation[:, 0], undirected=True),
            })
        except ValueError as exc:
            records.append({"valid": False, "reason": str(exc)})
    return {"seed": 25, "retained_fraction": .8, "samples": records,
            "interpretation": "仅为随机移除点后的敏感性，不是相对实物真值的误差"}


def circle_points(result):
    """生成拟合圆的示意线；圆线上没有观测的部分不补入点云。"""
    theta = np.linspace(0, 2 * np.pi, 181)
    return result.boundary_center_m + result.radius_m * (
        np.cos(theta[:, None]) * result.rotation[:, 0]
        + np.sin(theta[:, None]) * result.rotation[:, 1])


def write_classified_pcd(output, points, result, model_center, unit):
    """给真实观测点着色并导出 ASCII PCD，坐标单位沿用输入，便于 Windows 查看。"""
    rgb = np.zeros(len(points), dtype=np.uint32)
    rgb[result.face_indices] = 0x477C9C
    rgb[result.bridge_indices] = 0xDF8736
    rgb[result.boundary_indices] = 0x9D65A8
    xyz = points * (1000. if unit == "mm" else 1.)
    def write_cloud(name, selected_xyz, selected_rgb):
        header = (f"# length_unit {unit}; frame_id camera\nVERSION .7\nFIELDS x y z rgb\n"
                  "SIZE 8 8 8 4\nTYPE F F F U\nCOUNT 1 1 1 1\n"
                  f"WIDTH {len(selected_xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
                  f"POINTS {len(selected_xyz)}\nDATA ascii\n")
        rows = "".join(f"{p[0]:.12g} {p[1]:.12g} {p[2]:.12g} {int(c)}\n"
                       for p, c in zip(selected_xyz, selected_rgb))
        (output / f"{name}.pcd").write_text(header + rows, encoding="ascii")

    for name, indices in (("classified_cap", np.arange(len(points))),
                          ("face", result.face_indices), ("bridge", result.bridge_indices),
                          ("boundary", result.boundary_indices)):
        write_cloud(name, xyz[indices], rgb[indices])
    references = np.vstack((result.boundary_center_m, result.surface_reference_m, model_center))
    write_cloud("derived_reference_points", references * (1000. if unit == "mm" else 1.),
                [0x9D65A8, 0xAA2F38, 0x328259])


def write_preview(output, points, result, model_center):
    """用真实分组与同一份估计绘制静态三视图，长度显示为毫米。"""
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/cap-geometry-matplotlib")
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    fig, panels = plt.subplots(1, 3, figsize=(16, 5.5))
    fig.subplots_adjust(left=.055, right=.99, top=.86, bottom=.23, wspace=.30)
    circle = circle_points(result)
    aligned = (points - result.boundary_center_m) @ result.rotation
    for panel, dimensions, transformed, curve, title in (
        (panels[0], (0, 1), points, circle, "Camera XY"),
        (panels[1], (0, 1), aligned, (circle - result.boundary_center_m) @ result.rotation,
         "Estimated cap plane: X along bridge"),
        (panels[2], (0, 2), aligned, (circle - result.boundary_center_m) @ result.rotation,
         "Estimated X-Z: outward Z is positive"),
    ):
        a, b = dimensions
        for indices, color, label in ((result.face_indices, "#477c9c", "Face group"),
                                      (result.bridge_indices, "#df8736", "Bridge group")):
            q = transformed[indices] * 1000
            panel.scatter(q[:, a], q[:, b], s=3, c=color, label=label)
        panel.plot(curve[:, a] * 1000, curve[:, b] * 1000, "--", c="#9d65a8", lw=1.4,
                   label="Observed-boundary circle")
        reference = result.surface_reference_m
        if panel is not panels[0]:
            reference = (reference - result.boundary_center_m) @ result.rotation
        panel.scatter([reference[a] * 1000], [reference[b] * 1000], c="#aa2f38", marker="x",
                      s=65, zorder=10, label="Visible surface reference")
        model_reference = model_center
        if panel is not panels[0]:
            model_reference = (model_reference - result.boundary_center_m) @ result.rotation
        panel.scatter([model_reference[a] * 1000], [model_reference[b] * 1000], c="#328259", marker="+",
                      s=85, zorder=10, label="Grip center from model height")
        panel.set(title=title, xlabel=f'{"XYZ"[a]} / mm', ylabel=f'{"XYZ"[b]} / mm', aspect="equal")
        panel.grid(alpha=.18)
    handles, labels = panels[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(.5, .015), ncol=3, fontsize=8)
    offset_mm = np.linalg.norm(model_center - result.surface_reference_m) * 1000
    fig.suptitle(f"Cap geometry | green grip center uses the provisional model's {offset_mm:.2f} mm offset",
                 fontsize=12)
    fig.savefig(output / "preview.png", dpi=150)
    plt.close(fig)


def main():
    """显式指定点云单位；只生成离线几何结果，不构造有效 CapEstimate 或运动请求。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", type=Path, required=True)
    parser.add_argument("--length-unit", choices=("mm", "m"), required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--subsamples", type=int, default=12)
    parser.add_argument("--geometry-prior", type=Path, default=ROOT / "config/fuel_cap_geometry_prior.json")
    parser.add_argument("--observation", type=Path, help="配对采集的 observation.json，绑定后输出基座系诊断候选")
    parser.add_argument("--calibration", type=Path, help="默认使用观测目录内采集时保存的外参副本")
    parser.add_argument("--crop-manifest", type=Path, help="辅助裁剪的 crop.json，校验源文件与裁剪摘要")
    args = parser.parse_args()
    if args.subsamples < 0:
        parser.error("重复采样次数必须非负")
    output = args.output_dir.resolve()
    if output.exists():
        parser.error("输出目录已存在，请指定新目录，保留既有实验")
    cloud = load_pcd(args.pcd)
    points = cloud.xyz_raw * (.001 if args.length_unit == "mm" else 1.)
    result = estimate_cap_geometry(points)
    prior_config = json.loads(args.geometry_prior.read_text(encoding="utf-8"))
    if prior_config.get("schema_version") != 1:
        parser.error("未知的几何先验格式")
    urdf_path = ROOT / prior_config["source_urdf"]
    prior = read_grip_model_reference(urdf_path,
                                    top_width_reduction_m=prior_config["top_width_reduction_m"])
    offset = prior["top_surface_to_reference_m"]
    model_center = result.surface_reference_m - offset * result.rotation[:, 2]
    rotations = [result.rotation, result.rotation @ np.diag([-1., -1., 1.])]
    report = {
        "schema_version": 1, "mode": "cropped_cap_geometry", "frame_id": "camera",
        "length_unit": "m", "input_path": str(args.pcd.resolve()), "input_sha256": cloud.sha256,
        "input_length_unit": args.length_unit, "capture_timestamp_s": None,
        "pcd_exports_length_unit": args.length_unit,
        "source": "manual_cloud_crop_then_geometry_fit", "geometric_checks_passed": True,
        "motion_target_valid": False, "cap_estimate": None,
        "boundary_center_m": result.boundary_center_m.tolist(), "boundary_radius_m": result.radius_m,
        "visible_bridge_surface_reference_m": result.surface_reference_m.tolist(),
        "cap_grip_center_model_based_m": model_center.tolist(),
        "visible_surface_to_grip_center_offset_m_in_cap": [0., 0., -offset],
        "model_prior": prior,
        "model_prior_sources": {"urdf_path": str(urdf_path.resolve()), "urdf_sha256": digest(urdf_path),
                                "config_sha256": digest(args.geometry_prior)},
        "outward_axis_camera": result.rotation[:, 2].tolist(),
        "bridge_axis_camera_undirected": result.rotation[:, 0].tolist(),
        "orientation_candidates": [{"rotation_columns_xyz": rotation.tolist(),
                                    "ambiguity_yaw_deg": i * 180} for i, rotation in enumerate(rotations)],
        "pose_candidates_camera_model_based": [
            {"frame_id": "camera", "position_m": model_center.tolist(),
             "orientation_xyzw": Rotation.from_matrix(rotation).as_quat().tolist(),
             "ambiguity_yaw_deg": i * 180} for i, rotation in enumerate(rotations)],
        "diagnostics": result.diagnostics,
        "subsampling": subsampling_check(points, result, args.subsamples),
        "assumptions": ["已人工裁出整盖且外侧朝向相机", "观测外轮廓近圆且覆盖充分",
                        "凸起握梁与盖面存在明确高度分层", "点云原点是观察相机原点",
                        "用占位 URDF 的顶面到中心距离换算实际观测的参考点"],
        "unresolved": ["外轮廓受人工裁剪影响，其圆心与法向仍需实物核对",
                       "握梁参考点使用占位模型高度，实物吻合程度待核对", "握梁长轴有180度歧义",
                       "缺少本次采集的机器人姿态与采集时间，尚不能生成有效基座系运动目标"],
        "implementation_sha256": {"cap_geometry.py": digest(ROOT / "src/fuel_cap_task/cap_geometry.py"),
                                   "estimate_cap_geometry.py": digest(Path(__file__))},
        "versions": {"numpy": np.__version__, "scipy": __import__("scipy").__version__},
    }
    if args.observation is not None:
        from fuel_cap_task.cap_observation import bind_observation
        report = bind_observation(report, points, args.observation,
                                  calibration_path=args.calibration, crop_manifest_path=args.crop_manifest)
    elif args.calibration is not None or args.crop_manifest is not None:
        parser.error("--calibration 和 --crop-manifest 需要同时指定 --observation")
    output.mkdir(parents=True)
    np.savez_compressed(output / "groups.npz", points_camera_m=points,
                        face_indices=result.face_indices, bridge_indices=result.bridge_indices,
                        boundary_indices=result.boundary_indices)
    write_preview(output, points, result, model_center)
    write_classified_pcd(output, points, result, model_center, args.length_unit)
    (output / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
                                       + "\n", encoding="utf-8")
    print(json.dumps({"output_dir": str(output), "diagnostics": result.diagnostics}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
