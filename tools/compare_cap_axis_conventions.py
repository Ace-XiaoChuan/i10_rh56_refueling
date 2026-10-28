#!/usr/bin/env python3
"""保留旧 PCD 的 Y 镜像约定不确定性，比较两种基座系诊断假设。"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from fuel_cap_task.cap_geometry import estimate_cap_geometry
from fuel_cap_task.cap_observation import checked_transform
from fuel_cap_task.pcd_point_cloud import load_pcd


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    result_path = args.geometry_dir / "result.json"
    original = json.loads(result_path.read_text())
    if not original.get("robot_state_paired"):
        parser.error("需要已绑定停稳观测的几何结果")
    binding = original["observation_binding"]
    if digest(binding["observation_path"]) != binding["observation_sha256"]:
        raise ValueError("原始观测记录已经变化")
    cloud = load_pcd(original["input_path"])
    if cloud.sha256 != original["input_sha256"]:
        raise ValueError("裁剪输入已经变化")
    points_m = cloud.xyz_raw * {"mm": .001, "m": 1.}[original["input_length_unit"]]
    base_from_camera = checked_transform(binding["base_from_camera"], "base_from_camera")
    offset = original["model_prior"]["top_surface_to_reference_m"]
    hypotheses = []
    refit_checks = {}
    for name, reflection in (("native_xyz_matches_calibration", np.eye(3)),
                             ("reflect_y_before_calibration", np.diag([1., -1., 1.]))):
        # 镜像是点数据的坐标约定处理；重新拟合右手盖体系，不能把镜像矩阵当刚体外参。
        fit = estimate_cap_geometry(points_m @ reflection.T)
        reference = fit.surface_reference_m - offset * fit.rotation[:, 2]
        camera_from_cap = np.eye(4)
        camera_from_cap[:3,:3] = fit.rotation
        camera_from_cap[:3,3] = reference
        poses = []
        for yaw in (0, 180):
            camera_pose = camera_from_cap.copy()
            if yaw:
                camera_pose[:3,:3] = camera_pose[:3,:3] @ np.diag([-1., -1., 1.])
            base_pose = base_from_camera @ checked_transform(camera_pose, "camera_from_cap")
            poses.append({"position_m":base_pose[:3,3].tolist(),
                          "orientation_xyzw":Rotation.from_matrix(base_pose[:3,:3]).as_quat().tolist(),
                          "base_from_cap_grip_center":base_pose.tolist(),"ambiguity_yaw_deg":yaw})
        hypotheses.append({"name":name,"input_coordinate_mapping":reflection.tolist(),
            "coordinate_mapping_kind":"identity" if name.startswith("native") else "Y reflection of data before fit",
            "pose_candidates_base_model_based":poses,
            "outward_axis_base":(base_from_camera[:3,:3] @ fit.rotation[:,2]).tolist(),
            "motion_target_valid":False})
        # 与直接反射原始估计的闭式关系交叉检查，而不是依靠重新拟合恰好成功。
        reference_expected = reflection @ np.asarray(original["cap_grip_center_model_based_m"])
        normal_expected = reflection @ np.asarray(original["outward_axis_camera"])
        error = float(np.linalg.norm(reference-reference_expected))
        normal_error = float(np.rad2deg(np.arccos(np.clip(normal_expected @ fit.rotation[:,2],-1,1))))
        if error > 1e-6 or normal_error > .001:
            raise ValueError("镜像前后拟合未满足几何等价关系，不能生成对照")
        refit_checks[name] = {"reference_closed_form_difference_m":error,
                              "normal_closed_form_angle_deg":normal_error,
                              "radius_difference_m":abs(fit.radius_m-original["boundary_radius_m"])}
    first,second = hypotheses
    p0=np.asarray(first["pose_candidates_base_model_based"][0]["position_m"])
    p1=np.asarray(second["pose_candidates_base_model_based"][0]["position_m"])
    n0,n1=np.asarray(first["outward_axis_base"]),np.asarray(second["outward_axis_base"])
    report = {"schema_version":1,"selected_hypothesis":None,"frame_id":"base","length_unit":"m",
        "old_export_y_mirror_setting":"user_uncertain",
        "hypotheses":hypotheses,"position_distance_between_hypotheses_m":float(np.linalg.norm(p0-p1)),
        "normal_angle_between_hypotheses_deg":float(np.rad2deg(np.arccos(np.clip(n0@n1,-1,1)))),
        "reflection_refit_checks":refit_checks,
        "interpretation":"Y镜像是输入坐标约定不确定性；每种假设内部的180度是握梁无向轴歧义。单姿态不选择坐标假设，留待导出设置或多视角一致性核对。",
        "motion_target_valid":False,
        "source_sha256":{"geometry_result":digest(result_path),"crop":cloud.sha256,
                         "comparison_tool":digest(__file__),"geometry_module":digest(ROOT/'src/fuel_cap_task/cap_geometry.py')}}
    output=args.output_dir.resolve()
    output.mkdir(parents=True,exist_ok=False)
    (output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({"output_dir":str(output),"position_difference_mm":report["position_distance_between_hypotheses_m"]*1000,
                      "axis_difference_deg":report["normal_angle_between_hypotheses_deg"],
                      "positions_base_m":[p0.tolist(),p1.tolist()]},ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
