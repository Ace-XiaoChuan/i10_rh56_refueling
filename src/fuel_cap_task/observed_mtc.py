"""把实测候选和场景绑定到同一份 MTC 请求；保留仿真摆放的来源标记。"""

import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .voxel_octomap import rigid_matrix


def sha256(path):
    """记录实际读取的字节，不信任文件名所暗示的版本。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_binding(binding):
    """在启动规划前核对地图、盖体和清单；C++ 加载时还会独立复核。"""
    path = Path(binding["manifest_path"])
    if sha256(path) != binding["manifest_sha256"]:
        raise ValueError("实测场景清单摘要改变")
    manifest = json.loads(path.read_text())
    if (manifest["scene_representation"] != "environment_octomap_and_cap_collision_object"
            or manifest["planning_frame"] != "world" or manifest["allowed_collision_pairs"]
            or manifest["valid_for_hardware"]):
        raise ValueError("需要无接触规则的离线实测场景")
    for key, digest_key in [("map_path", "map_sha256"), ("cap_object_path", "cap_object_sha256")]:
        if sha256(path.parent/manifest[key]) != manifest[digest_key]:
            raise ValueError("实测碰撞几何摘要改变")
    for key in ("world_from_camera", "world_from_octomap"):
        rigid_matrix(manifest[key])
    return manifest


def model_directory(package, request):
    """按请求选择模型并核对来源；两个同名 world 坐标系不能只靠名字混用。"""
    package = Path(package)
    profile = request.get('robot_model_profile', 'single')
    if profile not in ('single', 'dual_arm'):
        raise ValueError('未知机器人模型配置')
    directory = package/('dual_arm' if profile == 'dual_arm' else 'config')
    if profile == 'single' and request.get('observed_scene',{}).get('robot_model_binding',{}).get('profile','single') != 'single':
        raise ValueError('不能把双臂世界坐标用于旧单臂模型')
    if profile == 'dual_arm':
        binding = request['observed_scene']['robot_model_binding']
        if binding['profile'] != profile:
            raise ValueError('场景与机器人模型配置不同')
        for name in ('robot.urdf','robot.srdf','kinematics.json','model.json'):
            if sha256(directory/name) != binding['files_sha256'][name]:
                raise ValueError('双臂机器人模型已改变：'+name)
    return directory


def observed_fields(manifest_path, candidates_path, fixed_grasp_path, settings, candidate_id, distance):
    """输出世界系手根目标与接近轴，两个方向共用同一张地图和同一个变换。"""
    path = Path(manifest_path).resolve()
    binding = {"manifest_path": str(path), "manifest_sha256": sha256(path),
               "replace_source_bodies": settings["replace_source_bodies"]}
    manifest = validate_binding(binding)
    profile = manifest.get('robot_model_profile','single')
    if profile == 'dual_arm':
        binding['robot_model_binding'] = manifest['robot_model_binding']
    candidates = json.loads(Path(candidates_path).read_text())
    fixed = json.loads(Path(fixed_grasp_path).read_text())
    if (sha256(fixed_grasp_path) != candidates["fixed_grasp_sha256"]
            or sha256(candidates["geometry_result"]) != candidates["geometry_result_sha256"]
            or candidates["pregrasp_channels"] != fixed["pregrasp_channels"]):
        raise ValueError("实测抓姿、固定手盖关系或手预型版本不同")
    # 地图摆放时读取的候选版本必须和本次目标相同，防止场景与目标错配。
    original = json.loads(Path(manifest["source_octomap_scene"]).read_text())
    if original["sources"].get(str(Path(candidates_path).resolve())) != sha256(candidates_path):
        raise ValueError("实测候选与地图摆放版本不同")
    candidate = next(c for c in candidates["candidates"] if c["candidate_id"] == candidate_id)
    transform = rigid_matrix(manifest["world_from_camera"])
    hand = transform @ rigid_matrix(candidate["camera_from_hand"])
    cap = transform @ rigid_matrix(candidate["camera_from_cap"])
    if not np.isfinite(distance) or distance <= 0:
        raise ValueError("接近距离须为有限正数")
    axis = cap[:3, 2]
    return {"observed_scene": binding, "observed_candidate_id": candidate_id,
            "robot_model_profile": profile,
            "fixed_left_joint_positions": manifest.get('fixed_left_joint_positions',{}),
            "observed_candidates_path": str(Path(candidates_path).resolve()),
            "observed_candidates_sha256": sha256(candidates_path),
            "hand_position": hand[:3, 3].tolist(),
            "pregrasp_position": (hand[:3, 3]+distance*axis).tolist(),
            "hand_orientation_xyzw": Rotation.from_matrix(hand[:3, :3]).as_quat().tolist(),
            "outward_axis_world": axis.tolist(), "target_frame_id": "world", "target_link": "base",
            "approach_distance_m": distance,
            "dense_joint_step_rad": settings["dense_joint_step_rad"],
            "dense_shape_motion_m": settings["dense_shape_motion_m"],
            "cap_estimate": {"source": ("paired_capture_and_dual_arm_installation" if profile=='dual_arm' else
                                        "measured_geometry_with_simulation_placement"),
                             "world_from_cap": cap.tolist(), "valid_for_hardware": False}}
