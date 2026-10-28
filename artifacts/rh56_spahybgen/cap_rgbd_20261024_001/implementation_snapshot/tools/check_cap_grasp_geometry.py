#!/usr/bin/env python3
"""在移除机械臂的独立手场景检查候选几何，并回放优化迭代；不积分物理。"""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import sys

os.environ.setdefault("MUJOCO_GL", "egl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import mujoco
import numpy as np
from PIL import Image
from fuel_cap_task.grasp_search import GraspSearchScene, SearchSettings
from fuel_cap_task.mujoco_scene import DEFAULT_SCENE, ARM_JOINTS
from fuel_cap_task.rgbd_point_cloud import digest


def save_json(path, value):
    """保存独立几何评价，禁止覆盖生成结果或改写优化排名。"""
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def place_world(scene, world_from_hand, full_names, full_angles):
    """只设置显示用手根和关节；每次更新后重新计算几何，不读取盖体系作为输入。"""
    rotation, position = world_from_hand[:3, :3], world_from_hand[:3, 3]
    carrier_rotation = rotation @ scene.carrier_from_hand_r.T
    carrier_position = position - carrier_rotation @ scene.carrier_from_hand_p
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, np.ascontiguousarray(carrier_rotation).ravel())
    scene.data.mocap_pos[scene.mocap] = carrier_position
    scene.data.mocap_quat[scene.mocap] = quaternion
    for name, value in zip(full_names, full_angles):
        scene.data.joint(name).qpos[0] = value
    mujoco.mj_forward(scene.model, scene.data)


def r6d_rotation(value):
    """与上游 robust_R6d2R9d 同一正交化，用于无 PyTorch 的迭代显示。"""
    def normalize(vector):
        return vector / max(float(np.linalg.norm(vector)), 1e-8)
    x, y = normalize(value[:3]), normalize(value[3:])
    middle, difference = normalize(x + y), normalize(x - y)
    x, y = normalize(middle + difference), normalize(middle - difference)
    return np.column_stack([x, y, normalize(np.cross(x, y))])


def check(args):
    """评价全部候选，分别报告自碰、目标穿透、环境干涉和靠近目标的手指数。"""
    folder = args.result_dir.resolve()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    result = json.loads((folder / "result.json").read_text())
    candidates = json.loads((folder / "candidates.json").read_text())
    observation_path = next(name for name in result["inputs_sha256"] if name.endswith("observation.json"))
    observation = json.loads((folder / observation_path).read_text())
    if observation.get("scene_xml_sha256") != digest(DEFAULT_SCENE):
        raise ValueError("仿真观测与当前评价场景不一致")
    for name in ("candidates.json", "scene_input.npz", "optimization.npz"):
        if digest(folder / name) != result["outputs_sha256"][name]:
            raise ValueError("候选或场景输入摘要不一致：" + name)
    cfg = result["settings"]["geometry"]
    scene = GraspSearchScene(settings=SearchSettings(penetration_m=cfg["penetration_tolerance_m"]), fixture=True)
    m, d = scene.model, scene.data
    present = [name for name in ARM_JOINTS if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0]
    if present:
        raise ValueError("独立手评价模型仍包含机械臂关节")
    cap_body = m.body("cap_body").id
    target_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] == cap_body
                    and (m.geom_contype[g] or m.geom_conaffinity[g])}
    finger_bodies = {"thumb": "thumb_distal", "index": "index_intermediate", "middle": "middle_intermediate",
                     "ring": "ring_intermediate", "pinky": "pinky_intermediate"}
    fingers = {finger: scene._collision_geom(body) for finger, body in finger_bodies.items()}
    rows = []
    for candidate in candidates["candidates"]:
        world_from_hand = np.asarray(candidate["world_from_hand"])
        place_world(scene, world_from_hand, candidates["full_joint_names"], candidate["full_joint_angles_rad"])
        actual = d.body("base")
        if not (np.allclose(actual.xpos, world_from_hand[:3, 3], atol=1e-8)
                and np.allclose(actual.xmat.reshape(3, 3), world_from_hand[:3, :3], atol=1e-6)):
            raise ValueError("候选世界位姿到独立手放置发生坐标偏差")
        collisions = []
        for contact in d.contact:
            a, b = map(int, contact.geom)
            if not ({a, b} & scene.hand_geoms) or contact.dist >= -cfg["penetration_tolerance_m"]:
                continue
            kind = ("hand_self" if a in scene.hand_geoms and b in scene.hand_geoms else
                    "hand_target" if {a, b} & target_geoms else "hand_environment")
            collisions.append({"kind": kind, "geoms": [scene.geom_label(a), scene.geom_label(b)],
                               "distance_m": float(contact.dist)})
        distances = {}
        for finger, geom in fingers.items():
            distances[finger] = min(float(mujoco.mj_geomDistance(m, d, geom, cap, .05, np.empty(6)))
                                    for cap in target_geoms)
        near = [finger for finger, distance in distances.items() if distance <= cfg["near_target_distance_m"]]
        reasons = sorted({contact["kind"] + "_penetration" for contact in collisions})
        if not candidate["hard_valid"]:
            reasons.append("invalid_joint_or_pose")
        if not result.get("prediction_evidence_sufficient", result["target_contact_peak"] >= .1
                          and result["target_center_peak"] >= .1):
            reasons.append("insufficient_network_evidence")
        if len(near) < cfg["minimum_near_fingers"]:
            reasons.append("too_few_fingers_near_target")
        rows.append({"id": candidate["id"], "geometrically_clear_and_near_target": not reasons,
                     "rejection_reasons": reasons, "finger_target_distances_m": distances,
                     "near_target_fingers": near, "penetrations": collisions,
                     "physical_grasp_verified": False})
    report = {"mode": "detached_hand_model_geometry_no_physics", "settings": cfg,
              "candidate_count": len(rows),
              "geometrically_clear_count": sum(row["geometrically_clear_and_near_target"] for row in rows),
              "rejection_counts": dict(Counter(reason for row in rows for reason in row["rejection_reasons"])),
              "arm_joint_names_present": present, "arm_planning_performed": False,
              "physics_steps": 0, "scene_geometry_used_only_for_evaluation": True,
              "source_sha256": {str(p.relative_to(ROOT)): digest(p) for p in
                                  (Path(__file__), DEFAULT_SCENE, ROOT / "src/fuel_cap_task/grasp_search.py")},
              "generation_result_sha256": digest(folder / "result.json"), "candidates": rows}
    save_json(output / "geometry.json", report)
    selected = next((index for index in candidates["rank"] if rows[index]["geometrically_clear_and_near_target"]),
                    candidates["rank"][0])
    if not args.no_render:
        render(scene, folder, output, candidates, rows, selected)
    report["display_candidate_id"] = selected
    report["outputs_sha256"] = {p.name: digest(p) for p in output.iterdir() if p.is_file() and p.name != "geometry.json"}
    save_json(output / "geometry.json", report)
    print(json.dumps({key: report[key] for key in ("candidate_count", "geometrically_clear_count", "rejection_counts")}, ensure_ascii=False))


def render(scene, folder, output, candidates, checks, selected):
    """渲染真实优化变量序列；动画展示求解过程，不是可执行运动轨迹。"""
    with np.load(folder / "scene_input.npz") as data:
        world_from_volume = data["world_from_volume"]
        target = data["points_volume_m"][data["target_point_mask"]]
        lookat = world_from_volume[:3, :3] @ np.median(target, axis=0) + world_from_volume[:3, 3]
    with np.load(folder / "optimization.npz") as data:
        trajectory = data["q_trajectory"][selected]
    model_path = (folder / candidates["model_dir"]).resolve()
    mapping = json.loads((model_path / "model.json").read_text())["mapping"]
    matrix, offset = np.asarray(mapping["matrix"]), np.asarray(mapping["offset"])
    m, d = scene.model, scene.data
    m.vis.global_.offwidth, m.vis.global_.offheight = 900, 700
    camera, options = mujoco.MjvCamera(), mujoco.MjvOption()
    camera.lookat[:] = lookat
    camera.distance, camera.azimuth, camera.elevation = .48, 90, -20
    options.geomgroup[0] = options.geomgroup[3] = 0
    options.sitegroup[:] = 0
    frames = []
    sample_indices = np.unique(np.rint(np.linspace(0, len(trajectory) - 1, min(61, len(trajectory)))).astype(int))
    with mujoco.Renderer(m, height=700, width=900) as renderer:
        for step in sample_indices:
            q = trajectory[step]
            transform = np.eye(4)
            transform[:3, :3], transform[:3, 3] = r6d_rotation(q[3:9]), q[:3]
            transform = world_from_volume @ transform
            full = q[9:] @ matrix.T + offset
            place_world(scene, transform, candidates["full_joint_names"], full)
            renderer.update_scene(d, camera=camera, scene_option=options)
            frame = Image.fromarray(renderer.render().copy())
            frames.append(frame)
            if step in (sample_indices[0], sample_indices[len(sample_indices) // 2], sample_indices[-1]):
                frame.save(output / f"iteration_{step:03d}.png")
        np.testing.assert_allclose(transform, candidates["candidates"][selected]["world_from_hand"], atol=1e-6)
    palette = [frame.quantize(colors=128) for frame in frames]
    durations = [70] * len(frames)
    durations[0] += 700
    durations[-1] += 1300
    palette[0].save(output / "optimization.gif", save_all=True, append_images=palette[1:],
                    duration=durations, loop=0, disposal=2)
    try:
        import cv2
    except ImportError:
        cv2 = None
    if cv2 is not None:
        writer = cv2.VideoWriter(str(output / "optimization.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 15, (900, 700))
        if writer.isOpened():
            for frame in [frames[0]] * 10 + frames + [frames[-1]] * 20:
                writer.write(cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR))
        writer.release()
    (output / "animation_caption.txt").write_text(
        f"候选 {selected} 的数值优化过程；几何检查：" +
        ("通过" if checks[selected]["geometrically_clear_and_near_target"] else "未通过") +
        "。画面是优化变量的显示，不是实际执行轨迹或抓稳结果。\n", encoding="utf-8")


def main():
    """显式指定一次生成结果和新的几何输出目录。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--no-render", action="store_true", help="仅做几何评价，省略基线对照的动画")
    check(parser.parse_args())


if __name__ == "__main__":
    main()
