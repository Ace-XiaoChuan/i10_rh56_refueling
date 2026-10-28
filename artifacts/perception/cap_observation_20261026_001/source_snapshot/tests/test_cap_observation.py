"""验证真实观测绑定、变换方向和原始点子集，不将数值残差当作实物精度。"""

import copy
import json
from pathlib import Path
import sys

import numpy as np
from numpy.testing import assert_allclose
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from fuel_cap_task.cap_observation import bind_observation, digest, extract_connected_crop


def write_json(path, value):
    path.write_text(json.dumps(value), encoding="utf-8")


@pytest.fixture
def observation_case(tmp_path):
    """已知几何：相机点先绕法兰 Y 转90°，再绕基座 Z 转90°。"""
    points_mm = np.array([[100., 20., 800.], [101., 20., 800.], [100., 21., 800.]])
    header = ("VERSION .7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\n"
              "WIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA binary\n")
    raw = tmp_path / "cloud.pcd"
    raw.write_bytes(header.encode("ascii") + points_mm.astype("<f8").tobytes())
    flange_from_camera = np.eye(4)
    flange_from_camera[:3, :3] = Rotation.from_euler("y", 90, degrees=True).as_matrix()
    flange_from_camera[:3, 3] = [.1, .2, .3]
    calibration = {"extrinsic": {"name": "flange_from_camera", "matrix_translation_unit": "m",
                                 "flange_from_camera_matrix": flange_from_camera.tolist()}}
    calibration_path = tmp_path / "fuel_cap_camera_calibration.json"
    write_json(calibration_path, calibration)
    base_from_flange = np.eye(4)
    base_from_flange[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    base_from_flange[:3, 3] = [1., 2., 3.]
    state = {"steady": True, "simulation": False, "q_rad": [0.]*6,
             "flange_pose_m_rpy_rad": [1., 2., 3., 0., 0., np.pi/2],
             "base_from_flange": base_from_flange.tolist()}
    observation = {"capture_complete": True,
        "point_cloud": {"path": raw.name, "sha256": digest(raw), "mirror": "none", "frame": "camera_sdk_native"},
        "calibration_sha256": digest(calibration_path), "state_before": state, "state_after": copy.deepcopy(state),
        "robot_before": {"end_utc": "2026-10-26T02:00:00.1+00:00"},
        "camera_cloud": {"start_utc": "2026-10-26T02:00:00.2+00:00", "end_utc": "2026-10-26T02:00:00.4+00:00"},
        "robot_after": {"start_utc": "2026-10-26T02:00:00.5+00:00"}, "time_meaning": "主机调用时间"}
    path = tmp_path / "observation.json"
    write_json(path, observation)
    report = {"input_sha256": digest(raw), "unresolved": ["缺少本次采集的机器人姿态与采集时间"],
        "implementation_sha256": {}, "boundary_center_m": [.1, .02, .8],
        "visible_bridge_surface_reference_m": [.1, .02, .8],
        "outward_axis_camera": [0, 0, 1], "bridge_axis_camera_undirected": [1, 0, 0],
        "pose_candidates_camera_model_based": [{"position_m": [.1, .02, .8], "ambiguity_yaw_deg": i*180} for i in range(2)],
        "orientation_candidates": [{"rotation_columns_xyz": np.diag([(-1.)**i, (-1.)**i, 1.]).tolist()} for i in range(2)]}
    return path, observation, report, points_mm*.001


def test_known_transform_direction_and_180_degree_candidates(observation_case):
    path, observation, report, points = observation_case
    result = bind_observation(report, points, path)
    # 手算：相机点(.1,.02,.8)→法兰(.9,.22,.2)→基座(.78,2.9,3.2)m。
    for candidate in result["pose_candidates_base_model_based"]:
        assert_allclose(candidate["position_m"], [.78, 2.9, 3.2], atol=1e-12)
        assert_allclose(np.linalg.det(np.asarray(candidate["base_from_cap_grip_center"])[:3,:3]), 1.)
    first, second = [np.asarray(v["base_from_cap_grip_center"]) for v in result["pose_candidates_base_model_based"]]
    assert_allclose(first[:3, 2], second[:3, 2], atol=1e-12)
    assert_allclose(first[:3, 0], -second[:3, 0], atol=1e-12)
    assert_allclose(result["outward_axis_base"], [0, 1, 0], atol=1e-12)
    assert result["robot_state_paired"] is True
    assert result["motion_target_valid"] is False
    assert result["observation_binding"]["camera_axis_assumption_verified_against_legacy_export"] is False
    assert "robot_state_paired" not in report


@pytest.mark.parametrize("change", ["source_hash", "calibration_hash", "moving", "simulation", "time", "pose_matrix", "mirror"])
def test_wrong_source_state_or_convention_is_rejected(observation_case, change):
    path, observation, report, points = observation_case
    if change == "source_hash": observation["point_cloud"]["sha256"] = "wrong"
    elif change == "calibration_hash": observation["calibration_sha256"] = "wrong"
    elif change == "moving": observation["state_before"]["steady"] = False
    elif change == "simulation": observation["state_after"]["simulation"] = True
    elif change == "time": observation["robot_after"]["start_utc"] = "2026-10-26T02:00:00+00:00"
    elif change == "pose_matrix": observation["state_before"]["base_from_flange"][0][3] += .1
    elif change == "mirror": observation["point_cloud"]["mirror"] = "vertical"
    write_json(path, observation)
    with pytest.raises(ValueError): bind_observation(report, points, path)


def test_crop_from_other_pose_is_not_silently_reused(observation_case):
    path, _, report, points = observation_case
    with pytest.raises(ValueError, match="原始子集"):
        bind_observation(report, points + [.001, 0, 0], path)


def test_reflected_extrinsic_is_rejected_even_with_matching_file_digest(observation_case):
    path, observation, report, points = observation_case
    calibration_path = path.parent / "fuel_cap_camera_calibration.json"
    calibration = json.loads(calibration_path.read_text())
    calibration["extrinsic"]["flange_from_camera_matrix"] = np.diag([1, -1, 1, 1]).tolist()
    write_json(calibration_path, calibration)
    observation["calibration_sha256"] = digest(calibration_path)
    write_json(path, observation)
    with pytest.raises(ValueError, match="右手刚体"):
        bind_observation(report, points, path)


def test_seeded_crop_preserves_original_row_order_and_unit():
    # 两个相隔20mm的连通区域，交错存储，选择第二组后仍保持原始行号。
    first = np.array([[i, 0, 5] for i in range(5)], dtype=float)
    second = first + [20, 0, 0]
    points = np.stack((first, second), axis=1).reshape(-1,3)
    kwargs = dict(bounds_mm=[-10, 40, -10, 10, -10, 20], seeds_mm=[[22,0,5]], radius_mm=1.1)
    indices, _ = extract_connected_crop(points, length_unit="mm", **kwargs)
    indices_m, _ = extract_connected_crop(points*.001, length_unit="m", **kwargs)
    assert_allclose(indices, [1,3,5,7,9])
    assert_allclose(indices_m, indices)
    assert_allclose(points[indices], second)


def test_crop_touching_roi_boundary_is_rejected():
    points = np.array([[i,0,5] for i in range(5)], dtype=float)
    with pytest.raises(ValueError, match="截断边界"):
        extract_connected_crop(points, length_unit="mm", bounds_mm=[-.1,10,-10,10,-10,20],
                               seeds_mm=[[2,0,5]], radius_mm=1.1)
