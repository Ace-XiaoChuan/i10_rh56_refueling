"""用已知几何、RGB 像素对应和错误标定检查点云重建。"""

import json
import struct

import numpy as np
from PIL import Image
import pytest

from fuel_cap_task.rgbd_point_cloud import (
    backproject_depth, digest, load_observation, transform_points, write_ply,
)


def test_optical_depth_keeps_z_constant_on_a_frontoparallel_plane():
    # 相机看向 Z=2 m 平面；边缘点到相机的斜距大于 2 m，Z 仍须为 2 m。
    points, pixels = backproject_depth(np.full((3, 3), 2.), np.ones((3, 3), dtype=bool),
                                      [[2, 0, 1], [0, 2, 1], [0, 0, 1]])
    np.testing.assert_allclose(points[[0, 4, 8]], [[-1, -1, 2], [0, 0, 2], [1, 1, 2]])
    np.testing.assert_array_equal(pixels[[0, 4, 8]], [[0, 0], [1, 1], [2, 2]])
    assert np.linalg.norm(points[0]) > 2


def test_invalid_depth_and_masked_pixels_do_not_create_points():
    depth = np.array([[0., np.nan, np.inf], [-1., 2., 3.]])
    valid = np.array([[True, True, True], [True, False, True]])
    points, pixels = backproject_depth(depth, valid, np.eye(3))
    np.testing.assert_array_equal(pixels, [[2, 1]])
    np.testing.assert_allclose(points, [[6, 3, 3]])


def test_camera_to_world_rotation_and_translation_direction():
    # 相机 +X 对应世界 +Y，相机原点在世界 (10,20,30)，不能误用逆变换。
    transform = np.array([[0, -1, 0, 10], [1, 0, 0, 20], [0, 0, 1, 30], [0, 0, 0, 1]])
    result = transform_points([[1, 0, 2], [0, 1, 2]], transform)
    np.testing.assert_allclose(result, [[10, 21, 32], [9, 20, 32]])


@pytest.mark.parametrize("matrix", [np.zeros((3, 3)),
    [[1, 1, 0], [0, 1, 0], [0, 0, 1]]])
def test_incompatible_camera_intrinsics_rejected(matrix):
    with pytest.raises(ValueError):
        backproject_depth(np.ones((2, 2)), np.ones((2, 2), dtype=bool), matrix)


def test_reflection_and_scaled_extrinsics_rejected():
    for diagonal in ([-1., 1., 1., 1.], [2., 1., 1., 1.]):
        with pytest.raises(ValueError, match="旋转"):
            transform_points([[0, 0, 1]], np.diag(diagonal))


@pytest.fixture
def observation(tmp_path):
    rgb = np.array([[[10, 20, 30], [40, 50, 60]], [[70, 80, 90], [100, 110, 120]]], dtype=np.uint8)
    Image.fromarray(rgb).save(tmp_path / "rgb.png")
    np.savez(tmp_path / "depth.npz", depth_m=np.ones((2, 2)),
             valid=np.array([[False, True], [True, False]]))
    metadata = {"source": "test_fixture", "frame_id": "test_optical", "timestamp_s": 0.,
                "timestamp_clock": "simulation", "depth_unit": "m", "depth_convention": "optical_z",
                "image_size_wh": [2, 2], "camera_matrix": np.eye(3).tolist(),
                "world_from_optical": np.eye(4).tolist(),
                "files": {name: digest(tmp_path / name) for name in ("rgb.png", "depth.npz")}}
    (tmp_path / "observation.json").write_text(json.dumps(metadata))
    return tmp_path


def test_colors_and_pixel_indices_stay_paired_without_truth_file(observation):
    cloud = load_observation(observation)
    np.testing.assert_array_equal(cloud.rgb, [[40, 50, 60], [70, 80, 90]])
    np.testing.assert_array_equal(cloud.pixel_uv, [[1, 0], [0, 1]])
    np.testing.assert_allclose(cloud.camera_xyz_m, [[1, 0, 1], [0, 1, 1]])
    assert set(cloud.input_sha256) == {"observation.json", "rgb.png", "depth.npz"}


def test_changed_observation_file_rejected(observation):
    Image.new("RGB", (2, 2)).save(observation / "rgb.png")
    with pytest.raises(ValueError, match="摘要"):
        load_observation(observation)


@pytest.mark.parametrize("change", [{"depth_unit": "mm"}, {"depth_convention": "range"},
                                  {"image_size_wh": [5, 4]}])
def test_wrong_units_depth_convention_and_image_size_rejected(observation, change):
    path = observation / "observation.json"
    metadata = json.loads(path.read_text())
    metadata.update(change)
    path.write_text(json.dumps(metadata))
    with pytest.raises(ValueError):
        load_observation(observation)


def test_exported_ply_vertex_layout_is_readable_by_standard_binary_types(tmp_path):
    path = tmp_path / "cloud.ply"
    write_ply(path, [[1.25, -2., 3.]], np.array([[12, 34, 56]], dtype=np.uint8))
    header, payload = path.read_bytes().split(b"end_header\n", 1)
    assert b"element vertex 1" in header
    assert struct.unpack("<fffBBB", payload) == (1.25, -2., 3., 12, 34, 56)
