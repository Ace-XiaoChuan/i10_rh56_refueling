"""用已知几何与刚体变换检查盖体几何基线，不将实测残差当作位姿真值误差。"""

import sys
from pathlib import Path

import numpy as np
import pytest
from numpy.testing import assert_allclose
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from fuel_cap_task.cap_geometry import estimate_cap_geometry, read_grip_model_reference


def synthetic_cap(noise=0.):
    """生成半径 35 mm、握梁表面凸出 20 mm 的已知曲面盖，外向为相机 -Z。"""
    rng = np.random.default_rng(127)
    xy = rng.uniform(-.035, .035, (14000, 2))
    radius = np.linalg.norm(xy, axis=1)
    keep = (radius < .035) & (np.abs(xy[:, 1]) > .007)
    xy = xy[keep]
    face = np.column_stack((xy, .400 + .002 * np.sum(xy**2, axis=1) / .035**2))
    angles = np.linspace(0, 2*np.pi, 500, endpoint=False)
    rim = np.column_stack((.035*np.cos(angles), .035*np.sin(angles), np.full(500, .402)))
    bridge_xy = rng.uniform([-.029, -.006], [.029, .006], (2000, 2))
    bridge = np.column_stack((bridge_xy, .380 + .002*(bridge_xy[:, 0]/.029)**2))
    points = np.vstack((face, rim, bridge))
    return points + rng.normal(0, noise, points.shape)


def angle_deg(first, second, directed=True):
    """比较有向法向或允许正负等价的握梁长轴，返回度。"""
    cosine = first @ second
    return np.rad2deg(np.arccos(np.clip(cosine if directed else abs(cosine), -1, 1)))


def test_known_shape_recovers_axes_circle_and_surface():
    result = estimate_cap_geometry(synthetic_cap())
    assert_allclose(result.boundary_center_m, [0, 0, .402], atol=1e-7)
    assert abs(result.radius_m - .035) < 1e-7
    assert angle_deg(result.rotation[:, 2], [0, 0, -1]) < .001
    assert angle_deg(result.rotation[:, 0], [1, 0, 0], directed=False) < 2
    assert_allclose(result.surface_reference_m, [0, 0, .380], atol=2e-5)
    assert_allclose(result.rotation.T @ result.rotation, np.eye(3), atol=1e-12)
    assert abs(np.linalg.det(result.rotation) - 1) < 1e-12


def test_rigid_transform_and_permutation_preserve_geometry():
    points = synthetic_cap()
    baseline = estimate_cap_geometry(points)
    rotation = Rotation.from_euler("xyz", [29, -21, 63], degrees=True).as_matrix()
    translation = np.array([.13, -.08, .22])
    transformed = points @ rotation.T + translation
    transformed = transformed[np.random.default_rng(6).permutation(len(points))]
    result = estimate_cap_geometry(transformed, camera_origin_m=translation)
    assert_allclose(result.boundary_center_m, rotation @ baseline.boundary_center_m + translation,
                    atol=1e-7)
    assert_allclose(result.surface_reference_m, rotation @ baseline.surface_reference_m + translation,
                    atol=1e-7)
    assert angle_deg(result.rotation[:, 2], rotation @ baseline.rotation[:, 2]) < .001
    assert angle_deg(result.rotation[:, 0], rotation @ baseline.rotation[:, 0], directed=False) < .001


def test_small_measurement_noise_preserves_axes():
    result = estimate_cap_geometry(synthetic_cap(noise=.00008))
    assert_allclose(result.boundary_center_m, [0, 0, .402], atol=.0005)
    assert abs(result.radius_m - .035) < .0005
    assert angle_deg(result.rotation[:, 2], [0, 0, -1]) < .5
    assert angle_deg(result.rotation[:, 0], [1, 0, 0], directed=False) < 2


def test_plain_disk_does_not_invent_bridge():
    angle = np.linspace(0, 2*np.pi, 500)
    points = np.column_stack((.035*np.cos(angle), .035*np.sin(angle), np.full(500, .4)))
    with pytest.raises(ValueError):
        estimate_cap_geometry(points)


def test_curved_face_without_bridge_is_rejected():
    with pytest.raises(ValueError):
        estimate_cap_geometry(synthetic_cap()[:-2000])


def test_half_cap_crop_does_not_claim_complete_circle():
    points = synthetic_cap()
    with pytest.raises(ValueError):
        estimate_cap_geometry(points[points[:, 0] > 0])


def test_model_reference_reads_height_and_width_as_different_directions(tmp_path):
    # 使用另一组尺寸，检查偏移真正取自轴向高度，而不是宽度的一半。
    urdf = tmp_path / "cap.urdf"
    urdf.write_text('''<robot><link name="cap_body"><visual name="cap_grip">
    <origin xyz="0 0 .04"/><geometry><box size=".024 .070 .020"/></geometry>
    </visual></link><joint name="cap_grip_center_joint"><origin xyz="0 0 .04" rpy="0 0 1.5707963267948966"/></joint>
    <joint name="cap_grip_top_center_joint"><origin xyz="0 0 .05"/></joint></robot>''')
    reference = read_grip_model_reference(urdf, top_width_reduction_m=.002)
    assert_allclose([reference[k] for k in ("length_m", "base_width_m", "top_width_m", "height_m",
                                          "top_surface_to_reference_m")],
                    [.070, .024, .022, .020, .010], atol=1e-12)


@pytest.mark.parametrize("points", [np.zeros((10, 3)), np.full((200, 3), np.nan),
                                  np.zeros((200, 3)), np.ones((200, 2))])
def test_invalid_input_is_rejected(points):
    with pytest.raises(ValueError):
        estimate_cap_geometry(points)
