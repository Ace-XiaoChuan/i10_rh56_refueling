"""新观测的相机定位必须进入当前网络坐标，错误绑定直接拒绝。"""
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
spec = importlib.util.spec_from_file_location('thumb_anchor_entry', ROOT / 'tools/run_thumb_anchor.py')
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


def test_camera_estimate_rebases_target_and_rejects_wrong_network(tmp_path):
    network = tmp_path / 'network_volume.npz'
    np.savez(network, marker=np.array([1]))
    expected = np.eye(4)
    expected[:3, :3] = Rotation.from_euler('xyz', [.2, -.1, .7]).as_matrix()
    expected[:3, 3] = [.08, .06, .09]
    camera_from_volume = np.eye(4)
    camera_from_volume[:3, 3] = [-.1, .04, .5]
    camera_from_grip = camera_from_volume @ expected
    estimate = tmp_path / 'estimate.json'
    estimate.write_text(json.dumps(dict(reference_frame='cap_grip_top_center', geometric_checks_passed=True,
        pose_candidates_camera=[dict(frame_id='camera', reference_frame='cap_grip_top_center',
            position_m=camera_from_grip[:3, 3].tolist(),
            orientation_xyzw=Rotation.from_matrix(camera_from_grip[:3, :3]).as_quat().tolist())])))
    config = dict(run_dir=str(tmp_path), grip_estimate=str(estimate), estimate_network_sha256=entry.digest_file(network))
    frame, source, _ = entry.observation_frame(config, dict(camera_from_volume=camera_from_volume))
    np.testing.assert_allclose(frame, expected, atol=1e-12)
    assert source == 'camera_geometry_estimate_with_known_grip_dimensions'
    config['estimate_network_sha256'] = '0' * 64
    with pytest.raises(ValueError, match='绑定'):
        entry.observation_frame(config, dict(camera_from_volume=camera_from_volume))


def test_invalid_camera_geometry_is_not_used_as_valid_localization(tmp_path):
    network = tmp_path / 'network_volume.npz'
    np.savez(network, marker=np.array([1]))
    estimate = tmp_path / 'estimate.json'
    estimate.write_text(json.dumps(dict(reference_frame='cap_grip_top_center', geometric_checks_passed=False,
        pose_candidates_camera=[dict(frame_id='camera', reference_frame='cap_grip_top_center',
            position_m=[0, 0, .4], orientation_xyzw=[0, 0, 0, 1])])) )
    config = dict(run_dir=str(tmp_path), grip_estimate=str(estimate), estimate_network_sha256=entry.digest_file(network))
    with pytest.raises(ValueError, match='定位'):
        entry.observation_frame(config, dict(camera_from_volume=np.eye(4)))


def test_degenerate_new_cap_cloud_is_rejected_instead_of_using_old_frame(tmp_path):
    network = tmp_path / 'network_volume.npz'
    np.savez(network, marker=np.array([1]))
    pcd = tmp_path / 'bad_cap.pcd'
    pcd.write_text('VERSION .7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 .4\n')
    config = dict(run_dir=str(tmp_path), cap_cloud=str(pcd), cap_cloud_unit='m',
                  estimate_network_sha256=entry.digest_file(network))
    with pytest.raises(ValueError, match='128'):
        entry.observation_frame(config, dict(camera_from_volume=np.eye(4)))


def test_new_segmented_cap_cloud_is_fitted_and_converted_from_millimeters(tmp_path):
    # 复用已有已知几何生成器；这里只检验新入口的单位和相机→网络衔接。
    helper_spec = importlib.util.spec_from_file_location('known_cap_geometry', ROOT / 'tests/test_cap_geometry.py')
    helper = importlib.util.module_from_spec(helper_spec)
    helper_spec.loader.exec_module(helper)
    points = helper.synthetic_cap() * 1000
    pcd = tmp_path / 'current_cap.pcd'
    with pcd.open('w') as stream:
        stream.write(f'VERSION .7\nFIELDS x y z\nSIZE 8 8 8\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n')
        np.savetxt(stream, points, fmt='%.12g')
    network = tmp_path / 'network_volume.npz'
    np.savez(network, marker=np.array([1]))
    config = dict(run_dir=str(tmp_path), cap_cloud=str(pcd), cap_cloud_unit='mm',
                  estimate_network_sha256=entry.digest_file(network))
    camera_from_volume = np.eye(4)
    camera_from_volume[:3, 3] = [-.2, .1, .3]
    frame, source, _ = entry.observation_frame(config, dict(camera_from_volume=camera_from_volume))
    np.testing.assert_allclose(frame[:3, 3], [.2, -.1, .08], atol=2e-5)
    np.testing.assert_allclose(frame[:3, 2], [0, 0, -1], atol=1e-4)
    assert source == 'current_cap_cloud_geometry_fit_with_known_grip_dimensions'
