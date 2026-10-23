"""用可手算格子验证合并、边界、颜色与原点平移，而不依赖仿真真值。"""

import numpy as np
import pytest

from fuel_cap_task.voxelization import cube_mesh, voxelize_points


def make_voxels(points, colors=None, *, size=.005, origin=(0, 0, 0)):
    """未指定颜色时使用固定测试色，不代表真实传感器采集。"""
    if colors is None:
        colors = np.full((len(points), 3), 100, dtype=np.uint8)
    return voxelize_points(points, np.asarray(colors, dtype=np.uint8),
                           size_m=size, origin_world_m=origin)


def test_hand_calculable_points_share_a_cell_and_keep_original_membership():
    voxels = make_voxels([[.012, .023, .041], [.014, .024, .044], [.016, .023, .041]],
                         [[200, 0, 0], [0, 0, 200], [0, 200, 0]])
    np.testing.assert_array_equal(voxels.indices_xyz, [[2, 4, 8], [3, 4, 8]])
    np.testing.assert_array_equal(voxels.point_counts, [2, 1])
    np.testing.assert_array_equal(voxels.point_to_voxel, [0, 0, 1])
    np.testing.assert_array_equal(voxels.mean_rgb, [[100, 0, 100], [0, 200, 0]])
    np.testing.assert_allclose(voxels.centers_world_m, [[.0125, .0225, .0425], [.0175, .0225, .0425]])


def test_negative_coordinates_and_exact_boundary_use_floor_not_truncation():
    voxels = make_voxels([[-.001, 0, 0], [0, 0, 0], [.005, 0, 0]])
    np.testing.assert_array_equal(voxels.indices_xyz, [[-1, 0, 0], [0, 0, 0], [1, 0, 0]])


def test_same_world_point_keeps_its_cell_when_cloud_extent_changes():
    before = make_voxels([[.012, 0, 0]])
    after = make_voxels([[-1, 0, 0], [.012, 0, 0], [2, 0, 0]])
    np.testing.assert_array_equal(before.indices_xyz[0], after.indices_xyz[after.point_to_voxel[1]])


def test_explicit_grid_origin_translates_cells():
    voxels = make_voxels([[1.012, 2.023, 3.041]], origin=(1, 2, 3))
    np.testing.assert_array_equal(voxels.indices_xyz, [[2, 4, 8]])
    np.testing.assert_allclose(voxels.centers_world_m, [[1.0125, 2.0225, 3.0425]])


def test_missing_cells_are_not_invented_between_surface_points():
    voxels = make_voxels([[0, 0, 0], [.025, 0, 0]])
    assert len(voxels.indices_xyz) == 2
    assert voxels.point_counts.sum() == 2
    assert not np.any(voxels.indices_xyz[:, 0] == 2)


def test_point_reordering_does_not_change_grid_or_statistics():
    points = np.array([[.012, 0, 0], [.014, 0, 0], [-.003, 0, 0]])
    before, after = make_voxels(points), make_voxels(points[::-1])
    np.testing.assert_array_equal(before.indices_xyz, after.indices_xyz)
    np.testing.assert_array_equal(before.point_counts, after.point_counts)
    np.testing.assert_array_equal(before.point_to_voxel, after.point_to_voxel[::-1])


@pytest.mark.parametrize("size", [0, -.005, float("nan"), float("inf"), True])
def test_invalid_voxel_sizes_rejected(size):
    with pytest.raises(ValueError):
        make_voxels([[0, 0, 0]], size=size)


def test_nonfinite_point_and_origin_rejected():
    with pytest.raises(ValueError):
        make_voxels([[np.nan, 0, 0]])
    with pytest.raises(ValueError):
        make_voxels([[0, 0, 0]], origin=(0, np.inf, 0))


def test_displayed_cube_matches_cell_bounds_and_side_length():
    voxels = make_voxels([[.012, .023, .041]])
    vertices, triangles = cube_mesh(voxels.centers_world_m, voxels.size_m)
    np.testing.assert_allclose(vertices.min(axis=0), [.010, .020, .040])
    np.testing.assert_allclose(vertices.max(axis=0), [.015, .025, .045])
    assert triangles.shape == (12, 3)
    assert set(triangles.ravel()) == set(range(8))
