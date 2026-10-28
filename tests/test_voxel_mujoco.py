"""检查 box 全边长与独立碰撞几何，防止凹槽被整体凸包填平。"""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from fuel_cap_task.voxel_mujoco import check_compiled_voxels, exposed_voxel_faces, voxel_scene_xml


def test_surface_batch_removes_only_shared_face_and_preserves_open_gap():
    rows, directions, corners = exposed_voxel_faces([[-1,0,0], [0,0,0], [2,0,0]])
    assert len(rows) == 16
    assert len(corners) == 16
    # 相邻格公共面的两侧删除，空隙两侧的面保留。
    assert not np.any((rows == 0) & (directions == 3))
    assert not np.any((rows == 1) & (directions == 5))
    assert np.any((rows == 1) & (directions == 3))
    assert np.any((rows == 2) & (directions == 5))
    # 每面仍为单位格的方形，没有用大三角形跨过空隙。
    edges = np.roll(corners, -1, axis=1)-corners
    np.testing.assert_array_equal(np.sum(edges*edges,axis=2), np.ones((16,4)))


def test_boxes_keep_meter_positions_half_extents_and_cap_collision():
    centers = np.array([[-.003,.001,.651], [.003,.001,.651]])
    xml, colors, _ = voxel_scene_xml(centers, .002, np.array([False,True]), centers.mean(0))
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    check_compiled_voxels(model, data, centers, .002)
    assert model.geom_group.tolist() == [0,1]
    assert len(np.unique(colors, axis=0)) == 2
    assert model.nmesh == 0


def test_independent_boxes_leave_gap_open_for_collision_detection():
    centers = np.array([[-.003,0,0], [.003,0,0]])
    xml, _, _ = voxel_scene_xml(centers, .002, np.array([False,False]), np.zeros(3))
    root = ET.fromstring(xml)
    # 动态探针放在两格之间应无接触，移入一个格子后必须报告接触。
    body = ET.SubElement(root.find("worldbody"), "body", name="probe")
    ET.SubElement(body, "freejoint")
    ET.SubElement(body, "geom", type="sphere", size=".0003", mass=".01")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.ncon == 0
    data.qpos[:3] = centers[0]
    mujoco.mj_forward(model, data)
    assert data.ncon > 0
