"""核对相机系叠加、MuJoCo 编译网格补偿和 CloudCompare 导出单位。"""

from pathlib import Path
import sys

import mujoco
import numpy as np
from numpy.testing import assert_allclose
import pytest
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from fuel_cap_task.cap_hand_overlay import HandOnlyScene, place_fixed_hand, extract_visual_meshes, write_hand_ply
from fuel_cap_task.fixed_grasp import DEFAULT_FIXED_GRASP, load_fixed_grasp
from fuel_cap_task.grasp_search import GraspSearchScene


@pytest.fixture(scope='module')
def setup_hand():
    grasp=load_fixed_grasp(DEFAULT_FIXED_GRASP,expected_mujoco_version=mujoco.__version__)
    return grasp,HandOnlyScene(grasp.provenance.source_scene_path)


def test_hand_only_model_preserves_previous_hand_geometry_without_environment(setup_hand):
    """独立手与历史完整模型的手一致，编译结果不含盖、箱体、机械臂或地面。"""
    grasp, scene = setup_hand
    old = GraspSearchScene(grasp.provenance.source_scene_path, fixture=True)
    pose = np.eye(4)
    pose[:3,:3] = Rotation.from_euler('xyz', [23., -17., 41.], degrees=True).as_matrix()
    pose[:3,3] = [.13, .02, .64]
    expected = place_fixed_hand(pose, grasp, scene=old)
    actual = place_fixed_hand(pose, grasp, scene=scene)
    assert_allclose(actual.vertices_camera_m, expected.vertices_camera_m, atol=1e-12, rtol=0)
    np.testing.assert_array_equal(actual.faces, expected.faces)
    assert actual.joint_positions_rad == expected.joint_positions_rad
    allowed = scene.hand_bodies | {0, scene.model.body('observed_hand_carrier').id}
    assert set(range(scene.model.nbody)) == allowed
    assert all(int(body) in scene.hand_bodies for body in scene.model.geom_bodyid)
    assert scene.model.njnt == 12 and scene.model.nu == 6


def test_180_degree_branch_rotates_hand_about_observed_cap_axis(setup_hand):
    """两方向围绕同一个实测盖轴转180°，同时旋转手根偏移和全部网格。"""
    grasp, scene = setup_hand
    pose = np.eye(4)
    pose[:3,:3] = Rotation.from_euler('xyz', [31., -18., 57.], degrees=True).as_matrix()
    pose[:3,3] = [.13, -.05, .62]
    first = place_fixed_hand(pose, grasp, scene=scene)
    opposite = pose @ np.diag([-1., -1., 1., 1.])
    second = place_fixed_hand(opposite, grasp, scene=scene)
    rotation = 2 * np.outer(pose[:3,2], pose[:3,2]) - np.eye(3)
    center = pose[:3,3]
    assert_allclose(second.camera_from_hand[:3,3],
                    center + rotation @ (first.camera_from_hand[:3,3] - center), atol=1e-12)
    assert_allclose(second.vertices_camera_m,
                    (first.vertices_camera_m - center) @ rotation.T + center, atol=1e-12)
    assert_allclose(second.camera_from_hand[:3,:3], rotation @ first.camera_from_hand[:3,:3], atol=1e-12)


def test_fixed_pose_composes_rotated_offset_and_model_fk(setup_hand):
    grasp,scene=setup_hand
    pose=np.eye(4)
    pose[:3,:3]=np.array([[0,-1,0],[1,0,0],[0,0,1]])
    pose[:3,3]=[.1,.2,.6]
    result=place_fixed_hand(pose,grasp,scene=scene)
    assert_allclose(result.camera_from_hand[:3,3],
                    [.1-grasp.cap_from_hand.position_m[1],
                     .2+grasp.cap_from_hand.position_m[0],
                     .6+grasp.cap_from_hand.position_m[2]],atol=1e-12)
    assert_allclose(scene.data.body('base').xpos,result.camera_from_hand[:3,3],atol=1e-12)
    assert scene.data.time==0
    assert result.coupling_residual_max_rad<1e-12
    assert {row['body'] for row in result.geometries}.issubset(
        {scene.model.body(i).name for i in scene.hand_bodies})


def test_every_mesh_vertex_and_fingertip_obeys_same_rigid_transform(setup_hand):
    grasp,scene=setup_hand
    initial=place_fixed_hand(np.eye(4),grasp,scene=scene)
    pose=np.eye(4)
    pose[:3,:3]=Rotation.from_euler('xyz',[31,-18,57],degrees=True).as_matrix()
    pose[:3,3]=[.13,-.05,.62]
    changed=place_fixed_hand(pose,grasp,scene=scene)
    assert_allclose(changed.vertices_camera_m,
                    initial.vertices_camera_m@pose[:3,:3].T+pose[:3,3],atol=1e-12)
    np.testing.assert_array_equal(changed.faces,initial.faces)
    for name,value in initial.fingertip_sites_camera_m.items():
        assert_allclose(changed.fingertip_sites_camera_m[name],pose[:3,:3]@value+pose[:3,3],atol=1e-12)


@pytest.mark.parametrize('bad',[np.zeros((3,3)),np.full((4,4),np.nan),
                               np.diag([2.,1.,1.,1.]),np.diag([-1.,1.,1.,1.])])
def test_invalid_transform_does_not_modify_model(bad,setup_hand):
    grasp,scene=setup_hand
    before=scene.data.qpos.copy(),scene.data.mocap_pos.copy(),scene.data.mocap_quat.copy()
    with pytest.raises(ValueError):
        place_fixed_hand(bad,grasp,scene=scene)
    for old,current in zip(before,(scene.data.qpos,scene.data.mocap_pos,scene.data.mocap_quat)):
        np.testing.assert_array_equal(old,current)


def test_compiled_mesh_centering_and_scaling_are_applied_once():
    authored=np.array([[.01,.02,.03],[.04,.02,.03],[.01,.06,.03],[.01,.02,.08]])
    vertices=' '.join(map(str,authored.ravel()))
    model=mujoco.MjModel.from_xml_string(f'''<mujoco><asset>
      <mesh name="test" vertex="{vertices}" face="0 2 1 0 1 3 0 3 2 1 2 3" scale="2 3 4"/>
      </asset><worldbody><body name="hand" pos=".1 -.2 .5" quat=".7071067811865476 0 0 .7071067811865476">
      <geom type="mesh" mesh="test" pos=".03 -.04 .02" quat=".7071067811865476 .7071067811865476 0 0" contype="0" conaffinity="0"/>
      </body></worldbody></mujoco>''')
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    actual,faces,_=extract_visual_meshes(model,data,{model.body('hand').id})
    body_rotation=np.array([[0,-1,0],[1,0,0],[0,0,1]])
    geom_rotation=np.array([[1,0,0],[0,0,-1],[0,1,0]])
    expected=((authored*[2,3,4])@geom_rotation.T+[.03,-.04,.02])@body_rotation.T+[.1,-.2,.5]
    assert cKDTree(expected).query(actual)[0].max()<3e-8
    assert cKDTree(actual).query(expected)[0].max()<3e-8
    assert faces.min()==0 and faces.max()==3


@pytest.mark.parametrize('unit,scale',[('mm',1000.),('m',1.)])
def test_ply_round_trip_preserves_unit_vertices_and_faces(unit,scale,tmp_path,setup_hand):
    grasp,scene=setup_hand
    overlay=place_fixed_hand(np.eye(4),grasp,scene=scene)
    path=tmp_path/'hand.ply'
    write_hand_ply(path,overlay.vertices_camera_m,overlay.faces,length_unit=unit)
    restored=trimesh.load(path,force='mesh',process=False)
    assert_allclose(restored.vertices,overlay.vertices_camera_m*scale,atol=1e-10,rtol=0)
    np.testing.assert_array_equal(restored.faces,overlay.faces)


def test_overlay_does_not_advance_dynamics(monkeypatch,setup_hand):
    grasp,scene=setup_hand
    def unexpected_step(*args,**kwargs):
        pytest.fail('静态叠加不应调用 mj_step')
    monkeypatch.setattr(mujoco,'mj_step',unexpected_step)
    place_fixed_hand(np.eye(4),grasp,scene=scene)
    assert scene.data.time==0
