#!/usr/bin/env python3
"""按已记录安装链和采集姿态，把既有碰撞组件放入双臂世界系。"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from fuel_cap_task.observed_mtc import sha256
from fuel_cap_task.voxel_octomap import cap_collision_object, load_occupied_voxels, rigid_matrix
from export_voxel_octomap import write_json
from build_dual_arm_model import check


def main():
    """不重建占据格；只替换整图变换及独立盖体的世界系 box 位姿。"""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args()
    config=json.loads((ROOT/'config/active_point_cloud_scene.json').read_text())
    # 保留原相机系组件作为重放来源，不随当前默认规划模型的切换而递归使用新场景。
    split=ROOT/config['collision_scene'].get('single_arm_mtc_manifest',config['collision_scene']['mtc_manifest'])
    original=json.loads(split.read_text())
    if original.get('robot_model_profile','single')!='single':
        raise ValueError('需要原单臂拆分归档作为相机系数据来源')
    output=(args.output_dir or (ROOT/config['collision_scene']['dual_arm_mtc_manifest']).parent).resolve()
    if output.exists():parser.error('输出目录已存在，请指定新目录')
    check()
    model_dir=ROOT/'src/i10_rh56_moveit_config/dual_arm'
    metadata=json.loads((model_dir/'model.json').read_text())
    candidates_path=ROOT/config['grasp_candidates']
    candidates=json.loads(candidates_path.read_text())
    geometry_path=Path(candidates['geometry_result'])
    if sha256(geometry_path)!=candidates['geometry_result_sha256']:
        raise ValueError('盖体几何来源已改变')
    geometry=json.loads(geometry_path.read_text())
    paired=geometry['observation_binding']
    capture_path=ROOT/config['source_capture']
    capture=json.loads(capture_path.read_text())
    if (sha256(capture_path)!=paired['observation_sha256'] or not capture['capture_complete']
            or capture['state_before']!=paired['robot_before'] or not capture['state_before']['steady']
            or capture['state_before']['simulation']):
        raise ValueError('采集姿态与盖体估计绑定不同或静止状态无效')
    calibration_path=capture_path.parent/'fuel_cap_camera_calibration.json'
    if sha256(calibration_path)!=capture['calibration_sha256']:
        raise ValueError('采集时相机外参摘要改变')
    calibration=json.loads(calibration_path.read_text())
    left_camera=rigid_matrix(capture['state_before']['base_from_flange']) @ rigid_matrix(
        calibration['extrinsic']['flange_from_camera_matrix'])
    np.testing.assert_allclose(left_camera,paired['base_from_camera'],atol=1e-12,rtol=0)
    mount=metadata['geometry']
    world_right=rigid_matrix(mount['world_from_right_base'])
    right_left=rigid_matrix(mount['right_base_from_left_base'])
    world_camera=world_right @ right_left @ left_camera
    np.testing.assert_allclose(world_right @ right_left,mount['world_from_left_base'],atol=1e-12,rtol=0)
    camera_map=rigid_matrix(original['camera_from_octomap'])
    indices,centers,size,origin,cap_mask=load_occupied_voxels(ROOT/config['octomap']['source_voxels'])
    with np.load(split.parent/original['voxel_components_path'],allow_pickle=False) as arrays:
        np.testing.assert_array_equal(indices,arrays['indices_xyz'])
        components=arrays['components'].copy()
    cap=cap_collision_object(centers[components==1],size,world_camera,
                             object_id=original['cap_object_id'],frame_id='world')
    selected=next(c for c in candidates['candidates'] if c['candidate_id']==original['alignment_candidate_id'])
    world_cap=world_camera @ rigid_matrix(selected['camera_from_cap'])
    for candidate in candidates['candidates']:
        np.testing.assert_allclose(left_camera @ rigid_matrix(candidate['camera_from_hand']),
                                   candidate['left_base_from_hand'],atol=1e-10,rtol=0)
    output.mkdir(parents=True)
    for name in [original['map_path'],original['voxel_components_path']]:
        if sha256(split.parent/name)!=original['map_sha256' if name==original['map_path'] else 'voxel_components_sha256']:
            raise ValueError('源碰撞组件摘要改变')
        shutil.copyfile(split.parent/name,output/name)
    shutil.copytree(split.parent/original['display_scene_dir'],output/'split_display')
    write_json(output/'cap_collision_object.json',cap)
    write_json(output/'scene_config.json',config)
    left_names=capture['state_before']['joint_order']
    left_q=dict(zip(('left_'+name for name in left_names),capture['state_before']['q_rad']))
    robot_binding={'profile':'dual_arm','files_sha256':{name:sha256(model_dir/name) for name in
                   ['robot.urdf','robot.srdf','kinematics.json','model.json']}}
    validation={**original['validation'],'placement_chain_verified':True,'source_octomap_byte_identical':True,
                'candidate_left_base_transform_verified':True,'left_arm_pose_source':'capture_state_before',
                'world_from_camera_determinant':float(np.linalg.det(world_camera[:3,:3])),
                'robot_model_profile':'dual_arm','moveit_scene_loaded':False}
    placement={'schema_version':2,'mode':'paired_capture_dual_arm_world','planning_frame':'world',
               'camera_frame':original['camera_frame'],'world_from_cap':world_cap.tolist(),
               'preview_arm_joints_rad':json.loads((ROOT/config['octomap']['placement_config']).read_text())['preview_arm_joints_rad'],
               'fixed_left_joint_positions':left_q,'valid_for_hardware':False,
               'description':'左臂保持本次拍照姿态，按安装链恢复地图位置；让位姿态和相机外形仍待补齐。'}
    write_json(output/'placement_config.json',placement)
    sources=[split,candidates_path,geometry_path,capture_path,calibration_path,model_dir/'model.json',ROOT/'config/dual_arm_mount.json',ROOT/config['octomap']['source_voxels']]
    manifest={**original,'created_utc':datetime.now(timezone.utc).isoformat(),'mode':placement['mode'],
              'robot_model_profile':'dual_arm','robot_model_binding':robot_binding,
              'world_from_camera':world_camera.tolist(),'world_from_octomap':(world_camera@camera_map).tolist(),
              'world_from_cap':world_cap.tolist(),'right_base_from_left_base':right_left.tolist(),
              'world_from_right_base':world_right.tolist(),'left_base_from_camera':left_camera.tolist(),
              'world_from_left_flange_at_capture':(world_right@right_left@rigid_matrix(capture['state_before']['base_from_flange'])).tolist(),
              'fixed_left_joint_positions':left_q,'left_arm_pose_source':'capture_state_before',
              'cap_object_sha256':sha256(output/'cap_collision_object.json'),
              'cap_candidates_world':[{'candidate_id':c['candidate_id'],'world_from_cap':(world_camera@rigid_matrix(c['camera_from_cap'])).tolist()} for c in candidates['candidates']],
              'placement_config_sha256':sha256(output/'placement_config.json'),'scene_config_sha256':sha256(output/'scene_config.json'),
              'sources':{str(p.resolve()):sha256(p) for p in sources},'source_split_scene':str(split),
              'source_octomap_scene':original['source_octomap_scene'],'validation':validation,
              'implementation_sha256':{'tools/place_scene_in_dual_arm.py':sha256(Path(__file__))},
              'world_center_bounds_m':[(centers@world_camera[:3,:3].T+world_camera[:3,3]).min(0).tolist(),
                                       (centers@world_camera[:3,:3].T+world_camera[:3,3]).max(0).tolist()]}
    write_json(output/'validation.json',validation)
    write_json(output/'octomap_scene.json',manifest)
    print(json.dumps({'output_dir':str(output),'world_from_camera':world_camera.tolist(),
                      'cap_position_world_m':world_cap[:3,3].tolist(),'fixed_left_joint_positions':left_q},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
