"""核对离线定位结果并归档自有源码；保留原始采集与既有结果不变。"""
from pathlib import Path
import ast
import hashlib
import json
import runpy
import shutil
import sys

import numpy as np

ROOT=Path('/home/ace/i10_rh56_refueling_ws')
sys.path.insert(0,str(ROOT/'src'))
from fuel_cap_task.pcd_point_cloud import load_pcd

folder=ROOT/'artifacts/perception/cap_observation_20261026_001'
crop=json.loads((folder/'crop/crop.json').read_text())
geometry=json.loads((folder/'geometry/result.json').read_text())
overlay=json.loads((folder/'overlay/overlay.json').read_text())
axes=json.loads((folder/'axis_conventions/result.json').read_text())
raw=load_pcd(crop['source_path'])
selected=load_pcd(folder/'crop/cap_crop.pcd')
indices=np.load(folder/'crop/source_indices.npz')['source_indices']
assert raw.sha256==crop['source_sha256']
assert selected.sha256==crop['crop_sha256']==geometry['input_sha256']
assert len(indices)==5742 and np.all(np.diff(indices)>0)
assert np.array_equal(raw.xyz_raw[indices],selected.xyz_raw)
assert len(selected.xyz_raw)==geometry['diagnostics']['point_count']
assert geometry['diagnostics']['face_count']+geometry['diagnostics']['bridge_count']==5742
assert geometry['robot_state_paired'] and geometry['motion_target_valid'] is False
assert axes['selected_hypothesis'] is None and axes['motion_target_valid'] is False
for hypothesis in axes['hypotheses']:
    assert hypothesis['motion_target_valid'] is False
    for pose in hypothesis['pose_candidates_base_model_based']:
        r=np.asarray(pose['base_from_cap_grip_center'])[:3,:3]
        assert np.allclose(r.T@r,np.eye(3),atol=1e-10)
        assert np.isclose(np.linalg.det(r),1,atol=1e-10)
binding=geometry['observation_binding']
assert binding['crop_to_source_max_nearest_error_m']==0
before=np.array(binding['robot_before']['base_from_flange'])
calib_path=Path(binding['observation_path']).parent/'fuel_cap_camera_calibration.json'
calib=json.loads(calib_path.read_text())
assert hashlib.sha256(calib_path.read_bytes()).hexdigest()==binding['calibration_sha256']
expected=before@np.array(calib['extrinsic']['flange_from_camera_matrix'])
assert np.allclose(expected,binding['base_from_camera'],atol=1e-12,rtol=0)
assert overlay['physics_steps']==0 and overlay['simulation_time_s']==0
assert overlay['relative_position_error_m']<1e-12
assert overlay['source_geometry_motion_target_valid'] is False
assert overlay['source_sha256']['geometry_result']==hashlib.sha256((folder/'geometry/result.json').read_bytes()).hexdigest()
assert all(v['valid'] for v in geometry['subsampling']['samples'])

metadata=runpy.run_path('/home/ace/robotics-from-zero-tutorial/scripts/update_doc_metadata.py')['updated_text']
for rel in ['doc/assembly_contract.md','doc/cap_geometry.md']:
    content=(ROOT/rel).read_text()
    assert metadata(content)==content,rel+' metadata is stale'
    # 本次编辑的文档链接只核对本地文件；外部链接保持来源原值。
    import re
    for target in re.findall(r'\[[^\]]*\]\(([^)]+)\)',content):
        target=target.split('#',1)[0]
        if not target or re.match(r'^[a-z]+:',target):continue
        assert ((ROOT/rel).parent/target).resolve().exists(),rel+' broken link '+target

files=['src/fuel_cap_task/cap_geometry.py','src/fuel_cap_task/cap_observation.py',
       'src/fuel_cap_task/pcd_point_cloud.py','src/fuel_cap_task/cap_hand_overlay.py',
       'tools/crop_cap_observation.py','tools/estimate_cap_geometry.py',
       'tools/compare_cap_axis_conventions.py','tools/preview_cap_cloud_grasp.py',
       'tests/test_cap_observation.py','tests/test_cap_geometry.py','tools/tests/test_pcd_point_cloud.py']
code=folder/'source_snapshot'
code.mkdir(exist_ok=False)
for rel in files:
    path=ROOT/rel
    ast.parse(path.read_text(),filename=rel)
    target=code/rel
    target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(path,target)
shutil.copy2(__file__,code/'finalize_localization.py')
for name,digest in geometry['implementation_sha256'].items():
    candidates=list(code.rglob(name))
    assert len(candidates)==1 and hashlib.sha256(candidates[0].read_bytes()).hexdigest()==digest,name

samples=geometry['subsampling']['samples']
validation={'tests':{'command':'python -B -m pytest tests/test_cap_observation.py tests/test_cap_geometry.py tools/tests/test_pcd_point_cloud.py -q',
                     'passed':34,'result_recorded_from_actual_tool_output':True},
    'crop_rows_exactly_match_source':True,'crop_point_count':5742,
    'crop_source_file_sha256_verified':True,'calibration_version_verified':True,
    'base_transform_chain_verified':True,'right_handed_pose_candidates_verified':True,
    'static_overlay_only':True,'geometry_to_overlay_binding_verified':True,
    'local_links_checked':True,'edited_document_metadata_checked':True,
    'subsampling_80_percent_trials':len(samples),
    'maximum_subsampling_surface_reference_delta_mm':max(v['surface_reference_delta_m'] for v in samples)*1000,
    'maximum_subsampling_normal_angle_deg':max(v['normal_angle_deg'] for v in samples),
    'interpretation':'文件、数值与软件流程校验；不将拟合残差或随机删点敏感性解释为实物绝对精度',
    'robot_motion_commands_sent':False}
(folder/'validation.json').write_text(json.dumps(validation,ensure_ascii=False,indent=2)+'\n')
manifest={'description':'配对停稳观测的盖体辅助提取、几何估计、基座候选及静态固定手叠加',
    'observation':binding['observation_path'],'observation_sha256':binding['observation_sha256'],
    'old_export_y_mirror_setting':'user_uncertain','selected_axis_hypothesis':None,
    'motion_target_valid':False,'robot_motion_commands_sent':False,
    'primary_result':'geometry/result.json','axis_hypotheses_result':'axis_conventions/result.json',
    'crop_preview':'geometry/preview.png','fixed_hand_preview':'overlay/preview.png',
    'length_unit_json':'m','exported_pcd_and_ply_unit':'mm',
    'files_sha256':{str(p.relative_to(folder)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(folder.rglob('*')) if p.is_file()}}
(folder/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
for name,digest in manifest['files_sha256'].items():
    assert hashlib.sha256((folder/name).read_bytes()).hexdigest()==digest
print(json.dumps({'artifact':str(folder),'archived_files':len(manifest['files_sha256']),
    'crop_count':5742,'observed_diameter_mm':geometry['boundary_radius_m']*2000,
    'base_positions_mm':[[x*1000 for x in h['pose_candidates_base_model_based'][0]['position_m']] for h in axes['hypotheses']],
    'axis_hypothesis_position_difference_mm':axes['position_distance_between_hypotheses_m']*1000,
    'validation':validation},ensure_ascii=False,indent=2))
