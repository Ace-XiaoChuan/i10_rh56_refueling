"""归档本次点云、配对状态和预览；不覆盖既有实验目录。"""
from pathlib import Path
import hashlib
import json
import shutil

root = Path('/home/ace/i10_rh56_refueling_ws')
source = Path('/tmp/i10-field/20261026T055151_608691Z_cloud')
alignment = Path('/tmp/i10-field/20261026T054645_133953Z_preview')
destination = root/'artifacts/hardware/vizum_i10_capture_20261026T055151Z'
destination.mkdir(parents=True, exist_ok=False)

for item in source.iterdir():
    if item.is_file():
        shutil.copy2(item, destination/item.name)
preview = destination/'alignment_preview'
preview.mkdir()
for name in ['left.png','right.png','observation.json','robot_before.json','robot_after.json','camera_preview.json']:
    shutil.copy2(alignment/name, preview/name)

code = destination/'source_snapshot'
code.mkdir()
for name in ['inspect-camera.cpp','acquire-preview.py','discover.cpp','archive-capture.py']:
    shutil.copy2(Path('/tmp/vizum-camera-setup')/name, code/name)

observation = json.loads((destination/'observation.json').read_text())
before,after = observation['state_before'], observation['state_after']
camera_text = observation['camera_cloud']['stdout']
final_text = json.loads((destination/'camera_configuration_after.json').read_text())['stdout']
assert 'scan_result=0 point_count=268852' in camera_text
assert before['steady'] and after['steady']
assert 'exposure_query=0 mode=0 time_raw=200' in final_text
assert 'roi_query=0 left=0,256,0,1920 right=960,1216,0,1920' in final_text
assert 'capture_status_query=0 capturing=0' in final_text
assert 'detect_status_query=0 detecting=0' in final_text
assert 'close=0' in final_text

manifest = {
    'description': 'Ik256431末端相机一次真实静态扫描，绑定采集前后机器人RPC状态及外参文件',
    'capture_directory_time_zone': 'UTC',
    'camera_ip': '192.168.1.20', 'camera_serial': 'Ik256431',
    'robot_ip': '192.168.1.32', 'robot_model': 'aubo_i10_JY',
    'sdk_distribution_name': 'DetectLaser_3.6.20.119_Windows&Linux&ARM',
    'sdk_runtime_reported_version': '3.6.20.112',
    'point_count':268852,
    'point_cloud': 'cloud_sdk_native.pcd',
    'point_cloud_frame': 'camera_sdk_native; no Y flip; no RGB map',
    'length_unit_note': 'SDK原值保留；按项目已确认的毫米单位解释，数值量级一致；本次未独立测距标定',
    'legacy_pcd_axis_convention_verified': False,
    'alignment_preview_note': '同一保持姿态期间的较早二维预览；曝光临时10000、全画幅ROI；不是此次扫描同步RGB，不提供像素映射',
    'host_time_note': '时间是主机接口调用起止时间，不是控制器采样或相机曝光硬件时间',
    'endpoint_max_abs_joint_delta_rad': observation['endpoint_max_abs_joint_delta_rad'],
    'camera_configuration_after_verified': {'exposure_raw':200,'left_roi':[0,256,0,1920], 'right_roi':[960,1216,0,1920],'capture_status':False,'auto_detect_status':False},
    'extrinsic_file':'fuel_cap_camera_calibration.json',
    'extrinsic_sha256': observation['calibration_sha256'],
    'extrinsic_physical_accuracy_verified':False,
    'valid_for_motion_target':False,
    'robot_motion_commands_sent_by_agent':False,
    'temporary_libraries_location':'/tmp/vizum-camera-setup/sdk',
    'files_sha256':{},
}
for path in sorted(destination.rglob('*')):
    if path.is_file():
        manifest['files_sha256'][str(path.relative_to(destination))] = hashlib.sha256(path.read_bytes()).hexdigest()
(destination/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
print('ARCHIVE='+str(destination))
print('FILES='+str(len(manifest['files_sha256'])))
print('MAX_JOINT_DELTA_RAD='+str(manifest['endpoint_max_abs_joint_delta_rad']))
