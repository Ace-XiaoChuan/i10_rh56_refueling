"""在确认停稳后记录原始双目预览及前后 RPC 状态；不触发三维扫描。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time

ROOT = Path('/tmp/vizum-camera-setup')
PROJECT = Path('/home/ace/i10_rh56_refueling_ws')
stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
is_cloud = '--cloud' in sys.argv[1:]
OUT = Path('/tmp/i10-field') / (stamp + ('_cloud' if is_cloud else '_preview'))
OUT.mkdir(parents=True, exist_ok=False)
record = {
    'purpose': '一次静态扫描及前后RPC状态' if is_cloud else '原始双目二维预览，不作为运动目标',
    'valid_for_motion_target': False,
    'camera_ip': '192.168.1.20', 'camera_serial': 'Ik256431',
    'robot_ip': '192.168.1.32',
    'time_meaning': 'UTC 主机调用起止时间及主机单调时间，不是相机曝光时间或控制器采样时间',
    'operator_report': '用户确认已大致对准目标且已停稳，要求观察期间保持姿态',
    'point_cloud': None,
}

def run(name, command, cwd, timeout):
    item = {'start_utc': datetime.now(timezone.utc).isoformat(),
            'start_monotonic_ns': time.monotonic_ns(), 'command': command}
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = str(ROOT / 'sdk/SDK/VzNLSDK/Linux/x64')
    try:
        p = subprocess.run(command, cwd=cwd, env=env, capture_output=True,
                           text=True, errors='replace', timeout=timeout)
        item.update(returncode=p.returncode, stdout=p.stdout, stderr=p.stderr)
    except subprocess.TimeoutExpired as error:
        item.update(returncode=None, timeout=True, error=str(error))
        raise
    finally:
        item.update(end_utc=datetime.now(timezone.utc).isoformat(),
                    end_monotonic_ns=time.monotonic_ns())
        record[name] = item
        (OUT / f'{name}.json').write_text(json.dumps(item, ensure_ascii=False, indent=2)+'\n')
    print(name, 'returncode=', p.returncode, flush=True)
    print(p.stdout, flush=True)
    if p.returncode != 0:
        raise RuntimeError(name + ' failed')
    return p.stdout

def state(text):
    data = {}
    for label, key in [('关节角_rad', 'q_rad'), ('法兰位姿_控制器原值', 'flange_pose_m_rpy_rad')]:
        match = re.search(re.escape(label) + r' = (\[[^\n]+\])', text)
        if not match:
            raise RuntimeError('missing ' + label)
        values = json.loads(match[1])
        if len(values) != 6 or not all(math.isfinite(x) for x in values):
            raise RuntimeError('invalid ' + label)
        data[key] = values
    data['steady'] = '当前静止 = true' in text
    data['simulation'] = '仿真模式 = true' in text
    if not data['steady'] or data['simulation']:
        raise RuntimeError('机器人未返回真机静止状态')
    x,y,z,r,p,w = data['flange_pose_m_rpy_rad']
    cr,sr,cp,sp,cw,sw = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(w),math.sin(w)
    data['base_from_flange'] = [
        [cw*cp, cw*sp*sr-sw*cr, cw*sp*cr+sw*sr, x],
        [sw*cp, sw*sp*sr+cw*cr, sw*sp*cr-cw*sr, y],
        [-sp, cp*sr, cp*cr, z], [0,0,0,1],
    ]
    data['pose_convention'] = '固定轴 XYZ RPY，R=Rz(yaw)Ry(pitch)Rx(roll)，平移 m'
    data['joint_order'] = ['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint']
    return data

try:
    calibration = PROJECT/'config/fuel_cap_camera_calibration.json'
    shutil.copy2(calibration, OUT/calibration.name)
    record['calibration_sha256'] = hashlib.sha256(calibration.read_bytes()).hexdigest()
    command = [str(PROJECT/'tools/aubo_sdk_probe/build/aubo_sdk_probe'), '--host', '192.168.1.32']
    before = run('robot_before', command, PROJECT, 25)
    record['state_before'] = state(before)
    camera_option = '--cloud' if is_cloud else ('--snapshot-full' if '--full-frame' in sys.argv[1:] else ('--snapshot-exposure' if '--temporary-exposure' in sys.argv[1:] else '--snapshot'))
    record['camera_option'] = camera_option
    capture = ''
    camera_error = None
    try:
        capture = run('camera_cloud' if is_cloud else 'camera_preview', [str(ROOT/'vizum-inspect'), camera_option], ROOT, 60 if is_cloud else 30)
    except Exception as error:
        camera_error = error
    finally:
        after = run('robot_after', command, PROJECT, 25)
        record['state_after'] = state(after)
    if camera_error is not None:
        raise camera_error
    if camera_option in ('--snapshot-exposure','--snapshot-full'):
        restore = re.search(r'exposure_restore=(\d+) verify=(\d+) mode=(\d+) time_raw=(\d+) expected_mode=(\d+) expected_time=(\d+)', capture)
        if not restore or restore.group(1,2) != ('0','0') or restore.group(3,4) != restore.group(5,6):
            raise RuntimeError('未确认曝光已恢复，需现场核对配置')
        record['temporary_exposure_restored_and_readback_verified'] = True
    if camera_option == '--snapshot-full':
        if 'roi_restore=0 verify=0 matches_original=1' not in capture:
            raise RuntimeError('未确认ROI已恢复，需现场核对配置')
        record['temporary_roi_restored_and_readback_verified'] = True
    for value in re.findall(r'^saved=(.+)$', capture, re.M):
        source = Path(value)
        if source.is_file() and source.parent.parent == ROOT:
            shutil.copy2(source, OUT/source.name)
    if is_cloud:
        point_path = OUT/'cloud_sdk_native.pcd'
        record['point_cloud'] = {
            'path': point_path.name,
            'sha256': hashlib.sha256(point_path.read_bytes()).hexdigest(),
            'frame': 'camera_sdk_native', 'xyz_unit': 'SDK native; verification pending',
            'dtype': 'float32 XYZ', 'mirror': 'none', 'rgb_map': False,
        }
    q0,q1 = record['state_before']['q_rad'],record['state_after']['q_rad']
    record['endpoint_max_abs_joint_delta_rad'] = max(abs(a-b) for a,b in zip(q0,q1))
    record['static_assessment'] = '用户报告停稳，前后RPC均返回静止；没有连续状态监测，不将两端快照解释为全程运动证明'
    record['capture_complete'] = True
except Exception as error:
    record['capture_complete'] = False
    record['error'] = str(error)
    print('ERROR:', error, flush=True)
finally:
    (OUT/'observation.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
    print('OUTPUT_DIR='+str(OUT), flush=True)
if not record.get('capture_complete'):
    raise SystemExit(1)
