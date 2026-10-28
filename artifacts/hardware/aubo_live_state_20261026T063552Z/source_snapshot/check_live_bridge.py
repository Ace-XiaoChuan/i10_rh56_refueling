"""在独立 ROS domain 中测量只读 RTDE 状态桥，不接运动控制器。"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import re
import signal
import subprocess
import time
import rclpy
from sensor_msgs.msg import JointState

root=Path('/tmp/i10-live-state-20261026')
out=root/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
out.mkdir(parents=True,exist_ok=False)
project=Path('/home/ace/i10_rh56_refueling_ws')
topic='/aubo/diagnostics/joint_states'
names=['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint']
record={'start_utc':datetime.now(timezone.utc).isoformat(),'robot_ip':'192.168.1.32','requested_hz':50.0,
        'topic':topic,'ROS_DOMAIN_ID':os.environ.get('ROS_DOMAIN_ID'),'RMW_IMPLEMENTATION':os.environ.get('RMW_IMPLEMENTATION'),
        'purpose':'只读静态联机检查；不验证示教移动、断流或MoveIt状态监视器',
        'frames':[],'robot_motion_commands_sent':False}
p=subprocess.run([str(project/'tools/aubo_sdk_probe/build/aubo_sdk_probe'),'--host','192.168.1.32'],cwd=project,
                 capture_output=True,text=True,timeout=25)
record['rpc_reference']={'returncode':p.returncode,'stdout':p.stdout,'stderr':p.stderr}
match=re.search(r'关节角_rad = (\[[^\n]+\])',p.stdout)
rpc_q=json.loads(match[1]) if p.returncode==0 and match else None
record['rpc_static_reference']='当前静止 = true' in p.stdout
rclpy.init()
node=rclpy.create_node('i10_live_bridge_diagnostic')
def receive(msg):
    now=time.time_ns()
    stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
    record['frames'].append({'host_unix_ns':now,'host_monotonic_ns':time.monotonic_ns(),
        'stamp_ns':stamp,'age_s':(now-stamp)/1e9,'name':list(msg.name),'position':list(msg.position),
        'velocity':list(msg.velocity),'effort':list(msg.effort)})
subscription=node.create_subscription(JointState,topic,receive,10)
command=[str(root/'build/joint_state_bridge'),'--ros-args','-p','host:=192.168.1.32',
         '-p','rate_hz:=50.0','-p','joint_states_topic:='+topic]
record['bridge_command']=command
with (out/'bridge.log').open('w') as log:
    bridge=subprocess.Popen(command,cwd=out,stdout=log,stderr=subprocess.STDOUT)
    try:
        started=time.monotonic()
        while time.monotonic()-started<12.0:
            rclpy.spin_once(node,timeout_sec=.1)
            if bridge.poll() is not None:break
    finally:
        if bridge.poll() is None:
            bridge.send_signal(signal.SIGINT)
            try:bridge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bridge.terminate()
                try:bridge.wait(timeout=3)
                except subprocess.TimeoutExpired:bridge.kill();bridge.wait()
        record['bridge_returncode']=bridge.returncode
        node.destroy_node()
        rclpy.shutdown()
frames=record['frames']
summary={'frame_count':len(frames),'bridge_returncode':bridge.returncode}
if frames:
    span=(frames[-1]['host_monotonic_ns']-frames[0]['host_monotonic_ns'])/1e9
    summary['received_hz']=(len(frames)-1)/span if span>0 else None
    summary['all_names_match']=all(f['name']==names for f in frames)
    summary['all_positions_six_finite']=all(len(f['position'])==6 and all(math.isfinite(v) for v in f['position']) for f in frames)
    summary['all_velocities_empty_or_six_finite']=all(len(f['velocity']) in (0,6) and all(math.isfinite(v) for v in f['velocity']) for f in frames)
    summary['max_host_age_s']=max(f['age_s'] for f in frames)
    summary['max_receive_gap_s']=max((b['host_monotonic_ns']-a['host_monotonic_ns'])/1e9 for a,b in zip(frames,frames[1:])) if len(frames)>1 else None
    summary['first_q_rad']=frames[0]['position']
    summary['last_q_rad']=frames[-1]['position']
    if rpc_q and record['rpc_static_reference']:
        summary['first_frame_max_abs_difference_from_rpc_rad']=max(abs(a-b) for a,b in zip(rpc_q,frames[0]['position']))
summary['static_link_check_passed']=bool(frames and summary['all_names_match'] and summary['all_positions_six_finite'])
record['summary']=summary
record['end_utc']=datetime.now(timezone.utc).isoformat()
record['source_sha256']=hashlib.sha256((project/'src/aubo_joint_state_bridge/src/joint_state_bridge.cpp').read_bytes()).hexdigest()
(out/'result.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(summary,ensure_ascii=False,indent=2))
print('OUTPUT_DIR='+str(out))
print((out/'bridge.log').read_text())
raise SystemExit(0 if summary['static_link_check_passed'] else 1)
