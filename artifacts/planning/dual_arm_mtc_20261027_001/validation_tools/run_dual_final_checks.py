"""保存本次双臂离线实验的标准入口、场景服务和 RViz 复核结果。"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys

ROOT = Path('/home/ace/i10_rh56_refueling_ws')
OUT = ROOT/'artifacts/planning/dual_arm_mtc_20261027_001'
PACKAGE = ROOT/'install/i10_rh56_moveit_config'
SHARE = PACKAGE/'share/i10_rh56_moveit_config'
CHECKS = OUT/'installed_checks'
CHECKS.mkdir()
env = os.environ.copy()
env.pop('CYCLONEDDS_URI',None)
env.update(ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID='48',ROS_LOG_DIR=str(CHECKS/'ros_logs'),
           QT_QPA_PLATFORM='xcb',QT_X11_NO_MITSHM='1',LIBGL_ALWAYS_SOFTWARE='1')
env['AMENT_PREFIX_PATH']=str(PACKAGE)+':'+env.get('AMENT_PREFIX_PATH','')

def write(path,data):
    """保存可检查的 JSON，不将非有限数伪装成有效结果。"""
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

def run(command,log,timeout=240):
    """逐项记录退出码，标准入口失败时停止后续显示。"""
    with (CHECKS/log).open('w') as stream:
        return subprocess.run(command,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=timeout).returncode

# 原生首轮完整保留；标准入口使用归档中的请求并另存成功结果。
failed_prepare=OUT/'direction_000'
if failed_prepare.exists():
    shutil.move(str(failed_prepare),str(CHECKS/'system_python_missing_mujoco'))
for name in ['direction_000','direction_180']:
    shutil.copytree(OUT/'native_run'/name,OUT/name)
request_path=OUT/'direction_000/request.json'
trajectory_path=OUT/'direction_000/trajectory.json'
summary={'mode':'offline_dual_arm_mtc_validation','robot_connections':0,'trajectory_execution':False}
print('Checking installed planning launch',flush=True)
code=run(['ros2','launch','i10_rh56_moveit_config','approach.launch.py',
          'request_path:='+str(request_path),'output_path:='+str(trajectory_path)],'planning.log')
summary['installed_planning_exit']=code
if code: raise RuntimeError('标准规划入口失败，见 installed_checks/planning.log')
shutil.copy2(CHECKS/'planning.log',OUT/'direction_000/planning.log')
print('Checking installed scene launch',flush=True)
code=run(['ros2','launch','i10_rh56_moveit_config','observed_scene.launch.py',
          'request_path:='+str(request_path),'output_path:='+str(CHECKS/'launch_scene_audit.json')],
         'scene_launch.log')
summary['installed_scene_exit']=code
if code: raise RuntimeError('标准场景入口失败')
request=json.loads(request_path.read_text())
parameters=json.loads((OUT/'native_run/direction_000/audit_parameters.yaml').read_text())
params=parameters['/**']['ros__parameters']
params.update(config_dir=str(SHARE/'config'),request_path=str(request_path))
wrong=dict(request,robot_model_profile='single')
write(CHECKS/'wrong_profile_request.json',wrong)
wrong_params=json.loads(json.dumps(parameters))
wrong_params['/**']['ros__parameters'].update(request_path=str(CHECKS/'wrong_profile_request.json'),
                                            output_path=str(CHECKS/'wrong_profile_result.json'))
write(CHECKS/'wrong_profile_parameters.yaml',wrong_params)
code=run([str(PACKAGE/'lib/i10_rh56_moveit_config/audit_observed_scene'),'--ros-args','--params-file',
          str(CHECKS/'wrong_profile_parameters.yaml')],'wrong_profile.log')
wrong_result=json.loads((CHECKS/'wrong_profile_result.json').read_text())
summary.update(wrong_profile_exit=code,wrong_world_model_rejected=(code!=0 and
               'Single-arm model cannot load dual-arm world coordinates' in wrong_result.get('error','')))
assert summary['wrong_world_model_rejected']
params.update(output_path=str(CHECKS/'published_scene_audit.json'),publish_seconds=80.0,
              trajectory_path=str(trajectory_path))
write(CHECKS/'publisher_parameters.yaml',parameters)
viewer={'robot_description':(SHARE/'dual_arm/robot.urdf').read_text(),
        'robot_description_semantic':(SHARE/'dual_arm/robot.srdf').read_text(),
        'robot_description_kinematics':params['robot_description_kinematics']}
write(CHECKS/'viewer_parameters.yaml',{'/**':{'ros__parameters':viewer}})
print('Checking actual PlanningScene service and rendering',flush=True)
with (CHECKS/'publisher.log').open('w') as log:
    process=subprocess.Popen([str(PACKAGE/'lib/i10_rh56_moveit_config/audit_observed_scene'),
        '--ros-args','--params-file',str(CHECKS/'publisher_parameters.yaml')],env=env,stdout=log,stderr=subprocess.STDOUT)
    try:
        # rclpy 使用同一隔离域；读回序列化场景，检查左臂状态与整图位置。
        os.environ.update({k:env[k] for k in ['ROS_LOCALHOST_ONLY','ROS_DOMAIN_ID','ROS_LOG_DIR']})
        os.environ.pop('CYCLONEDDS_URI',None)
        import rclpy
        import numpy as np
        from rclpy.serialization import serialize_message
        from moveit_msgs.srv import GetPlanningScene
        rclpy.init()
        node=rclpy.create_node('dual_arm_scene_archive_check')
        client=node.create_client(GetPlanningScene,'get_planning_scene')
        assert client.wait_for_service(timeout_sec=25)
        call=GetPlanningScene.Request();call.components.components=0xffff
        future=client.call_async(call)
        rclpy.spin_until_future_complete(node,future,timeout_sec=20)
        response=future.result()
        assert response is not None
        scene=response.scene
        summary['published_robot_model_name']=scene.robot_model_name
        assert scene.robot_model_name=='i10_dual_arm_rh56'
        joints=dict(zip(scene.robot_state.joint_state.name,scene.robot_state.joint_state.position))
        summary['published_left_pose_matches_capture']=all(abs(joints[k]-v)<1e-12 for k,v in
                                                       request['fixed_left_joint_positions'].items())
        manifest=json.loads(Path(request['observed_scene']['manifest_path']).read_text())
        pose=scene.world.octomap.origin
        q=pose.orientation;x,y,z,w=q.x,q.y,q.z,q.w
        transform=np.eye(4)
        transform[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                          [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                          [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
        transform[:3,3]=[pose.position.x,pose.position.y,pose.position.z]
        summary['published_octomap_transform_verified']=bool(np.max(np.abs(transform-
                                                             manifest['world_from_octomap']))<1e-12)
        assert summary['published_left_pose_matches_capture'] and summary['published_octomap_transform_verified']
        data=serialize_message(scene)
        (CHECKS/'planning_scene.cdr').write_bytes(data)
        summary.update(scene_snapshot_sha256=hashlib.sha256(data).hexdigest(),snapshot_bytes=len(data))
        node.destroy_node();rclpy.shutdown()
        code=run(['/tmp/dual_arm_rviz_capture/build/capture',str(SHARE/'config/observed_dual_arm.rviz'),
                  str(OUT/'rviz_scene.png'),'--ros-args','--params-file',str(CHECKS/'viewer_parameters.yaml')],
                 'capture.log',90)
        summary.update(rviz_capture_exit_code=code,rviz_render_saved=(OUT/'rviz_scene.png').is_file())
        assert summary['rviz_render_saved']
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate();process.wait(timeout=10)
trajectory=json.loads(trajectory_path.read_text())
summary.update(validated_waypoints=sum(len(s['points']) for s in trajectory['stages']),
               endpoint_error_m=trajectory['endpoint_position_error_m'],
               endpoint_orientation_error_rad=trajectory['endpoint_orientation_error_rad'],
               left_arm_fixed=trajectory['fixed_left_arm_verified'],
               dense_fcl_check_passed=trajectory['dense_fcl_check_passed'],
               endpoint_check_passed=trajectory['endpoint_check_passed'],
               direction_180=json.loads((OUT/'native_run/summary.json').read_text())['candidates']['direction_180'])
write(CHECKS/'summary.json',summary)
print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
