"""整理已完成的双臂离线规划结果，并保存源码快照和逐文件摘要。"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

ROOT=Path('/home/ace/i10_rh56_refueling_ws')
OUT=ROOT/'artifacts/planning/dual_arm_mtc_20261027_001'
summary=json.loads((OUT/'installed_checks/summary.json').read_text())
audit=json.loads((OUT/'installed_checks/launch_scene_audit.json').read_text())
native=json.loads((OUT/'native_run/summary.json').read_text())
assert summary['installed_planning_exit']==0 and summary['installed_scene_exit']==0
assert summary['dense_fcl_check_passed'] and summary['endpoint_check_passed'] and summary['left_arm_fixed']
assert all(audit['robot_collision_geometry_participation'].values())
assert len(audit['robot_collision_geometry_participation'])==30
assert audit['intersection_separation_tests']['environment_intersection']['collision']
assert audit['intersection_separation_tests']['cap_intersection']['collision']
assert not audit['intersection_separation_tests']['separated']['collision']
assert not audit['initial_state']['collision'] and audit['initial_state']['satisfies_bounds']
assert native['candidates']['direction_180']['mode']=='mtc_failed_no_replay'
verification={'created_utc':datetime.now(timezone.utc).isoformat(),
    'installed_checks':summary,'native_run':native,
    'active_robot_collision_links':len(audit['robot_collision_geometry_participation']),
    'left_capture_fk_check':audit['left_capture_fk_check'],
    'unit_test_command':'python3 -m unittest discover -s tools/tests -p test_observed_mtc.py -v',
    'observed_mtc_unit_tests':{'exit_code':0,'passed':6},
    'dual_model_check':{'exit_code':0,'links':33,'joints':32,'independent_joints':18},
    'single_model_provenance_check':{'exit_code':0,'inputs':237,'generated_files':233},
    'collision_object_roundtrip':{'exit_code':0,'boxes':1502,'all_grid_indices_verified':True,
                                'all_box_rotations_verified':True,'disjoint_union_cells':80298},
    'robot_connections':0,'trajectory_execution':False}
(OUT/'validation_summary.json').write_text(json.dumps(verification,ensure_ascii=False,indent=2)+'\n')
files=[
 'config/active_point_cloud_scene.json','config/dual_arm_mount.json',
 'tools/place_scene_in_dual_arm.py','tools/run_observed_mtc_validation.py','tools/prepare_cap_approach.py',
 'tools/check_split_voxel_scene.py','tools/build_dual_arm_model.py','tools/tests/test_observed_mtc.py',
 'tools/view_voxel_map_placement.py','src/fuel_cap_task/observed_mtc.py','src/fuel_cap_task/voxel_octomap.py',
 'src/i10_rh56_moveit_config/src/observed_scene_loader.hpp',
 'src/i10_rh56_moveit_config/src/audit_observed_scene.cpp',
 'src/i10_rh56_moveit_config/src/plan_approach.cpp',
 'src/i10_rh56_moveit_config/launch/approach.launch.py',
 'src/i10_rh56_moveit_config/launch/observed_scene.launch.py',
 'src/i10_rh56_moveit_config/config/observed_dual_arm.rviz',
 'src/i10_rh56_moveit_config/config/provenance.json',
 'src/i10_rh56_moveit_config/config/generated_hashes.json',
 'src/i10_rh56_moveit_config/config/scene.json',
 'src/i10_rh56_moveit_config/dual_arm/robot.urdf',
 'src/i10_rh56_moveit_config/dual_arm/robot.srdf',
 'src/i10_rh56_moveit_config/dual_arm/kinematics.json',
 'src/i10_rh56_moveit_config/dual_arm/model.json',
 'src/i10_rh56_moveit_config/CMakeLists.txt','src/i10_rh56_moveit_config/package.xml',
 'src/i10_rh56_moveit_config/README.md','doc/assembly_contract.md','doc/point_cloud_voxelization.md']
for name in files:
    target=OUT/'source_snapshot'/name
    target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(ROOT/name,target)
    assert target.read_bytes()==(ROOT/name).read_bytes()
helpers=OUT/'validation_tools'
helpers.mkdir()
for source in ['/tmp/run_dual_final_checks.py','/tmp/finalize_dual_arm_archive.py',
               '/tmp/dual_arm_rviz_capture/capture.cpp','/tmp/dual_arm_rviz_capture/CMakeLists.txt']:
    shutil.copy2(source,helpers/Path(source).name)
binaries={}
for name in ['plan_approach','audit_observed_scene']:
    path=ROOT/'install/i10_rh56_moveit_config/lib/i10_rh56_moveit_config'/name
    binaries[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
(OUT/'binaries_sha256.json').write_text(json.dumps(binaries,indent=2)+'\n')
readme=f'''# 双臂、T 型架与实测体素的 MTC 接近

用户提供的 `src/i10_rh56_moveit_config/dual_arm/` 模型已用于实际 PlanningScene 与 MTC。地图采用 [双臂碰撞场景](../../perception/dual_arm_collision_scene_20261027_001/README.md)，原点云、2 mm 网格与组件划分不变，旧单臂结果保持原样。

## 结果

| 项目 | 实际结果 |
| --- | --- |
| 方向 0 | 已安装的标准 launch 完成绕障碍到预抓取位和 0.100 m 轴向接近；{summary['validated_waypoints']:,} 个加密路点通过 FCL 与限位检查 |
| 方向 0 手根终点误差 | 位置 {summary['endpoint_error_m']*1000:.4f} mm，姿态 {summary['endpoint_orientation_error_rad']:.8f} rad；这是模型计算误差 |
| 方向 180 | 预抓取有无碰撞 IK；采样抓取目标均含拇指中节与环境格相交，未生成完整接近轨迹 |
| 碰撞模型 | 33 个 link，其中 30 个含碰撞几何；逐 link 探针全部命中，含两臂、T 型架、右臂传感器、adapter、掌部和全部手指 |
| 环境与盖体对照 | 两种明确相交均检出，明确分离不报碰撞；起点无碰撞且不超限 |
| 左臂 | 全路径保持采集姿态不动；模型法兰 FK 与采集记录位置差 {audit['left_capture_fk_check']['position_error_m']:.3g} m |
| 场景一致性 | 已安装场景入口及 `get_planning_scene` 读回的模型、左臂关节和地图变换一致 |
| 坐标错配检查 | 将双臂请求改成单臂模型会被拒绝 |

完整数据见 [验证汇总](validation_summary.json)、[方向 0 成功路径](direction_000/trajectory.json)、[方向 180 场景诊断](direction_180/scene_audit.json)及 [RViz 实际画面](rviz_scene.png)。RRTConnect 采样路径会变化，`native_run/` 保留另一次成功路径及两方向完整日志；主目录方向 0 使用标准 launch 的结果。

## 坐标和固定状态

地图摆放使用采集记录，而不是旧单臂的盖体临时对齐：

```text
world_from_camera = world_from_right_base @ right_base_from_left_base @ left_base_from_camera_at_capture
world_from_octomap = world_from_camera @ camera_from_octomap
```

盖体参考中心约为 `[-0.3070661, -1.1266202, 0.8557977]` m。左臂固定为 10 月 26 日采集前的六轴角；该姿态是拍照姿态，未指定让位姿态。右臂是离线起点 `[0,-0.5,1.2,0,0,0]` rad，手保持 650 预型。模型类型、模型文件、地图和盖体摘要共同绑定，拒绝双臂世界坐标配旧单臂模型。相机外形尚未包含，完整现场障碍和实物安装精度仍需复核。

只有固定安装接口及架子支撑地面的接触例外；手指对盖体、环境以及两臂间的碰撞正常检查。路径为加密离散检查，不是连续碰撞证明。本次没有连接设备或执行轨迹，十秒显示动画不作为执行时序。

## 查看实际场景与成功轨迹

从项目根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
env -u CYCLONEDDS_URI ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=48 ROS_LOG_DIR=/tmp/dual_arm_scene_view_logs \\
  ros2 launch i10_rh56_moveit_config observed_scene.launch.py \\
  request_path:="$PWD/artifacts/planning/dual_arm_mtc_20261027_001/direction_000/request.json" \\
  output_path:="/tmp/dual_arm_scene_audit_view.json" \\
  trajectory_path:="$PWD/artifacts/planning/dual_arm_mtc_20261027_001/direction_000/trajectory.json" \\
  publish_seconds:=600.0 rviz:=true
```

该入口先复核模型和碰撞，再发布实际场景及成功轨迹。安装查看器 `view_dual_arm.py` 使用另一组历史显示姿态，单臂 MuJoCo 地图查看器不支持本双臂世界系。重新规划的命令见[项目操作说明](../../../doc/point_cloud_voxelization.md#实测场景接入-mtc)。

本机 WSLg 的 RViz 可显示并保存原生画面，但退出阶段发生段错误；本次截图进程退出码为 `{summary['rviz_capture_exit_code']}`，日志保存在 `installed_checks/capture.log`。规划、场景对照和服务读回在独立进程完成。

## 关键实现

- [地图摆放](../../../tools/place_scene_in_dual_arm.py)：校验采集、外参与两基座变换，将整图和独立盖体一起转入双臂世界系，保持体素格号。
- [请求绑定](../../../src/fuel_cap_task/observed_mtc.py)：写入双臂模型摘要、方向目标与固定左臂关节。
- [原生场景加载](../../../src/i10_rh56_moveit_config/src/observed_scene_loader.hpp)：同一加载函数供 MTC、相交/分离对照和 RViz 使用，核对左臂采集 FK。
- [路径检查](../../../src/i10_rh56_moveit_config/src/plan_approach.cpp)：仅右臂规划，加密后逐状态检查全模型碰撞、限位及左臂不动。

`source_snapshot/` 保存本次实现、模型参数与文档；`archive.json` 保存逐文件摘要。碰撞几何本体仍从已绑定的双臂场景和项目 mesh 读取，文件版本改变时应生成新场景与请求。
'''
(OUT/'README.md').write_text(readme)
hashes={str(p.relative_to(OUT)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(OUT.rglob('*')) if p.is_file() and p.name!='archive.json'}
(OUT/'archive.json').write_text(json.dumps({'created_utc':datetime.now(timezone.utc).isoformat(),
    'kind':'offline_dual_arm_mtc_approach','files_sha256':hashes},ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'archive':str(OUT),'files':len(hashes),'validated_waypoints':summary['validated_waypoints'],
                  'endpoint_error_mm':summary['endpoint_error_m']*1000},ensure_ascii=False),flush=True)
