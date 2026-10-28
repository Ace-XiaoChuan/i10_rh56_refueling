# 双臂、T 型架与实测体素的 MTC 接近

用户提供的 `src/i10_rh56_moveit_config/dual_arm/` 模型已用于实际 PlanningScene 与 MTC。地图采用 [双臂碰撞场景](../../perception/dual_arm_collision_scene_20261027_001/README.md)，原点云、2 mm 网格与组件划分不变，旧单臂结果保持原样。

## 结果

| 项目 | 实际结果 |
| --- | --- |
| 方向 0 | 已安装的标准 launch 完成绕障碍到预抓取位和 0.100 m 轴向接近；5,762 个加密路点通过 FCL 与限位检查 |
| 方向 0 手根终点误差 | 位置 0.0853 mm，姿态 0.00008055 rad；这是模型计算误差 |
| 方向 180 | 预抓取有无碰撞 IK；采样抓取目标均含拇指中节与环境格相交，未生成完整接近轨迹 |
| 碰撞模型 | 33 个 link，其中 30 个含碰撞几何；逐 link 探针全部命中，含两臂、T 型架、右臂传感器、adapter、掌部和全部手指 |
| 环境与盖体对照 | 两种明确相交均检出，明确分离不报碰撞；起点无碰撞且不超限 |
| 左臂 | 全路径保持采集姿态不动；模型法兰 FK 与采集记录位置差 6.97e-13 m |
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
env -u CYCLONEDDS_URI ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=48 ROS_LOG_DIR=/tmp/dual_arm_scene_view_logs \
  ros2 launch i10_rh56_moveit_config observed_scene.launch.py \
  request_path:="$PWD/artifacts/planning/dual_arm_mtc_20261027_001/direction_000/request.json" \
  output_path:="/tmp/dual_arm_scene_audit_view.json" \
  trajectory_path:="$PWD/artifacts/planning/dual_arm_mtc_20261027_001/direction_000/trajectory.json" \
  publish_seconds:=600.0 rviz:=true
```

该入口先复核模型和碰撞，再发布实际场景及成功轨迹。安装查看器 `view_dual_arm.py` 使用另一组历史显示姿态，单臂 MuJoCo 地图查看器不支持本双臂世界系。重新规划的命令见[项目操作说明](../../../doc/point_cloud_voxelization.md#实测场景接入-mtc)。

本机 WSLg 的 RViz 可显示并保存原生画面，但退出阶段发生段错误；本次截图进程退出码为 `-11`，日志保存在 `installed_checks/capture.log`。规划、场景对照和服务读回在独立进程完成。

## 关键实现

- [地图摆放](../../../tools/place_scene_in_dual_arm.py)：校验采集、外参与两基座变换，将整图和独立盖体一起转入双臂世界系，保持体素格号。
- [请求绑定](../../../src/fuel_cap_task/observed_mtc.py)：写入双臂模型摘要、方向目标与固定左臂关节。
- [原生场景加载](../../../src/i10_rh56_moveit_config/src/observed_scene_loader.hpp)：同一加载函数供 MTC、相交/分离对照和 RViz 使用，核对左臂采集 FK。
- [路径检查](../../../src/i10_rh56_moveit_config/src/plan_approach.cpp)：仅右臂规划，加密后逐状态检查全模型碰撞、限位及左臂不动。

`source_snapshot/` 保存本次实现、模型参数与文档；`archive.json` 保存逐文件摘要。碰撞几何本体仍从已绑定的双臂场景和项目 mesh 读取，文件版本改变时应生成新场景与请求。
