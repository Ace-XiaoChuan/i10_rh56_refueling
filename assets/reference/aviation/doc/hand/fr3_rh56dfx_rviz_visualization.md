# FR3 + RH56DFX RViz 模型验证记录

## 1. 背景

本记录来自 `/home/ace/franka_ros2_ws/src/franka_description` 中的一次
FR3 + RH56DFX 右手 RViz 可视化验证。

目标不是接入真实 RH56DFX 控制，也不是完成 MoveIt 手部规划，而是先确认：

- FR3 模型可以挂载 RH56DFX 右手外观模型。
- `robot_state_publisher` / `joint_state_publisher_gui` / RViz 的 TF 链路可用。
- RH56DFX mesh、joint tree、默认姿态在 RViz 中能稳定显示。
- 当前问题和后续集成边界被记录下来，避免混入 MVP-1 fake 控制链路。

## 2. 当前有效结论

### 2.1 DDS 结论

本次调试中，严重卡顿主要不是 mesh 本身造成的。

现象：

- `/joint_states` 约 10 Hz，滑条位置会立刻变化。
- `/tf` 和 RViz 响应曾经延迟一两分钟。
- 切换到 Fast DDS 后，详细 RH56DFX 模型可以正常响应。

推荐 RViz 验证环境：

```bash
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

CycloneDDS 配置导致的 `/tf` 延迟应作为本机环境问题记录，不应误判为
RH56DFX mesh 过重。

### 2.2 RH56DFX 模型结论

已在 `franka_description` 中整理出正式 end-effector ID：

```text
rh56dfx_right
```

调试过程中曾使用轻量级手模型排查性能，但最终确认不再需要保留轻量模型。
当前应以详细 RH56DFX mesh 版本作为 RViz 验证基线。

### 2.3 joint_state_publisher_gui 默认姿态

`joint_state_publisher_gui` 默认/Center 规则会让部分 RH56DFX 关节处在不一致姿态：

- 食指因为下限大于 0，会被放到中值附近。
- 中指、无名指、小指下限为 0，会停在 0。
- 拇指 yaw 也会停在 0 的极限位。

这会造成 RViz 中看起来“手指和手掌断裂”的形态。

在 `franka_description` 的 launch 中，已为 `rh56dfx_right` 显式设置 GUI 默认零位，
使初始/Center 姿态更适合模型检查。

## 3. 当前验证命令

在 `/home/ace/franka_ros2_ws` 中：

```bash
source install/setup.bash

ros2 daemon stop
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

ROS_LOG_DIR=/tmp/franka_ros2_ws_ros_logs ros2 launch franka_description visualize_franka.launch.py \
  robot_type:=fr3 \
  ee_id:=rh56dfx_right \
  rpy_ee:="0 0 3.14159" \
  xyz_ee:="0 0.015 0" \
  tcp_xyz:="0.070 0.016 0.155"
```

## 4. 已提交位置

本次 `franka_description` 本地提交：

```text
branch: fr3-rh56dfx-rviz
commit: cea9d48 Add RH56DFX right hand visualization
```

未 push。

## 5. 对 bimanual_dexterous_mvp_ws 的建议

短期不建议把 `franka_description` 的代码直接复制进本仓库。

更合理的边界：

- 本仓库保留这份验证记录和后续集成要求。
- RH56DFX 原始/第三方资料继续放在 `third_party/`。
- FR3/RH56DFX description 修改保留在 `franka_description` fork 或独立分支。
- 进入 MVP-2 真正模型接入时，再决定使用 submodule、vendor package，或固定 fork commit。

## 6. 后续任务

- 冻结 FR3 到 RH56DFX 的真实安装位姿，而不是继续使用临时 RViz 对齐参数。
- 明确 RH56DFX joint name 与真实硬件 6 自由度命令之间的映射。
- 若进入 MoveIt 集成，需要补 SRDF、planning group、collision matrix 和手部控制适配层。
- 真实手控制逻辑不应混入 `fake_hand_control`，应进入后续 `rh56dfx_hand_driver` 或等价适配包。
