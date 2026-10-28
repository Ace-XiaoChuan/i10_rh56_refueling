# AUBO 六轴实时关节状态

本包使用本地 AUBO SDK 0.23.1，只订阅 RTDE 输出，将六轴位置和可用的速度发布成 ROS 2 `sensor_msgs/msg/JointState`。MoveIt 可据此读取当前臂构型。节点不提供运动或控制器配置接口。

当前参数、进度和现场待核对项统一见[项目说明](../../doc/assembly_contract.md#近期真机目标)。

下方 `192.168.1.32` 及已有联机验证属于左侧拍照臂。真机为左臂拍照、右臂旋拧，见[双臂分工与坐标传递](../../doc/assembly_contract.md#双臂分工与坐标传递)。本包当前使用一套无左右前缀的六轴关节名和默认 `/joint_states`；不能同时将两臂状态发布到该话题，也不能把左臂状态作为右臂规划起点。双臂接入需分别绑定控制盒、状态话题、关节名及模型；右臂状态接入尚未验证。

## 构建与启动

从项目根目录运行，厂家 SDK 路径沿用 `tools/sdk/`：

```bash
source /opt/ros/humble/setup.bash
# 本机跨进程验证使用已安装的 CycloneDDS，所有终端使用同一中间件设置。
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --packages-select aubo_joint_state_bridge i10_rh56_moveit_config \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 launch aubo_joint_state_bridge joint_states.launch.py host:=192.168.1.32
```

修改开发包位置时，构建增加 `-DAUBO_SDK_ROOT=/实际SDK目录`。节点默认使用已验证的厂家示例认证；现场修改密码时，可先通过终端隐藏读取并导出 `AUBO_RPC_PASSWORD` 环境变量。节点不将密码作为 ROS 参数或日志输出。自定义用户名和 RTDE 前缀使用原生节点的 `user`、`rtde_prefix` 参数；前缀必须对应控制柜输出菜单中的目标机器人，当前单机器人为 `R1`。

在另一个同样加载 ROS 和工作区环境的终端中：

```bash
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
ros2 launch i10_rh56_moveit_config arm_state_snapshot.launch.py \
  output_path:=/tmp/aubo_current_arm.json
```

`snapshot_arm_state` 使用 MoveIt 自身的 `CurrentStateMonitor`。只有六个臂关节均有新状态、时间有效且角度满足模型限位时才返回有效快照；失败时返回非零，输出文件保持 `valid=false`。快照是一次读取结果，后续使用仍需重新检查时效。

## 数据如何进入 MoveIt

输入是 `R1_actual_q` 和可用的 `R1_actual_qd`，按以下次序输出：

```text
SDK[0..5] → shoulder_joint, upperArm_joint, foreArm_joint,
            wrist1_joint, wrist2_joint, wrist3_joint
```

位置单位 rad，速度单位 rad/s，不转换正负号、零点或角度绕数；缺少速度时数组留空，`effort` 也留空。数组与模型计算的对应关系已由已有控制器 FK 探针核对，实物方向及安装仍需现场复核。节点只发布六轴测量，不补造 RH56 手指反馈，因此全机器人状态仍不完整。

关键发布逻辑见 [`receive()` 和 `tick()`](src/joint_state_bridge.cpp)：

```cpp
// 接到新有效帧时保留主机接收时间，不使用无效的控制器时间戳。
pending_.header.stamp = stamp;
// 每帧最多发布一次；数据断流后退出，不刷新旧帧的时间戳。
if (received_count_ != published_count_) publisher_->publish(pending_);
```

主机接收时间用于 ROS 状态时效检查，不等于控制器采样时间，也没有完成相机时钟同步。无效位置/速度、首帧超时或断流会使进程非零退出，不自动重连。请求频率和实际接收频率分别查看启动参数与 `ros2 topic hz`。

已有 `approach.launch.py` 可显式从状态话题取臂起点：

```bash
ros2 launch i10_rh56_moveit_config approach.launch.py \
  request_path:=/绝对路径/approach_request.json \
  output_path:=/绝对路径/approach_result.json \
  start_source:=joint_states
```

它会替换请求中的六轴起点，并在结果中保存测量话题、时间和实际起始角。手部预型、目标和环境仍由请求提供；该入口输出几何规划结果，轨迹执行、真实工具标定及现场场景接入按项目计划继续完成。默认 `start_source:=request` 保留已有离线行为；已选构型和重新抓握入口仍使用其原有请求起点。

## 离线验证

回放只接受采集器的 JSONL 帧：`parse_ok`、`host_monotonic_s`、`R1_actual_q`，以及可选的 `R1_actual_qd`。保留帧间隔，按本次回放的接收时间发布，不使用历史控制器时间戳。回放结束后按断流退出；禁止发布到真机 `/joint_states`。

```bash
ros2 run aubo_joint_state_bridge joint_state_bridge --ros-args \
  -p source:=replay \
  -p replay_path:=/绝对路径/rtde_samples.jsonl
```

默认回放话题为 `/aubo/replay/joint_states`，检查时给快照工具传入相同的 `joint_states_topic`。回放节点会明确记录历史来源。

集成检查使用独立 ROS domain，并启动状态桥、ROS 订阅者和 MoveIt 状态监视器；测试全部离线，不连接控制柜：

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1 \
  /usr/bin/python3 tools/tests/test_aubo_joint_state_bridge.py \
  --install-base install -v
colcon test --packages-select aubo_joint_state_bridge
```

需要使用可导入 ROS 2 `rclpy` 的 Python；项目 MuJoCo 虚拟环境不作为此测试入口。规划用例会单独调用项目 `.venv` 生成请求，并使用已有几何复核通过的构型检查起点替换。

本机 WSL 的 Fast DDS 在部分只含回放器和 Python 订阅者的回环用例中未完成发现；相同用例在 CycloneDDS 下能正常通信。上述命令显式选用本机已有的 CycloneDDS，避免将发现失败误判为状态值错误。
