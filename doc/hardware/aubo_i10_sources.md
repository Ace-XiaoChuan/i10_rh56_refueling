# AUBO i10 模型与驱动

实际设备采用左臂拍照、右臂旋拧，两臂由 T 型架固定。当前旋拧模型使用 AUBO i10，安装链为 i10 → 传感器 → adapter → RH56DFX 右手；双臂实物模型与安装仍需核对。原有 SDK 采集及控制器模型比对来自左臂，设备归属和坐标传递见[项目说明](../assembly_contract.md#双臂分工与坐标传递)。

## 当前模型

URDF 与 SRDF 位于 `assets/robots/aubo_i10/urdf/`，MuJoCo 模型位于 `assets/robots/aubo_i10/mjcf/`。完整臂手规划模型位于 `src/i10_rh56_moveit_config/`。

六关节顺序为：

```text
shoulder_joint, upperArm_joint, foreArm_joint,
wrist1_joint, wrist2_joint, wrist3_joint
```

机器人基座为 `base_link`，法兰为 `wrist3_Link`。资产路径和原件见[资产索引](../../assets/README.md)，安装变换见[项目说明](../assembly_contract.md#当前装配变换)。

## 官方资料

| 来源 | 用途 |
| --- | --- |
| [AuboRobot/aubo_description](https://github.com/AuboRobot/aubo_description) | 型号描述、URDF 和网格 |
| [i10 URDF](https://github.com/AuboRobot/aubo_description/blob/main/urdf/aubo_i10.urdf) | 关节链、限位及几何来源 |
| [AuboRobot/aubo_ros2_driver](https://github.com/AuboRobot/aubo_ros2_driver) | ROS 2 驱动与控制接入参考 |

## 真机接入需要核对

真机只读查询、资料采集与保存文件说明见[SDK 连接与采集](aubo_sdk_connection.md)，当前设备参数与进度见[项目说明](../assembly_contract.md#近期真机目标)。

设备铭牌、i10 具体版本、控制柜软件与 SDK；关节零位、限位、数组顺序和法兰变换；状态读取、轨迹控制、停止保持及力反馈接口。官方不同 i10 型号的参数要按实际设备选择，驱动示例中的型号也需对应修改。
