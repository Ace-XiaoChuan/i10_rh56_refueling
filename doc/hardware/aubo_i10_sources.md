# AUBO i10 模型与驱动来源

查阅日期：2026-09-15。当前 MVP 硬件为单臂 AUBO i10 + adapter + RH56DFX，双臂为后续扩展。
本次完成网页和模型文本查阅，未克隆模型仓库、安装驱动、连接实机或验证 Humble 构建。

## 官方 GitHub 入口

官方仓库地址：<https://github.com/AuboRobot/aubo_description/tree/main>

| 来源 | 已查阅的内容 | 用途 |
| --- | --- | --- |
| [AuboRobot/aubo_description](https://github.com/AuboRobot/aubo_description) | [型号与资源说明](https://github.com/AuboRobot/aubo_description/blob/main/README.md) | 型号、URDF、网格和配置的来源 |
| [i10 URDF](https://github.com/AuboRobot/aubo_description/blob/main/urdf/aubo_i10.urdf) | 六个 revolute joint、惯量与 visual/collision mesh 引用 | 接入模型的候选入口，尚未匹配实机版本 |
| [AuboRobot/aubo_ros2_driver](https://github.com/AuboRobot/aubo_ros2_driver) | ROS 2 驱动说明、MoveIt 配置目录及模型子模块 | 执行后端的候选来源，示例默认使用 i5，不能原样当作 i10 启动验证 |

本次查看的 i10 URDF 关节依次为 `shoulder_joint`、`upperArm_joint`、`foreArm_joint`、
`wrist1_joint`、`wrist2_joint`、`wrist3_joint`；网格引用位于
`package://aubo_description/meshes/aubo_i10/visual/` 和 `collision/`。
这些是所查候选 URDF 的事实，不是对实机版本的最终确认。

## 版本选择

官方说明区分 `aubo_i10`、`aubo_i10_10`、`aubo_i10_11`、`aubo_i10_30` 等条目；
不能只按“i10”名称选择关节限位或零位。应核对设备铭牌、具体型号版本和控制器。

查阅时 ROS 2 驱动的模型子模块链接指向
[`47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1`](https://github.com/AuboRobot/aubo_description/tree/47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1)。
这是网页当时显示的关联版本，不是本工程已锁定或通过验证的依赖。
`main` 为可变分支；正式引入时要固定选定提交、保留许可/来源并核验网格依赖。

## 接入边界

1. 先确定 i10 版本及其 URDF、网格、关节限制，再在本工程接入完整模型资源。
2. 以 `adapter` 表示当前单臂与 RH56 之间的转接件；已确定采用 RH56DFX 右手，命名约定在接入时统一，双臂实例化不属于当前需求。
3. 定义 i10 法兰 → adapter → RH56 基座的固定变换；照片不能确定两侧安装方向及数值。
4. 更新 MoveIt 规划组、IK、控制器与状态映射；重新建立 MuJoCo 的 i10 动力学/执行器模型。
5. 核对 ROS 2 Humble、控制柜软件与 SDK 兼容性；官方默认模型不等于实机运动学校准结果。

上述是后续接入工作，本次不宣称已有 i10 + RH56 的可运行仿真。
旧 FR3、旧安装合同、旧控制增益和旧验收结论均不自动适用于 i10。

## 照片与机械信息

见 [四张硬件照片及观察记录](../../assets/hardware_photos/README.md)。
当前优先确认 adapter 的 CAD/安装测量，以及油枪握把、护圈、扳机和油箱口几何；需要验证中指、无名指、小指操作扳机时的稳定握持。
油箱口默认敞开，盖体照片仅保留为后续参考。此前确认的先旋转再拔出、先插入再旋转机制，不属于当前单臂 MVP 任务。
