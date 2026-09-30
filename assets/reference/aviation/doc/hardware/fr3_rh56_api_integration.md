# FR3 与 RH56DFX API 集成说明

本文档汇总 Franka Research 3 API 体系、MVP-2 MoveIt 仿真接入边界，以及
`doc/hand/` 中现有 RH56DFX 灵巧手资料。它用于指导后续从 MVP-1 fake
链路过渡到 MoveIt、仿真和真机支线，不替代厂商手册或实验记录。

更新时间：2026-06-29

## 1. 总结论

当前项目不需要在 MVP-1 接入 FR3 或 RH56DFX 的真实 API。MVP-1 仍然只验证
fake 单臂 Pick-and-Place 状态机。

从 MVP-2 开始，需要引入 FR3 的 MoveIt / ROS 2 生态，但优先接入的是
`franka_ros2`、MoveIt 2 和 `ros2_control` 这一层，而不是直接在任务节点中
调用 `libfranka`。

推荐边界：

```text
assembly_task / MoveArm.action / ControlHand.srv
        |
        v
硬件或仿真适配层
        |
        +-- FR3: MoveIt 2 / franka_ros2 / ros2_control
        |
        +-- RH56DFX: 串口驱动节点 / 抓型库 / 状态监控
```

任务层不应直接操作 FR3 FCI 命令或 RH56DFX 串口帧。所有真实设备细节应封装
在独立适配层中。

## 2. FR3 API 与仿真/真机分层

### 2.1 官方栈

Franka Research 3 的官方生态可按以下层次理解：

| 层级 | 作用 | 项目中的使用阶段 |
| --- | --- | --- |
| FCI / `libfranka` | 真实 FR3 的底层实时控制接口 | 真机支线或后续接触控制 |
| `pylibfranka` | `libfranka` 的 Python binding | 研究验证，可选 |
| `franka_ros2` | 将 `libfranka` 接入 ROS 2 / `ros2_control` | MVP-2 起重点关注 |
| MoveIt 2 / MTC | 规划、PlanningScene、attached object、轨迹执行接口 | MVP-2 主线 |
| dummy / fake hardware | 无真机时验证 MoveIt 配置和控制接口 | MVP-2 仿真入口 |
| Gazebo / 其它仿真 | 动力学或可视化仿真入口 | 可选，MuJoCo 仍按项目路线单独评估 |

`libfranka` / FCI 面向真机实时通信；MoveIt 仿真阶段通常不直接使用它。
仿真与真机的解耦主要发生在 MoveIt 2、`ros2_control` 和 `franka_ros2`
配置层，而不是在任务节点里手写一套 if/else。

### 2.2 MVP-2 推荐接入方式

MVP-2 目标是把 MVP-1 中的 fake arm 替换为 MoveIt 规划执行，同时尽量保持
`assembly_task` 的上层状态机和接口稳定。

推荐新增一个适配包或节点，例如：

```text
moveit_arm_control
fr3_moveit_adapter
```

它对上继续提供项目内的 `MoveArm.action`，对下调用 MoveIt 2：

```text
MoveArm(right_arm, home)
        |
        v
MoveIt named target / pose target
        |
        v
Planning pipeline
        |
        v
ros2_control controller
        |
        +-- fake hardware: MVP-2 RViz 仿真
        +-- real FR3: 真机支线
```

MVP-2 不建议让 `assembly_task_node` 直接依赖 `move_group`、`libfranka` 或厂商
API。任务层只关心动作是否成功、失败码和 TaskState。

### 2.3 仿真与真机是否解耦

结论：工程上可以解耦，但不是完全自动无差别切换。

可复用部分：

* `assembly_task` 状态机。
* `MoveArm.action` 等项目上层接口。
* MoveIt 的规划目标、PlanningScene、collision object / attached object
  语义。
* FR3 URDF / SRDF / joint name / planning group。

需要区分的部分：

* fake hardware 不连接真实 FR3，也不需要 robot IP。
* 真机需要 FCI / `libfranka`、robot IP、实时内核、网络质量和安全配置。
* 真机必须额外验证急停、限速、碰撞阈值、工作空间边界和低速空载动作。
* 仿真中的轨迹成功不等于真机可安全执行。

## 3. RH56DFX 现有资料摘要

`doc/hand/` 当前包含：

```text
doc/hand/README.md
doc/hand/rh56_angle_cmd.py
doc/hand/rh56_watch.py
```

这些资料已经覆盖了最小串口读写和人工调试流程。

### 3.1 已知硬件通信信息

| 项 | 当前资料中的值 |
| --- | --- |
| 通信方式 | RS485 串口 |
| 默认端口 | `/dev/ttyUSB_robot_485` |
| 波特率 | 115200 |
| 左手 ID | 1 |
| 右手 ID | 2 |
| 回复帧头 | `90 eb` |
| 命令帧头 | `eb 90` |
| 校验 | 从 hand id 开始求和后取低 8 位 |

两只手接在同一条 485 总线上时，ID 必须不同。当前资料默认左手为 1，右手为
2。

### 3.2 六自由度顺序

所有角度、力、温度和错误数组都按同一顺序解释：

```text
little ring middle index thumb_bend thumb_rotate
```

中文含义：

```text
小拇指 无名指 中指 食指 大拇指弯曲 大拇指旋转
```

### 3.3 角度控制脚本

`rh56_angle_cmd.py` 当前用于向指定手 ID 下发 6 个自由度的目标角度。

已知规则：

```text
1000 = 张开
0    = 最大弯曲
-1   = 该自由度保持当前状态
```

脚本会先读取当前角度、受力、温度和错误码，再写入目标角度，随后持续观察一段
时间。

当前用到的寄存器：

| 名称 | 地址 | 长度/类型 | 用途 |
| --- | --- | --- | --- |
| `ANGLE_SET` | 1486 | 6 个 int16 | 下发目标角度 |
| `ANGLE_ACT` | 1546 | 6 个 int16 | 读取实际角度 |
| `FORCE_ACT` | 1582 | 6 个 int16 | 读取受力 |
| `ERROR` | 1606 | 6 个 uint8 | 读取错误码 |
| `TEMP` | 1618 | 6 个 uint8 | 读取温度 |

### 3.4 实时监控脚本

`rh56_watch.py` 当前用于持续读取单手或双手状态。

已显示字段：

```text
angle
force(N)
dF(N)
temp
err
```

其中 `force(N)` 由 gf 换算：

```text
force_N = force_gf * 0.00980665
```

`dF(N)` 是脚本启动时基线之后的受力变化量，适合用来观察手指受力趋势。该值
不应直接当作高精度力传感器标定结果。

脚本中还定义了以下寄存器，但当前没有实际展示或解释：

| 名称 | 地址 | 当前状态 |
| --- | --- | --- |
| `CURRENT` | 1594 | 已定义，未读取展示 |
| `STATUS` | 1612 | 已定义，未读取展示 |

## 4. RH56DFX 后续工程封装建议

MVP-1 的 `fake_hand_control` 只保留高层命令：

```text
open
preshape
close
hold
release
```

后续真实手部接入时，不建议让 `assembly_task` 直接调用
`rh56_angle_cmd.py` 或直接打开串口。应新增一个 ROS 2 驱动适配节点：

```text
rh56dfx_hand_driver
```

推荐职责：

* 独占 `/dev/ttyUSB_robot_485` 串口。
* 对上实现 `ControlHand.srv`，先支持 MVP 高层命令。
* 内部维护 `grasp_library.yaml`，把 `open`、`preshape`、`close` 等命令映射到
  六自由度角度数组。
* 持续发布真实手部状态，例如角度、受力、温度、错误码。
* 提供错误码映射和安全限幅，不允许任务层直接发送任意危险角度。

后续如果要做灵巧手精细控制，再逐步扩展接口：

```text
SetHandJointTarget
GetHandState
SetGraspPreset
CalibrateHand
ClearHandError
```

这些不应阻塞 MVP-1 / MVP-2。

## 5. 当前资料缺口

`doc/hand/` 中的资料已经足够做人工串口冒烟和单指小幅测试，但还不足以支撑
完整 ROS 2 驱动、MoveIt/MuJoCo 模型和真机安全集成。后续至少需要补齐：

### 5.1 厂商协议与设备信息

* RH56DFX-2R 官方通信协议文档版本。
* 完整寄存器表，包括 `CURRENT`、`STATUS`、错误码位含义和单位。
* 左右手序列号、固件版本、ID 设置方式和出厂配置。
* 角度 0-1000 与真实关节角之间的映射关系。
* 速度、力、力矩、电流或使能模式是否可控。
* 写命令 ack 的完整语义和失败码。

### 5.2 安全与标定信息

* 每个自由度的安全角度范围、速度限制和持续夹紧限制。
* 温度、电流、受力的安全阈值。
* 错误码非零时的停机和恢复流程。
* 左右手各自由度方向是否完全一致。
* `1000=open`、`0=close` 在所有自由度上的真实动作方向确认记录。
* force 读数的标定方法、漂移范围和可用精度。

### 5.3 ROS 2 与模型信息

* RH56DFX-2R URDF / mesh / joint name / joint limit 来源。
* 手掌坐标系、指尖坐标系和各关节坐标轴定义。
* FR3 法兰到 RH56DFX 手掌的固定安装变换。
* MoveIt planning group 是否需要包含手指关节。
* MuJoCo 或其它仿真模型来源、质量、惯量、摩擦和接触参数。
* `right_hand` / `left_hand` 与真实设备 ID、串口、命名空间的映射规则。

### 5.4 工程资料与测试证据

* `doc/hand/` 两个脚本的来源、维护者和可使用范围说明。
* 串口设备的 udev 规则，避免 `/dev/ttyUSB*` 漂移。
* Python 依赖安装说明，例如 `pyserial`。
* 单手、双手、长时间监控、错误码触发和恢复的测试记录。
* 推荐抓型库的第一版参数：`open`、`preshape`、`close`、`hold`、`release`。
* ROS 2 驱动单元测试和无硬件 fake backend 方案。

## 6. 建议的文档落点

后续资料建议按职责放置：

| 文档 | 负责内容 |
| --- | --- |
| `doc/hardware/hardware_baseline.md` | 型号、命名、阶段边界、硬件总路线 |
| `doc/hardware/fr3_rh56_api_integration.md` | FR3 API 分层、RH56DFX 资料摘要、缺口清单 |
| `doc/hand/README.md` | RH56DFX 串口脚本使用说明和人工测试流程 |
| `doc/mvp1/tasks/03_MVP1_Fake手部控制器.md` | MVP-1 fake 手部服务验收要求 |
| 后续 `doc/mvp2/` | MoveIt / FR3 仿真接入任务拆解 |
| 后续 `doc/bringup/` | 真机启动、安全检查和实验记录 |

## 7. 参考资料

官方资料：

* Franka FCI documentation: <https://frankarobotics.github.io/docs/>
* Franka FCI overview: <https://frankarobotics.github.io/docs/overview.html>
* Franka ROS 2 repository: <https://github.com/frankarobotics/franka_ros2>

本地资料：

* `doc/hardware/hardware_baseline.md`
* `doc/hand/README.md`
* `doc/hand/rh56_angle_cmd.py`
* `doc/hand/rh56_watch.py`
* `doc/mvp1/tasks/03_MVP1_Fake手部控制器.md`
