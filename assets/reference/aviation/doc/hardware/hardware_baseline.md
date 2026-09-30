# Hardware Baseline

本文件记录当前项目的真实硬件平台、工程命名和后续集成约定。它不是产品
说明书，也不是论文资料归档；只沉淀对 URDF / SRDF、MoveIt、MuJoCo、
`ros2_control` 和真机 bringup 有直接影响的信息。

MVP-0 / MVP-1 仍以纯软件 fake 系统为主，不加载真实硬件驱动，也不下发
真实关节命令。本文档中的硬件信息用于保证后续模型、命名、接口和实验路线
提前对齐。

FR3 API 分层、MoveIt 仿真/真机解耦方式，以及 `doc/hand/` 中 RH56DFX
串口资料的摘要和缺口清单，见
`doc/hardware/fr3_rh56_api_integration.md`。

当前 RH56 MuJoCo 模型中，六通道目标到关节角度、执行器力矩的计算及其对
夹持的影响，见 [RH56 关节目标与驱动力矩](rh56_mujoco_position_torque.md)。

## 1. 已确定硬件

| 角色 | 型号 | 当前工程映射 | MVP 阶段用法 |
| --- | --- | --- | --- |
| 机械臂 | Franka Research 3 | `right_arm` 先映射到首个或右侧 FR3 | 仅做逻辑名和后续模型基线 |
| 灵巧手 | 因时 RH56DFX-2R | `right_hand` 先映射到首个或右侧 RH56DFX-2R | 先降维为夹爪化或预设抓型执行器 |

后续若增加第二套机械臂、左手机械手或新的视觉设备，应在本文档中补充型号、
序列号、IP、控制器命名、TF 前缀和接入状态。

## 2. 工程命名与命名空间

逻辑名用于上层任务接口，不等同于设备序列号或驱动节点名。

| 逻辑名 | 硬件含义 | 当前状态 |
| --- | --- | --- |
| `right_arm` | 首个或右侧 Franka Research 3 | 当前单臂链路默认目标 |
| `right_hand` | 首个或右侧因时 RH56DFX-2R | 当前单手链路默认目标 |
| `left_arm` | 后续左侧或第二台 Franka Research 3 | 预留 |
| `left_hand` | 后续左侧或第二只灵巧手 | 预留 |

命名约定：

* ROS 2 上层接口继续使用 `right_arm`、`right_hand`、`left_arm`、`left_hand`
  这类逻辑名，避免任务层绑定具体硬件序列号。
* 真机驱动、控制器和 TF 前缀可在 bringup 层映射到具体设备。
* 双臂扩展前，不应把 `left_arm` / `left_hand` 写成已经可用能力。
* 若后续需要多机协同，应在 launch、参数文件和测试文档中显式区分左右设备。

## 3. Franka Research 3 机械臂基线

Franka Research 3 是当前机械臂硬件基线。仓库中的任务层先把它抽象为一个
可接收高层运动请求的 `right_arm`，后续再逐步替换 fake 控制器。

工程关注点：

* 模型层：需要维护 FR3 的 URDF、mesh、关节限制、末端法兰和碰撞模型。
* 规划层：MoveIt / MoveIt Task Constructor 负责把命名目标、抓取位姿或笛卡尔
  目标转换为可执行轨迹。
* 控制层：`ros2_control` 或真实驱动适配层负责关节状态、控制器加载和轨迹执行。
* 任务层：`assembly_task` 只应调用稳定的运动接口，不直接写入厂商私有命令。
* 安全层：真机调试必须从使能、急停、限速、低速空载动作和单节点 bringup 开始。

MVP 阶段简化：

* MVP-0 只验证 `MoveArm(right_arm, home)` 的 action 链路。
* MVP-1 中的 `home`、`pregrasp`、`grasp`、`lift`、`preplace`、`place`、`retreat`
  是 fake 命名目标，不代表已经完成真实 IK、轨迹规划或硬件执行。
* FR3 的 FK / IK、碰撞检测和真实轨迹执行应放在 MoveIt / MuJoCo / 真机阶段验证。

## 4. 因时 RH56DFX-2R 灵巧手基线

因时 RH56DFX-2R 是当前灵巧手硬件基线。`RH56DFX-2R` 是本仓库采用的权威
型号命名，不应再使用旧的错误型号名。

工程关注点：

* 手部模型：后续需要明确手掌坐标系、各手指关节、指尖坐标系和可用抓型。
* 控制接口：真实驱动应被封装在手部控制适配层之后，上层任务只发送高层命令。
* 抓型库：常用 `open`、`preshape`、`close`、`hold`、`release` 可先作为预设抓型。
* 状态反馈：后续应记录关节状态、抓取状态、错误状态；若有触觉或力反馈再单独接入。
* 安装关系：手掌坐标系相对 FR3 末端法兰的固定变换必须在真实安装后标定。

MVP 阶段简化：

* MVP-1 只把 `right_hand` 当作夹爪化或预设抓型执行器。
* 当前不要求 RH56DFX-2R 全关节精细控制。
* 当前不要求真实手部驱动、真实接触反馈或多指独立抓取策略。

## 5. 臂/手组合与坐标系

臂/手组合的关键不是单独知道机械臂或灵巧手型号，而是明确二者之间的固定
安装关系和任务坐标约定。

后续模型中应至少明确以下坐标关系：

| 坐标对象 | 含义 | 当前状态 |
| --- | --- | --- |
| `world` / `base` | 场景或机器人工作站参考坐标系 | 待模型阶段冻结 |
| `right_arm_base` | 右侧 FR3 基座坐标系 | 待 URDF / TF 确认 |
| `right_arm_flange` | FR3 末端法兰坐标系 | 待 URDF / TF 确认 |
| `right_hand_palm` | RH56DFX-2R 手掌坐标系 | 待安装标定 |
| `object` | 被抓取或装配零件坐标系 | 待视觉/仿真阶段定义 |
| `connector_plug` | 航空插头本体坐标系 | 后续插接阶段定义 |
| `connector_socket` | 航空插座或目标接口坐标系 | 后续插接阶段定义 |

臂手标定要求：

* 真实安装后应记录 `right_arm_flange -> right_hand_palm` 的固定变换。
* 抓取位姿、预抓取位姿和放置位姿应统一表达在明确的参考坐标系下。
* 物体附着到末端时，需要定义附着 link 和物体坐标系之间的关系。
* 航空插头插接阶段还需要定义插头键位方向、插座坐标系、插接轴线和末端接触坐标系。

## 6. 仿真与真实硬件接入路线

硬件能力应按层次逐步接入，不应从完整 Pick-and-Place 或航空插头插接直接上真机。

| 阶段 | 目标 | 最小验证 |
| --- | --- | --- |
| URDF / SRDF | 能加载 FR3、RH56DFX-2R 和臂手安装关系 | 模型可视化、TF 连通、关节限制合理 |
| MoveIt / MTC | 能规划单臂抓取相关目标 | 命名目标、FK / IK、碰撞检测、轨迹可视化 |
| MuJoCo 或等价仿真 | 能验证基础接触、物体状态和场景重置 | 单臂夹爪化 Pick-and-Place 仿真闭环 |
| `ros2_control` | 能连接控制器和关节状态 | `joint_states`、控制器加载、低速空载动作 |
| 真机 bringup | 能安全驱动单节点和单动作 | 急停、限速、使能、单目标低速运动 |
| 任务集成 | 能把硬件能力接回任务系统 | 先单臂单手，再考虑双臂或航空插头插接 |

每一层完成后都应补充对应的启动命令、测试记录、已知限制和失败退出条件。

## 7. 当前 MVP 边界

MVP-0 当前只验证最小 ROS 2 任务链路：

```text
StartTask
-> ResetScene
-> MoveArm(right_arm, home)
-> TaskState
```

MVP-1 的目标是扩展为 fake 单臂 Pick-and-Place 闭环：

```text
StartTask(pick_and_place_mvp)
-> ResetScene
-> MoveArm(right_arm, pregrasp / grasp / lift / preplace / place / retreat)
-> ControlHand(right_hand, open / preshape / close / hold / release)
-> AttachObject / DetachObject
-> ExecuteTerminalOperation(PLACE)
-> TaskState
```

MVP-0 / MVP-1 不包含：

* 真实 Franka Research 3 驱动。
* 真实 RH56DFX-2R 全关节控制。
* MoveIt / MTC 真实规划执行。
* MuJoCo 接触仿真闭环。
* 真实视觉定位和手眼标定。
* 柔顺控制、力反馈、双臂协同或航空插头插接。

## 8. 后续待补充项

硬件进入仿真或真机阶段前，应逐项补齐以下信息：

* FR3 序列号、IP、控制器版本和驱动来源。
* RH56DFX-2R 序列号、通信方式、驱动来源和控制模式。
* FR3 的 `franka_ros2` / MoveIt / `ros2_control` 接入配置和验证命令。
* RH56DFX-2R 官方通信协议版本、完整寄存器表、错误码含义和安全阈值。
* URDF、mesh、SRDF、MoveIt 配置包来源。
* `right_arm_flange -> right_hand_palm` 标定结果。
* 关节限位、速度限制、力矩限制和真机调试限速策略。
* 急停、碰撞保护、工作空间边界和人员安全要求。
* 单节点 bringup 命令、低速空载测试记录和失败退出条件。
* 后续视觉设备、相机标定和手眼标定结果。

## 9. 更新规则

* 只记录已经确定或明确规划的工程信息，不把临时论文资料直接搬入本文件。
* 若信息来自厂商文档、真机测试或标定结果，应在对应小节写明来源和日期。
* README 只保留硬件基线摘要和链接，详细内容维护在本文件中。
* 任何型号名、逻辑名或 TF 约定变更，都应同步检查 README、MVP 文档和接口测试。
