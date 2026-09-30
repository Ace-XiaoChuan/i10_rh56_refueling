# 航插工程参考资产

本目录是从 `/home/ace/bimanual_dexterous_mvp_ws` 当前工作树筛选的本地副本。
源工程未被移动或修改；不依赖指向源工程的符号链接。逐文件来源、大小和 SHA-256
见 [manifest.json](manifest.json)。源工程存在未提交修改，因此文件哈希优先于 Git HEAD。

## 入口与用途

| 资源 | 本地入口 | 用途与限制 |
| --- | --- | --- |
| FR3 + 可动 RH56 右手 MJCF | [fr3_rh56_right.xml](derived/fr3_rh56_right/fr3_rh56_right.xml) | 从当前航插 MJCF 提取机器人子树，保留 7 个臂关节、12 个手关节和 13 个执行器；不是双臂场景 |
| RH56 左、右单手 MJCF | [左手](third_party/correlllab_rh56_controller_h12/mujoco_inspire/inspire_left.xml)、[右手](third_party/correlllab_rh56_controller_h12/mujoco_inspire/inspire_right.xml) | 上游独立参考，两者均有六个执行器；其关节命名、零位、耦合及控制参数需与当前手模型核对，不能直接互换 |
| FR3 描述源节选 | [FR3 Xacro](src/third_party/franka_description/robots/fr3/fr3.urdf.xacro) | 保留 FR3/common Xacro、FR3 参数和 visual/collision 网格；是节选，不是完整可构建包 |
| 转接件与开手描述 | [描述包说明](src/fr3_rh56_adapter_description/README.md) | 原样保留 Xacro、SRDF、预览工具与包内网格；MoveIt 手部是固定开手模型 |
| 安装参数 | [暂定安装合同](src/fr3_rh56_adapter_description/config/fr3_rh56_mount_provisional.yaml) | 未经实物标定；右侧参数不能直接当作左侧参数 |
| 转接件 CAD 导出证据 | [adapter_1](third_party/fr3_rh56_adapter_assets/solidworks_exports/adapter_1_flange_frame/urdf/装配体2.urdf)、[adapter_2](third_party/fr3_rh56_adapter_assets/source_metadata/adapter_2_export.urdf) | 保存原始 STL、CSV、URDF；原导出中的 package URI 仅作来源记录 |
| 灵巧手串口接口 | [使用说明](doc/hand/README.md)、[角度脚本](doc/hand/rh56_angle_cmd.py)、[状态脚本](doc/hand/rh56_watch.py) | 原样参考，本次没有运行脚本或连接硬件 |
| 硬件接口与基线 | [API 说明](doc/hardware/fr3_rh56_api_integration.md)、[模型清单](doc/hardware/fr3_rh56_model_resource_inventory.md)、[力矩说明](doc/hardware/rh56_mujoco_position_torque.md) | 保留历史上下文，不作为新工程验收记录 |
| 依赖与来源 | [MuJoCo 版本](src/assembly_mujoco/requirements.txt)、[模型来源](src/assembly_mujoco/mjcf/MODEL_PROVENANCE.md)、[ROS 依赖参考](dependencies/franka_ros2_humble.repos) | 原 ROS 清单使用分支名，不是完整锁定清单；本次未安装依赖 |

## 机器人 MJCF 的提取边界

提取工具为 [extract_robot.py](derived/extract_robot.py)。它保留源 MJCF 的 `base`
机器人子树、运动学、惯量、执行器、关节耦合和仿真选项；移除航插物体、夹具、
放置台、地面、物体接触对和包含物体位姿的任务 keyframe；只复制被保留模型实际引用的网格。
提取没有重新标定参数，也没有把源场景改造成加油仿真。没有任务初始姿态、地面或控制器，
不应直接将默认零位的自由仿真当作稳定性验收。

该模型保留旧工程实际使用的右手安装方式；与转接件预览合同可能不同。二者用途分别记录，
后续须统一安装来源，不能将它们视为已经一致的双臂机器人描述。

## 依赖与历史资料

- 全部文件位于 `assets/reference/`，并设置 `COLCON_IGNORE`，不会被 colcon 当成运行时包构建。
- 单独的三个 MJCF 及其 mesh 在本目录内自包含；不需要旧工作空间的 mesh 路径。
- Xacro 节选和旧预览包尚未接入新工程。预览依赖 `franka_description`，MoveIt 覆盖描述还依赖
  `franka_bringup`/`franka_hardware`；本次没有迁移这些运行节点，也没有验证 ROS 启动。
- 原样保留的历史 README/资料可能含未复制文件的相对链接、旧绝对路径和旧阶段结论；它们用于
  查考来源，不应按旧命令直接启动新工程。当前迁移入口以本文件为准。
- Correll Lab 原 README 描述的是较大参考集合；这里只复制左右单手模型、其网格与许可来源。
- FR3 Apache-2.0、RH56 描述 MIT、Correll 模型 MIT 的许可文件和已有 NOTICE 已随相应参考目录保留。
  转接件保留原始机械来源，不额外赋予新许可证。

## 本次未复制

航空插头 STEP/STL、装配任务与配置、放置调参和验收日志、build/install/log、Git 历史、
其他机器人型号资产、相机支架、用途或授权不明确的第三方控制脚本。加油枪、油箱口和盖体模型尚未提供。

## 本次验证

使用本机 `/home/ace/mujoco-env/bin/python`（MuJoCo 3.8.1），三个 MJCF 均独立加载成功，
执行一次 `mj_forward` 后状态与加速度为有限值。右臂组合模型有 19 个关节、13 个执行器；
两个单手模型各有 12 个关节、6 个执行器。详细数据见 [model_checks.json](model_checks.json)。
这仅验证模型可解析和依赖齐全，未验证动态稳定性、抓取、开盖、双臂协同或真机。
迁移时逐文件核验 SHA-256；未运行串口脚本、ROS 节点或 GUI。
