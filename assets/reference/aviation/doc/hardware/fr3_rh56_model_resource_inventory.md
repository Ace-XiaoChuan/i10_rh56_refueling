# FR3-RH56 模型资源清单与来源合同

## 1. 目的与边界

本文冻结 **FR3 + 转接件 + RH56 + connector** 的当前资源清单、来源角色与迁移规则，
用于后续建立同一套近似 URDF/Xacro 与 MJCF。它不冻结真实安装标定，也不证明任何
碰撞、质量、惯量、摩擦或接触参数正确。

当前实施范围仅为右侧单臂单手链：

```text
fr3_link8
-> adapter_1
-> adapter_2
-> rh56_right_base_link
-> rh56_grasp_tcp
```

第二条臂/手应复用本合同的结构和命名规则，但不得假定其安装变换与右侧相同。

## 2. 资源角色

| 角色 | 定义 | 能否直接作为运行时依赖 |
| --- | --- | --- |
| 原始资产 | 从 CAD、供应商或机械设计获得的 STEP/STL；保留原始坐标和文件名 | 否；须经单位、坐标系和许可证检查 |
| 描述源 | Xacro、joint limit、joint tree、固定安装合同 | 是；它定义 URDF/MJCF 的运动学事实 |
| 运行时资产 | 随 ROS package 安装的 visual/collision mesh 和已生成 MJCF | 是；必须自包含，不依赖个人目录 |
| 派生产物 | open-pose 合成 mesh、MJCF、复制到 package share 的 mesh | 否；必须能追溯到描述源或原始资产 |

任何文件在成为 MoveIt 或 MuJoCo 输入前，至少要记录：来源、单位、坐标系、用途
（visual/collision/inertia）及其确认等级。

确认等级如下：

| 等级 | 含义 | 允许用途 |
| --- | --- | --- |
| R0 raw | 未检查的原始 CAD/STL | 归档与人工检查 |
| R1 visual | 已能在 RViz 连续显示，但安装位姿未标定 | 外观、TF、粗略工作空间检查 |
| R2 collision | 有已检查的单位、坐标和简化 collision 几何 | MoveIt 的无接触粗规划 |
| R3 physical | 有测量/CAD 依据的安装、质量、惯量和接触参数 | 接触仿真或真机相关结论 |

当前所有 FR3-RH56 安装关系最多是 **R1**；不得将它用于 R3 结论。

## 3. 当前资源清单

### 3.1 FR3 与 RH56 上游模型

| 项目 | 当前位置 | 当前状态 | 规定用途 |
| --- | --- | --- | --- |
| FR3 MJCF、关节动力学、position actuator、mesh | MuJoCo Menagerie `franka_fr3`，固定 commit 见 `src/assembly_mujoco/mjcf/MODEL_PROVENANCE.md`；其 mesh 已复制进 `src/assembly_mujoco/mjcf/meshes/` | MuJoCo 中的 FR3 唯一上游源 | 不手改派生 MJCF；通过生成器更新上游并重建场景 |
| RH56 右手 Xacro、12 physical joint、mimic | 本机 `franka_description` fork；来源 hash 见 `src/assembly_mujoco/mjcf/MODEL_PROVENANCE.md` | 右手运动学和 mimic 已在 MJCF 使用 | 作为详细手部 joint tree 的来源 |
| RH56 右手 collision mesh | `src/assembly_mujoco/mjcf/meshes/rh56dfx_description__*`，35 个文件、5,764,640 bytes | 已随 MuJoCo package 运行时安装 | MuJoCo collision；MoveIt 版本须建立可追溯的 package 资源入口 |
| FR3 collision/visual mesh | `src/assembly_mujoco/mjcf/meshes/link*.stl` 与 `link*.obj` | 已随 MuJoCo package 运行时安装 | 由固定 Menagerie FR3 MJCF 引用 |

上游 `/home/ace/franka_ros2_ws` 只可作为**生成期来源**，不是本项目运行时依赖。
若将 RH56 模型引入 MoveIt，须固定 fork commit/hash 或以受许可证约束的 vendor package
方式纳入本工作区。

### 3.2 转接件与手部预览资源

| 逻辑资源 | 当前文件 | 确认等级 | 已知事实 / 限制 |
| --- | --- | --- | --- |
| adapter_1（法兰坐标系） | `third_party/fr3_rh56_adapter_assets/solidworks_exports/adapter_1_flange_frame/` | R2 | SolidWorks “坐标系1”导出；URDF 的 visual/collision 均为零偏移，网格单位为米。运行时副本为 `src/fr3_rh56_adapter_description/meshes/adapter_1_flange_frame.stl`；旧 adapter1 STL 已清理 |
| adapter_2 | `third_party/fr3_rh56_adapter_assets/meshes/adapter_2.stl` | R1 | SolidWorks metadata 记录质量 `0.035043 kg` 与惯量；仍未确认它相对 FR3/RH56 的装配变换 |
| camera mount | `third_party/fr3_rh56_adapter_assets/meshes/camera_mount.stl` | R0 | 与 `assets/相机安装.STL` 的 SHA-256 相同；不进入本轮臂手模型 |
| RH56 张开位单一预览 mesh | `third_party/fr3_rh56_adapter_assets/meshes/rh56_right_open_preview.stl` | R1 | 由 `inspire_right.xml` 零位 visual mesh 合成；仅用于 RViz 外观，不是可动手或 collision 模型 |
| 视觉预览 Xacro | `src/fr3_rh56_adapter_description/urdf/fr3_rh56_visual_preview.urdf.xacro` | R1 | 已有固定链和 `rh56_grasp_tcp` 骨架；没有 collision；实际引用的 preview mesh 已随描述包源树和 install space 提供 |

法兰坐标系导出的 `base_link.STL` 为 361,884 bytes，`adapter_2.stl` 为 24,784 bytes；
二者是不同零件，不得仅按旧文件名或网格外观替换。

### 3.3 Connector 与其他原始设计资产

| 资源 | 当前位置 | 确认等级 | 规定用途 |
| --- | --- | --- | --- |
| connector plug/socket visual STL | `assets/meshes/connector_*_visual.stl` | R1 | connector 预览及当前 MJCF 的派生运行时副本 |
| connector STEP | `assets/移动插头.STEP`、`assets/母.STEP`、`assets/航空插头装配体.STEP` | R0 | 后续确定功能端面、插接轴和 collision proxy 的原始设计输入 |
| 相机安装 STL | `assets/相机安装.STL` | R0 | 原始归档；不属于本轮臂手链 |

Connector mesh 已被复制到 `src/assembly_mujoco/mjcf/meshes/`。该副本是 MuJoCo 运行时
资产，`assets/meshes/` 才是当前项目中的可读原始入口；任何更新必须同时记录复制/生成方式。

## 4. 已验证的重复关系

以下 SHA-256 在 2026-08-09 本工作区中一致：

```text
assets/相机安装.STL
  = third_party/fr3_rh56_adapter_assets/meshes/camera_mount.stl
  = 40d4bd51923dbf92b49e87d20b9cf5ba6a6f0d859301c475c334295b1e93cedb
```

旧 adapter1 的两份字节相同副本已在确认法兰坐标系导出可用后删除。相机安装这对文件仍按
原始归档与第三方资产分别保留；是否去重由维护者决定。

## 5. 统一来源与迁移规则

后续实施遵循以下规则：

1. `assets/` 只保存原始设计输入，不作为 ROS package 的隐式运行时依赖。
2. `third_party/fr3_rh56_adapter_assets/` 保存带来源/许可证信息的第三方或机械输入；不直接
   用 package URI 指向它。
3. `fr3_rh56_adapter_description` 应成为 FR3-RH56 覆盖描述的唯一 Xacro 所有者，并在 package
   内安装它实际引用的 visual/collision mesh。
4. 新增的安装合同必须单独版本化，例如
   `config/fr3_rh56_mount_provisional.yaml`；禁止在 launch、URDF、MJCF 中各写一份 `xyz/rpy`。
5. MoveIt 读取 FR3/RH56 Xacro；MuJoCo 由固定的 FR3 MJCF、RH56 模板和安装合同生成运行时场景。
6. `mvp3_connector_assembly.xml` 及其 mesh 只视为生成后的运行时快照；修改其运动学前必须先
   修改描述源或安装合同。

## 6. 进入下一步前必须确认的缺口

| 缺口 | 影响 | 本轮处理方式 |
| --- | --- | --- |
| `T_fr3_link8_rh56_right_base_link` 未测量 | TCP、碰撞、预抓取全都会偏移 | 先标为 provisional，允许 RViz 拟合，禁止真机/接触结论 |
| `T_fr3_link8_adapter_1` 尚未实物/装配确认 | TCP、碰撞、预抓取全都会偏移 | adapter_1 的 local frame 已确认；固定关节仍仅限 RViz 拟合，禁止真机/接触结论 |
| 预览包没有 collision | 不能直接接入 MoveIt | 下一步基于已整理的运行时 mesh 建立可追溯的 collision 版本 |
| RH56 active model 与 open preview 不同 | 不能用单一 preview STL 做抓取碰撞 | MoveIt 使用详细 RH56 joint tree 或保守 open-hand proxy |
| 质量/惯量/接触参数不完整 | 不能宣称 MuJoCo 抓取可信 | 仅填 provisional 值并在 MJCF provenance 中说明 |

## 7. 本清单后的下一项工作

下一步不是立刻替换 MoveIt，而是创建并验证 R1 的统一安装合同：在 RViz 中按
`FR3 -> adapter_1 -> adapter_2 -> RH56 palm -> TCP` 逐段显示，记录每段变换的来源和
置信度。通过后再将同一合同接入 URDF/MJCF 生成链。
