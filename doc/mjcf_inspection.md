# 模型预览与排障

当前唯一任务是油箱门已打开条件下拧油箱盖，见[项目合同](assembly_contract.md)。组合预览已按合同设置默认开门初态（135°）；独立开关门工具及场景已删除。

## 文件入口

- `assets/robots/aubo_i10/mjcf/aubo_i10.xml`：i10 单体，6 个关节。
- `assets/objects/fuel_cap/mjcf/fuel_cap.xml`：油箱盖单体，1 个旋转关节。
- `assets/scenes/i10_fuel_cap.xml`：组合展示场景，7 个关节。
- `assets/robots/aubo_i10/mjcf/meshes/`：由 DAE 转换的外观 OBJ。

所有网格引用均为相对路径。原 URDF 保留为当前源文件，生成的 MJCF 不替代 ROS 描述。
修改 URDF 后须重新运行转换脚本，组合场景也会一起更新。

## 使用

工作空间已创建独立 `.venv`，使用 MuJoCo 3.13.0、trimesh 5.1.0、
pycollada 0.9.3；完整版本记录在 `dependencies/mujoco-inspection.txt`。
新环境可通过 `python3 -m venv .venv` 和
`.venv/bin/pip install -r dependencies/mujoco-inspection.txt` 安装。

```bash
# 从 URDF 重建 i10、盖体 MJCF、OBJ 及臂盖场景（臂手组合另见下文）
.venv/bin/python tools/build_mjcf.py
# 组合预览
.venv/bin/python tools/view_mjcf.py
# 油箱盖近景
.venv/bin/python tools/view_mjcf.py --cap-only
# 无窗口加载检查
.venv/bin/python tools/view_mjcf.py --check
```

预览使用 `mj_forward` 更新运动学，不执行 `mj_step`，避免未配置驱动时自由下落。
MuJoCo 窗口右侧 Joints 面板可调整关节。外观为 group 1，碰撞几何为 group 0，
脚本默认隐藏碰撞几何。三个参考坐标系以固定 body 和同名 site 保留，默认显示 site 坐标轴。
图形窗口仍依赖桌面环境；WSLg COPY MODE 问题见本文末尾排障记录。

## 转换边界

### 临时臂手组合预览

```bash
.venv/bin/python tools/build_i10_hand_preview.py
.venv/bin/python tools/view_mjcf.py --with-hand
```

生成场景为 `assets/scenes/i10_rh56_fuel_cap_preview.xml`，包含 i10、传感器、adapter、RH56 右手、油箱盖、门板和立起的油箱。油箱保留分段碰撞网格，场景位置与有效性见[合同](assembly_contract.md#场景记录与有效性)；生成脚本中的 `BASE_TO_TANK_FACE` 控制底座与开孔外表面距离。修改油箱 URDF 后须重新生成组合场景，布局尚未做可达性或接触操作验证。

右手来源为 `assets/reference/aviation/third_party/correlllab_rh56_controller_h12/mujoco_inspire/inspire_right.xml`，沿用其网格、关节、耦合约束与六个执行器；不代表已与 RH56DFX 实物核对。

安装链为 `wrist3_Link → sensor_link → sensor_output_frame → adapter → rh56_provisional_mount → base`。当前尺寸、15 mm 插入关系、坐标变换及有效性统一见[装配合同](assembly_contract.md#当前装配变换)，网格来源与转换见[连接件说明](../assets/robots/adapter/README.md)。实物安装尚未标定，传感器与 adapter 缺少已确认的碰撞、惯量和测量模型，不能作为已验证接触仿真基线。

油箱盖与 `tank_filler_top` 同轴，灰色盖体下表面贴合环口端面；白色下部直径 40 mm，伸入直径 50 mm 的孔中 20 mm。盖体基准中心位于世界坐标 `(0, 1.037, 0.4)` m。这是几何展示摆放，未模拟螺纹锁紧、密封或抓握。仅运行 `mj_forward`；Joints 面板可查看关节姿态，手指耦合不会通过动力学自动求解。早期未加入门铰链时检查为 19 个关节；加入门铰链后为 20 个关节，仍为 6 个手部执行器。既有加载和前向计算检查通过。

- 保留 URDF 的关节层级、轴、位置限位和 i10 惯量。同步 SRDF 中经 URDF 父子关系核对的 `Adjacent` 碰撞排除；其他 SRDF 规划语义与 `Never` 非相邻排除不转换。
- 外观 DAE 的节点变换烘焙到 OBJ；使用 URDF 颜色，不复现 DAE 内部材质或纹理。
- 碰撞沿用 STL/基本几何，MuJoCo 的网格接触使用凸包，不等价于原三角面接触。
- 油箱盖缺少实测质量和惯量，按碰撞几何与默认密度 1000 kg/m³估算，
  仅供模型编译和预览。未标定摩擦、螺纹、接触或控制参数。
- 原 `i10_fuel_cap.xml` 场景中的盖体展示位置仍为 `(0.65, -0.35, 0.55)` 米；`--with-hand` 组合场景使用上述油箱口同轴摆放。
- 原臂盖场景未加入执行器；`--with-hand` 场景保留右手的六个参考执行器，尚无机械臂执行控制、硬件接口或拧盖任务，未进行动力学动作验收。

2026-09-16 验证：三个 MJCF 均编译成功；两个单体的全部 link 在零位及
0.37 rad 关节角下与 URDF 正运动学一致（位置/旋转矩阵容差 2e-6）；
油箱盖配色、白色部分直径、三个 site 位置和方向检查通过，EGL 离屏渲染成功。
本轮未验证桌面窗口可见性。

转换依据：[MuJoCo 建模文档](https://mujoco.readthedocs.io/en/stable/modeling.html)
与 [MJCF XML 参考](https://mujoco.readthedocs.io/en/latest/XMLreference.html)。

## RViz 与单体预览

在工作区根目录执行：

```bash
# RViz：机械臂与盖体、关节滑块
bash tools/view_i10_fuel_cap_rviz.sh
# RViz：盖体近景
bash tools/view_i10_fuel_cap_rviz.sh --cap-only
# MuJoCo：传感器或油箱单体
.venv/bin/python tools/view_mjcf.py --sensor-only
.venv/bin/python tools/view_mjcf.py --tank-only
```

RViz 读取 i10 与盖体的当前 URDF，默认 ROS domain 56。窗口中左键拖动旋转、滚轮缩放；关闭 RViz 会结束配套节点。早期未读取当前 URDF 的手写示意工具已于 2026-09-30 删除，使用上面的入口。

## 当前候选 IK 完整装配

入口 `.venv/bin/python tools/preview_cap_ik.py --view`，操作与报告位置见 [README](../README.md#当前候选的完整装配-ik-预览)。此工具把当前候选通过六轴 IK 接回完整机械臂，仅做端点前向计算；接近预型和历史实际闭合手指姿态可切换，不代表执行了物理动作。

## 抓取位姿与局部接触探索

当前人工调姿入口为 `.venv/bin/python tools/tune_cap_grasp.py`，操作步骤见 [README](../README.md#手动调抓姿)。它打开控制面板和 MuJoCo 视窗，不需要先完成批量搜索。

入口及运行命令见 [README](../README.md#抓取位姿探索)，参数与验证边界见[合同](assembly_contract.md#抓取位姿搜索合同)。该工具在内存中分离臂手装配并固定盖体/门板，原组合预览仍只运行运动学。窗口及离屏图默认显示记录的实际状态，而非穿入物体的空载驱动目标。

## 组合场景门板

门板作为当前组合场景的环境几何保留，默认打开姿态（135°）见[项目合同](assembly_contract.md#场景记录与有效性)。尺寸、质量和铰链属性仍为软件占位参数，不代表实物标定：

| 参数 | 当前占位值 |
| --- | --- |
| 门板 | 170 × 170 × 3 mm，质量 0.15 kg |
| 铰链 | 按用户修正，正对油箱时在右侧，竖直转轴，向外打开 |
| 安装 | 箱体局部铰链坐标 `(0.085, 0, 0.405)` m；闭合门板中面距开孔外表面 5 mm |
| 铰链范围 | 0°～100°，0°为关闭；不表示默认初态或执行动作 |
| 阻尼 / 干摩擦 | 0.015 N·m·s/rad / 0.002 N·m |

`tools/fuel_door.py` 仅构建组合场景中的门板与铰链，由 `tools/build_i10_hand_preview.py` 调用。组合预览只运行 `mj_forward`；门板避让空间和物理保持尚未验证。

独立开关门预览、自动开合检查、被动力矩回调及独立场景生成已按用户要求删除，不再提供对应运行入口。

## WSLg 窗口不可见排查

2026-09-16 本机检查结果：WSL 2.7.8.0、WSLg 1.0.73.2，内核
6.18.33.1-microsoft-standard-WSL2。

`/mnt/wslg/weston.log` 中有：

```text
rdp_allocate_shared_memory: Failed to open "/mnt/shared_memory/{...}" with error: Input/output error
RDP backend: use_gfxredir = 0
```

系统发行版中 `/mnt/shared_memory` 已挂载为 virtiofs，不能将其误判为
Ubuntu 中缺少目录，也不应通过在 Ubuntu 挂载 tmpfs 来替代跨系统共享内存。
RViz、关节状态和 TF 曾正常运行，Windows 端仍无法显示窗口；后台运行成功
不能作为用户可见的验证结果。

本机症状与以下上游记录一致，但未对本机执行底层 FUSE 跟踪来完全确认内部成因：

- https://github.com/microsoft/wslg/issues/1456
- https://github.com/microsoft/openvmm/issues/4274

### 恢复步骤

先保存所有 WSL 程序中的工作，然后在 **Windows PowerShell** 执行：

```powershell
wsl --shutdown
```

这会停止所有 WSL 发行版及当前开发会话。重新打开 Ubuntu 后执行：

```bash
cd ~/i10_rh56_refueling_ws
bash tools/view_i10_fuel_cap_rviz.sh
```

检查窗口实际能显示并交互，检查新的 `/mnt/wslg/weston.log` 是否恢复
`use_gfxredir = 1`。只关闭 RViz 或单独终止 Ubuntu 不等价于完整关闭 WSL VM。

若仍复发，在 Windows PowerShell 运行 `wsl --update`，完成后再次
`wsl --shutdown` 并重试；不能保证当前更新通道已包含上游修复。
截至此次诊断，尚未执行更新或完整重启，恢复效果待验证。
