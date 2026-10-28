# i10 + RH56 MoveIt 规划

本包提供完整臂手 URDF/SRDF、KDL 配置、场景和 MTC 无接触路径规划。轨迹导出后可在 MuJoCo 中回放；控制器执行待接入。[六轴状态桥](../aubo_joint_state_bridge/README.md)提供 RTDE 关节反馈、MoveIt 状态检查与显式选择当前臂起点的入口。

任务参数及诊断设置见[项目说明](../../doc/assembly_contract.md#几何规划与局部实验设置)。以下命令从项目根目录执行。

真机采用左臂拍照后让位、右臂旋拧。`config/robot.urdf` 仍为现有单臂规划模型；`dual_arm/robot.urdf` 已组合左臂与 T 型架，用于单独检查安装几何。已有控制盒采集与状态桥联机验证来自左臂。右臂规划前需使用右臂自身状态，并经两基座固定变换转换左臂观测，详见[双臂分工与坐标传递](../../doc/assembly_contract.md#双臂分工与坐标传递)。不能直接将左臂 `/joint_states` 接入该模型作为真机右臂起点。

实测 2 mm 环境 OctoMap 与独立盖体 `fuel_cap` 已接入 MTC。请求准备时传入 `--observed-scene`，会以两个组件替换旧油箱、盖体和门板；同一加载函数供规划、FCL 对照与 RViz 使用。当前临时单臂摆放下，`direction_000` 完整接近通过，`direction_180` 因拇指中节与环境相交被拒绝。命令、结果与源码说明见[实测场景接入 MTC](../../doc/point_cloud_voxelization.md#实测场景接入-mtc)。

## 双臂和 T 型架查看

安装参数与竖直方向来源见[项目说明](../../doc/assembly_contract.md#双臂与-t-型架安装参数)。生成器复用现有右臂网格、六轴关节和末端装配，左臂加 `left_` 前缀；两臂安装在两个相交圆柱构成的 T 型架上。双臂模型的世界系原点在支架正下方地面，+Z 竖直向上，与原单臂仿真世界系分开使用。

```bash
python3 tools/build_dual_arm_model.py
python3 tools/build_dual_arm_model.py --check
.venv/bin/python tools/view_dual_arm.py --render

source /opt/ros/humble/setup.bash
python3 tools/view_dual_arm.py --view
```

静态预览保存到 `dual_arm/preview.png`。RViz 提供两臂关节滑块，默认开启 TF 和基座坐标轴；默认显示左臂历史采集姿态和右臂已有规划参考姿态。查看器将本地网格路径转换为 `file://` URI 供 RViz 加载。右臂基座名为 `base_link`，左臂为 `left_base_link`。`dual_arm/robot.srdf` 保留原有 `arm`、`hand`，新增 `right_arm`、`left_arm` 和 `dual_arms`，支架与两臂均有碰撞几何。现有 MTC 入口仍读取单臂 `config/`；双臂规划和左臂相机几何尚未接入。

## 单臂规划模型生成、构建与检查

```bash
.venv/bin/python tools/preview_cap_ik.py
.venv/bin/python tools/build_moveit_model.py
python3 tools/check_moveit_model_sources.py
source /opt/ros/humble/setup.bash
colcon build --packages-select i10_rh56_moveit_config
source install/setup.bash

ros2 launch i10_rh56_moveit_config audit.launch.py coverage:=source \
  output_path:="$PWD/artifacts/planning_model/source_audit.yaml"
ros2 launch i10_rh56_moveit_config audit.launch.py coverage:=planning \
  output_path:="$PWD/artifacts/planning_model/planning_audit.yaml"
```

模型来源变化后重新生成、构建，再加载安装副本。生成使用共享 MuJoCo 环境；凸包通过系统 Python 的 SciPy 独立计算。

| 配置 | 内容 |
| --- | --- |
| `config/robot.urdf`、`robot.srdf` | 含传感器、adapter 和地面的规划覆盖 |
| `config/robot_source_coverage.*` | 与 MuJoCo 原碰撞覆盖对照 |
| `config/kinematics.json` | 六轴 `arm` 组的 KDL，末端为手根 `base` |
| `config/scene.json` | 油箱、盖体、开门和地面的场景快照 |
| `config/audit_reference.json` | 多构型 FK、手预型和碰撞参考 |
| `config/provenance.json`、`generated_hashes.json` | 输入、生成文件和摘要 |

手部保持给定预型，mimic 表示关节耦合。手部 velocity/effort=0 用于禁用该阶段的手轨迹。规划侧新增安装件和地面碰撞，诊断同时检查 MuJoCo 原覆盖。

## 接近与回放

```bash
.venv/bin/python tools/prepare_cap_approach.py \
  --start 0 -.5 1.2 0 0 0 --distance .1 \
  --output artifacts/planning/my_approach_request.json

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR="$PWD/log/mtc_approach"
ros2 launch i10_rh56_moveit_config approach.launch.py \
  request_path:="$PWD/artifacts/planning/my_approach_request.json" \
  output_path:="$PWD/artifacts/planning/my_approach.json"

.venv/bin/python tools/replay_cap_approach.py \
  --trajectory artifacts/planning/my_approach.json --view
```

`FixedState` 指定起点，`MoveTo` 用 RRTConnect 到预抓取构型，`MoveRelative` 用 CartesianPath 沿盖轴接近。准备工具从仿真估计接口读取盖体位姿，并通过公共 `GraspPreparation` 传递同一次观测的抓取及接近目标。MTC 的 `setIKFrame("base")` 使用世界系手根目标；数值 IK 使用法兰目标。接近期间手保持候选预型，请求中的闭合目标由后续 E3 执行后端在抓握阶段使用。完整字段说明见[项目说明](../../doc/assembly_contract.md#盖体坐标系)。

回放检查关节顺序、限位、连续性、预型、端点和模型来源，设置 qpos 后运行 `mj_forward`。Space 暂停/继续，R 重播，F 切换视角，C 显示碰撞几何。`--render-dir` 保存动画和静态图。

准备时加入 `--random-seed SEED` 可生成单个随机场景。请求会保存采样、观测和世界几何快照；MTC 和回放使用同一场景。

## 旋转空间与构型筛选

```bash
.venv/bin/python tools/check_rotation_margin.py
.venv/bin/python tools/check_regrasp_margin.py
.venv/bin/python tools/select_initial_configuration.py --verify-only
.venv/bin/python tools/select_regrasp_configuration.py --verify-only
```

| 工具 | 用途 |
| --- | --- |
| `check_rotation_margin.py` | 从接近终点扫描连续旋转，记录各关节余量与首个拒绝状态 |
| `check_regrasp_margin.py` | 比较同一盖姿态重新接近前后的可转范围 |
| `select_initial_configuration.py` | 筛选初始构型，规划接近，再检查实际末态余量 |
| `select_regrasp_configuration.py` | 筛选重新接近构型，规划外退/调整/接近，再检查实际末态余量 |

新计算使用 `--output-dir` 指定新目录；`--verify-only` 检查已有筛选结果。所选路径以原始关节角指定目标，保留关节绕数。对应原生入口为 `selected_approach.launch.py` 和 `selected_regrasp.launch.py`。

诊断固定轴向深度、手盖关系和接近预型，用来比较几何旋转空间。局部接触、螺纹轴向运动及停止余量按执行模块继续开发。

## 分段几何实验

```bash
.venv/bin/python tools/check_segmented_rotation.py \
  --output-dir artifacts/planning/my_segmented_run
MUJOCO_GL=glfw .venv/bin/python tools/preview_segmented_rotation.py \
  --summary artifacts/planning/my_segmented_run/summary.json --view
```

每段用连续 IK 检查旋转，重新接近用 MTC 规划外退、调整和轴向接近。查看器展示保存的通过前缀。结果中的手保持预型，实际释放和再次闭合由后续执行层实现。

单分支旋转对照保留 `tools/check_cap_rotation.py`、`rotation_audit.launch.py` 和 `tools/preview_cap_rotation.py`。
