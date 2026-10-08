# AUBO i10 + RH56DFX 拧油箱盖

油箱门默认已打开，单臂配合右手拧开并取下油箱盖，结束时持盖；首轮交付仿真。装回、开关门与加油操作不在当前范围。

## 文档入口

| 文档 | 用途 |
| --- | --- |
| [项目合同](doc/assembly_contract.md) | 唯一的当前需求、通道目标、装配参数、实现状态与待办记录 |
| [项目架构合同](doc/assembly_contract.md#项目架构合同) | 已确认的四模块架构与开发边界 |
| [功能清单与实施顺序](doc/assembly_contract.md#功能清单与实施顺序) | 四模块待开发功能、当前交付顺序与验证边界 |
| [长期目标与近期工作计划](doc/assembly_contract.md#长期研究目标与近期工作计划) | 博士路线的采用方式、新方法与工程对照；当前顺序查功能清单 |
| [合作方路线原文](doc/research_route_reference.md) | 用户提供的第五、六章参考提纲；项目采用方式以合同为准 |
| [当前技术路线](doc/current_technical_route.md) | VLM 整盖定位、分割/深度、SpaHybGen 接触与抓姿、旋拧评价、MTC 路径及力反馈执行 |
| [伟景相机与 SDK 交接](doc/hardware/vizum_camera_sdk_notes.md) | 型号疑点、已有软件检查和回公司后的最小验证步骤 |
| [模型预览与排障](doc/mjcf_inspection.md) | 启动、重建、验证边界及窗口排障 |
| [MoveIt 规划模型](src/i10_rh56_moveit_config/README.md) | 模型审计、MTC 接近规划与 MuJoCo 回放 |
| [资产索引](assets/README.md) | CAD/网格、照片和来源说明 |
| [i10 官方来源](doc/hardware/aubo_i10_sources.md) | 官方模型、驱动及版本边界 |

## 开发基线

采用“感知、规划、执行、任务流程”四模块架构，见[架构合同](doc/assembly_contract.md#项目架构合同)。2026-10-08 已将当前路线替换为 VLM 整盖定位与 SpaHybGen 自动抓姿生成，保留 MTC；采用状态、下一步与验收统一见[合同实施顺序](doc/assembly_contract.md#实施顺序与交付)。下方已有工具和报告保留为开发基础与手动抓姿对照，不代表新感知、自动候选或物理取盖已经实现。

## 快速预览

在工作区根目录运行：

```bash
# MuJoCo：完整臂手与油箱组合
.venv/bin/python tools/view_mjcf.py --with-hand
# RViz：机械臂与油箱盖
bash tools/view_i10_fuel_cap_rviz.sh
```

已有模型加载、前向计算和渲染检查。当前组合仅运行 `mj_forward`。任务状态机与 mock 后端已实现；拧盖动力学、MTC 执行和图像识别尚未接入，已确认参数尚未用于物理动作验收。

## MoveIt 规划与回放

已建立完整臂手 URDF/SRDF、KDL 配置与规划碰撞场景，并实现当前候选的 MTC 抓取前运动及轴向接近规划；实际轨迹可在 MuJoCo 中运动学回放。动力学执行尚未接入。入口和构建命令见 [配置包说明](src/i10_rh56_moveit_config/README.md)，实际结果见 [项目合同](doc/assembly_contract.md)。

```bash
# 回放当前已验证的 MTC 接近轨迹
.venv/bin/python tools/replay_cap_approach.py --view
```

Space 暂停/继续，R 显式重播，F 全景/近景，C 碰撞几何；结束保持末态。
[回放动画](artifacts/planning/views/index.html)和[验证报告](artifacts/planning/replay_validation.json)已保存。
回放仅运行 `mj_forward`，不代表抓稳、动力学执行或完整取盖通过。

模型派生使用项目 `.venv`，与当前候选记录版本一致；`~/mujoco-env/bin/activate` 是另一套环境，版本区别见合同。模型变更后先运行 `python3 tools/check_moveit_model_sources.py`，失效时重新生成并构建；RViz 不是依赖。

当前接近终点的单构型余量诊断入口如下；扫描设置和验证边界见[合同](doc/assembly_contract.md#任务点-2-首轮单构型旋转余量诊断)。工具按微元盖角求连续 IK，再审计完整规划碰撞覆盖，输出六个关节的上下限距离、共同通过前缀与首个拒绝状态。

```bash
.venv/bin/python tools/check_rotation_margin.py
```

结果保存到独立目录 `artifacts/planning/rotation_margin_single/`，阅读 `report.md`，逐状态数据见 `joint_margins.csv`。目录已存在时拒绝覆盖，可用 `--output-dir` 指定新目录。`--max-angle-deg` 和 `--angle-step-deg` 只改变数值扫描预算，实际设置写入报告。发现几何边界也是完成诊断，退出码为 0；输入失效或程序故障才返回非零。若完整覆盖审计因环境故障未完成，保留诊断与日志；另行完成同一诊断的审计后，可用 `--summarize-only` 汇总已有数据，不重新求解或规划。

该结果仅为固定深度、预型手的运动学诊断，不给出执行停转阈值，也不证明抓稳或脱离；构型筛选与重新抓握评价的最新范围见下方入口及合同。

重新抓握前后的余量比较复用已有第一次成功重新接近记录，在同一盖姿态下分别计算两侧的可转范围：

```bash
.venv/bin/python tools/check_regrasp_margin.py
```

来源选择、扫描预算和边界见[合同](doc/assembly_contract.md#任务点-2-第二轮重新抓握前后的旋转余量)。输出目录为 `artifacts/planning/regrasp_margin_single/`，入口 `report.md` 同时展示追加转角、累计转角及停止原因。`--segment-dir` 选择具有第一次成功重新接近节点的原分段目录；`--output-dir` 必须是工作区内不存在的新目录。`--max-additional-angle-deg` 指定两侧共同扫描预算，`--angle-step-deg` 指定数值步长；不改变执行参数。

`--prepare-only` 只保存两侧数值诊断，仍待完整覆盖审计；`--summarize-only` 只复核汇总已有诊断和审计，不重新运行 IK 或 MTC。工具保留原始关节绕数，不自动换分支、不覆盖已有结果；一次成功重新接近可能减少后续可转范围，报告如实记录。这里的“重新抓握”只表示已有几何路径的节点，实际松手和再次抓稳仍未验证。


重新抓握构型筛选会保留候选绕数、评价连续旋转前缀、指定预抓取关节目标进行真实 MTC 规划，再检查实际到达后的余量：

```bash
# 首次使用先构建新增规划器；原模型无需重新生成
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select i10_rh56_moveit_config
.venv/bin/python tools/select_regrasp_configuration.py
# 已有结果仅复核，不重新计算或规划
.venv/bin/python tools/select_regrasp_configuration.py --verify-only
```

有限候选预算、筛选规则及验证边界见[合同](doc/assembly_contract.md#任务点-2-第三轮重新抓握构型筛选与路径衔接)。输出位于 `artifacts/planning/regrasp_selection_single/`，入口为 `report.md`；保留候选池、未入选项、独立诊断、实际规划路径和拒绝记录。`--output-dir` 要求工作区内不存在的新目录，不覆盖或自动续跑。离线筛选不改变任务运行时的失败停止规则；几何路径和旋转余量不代表允许释放或实际抓稳。

## 初始到达构型筛选

在原单随机场景、仿真起点和预型下筛选初始机械臂构型，真实 MTC 两段接近后重新检查旋转空间：

```bash
# 新增原生规划器须先构建，模型无需重新生成
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select i10_rh56_moveit_config
.venv/bin/python tools/select_initial_configuration.py
# 只读复核保存的来源、路径和共同通过范围
.venv/bin/python tools/select_initial_configuration.py --verify-only
```

数值预算、失败记录、采用状态和物理边界统一见[合同](doc/assembly_contract.md#任务点-2-第四轮初始到达构型筛选)。结果入口为 `artifacts/planning/initial_selection_single_v2/report.md`；首版未完成的拒绝记录、算法快照另存于 `artifacts/planning/initial_selection_single/`，不计为通过。`--trajectory` 选择原始两段接近基线，`--output-dir` 必须为工作区内不存在的新目录；不覆盖或自动续跑。可用所选 `path/mtc.json` 的真实几何轨迹单独回放，保留预型且只运行 `mj_forward`。筛选结果不写入正式任务参数，不能作为抓稳、脱离或允许松手的证据。

## 状态机演示

使用 Python 3.10+，无需安装额外依赖；此入口只产生合成反馈，不驱动 MuJoCo 或硬件。

```bash
# 正常流程，输出结果及事件；可选写入 JSONL 文件
python3 tools/run_cap_task.py --events /tmp/cap-task.jsonl
# 未达到参考角度就脱离，以及一次计划重新抓握
python3 tools/run_cap_task.py --scenario early_detach
python3 tools/run_cap_task.py --scenario regrasp
# 支撑未知：保持已有抓握并等待人工，不调用释放
python3 tools/run_cap_task.py --scenario support_unknown
# 在拧转阶段注入失败：停止并等待人工
python3 tools/run_cap_task.py --fail-at twist
# 执行中取消；Ctrl+C 也会请求停止
python3 tools/run_cap_task.py --delay 0.2 --cancel-after 0.7
# 行为测试
PYTHONPATH=src python3 -m unittest discover -s tests -v
```

成功退出码为 0，失败/取消为 1。`--timeout` 设置演示阶段及停止回执等待上限；`--total-timeout` 与 `--max-segments` 限制总软件等待和旋拧段数，默认段数对应所选有限脚本。这些均不是物理控制参数，入口通过 `RunLimits` 显式传入。状态机代码见 [state_machine.py](src/fuel_cap_task/state_machine.py)，新执行回执见 [execution_contract.py](src/fuel_cap_task/execution_contract.py)，位姿与参数继续复用 [interfaces.py](src/fuel_cap_task/interfaces.py)。旧执行回执不作为新版有效回执。实际检查结果见[合同实施记录](doc/assembly_contract.md#任务点-1-实施与验证)和[模拟报告](artifacts/task_orchestration/feedback_v1/report.json)；真实规划、支撑判据、控制交接及保持能力仍待后端接入验证。

## 有边界的随机场景

```bash
# 默认：基线 + 四类各4个样本，逐样本保存 IK/干涉结果
.venv/bin/python tools/sample_cap_scenes.py --view
# 已生成的17个样本直接查看，无需重新求解
.venv/bin/python tools/sample_cap_scenes.py --load-report artifacts/random_scenes/report.json --view
# 更换随机种子或样本数；建议使用独立输出文件保留旧结果
.venv/bin/python tools/sample_cap_scenes.py --seed 1 --per-group 8 --output artifacts/random_scenes/seed1.json
# 从已有报告生成图集
.venv/bin/python tools/sample_cap_scenes.py --load-report artifacts/random_scenes/report.json --render-dir artifacts/random_scenes/views
```

采样范围统一见[合同](doc/assembly_contract.md#有边界的随机场景)。四类分别改变位置、油箱偏航、盖体朝向以及混合变化，目标位姿通过仿真估计接口更新。位置约束针对开孔中心，不是整个箱体包围盒；旋转后整体平移到指定开孔位置。

窗口按 N/P 切换样本（关闭旧视窗并打开该样本的独立模型），[/] 切换该样本的逆解，F 全景/近景，C 碰撞几何；终端显示样本参数及结果，关闭窗口退出。这里仅显示接近预型，不将原场景闭合采样当成新场景的接触结果。

报告保留全部合法采样及端点结果，逐样本写检查点；中断时 `complete=false`，可以查看已完成部分，不自动重试。`endpoint_clear` 只表示至少一个收敛逆解在启用碰撞几何中没有超过软件诊断尺度的穿透；`endpoint_interference` 表示已找到的逆解都有该类干涉；`ik_not_found` 仅表示本次数值搜索未找到解，画面显示初始机械臂及目标标记。未验证接近路径、完整拧盖或物理持稳。

首次种子0的17个样本均为 `endpoint_clear`，不据此宣称整个区域可达。图集入口：`artifacts/random_scenes/views/index.html`。

## 当前候选的完整装配 IK 预览

```bash
# 求当前候选的机械臂逆解并打开完整装配（不执行运动）
.venv/bin/python tools/preview_cap_ik.py --view
# 仅求解并保存报告、离屏图
.venv/bin/python tools/preview_cap_ik.py --render-dir artifacts/arm_ik/views
```

默认读取已选手盖候选，报告位于 `artifacts/arm_ik/current_candidate.json`。窗口按 N/P 切换逆解，O 查看接近预型，G 查看候选中记录的实际闭合手指姿态，F 切换全景/近景，C 显示碰撞几何。切换只显示端点，不表示执行了接近或闭合轨迹。

当前首选解的手根误差约 0.017 mm / 0.020°；已同步 i10 的 6 对 SRDF 相邻碰撞排除，当前首选端点在已启用的碰撞几何范围内未检测到穿透。传感器/adapter 与地面碰撞、接近路径、负载和完整拧转仍未验证，详见[合同](doc/assembly_contract.md#当前候选的机械臂-ik-与完整装配预览)。

## 手动调抓姿

```bash
.venv/bin/python tools/tune_cap_grasp.py
# 重新载入自己保存的候选初态
.venv/bin/python tools/tune_cap_grasp.py --load artifacts/manual_grasp/候选文件名.json
```

已保存用户选定的手动候选（未通过抓稳验收），可直接打开：

```bash
.venv/bin/python tools/tune_cap_grasp.py --load artifacts/manual_grasp/user_selected_20260930.json
```

打开数值控制面板和 MuJoCo 视窗。位置为盖体系毫米，姿态为固定轴 XYZ RPY 角度，六通道字段仅编辑接近预型；最终闭合通道只读。

1. 修改位置、姿态和预型，点击“应用调姿”；初值仅是几何种子。
2. 点击“闭合”，观察实际手指运动、接触点、拇/食/中末端对握梁法向力及其他接触。初态干涉会阻止实验并列出部分碰撞对。
3. “暂停”保留当前实际状态；“重置”返回已应用的预型。调整后可由人再次开始独立实验。
4. “保存已应用候选”将当前已应用位姿、预型和实验记录保存到 `artifacts/manual_grasp/` 的独立时间戳文件；未应用的输入框编辑不会保存。载入候选恢复初态，不重放或认定上次抓取成功。

闭合时禁用调姿、保存与载入；可暂停或重置。MuJoCo 视窗支持鼠标旋转/缩放，可勾选显示碰撞几何；关闭任一窗口结束工具。模型参数未标定，手根和盖体固定；保存不修改正式任务参数，不证明抓稳、持起、拧盖或机械臂可达。具体边界见[合同](doc/assembly_contract.md#手动抓姿工具)。

## 抓取位姿探索

固定已确认的闭合通道目标，联合搜索适度张开的接近预型和手根相对盖体系的位姿。默认用当前模型参数做局部接触探索：手根、盖体和门板固定，手指由位置伺服器驱动，记录实际姿态与接触力。该实验不代表抓稳、持起或拧盖通过，参数和边界见[合同](doc/assembly_contract.md#抓取位姿搜索合同)。

2026-09-30 已清理历史无效接触报告和旧静态诊断图。下面的重放命令只适用于之后重新生成并核对有效的报告；当前没有已选定的有效抓取结果。

```bash
# 搜索并保存报告；已有 .venv 依赖即可，无新增依赖
.venv/bin/python tools/search_cap_grasp.py --output artifacts/grasp_search/contact_report.json
# 重放报告中的实际闭合状态；N/P 切换候选，O/G 查看候选预型/实际末态，[ / ] 查看采样
.venv/bin/python tools/search_cap_grasp.py --load-report artifacts/grasp_search/contact_report.json --view
# 无窗口生成实际末态三视图及 HTML 查看页
.venv/bin/python tools/search_cap_grasp.py --load-report artifacts/grasp_search/contact_report.json --render-dir artifacts/grasp_search/contact_views
# 场景、位姿转换和局部接触行为测试
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests/scene -v
```

`--trials` 控制候选接触实验数量，`--keep` 控制保存的详细候选数量。报告输出 `cap_from_hand`，其中 `base` 为手根，盖体系遵循估计接口约定。`triple_contact_observed` 只表示末段记录到拇指、食指和中指接触力；实际穿透、其他部位接触和速度同时保留，未设物理抓稳通过门槛。工具不会修改任务抓取参数或原始模型。

`--static-diagnostic` 仅查看强设空载目标形状的旧几何诊断；目标形状穿透不能据此否决实际抓取，不能将该模式的结果用于抓稳判断。

## 目录

- `doc/`：合同、预览说明及硬件来源。
- `assets/`：当前模型、原始资料及照片；`assets/reference/aviation/` 为航插参考副本，使用 `COLCON_IGNORE` 隔离，FR3 参数不作为 i10 基线。
- `tools/`：模型转换、生成和预览脚本。
- `src/fuel_cap_task/`：独立 Python 任务状态机、接口与 mock 后端；尚未封装 ROS 包。
- `tests/`：状态机行为测试。
- `config/`：预览配置及后续运行配置。
- `dependencies/`：依赖版本记录。

技术路线为 Ubuntu 22.04、ROS 2 Humble、Python 3.10、colcon、MoveIt/MTC 与 MuJoCo。新环境的预览依赖安装见使用说明。
