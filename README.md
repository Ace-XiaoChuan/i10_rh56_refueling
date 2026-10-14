# AUBO i10 + RH56DFX 油箱盖旋拧

用单臂 i10 和 RH56DFX 右手拧开油箱盖，完全取下后持续持盖。油箱门默认已打开，先完成仿真，再接真机。

## 项目做到哪里了

已有本机二维定位与分割、RH56 抓姿优化适配、MTC 几何路径、构型筛选和反馈状态机。接下来接深度与油箱盖自动抓姿，再实现抓稳、柔顺旋拧和脱离判断，完成取盖闭环。

当前需求、参数与进度统一见[项目说明](doc/assembly_contract.md)。

## 从哪里开始看

| 想了解什么 | 文档 |
| --- | --- |
| 项目目标、参数、进度和下一步 | [项目说明](doc/assembly_contract.md) |
| 感知、抓姿、规划和执行怎样连接 | [技术路线](doc/current_technical_route.md) |
| 对自己的图片定位和分割 | [图像操作说明](doc/grounded_sam_perception.md) |
| 整理标注和重新训练定位模型 | [定位数据](doc/cap_training_data.md)、[云端训练](doc/cap_dino_cloud_training.md) |
| 查看模型、调抓姿和闭合接触 | [模型与抓姿工具](doc/mjcf_inspection.md) |
| 规划接近、回放路径和筛选构型 | [MoveIt 工具](src/i10_rh56_moveit_config/README.md) |
| 理解空间接触预测和 RH56 优化 | [SpaHybGen 示例](doc/spahybgen_reproduction.md)、[RH56 适配](doc/rh56_spahybgen_adapter.md) |
| 查找模型、照片和设备资料 | [资产索引](assets/README.md)、[i10 资料](doc/hardware/aubo_i10_sources.md)、[相机资料](doc/hardware/vizum_camera_sdk_notes.md) |
| 研究方法和报告章节 | [合作方路线](doc/research_route_reference.md)、[报告目录](doc/refueling_twisting_report_outline.md) |

## 常用入口

以下命令在项目根目录执行。仿真和 SAM 2 使用 `~/mujoco-env`，项目 `.venv`、`.venv-perception` 为软链接；检测与 SpaHybGen 各有独立环境。

### 查看完整模型

```bash
.venv/bin/python tools/view_mjcf.py --with-hand
```

窗口可调整关节和观察几何，使用前向计算更新姿态。

### 图片定位与分割

```bash
~/mujoco-env/bin/python tools/segment_cap_local.py \
  --image pic/带油箱盖_正面.jpg \
  --image-source offline_image \
  --output-dir artifacts/perception/my_prediction
```

把 `--image` 换成自己的图片路径，输出目录选新名字。完成后打开结果目录中的 `index.html` 查看框和掩膜。

### 手动调抓姿

```bash
.venv/bin/python tools/tune_cap_grasp.py \
  --load artifacts/manual_grasp/user_selected_20260930.json
```

在控制面板调整盖体系中的手根位姿和预型，点击“应用调姿”和“闭合”观察接触。保存的候选供后续规划和执行评价使用。

### 回放 MTC 接近路径

```bash
.venv/bin/python tools/replay_cap_approach.py --view
```

Space 暂停/继续，R 重播，F 切换视角，C 显示碰撞几何。回放展示保存的几何路径。

### 状态机演示

```bash
python3 tools/run_cap_task.py --scenario early_detach
python3 tools/run_cap_task.py --scenario regrasp
python3 tools/run_cap_task.py --scenario support_unknown
```

这些演示使用模拟反馈，分别展示提前脱离、计划重新抓握和支撑未知时停止。`--events /tmp/cap-task.jsonl` 保存事件；`--timeout`、`--total-timeout`、`--max-segments` 设置软件等待和段数预算。成功退出 0，停止或取消退出 1。

## 目录与开发环境

| 目录 | 内容 |
| --- | --- |
| `src/fuel_cap_task/` | 感知、运动学、抓姿、任务接口及状态机 |
| `src/i10_rh56_moveit_config/` | ROS 2 MoveIt/MTC 规划 |
| `tools/` | 运行、转换、采集、训练和查看入口 |
| `tests/` | 软件、场景及规划检查 |
| `config/` | 当前可运行配置 |
| `doc/` | 项目与使用说明 |
| `assets/` | 模型、照片和原始资料 |
| `artifacts/` | 实验输出和模型回传 |
| `dependencies/` | 依赖清单与第三方源码 |

开发采用 Ubuntu 22.04、ROS 2 Humble、Python 3.10、colcon、MoveIt/MTC 和 MuJoCo。日常编辑打开 `refueling-lite.code-workspace`，终端仍从项目根目录运行。
