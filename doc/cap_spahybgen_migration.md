# 油箱盖的 SpaHybGen 离线抓姿生成

当前已接通仿真 RGB-D、人工整盖标注、原方法体素构建、网络预测、RH56 候选优化和独立手几何检查。输出可以直接在网页中旋转查看，并包含优化过程动画。当前使用仿真观测，真实 PCD 尚未接入；本阶段没有机械臂规划或真机执行。

## 查看当前结果

[总览](../artifacts/rh56_spahybgen/cap_rgbd_20261024_001/index.html) · [交互候选](../artifacts/rh56_spahybgen/cap_rgbd_20261024_001/views/index.html) · [接触预测](../artifacts/rh56_spahybgen/cap_rgbd_20261024_001/views/contact_map.html) · [优化动画](../artifacts/rh56_spahybgen/cap_rgbd_20261024_001/geometry_003/optimization.gif)

当前生成了 16 个满足关节范围和位姿数值条件的候选，独立几何筛选通过 0 个，主要问题是手与盖体及周边环境干涉。扩大初始关节采样和迭代预算后仍未取得几何通过候选。网页和图片明确显示筛选状态，优化动画表示求解变量变化。

这组结果用于分析迁移效果。下一步应改进接触区域与手型适配、环境干涉约束，并与当前人工抓姿比较；抓稳、承载和旋拧能力需要后续评价。当前参数和范围统一见[项目说明](assembly_contract.md#油箱盖离线迁移)。

## 数据怎样连接

1. 读取同一帧 RGB、光轴 Z 深度、有效标记、内参及相机到世界变换。人工在 RGB 中标注整个可见盖体；标注通过图像摘要绑定输入。
2. 用整盖观测点的中位数放置局部体积。该位置用于局部坐标，不是完整盖体位姿估计。体积为 0.4 m、80³，保持真实尺寸，Z 朝观察者。
3. 调用上游 `VoxelVolume.integrate()` 对完整深度雕刻，保留周边场景。另存由观测支持的空闲格，未观测区域不自动当作空闲。现有“点云表面归格”工具的输出不直接替代这个输入。
4. 网络预测接触质量、接触方向和中心分布。接触候选限制在整盖观测的 10 mm 邻域，中心限制在 20 mm 邻域；没有预设握梁抓点。
5. 优化手根位置、朝向和六个驱动角。驱动通过耦合展开为十二个关节，迭代后投影到可行范围，末态重新计算排序。
6. 用独立的 MuJoCo 手和当前仿真环境评价候选，检查穿透及至少两个不同手指靠近盖体。此时才读取盖体几何，生成过程不读取 `evaluation_only.npz` 或盖体真值位姿。

核心实现：[输入体积](../src/fuel_cap_task/cap_spahybgen_input.py)、[单目标优化](../src/fuel_cap_task/cap_spahybgen_optimizer.py)、[生成入口](../tools/generate_cap_grasps.py)、[独立几何检查](../tools/check_cap_grasp_geometry.py)。

## 网络模式

对照发现，当前权重使用 `eval()` 时，原论文示例的预测也接近零；作者默认模式下，原示例和油箱盖输入均能输出明显响应。因此当前基线沿用作者的训练模式前向计算，并固定随机种子、每次重新加载权重。前向在 `no_grad()` 内进行，没有网络参数优化，也不保存修改后的权重。

该模式会使用当前输入的 BatchNorm 统计。迁移结果应按这一设置解释，接触得分不是抓取成功概率。原始对照保存在结果目录的 `network_mode_comparison.json`。

## 重新运行

在项目根目录使用新的输出目录：

```bash
python3 tools/run_cap_spahybgen_demo.py \
  --output-dir artifacts/rh56_spahybgen/my_cap_rgbd

.venv/bin/python tools/finalize_cap_spahybgen_demo.py \
  --result-dir artifacts/rh56_spahybgen/my_cap_rgbd
```

入口顺序保存三组模式与初始手型对照、几何评价、展示及检查日志。中断后可在核对来源一致的前提下给第一个命令增加 `--resume`，续用已经完成的阶段。输入或源码发生变化时，应使用新目录。

按步骤单独运行时：

```bash
CAP_DEMO_DIR=artifacts/rh56_spahybgen/my_cap_rgbd

.venv/bin/python tools/build_rh56_spahybgen_model.py \
  --output-dir "$CAP_DEMO_DIR/hand_model"

.venv/bin/python tools/capture_cap_rgbd.py \
  --width 1280 --height 960 --lookat-world-m 0 .94 .62 \
  --distance-m .9 --azimuth-deg 90 --elevation-deg -20 \
  --output-dir "$CAP_DEMO_DIR/observation"

.venv-spahybgen/bin/python tools/generate_cap_grasps.py \
  --observation-dir "$CAP_DEMO_DIR/observation" \
  --model-dir "$CAP_DEMO_DIR/hand_model" \
  --config config/spahybgen_cap_demo.json \
  --output-dir "$CAP_DEMO_DIR/run"

.venv/bin/python tools/check_cap_grasp_geometry.py \
  --result-dir "$CAP_DEMO_DIR/run" --output-dir "$CAP_DEMO_DIR/geometry"

.venv-spahybgen/bin/python tools/render_cap_grasps.py \
  --result-dir "$CAP_DEMO_DIR/run" --geometry-dir "$CAP_DEMO_DIR/geometry" \
  --output-dir "$CAP_DEMO_DIR/views"
```

当前配置绑定 1280×960 的示例 RGB。新采集图片变化后，需重新检查整盖标注并保存对应图像摘要。配置中的范围不能未经检查套用到其他图片。

当前归档来源核对：

```bash
.venv-spahybgen/bin/python tools/generate_cap_grasps.py \
  --output-dir artifacts/rh56_spahybgen/cap_rgbd_20261024_001/run_003 \
  --verify-only
```

生成使用 Python 3.8 的独立 SpaHybGen 环境；模型派生、仿真采集与几何检查使用共享 Python 3.10/MuJoCo 环境。所有实验输出保存到新目录，现有固定抓姿和历史结果继续保留。
