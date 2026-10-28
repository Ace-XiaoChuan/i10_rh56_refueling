# 汇报可用图片与动画

## 机械臂接近油箱盖

- 基准场景：[MP4 视频](arm_approach_baseline.mp4)、[GIF 动图](arm_approach_baseline.gif)。
- 目标位置变化后的场景：[MP4 视频](arm_approach_changed_scene.mp4)、[GIF 动图](arm_approach_changed_scene.gif)。

两段均为已有仿真路径回放，首尾各停留一秒，播放时长约 6 秒；播放速度不代表真实设备的执行速度。画面展示接近动作，手保持接近预型。

PPT 标题可写“机械臂接近动作仿真”。讲解可用：“已经能够在电脑中规划机器人靠近油箱盖的动作，并检查途中是否发生碰撞。”

视频从 `artifacts/e2_checks/fixed_grasp_20261020/` 中的基准和混合变化场景动画转换，来源及转换核对见 [media_validation.json](media_validation.json)。

## 灵巧手抓握方案

[已有抓握截图](../../artifacts/manual_grasp/user_selected_20260930.png)展示人工选定的抓握姿态及固定夹具下的手指闭合实验。

PPT 标题可写“灵巧手抓握方案初步验证”。讲解可用：“已经确定一种初步抓法，并在仿真中检查手指与油箱盖的接触情况。”

## 自动抓姿方法探索

- [油箱盖邻域接触预测](../../artifacts/rh56_spahybgen/cap_rgbd_20261024_001/views/contact_map.png)。
- [油箱盖抓姿候选对照](../../artifacts/rh56_spahybgen/cap_rgbd_20261024_001/views/candidate_comparison.png)。
- [油箱盖候选交互页](../../artifacts/rh56_spahybgen/cap_rgbd_20261024_001/views/index.html)。
- [优化过程 MP4](../../artifacts/rh56_spahybgen/cap_rgbd_20261024_001/geometry_003/optimization.mp4)、[GIF](../../artifacts/rh56_spahybgen/cap_rgbd_20261024_001/geometry_003/optimization.gif)。
- [论文原示例图片](../../artifacts/spahybgen/original_20261008_v2/preview.png)。
- [论文原示例交互页面](../../artifacts/spahybgen/original_20261008_v2/best_grasp.html)。
- [RH56 历史离线候选图片](../../artifacts/rh56_spahybgen/original_20261008/preview.png)。

PPT 标题可写“油箱盖自动抓取姿态生成初步实验”。讲解可用：“已将论文方法接入油箱盖仿真观测，可以预测接触区域并自动生成多种抓法，目前正在改进手指与周围结构的干涉问题。”

RH56 已按当前模型重新派生并复核。最新一轮油箱盖实验生成 16 个关节范围和位姿数值条件有效的候选，几何筛选通过 0 个；优化动画展示的是数值求解过程。当前使用仿真 RGB-D 和人工整盖标注，真实油箱盖点云尚未接通。抓稳与旋拧能力仍需后续验证。当前状态统一见[项目说明](../assembly_contract.md#spahybgen-与-rh56)。
