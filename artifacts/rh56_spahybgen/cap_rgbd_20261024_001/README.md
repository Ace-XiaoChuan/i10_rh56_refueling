# 油箱盖离线抓姿迁移结果

最新一轮生成 16 个满足关节范围和位姿数值条件的候选，几何筛选通过 0 个。当前候选存在盖体或周边环境干涉，尚不能作为有效抓取结果。

输入为仿真 RGB-D 与人工整盖标注，网络预测接触区域，优化器生成 RH56 抓姿。未开展机械臂规划或真机执行。

[候选交互页](views/index.html) · [接触预测](views/contact_map.html) · [优化动画](geometry_003/optimization.gif) · [MP4](geometry_003/optimization.mp4)

| 对照 | 网络模式 | 初始关节采样比例 | 迭代 | 数值有效候选 | 几何通过 |
| --- | --- | --- | --- | --- | --- |
| 1 | eval | 0.2 | 120 | 16 | 0 |
| 2 | upstream_train | 0.2 | 120 | 16 | 0 |
| 3 | upstream_train | 1.0 | 240 | 16 | 0 |

网络模式对照见 network_mode_comparison.json。作者模式使用当前输入的 BatchNorm 统计，不训练或改写网络权重。

输入、手模型和产物摘要核对通过；20 项检查通过。日志位于 logs/，源码快照位于 implementation_snapshot/。

下一步优先改进手型与接触组合、凹槽和环境干涉约束，再评价抓稳与旋拧能力。复现说明见项目 doc/cap_spahybgen_migration.md。
