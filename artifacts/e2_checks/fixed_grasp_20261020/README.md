# E2 变化场景与失败检查

对象为当前 `config/fixed_grasp.json` 和来源绑定模型。结果为目标计算、端点 IK 与实际 MTC 几何路径检查。手保持接近预型，闭合和动力学执行由 E3 完成。

[可视化总览](visualization/index.html) · [汇总 JSON](summary.json) · [全部数值](scenarios.json) · [求解前预测](scenarios.predictions.json)

| 场景 | 目标计算 | IK 解数 | 路径检查 |
| --- | --- | --- | --- |
| 基准 | 与独立预测一致 | 12 | 通过真实 MTC + MuJoCo 复核 |
| 世界平移 (40, -60, 30) mm | 与独立预测一致 | 11 | 未规划（本项检查端点） |
| 油箱整体偏航 +20° | 与独立预测一致 | 11 | 未规划（本项检查端点） |
| 盖体绕自身 Z 转 +90° | 与独立预测一致 | 8 | 未规划（本项检查端点） |
| 远距不可达 | 与独立预测一致 | 0 | 未规划（本项检查端点） |
| 混合变化，固定种子 20261002 | 与独立预测一致 | 8 | 通过真实 MTC + MuJoCo 复核 |

预测与实际输出的最大矩阵元素误差为 2.44e-15。远距目标距离基座约 4.236 m，超过当前臂链保守半径上界 1.8337 m；其他案例的 `ik_not_found` 只能解释为既定搜索预算内未找到解。

| 实际路径 | 路点数 | 终点位置误差（mm） | 最大轴向横偏（mm） | 机器人最大穿透（mm） |
| --- | --- | --- | --- | --- |
| baseline | 606 | 0.0579 | 0.0200 | 0.0000 |
| mixed | 311 | 0.0604 | 0.0605 | 0.0000 |

## 失败情况

- [无效输入](scenarios.json)：29 项观测、候选、安装变换、距离或 MTC 字段错误全部被公共入口拒绝。
- [坐标故障注入](scenarios.json)：安装不取逆、固定世界外退轴、沿用旧目标三项错误均被独立预测发现。
- [回放拒绝记录](replay_rejections.json)：九份篡改轨迹分别改变目标、轴、朝向、指令、关节顺序、连续性、终点、来源和场景；加上失败规划报告，十项均被拒绝。
- [障碍案例](blocked/planner.log)：在预抓手根位置加半径 0.06 m 的诊断球，起点无碰撞，预抓状态与障碍碰撞。相同目标仍有数值 IK 解，但 MTC 报 Invalid goal state，在 move to pregrasp 阶段失败，进程返回 1。结果保留无完成路径标志，回放入口拒绝读取。
- 障碍场景直接传给新编译的规划节点，用于诊断注入；正常 launch 的场景形状和来源校验未修改。其 MuJoCo 派生场景和碰撞证据保存在 [scene.xml](blocked/scene.xml) 与 [obstacle_check.json](blocked/obstacle_check.json)。

## 重跑

在仓库根目录执行当前模型检查：

```bash
.venv/bin/python tools/check_grasp_scenarios.py --distance .1 \
  --output /tmp/e2_recheck/scenarios.json
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m unittest discover -s tools/tests -v
```

当前测试记录为 12 项公共适配/变化检查，加上 3 项原有 ArmIK 和 3 项原有随机场景检查，共 18 项通过；日志分别为 regression.log、arm_regression.log、scene_regression.log。

真实路径输入保存在 baseline/request.json 和 mixed/request.json；按规划包 README 的 approach.launch.py 入口运行，再用 tools/replay_cap_approach.py 验证。标准 ROS 入口须先构建同步模型。本次将 plan_approach 单独编译到临时目录，直接节点读取当前源码目录的 config，以免修改常用 install；已核对现有 install 的全部模型几何与当前来源一致（差异仅旧审计/来源记录），源与生成文件检查通过。params.yaml 保留本次实际参数及临时路径，重跑时需设置新的请求和输出路径。

RRTConnect 路径不保证每次逐点相同，应比较约束与残差。ROS 日志中的网络权限提示来自沙箱内 DDS；本地模型载入、OMPL/MTC 规划和结果写入已实际完成。

[代码入口](../../../tools/check_grasp_scenarios.py) · [变化回归](../../../tools/tests/test_grasp_scenarios.py)

来源摘要与求解器设置保存在报告；本次关键源码快照见 sources/。
