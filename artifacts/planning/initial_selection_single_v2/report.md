# 初始到达构型筛选

同一观测初态、起点和预型；固定深度，累计盖角从 0° 开始。

| 候选 | 对照 | 共同通过盖角 ° | FCL 通过状态数 | 首个拒绝 |
| --- | --- | ---: | ---: | --- |
| candidate_000 | baseline | 189.0 | 255 | mujoco_robot_interference |
| candidate_001 |  | 224.0 | 261 | mujoco_robot_interference |
| candidate_012 |  | 189.0 | 255 | mujoco_robot_interference |
| candidate_014 |  | 189.0 | 255 | mujoco_robot_interference |
| candidate_016 |  | 189.0 | 255 | mujoco_robot_interference |
| candidate_017 |  | 163.0 | 203 | continuous_ik_not_converged |
| candidate_018 |  | 224.0 | 261 | mujoco_robot_interference |
| candidate_020 |  | 224.0 | 261 | mujoco_robot_interference |
| candidate_021 |  | 74.0 | 89 | continuous_ik_not_converged |
| candidate_023 |  | 163.0 | 195 | continuous_ik_not_converged |
| candidate_035 |  | None | 0 | fcl_robot_collision |
| candidate_074 |  | None | 0 | fcl_robot_collision |

最终状态：selected_initial_path_and_margin_verified。
所选 candidate_001 的真实 MTC 路径到达后，共同通过 224.0°，比原接近终点增加 35.0°。
真实路径：`artifacts/planning/initial_selection_single_v2/candidate_001/path/mtc.json`。

## 路径尝试

- candidate_001：通过

候选绕数是待规划目标，不能直接改写实际状态；有限搜索不证明全局最优。
几何通过不代表抓稳、允许松手、轴向柔顺、脱离或动力学执行；停止裕量未知。
