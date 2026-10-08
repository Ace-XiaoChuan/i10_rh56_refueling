# 重新抓握构型筛选

固定盖姿态、预型、轴向深度和手盖关系；仅作几何验证。

| 候选 | 对照 | 共同通过追加角 ° | 通过状态数 | 首个拒绝 |
| --- | --- | ---: | ---: | --- |
| candidate_000 | before | 99.0 | 154 | mujoco_robot_interference |
| candidate_001 | old_after | 73.0 | 91 | continuous_ik_not_converged |
| candidate_005 |  | 99.0 | 154 | mujoco_robot_interference |
| candidate_007 |  | None | 0 | fcl_robot_collision |
| candidate_009 |  | None | 0 | fcl_robot_collision |
| candidate_016 |  | 99.0 | 154 | mujoco_robot_interference |
| candidate_023 |  | 134.0 | 157 | mujoco_robot_interference |
| candidate_034 |  | None | 0 | fcl_robot_collision |
| candidate_054 |  | None | 0 | fcl_robot_collision |
| candidate_055 |  | None | 0 | fcl_robot_collision |
| candidate_058 |  | None | 0 | fcl_robot_collision |
| candidate_063 |  | None | 0 | fcl_robot_collision |

最终状态：selected_path_and_margin_verified。
所选 candidate_023 经真实 MTC 路径后，追加共同通过 134.0°，累计 224.0°，比节点前增加 35.0°。
实际末态原始关节差最大 0.00017169461685878629 rad。

## 路径尝试

- candidate_023：通过

有限搜索不能证明全局最优；预算通过不等于最大转角或物理脱离角。
没有验证释放、抓稳、轴向柔顺、螺纹、动力学或真机；实际停转阈值与停止余量未知。
