# Correll Lab RH56DFX Reference Materials

本目录保存从 Correll Lab 相关仓库筛选出的 RH56DFX / Inspire Hand 参考资料。
这些文件不是本仓库自己的实现，也不应直接混入 `src/` ROS 2 package。当前用途是
为后续 RH56DFX MuJoCo 仿真、抓取实验、真实手控制接口和汇报材料提供参考。

## Source

主仓库：

```text
https://github.com/correlllab/rh56_controller/tree/h12
commit: 3db830161fcae85635126a5b0d0de8449a32b870
```

MuJoCo 模型子仓库：

```text
https://github.com/correlllab/h1_mujoco
commit: fa8795a1ce1c2be418e8d87b72761c3a452efcd6
```

下载日期：2026-07-01

## Contents

```text
third_party/correlllab_rh56_controller_h12/
├── mujoco_inspire/              # RH56DFX / Inspire Hand MuJoCo XML、assets 和示例场景
├── rh56_controller_selected/    # 从 rh56_controller 中筛选的相关 Python 参考脚本
├── docs/                        # 原仓库中与仿真、真实手、ROS 2 和力控相关的 README
└── h1_mujoco/                   # h1_mujoco 的 README 和 LICENSE
```

## Most Relevant Files

当前最值得优先看的 MuJoCo 模型：

```text
mujoco_inspire/inspire_right.xml
mujoco_inspire/inspire_left.xml
mujoco_inspire/inspire_scene.xml
mujoco_inspire/inspire_grasp_scene.xml
mujoco_inspire/inspire_force_scene.xml
mujoco_inspire/inspire_right_floating.xml
mujoco_inspire/assets/
```

当前最值得优先看的控制和仿真脚本：

```text
rh56_controller_selected/mujoco_bridge.py
rh56_controller_selected/grasp_geometry.py
rh56_controller_selected/hand_mirror.py
rh56_controller_selected/real2sim_viz.py
rh56_controller_selected/rh56_hand.py
rh56_controller_selected/rh56_driver.py
rh56_controller_selected/kinematics.py
```

当前最值得优先看的说明文档：

```text
docs/README_SIM.md
docs/README_GRASP_VIZ.md
docs/README_REAL.md
docs/README_ROS2.md
docs/README_FORCE_CONTROL.md
docs/THUMB_TANGENTIAL.md
```

## Relationship To This Project

当前 MVP-1 仍然只实现 fake 手部命令：

```text
open / preshape / close / hold / release
```

这些命令后续可以映射到 MuJoCo 模型中的 position actuator 或 keyframe。例如：

```text
open     -> open-like ctrl / keyframe
preshape -> 30mm or 40mm keyframe
close    -> closed ctrl / grasp keyframe
hold     -> keep current ctrl
release  -> return to open ctrl
```

因此，本目录更适合服务后续阶段：

```text
MVP-3: MuJoCo 单臂 + 夹爪化 RH56DFX Pick-and-Place
Post-MVP: RH56DFX 抓取、接触、力反馈和真实手控制接口
```

## Notes

- `mujoco_inspire/` 中的 XML 是 MuJoCo MJCF，不是 URDF，也不是 MoveIt 配置。
- `inspire_right.xml` / `inspire_left.xml` 适合作为单手模型入口。
- `inspire_grasp_scene.xml` 和 `inspire_scene.xml` 更适合看抓取和接触场景。
- `ur5_inspire.xml`、H1 相关 XML 可参考组合方式，但机械臂不是本项目的 FR3。
- `rh56_controller` 主仓库未在克隆根目录发现 LICENSE 文件；使用其中代码前应再次确认授权。
- `h1_mujoco` 的 LICENSE 已保存在 `h1_mujoco/LICENSE`。
