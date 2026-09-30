# M3-07/M3-08 模型来源与边界

`mvp3_connector_assembly.xml` 由 `tools/generate_fr3_rh56_scene.py` 生成。FR3 基底来自
MuJoCo Menagerie 的 `franka_fr3/fr3.xml`，RH56 子树仍由经过 RViz 检查的
FR3 + RH56DFX Xacro 生成的模板场景导入。这样 FR3 的 MJCF 关节、动力学默认值和 position
actuator 以固定上游版本为唯一来源，而 RH56、connector、socket 与 fixture 仍由本项目合同
追加；输出 XML 只是运行时快照，不是手工维护源。

`rh56_grasp_tcp` 的 site 同样由
`assembly_bringup/config/mvp3_connector_assembly.yaml` 的
`mujoco_contract.rh56_grasp_tcp` 生成；当前位置来自闭手 collision mesh 扫描，
不是实物 TCP 标定。

当前冻结的 FR3 上游来源：

```text
repository: https://github.com/google-deepmind/mujoco_menagerie
commit: da76818e269b82289eba39808e2fb91d679d6994
path: franka_fr3/fr3.xml
license: Apache-2.0
```

上游与模板目录只用于生成，不是运行时依赖；运行时 MJCF 和全部 mesh 都安装在
`assembly_mujoco` package share 中。用已下载的上游副本重建时，命令形如：

```bash
python3 tools/generate_fr3_rh56_scene.py \
  --fr3-mjcf /path/to/mujoco_menagerie/franka_fr3/fr3.xml \
  --rh56-template mjcf/mvp3_connector_assembly.xml
```

RH56 模板生成时的模型来源：

```text
franka_description branch: fr3-rh56dfx-rviz
franka_description base commit: cea9d48 Add RH56DFX right hand visualization
fr3.urdf.xacro sha256:
  5a1bd50e95865416b83036afe085690adb3967a628c473acf26bb86bddf54291
franka_robot.xacro sha256:
  4e86eccfa40f70273352da5b03c3e0c5dbaab80348118e047767cc210ed8f87e
rh56dfx_hand.xacro sha256:
  8c588e3307d26907b759c3ab0db149d1496ef872b526c4183a9ce35402977738
rh56dfx_macro.xacro sha256:
  a397aed9943f9dd4ce22c071ba7c9d1d0f57b6f7daf98d25b0a8022b95ab7c0c
```

## 临时安装基线

```text
parent: fr3_link8
child: fr3_rh56dfx_right_hand_root
xyz: [0.0, 0.015, 0.0] m
rpy: [0.0, 0.0, 3.14159] rad
```

这组数值只是在 RViz 中整理模型时采用的直接安装基线，没有包含真实转接件链，也没有
机械测量依据。后续获得 adapter/CAD/实物测量后必须替换，不能用于真机、精确碰撞或
接触结论。

M3-08 保留 RH56 全部 12 个转动关节，并将 6 个独立驱动 joint 与其余 6 个从动
joint 的 RH56 Xacro mimic 关系写为 MuJoCo joint equality。handoff snapshot 必须
包含配置指定的 FR3 7 个与 RH56 12 个 physical joint，不能少传、重传或传入未知名称。

M3-20 为 6 个 RH56 driver joint 增加 MuJoCo `position` actuator。它们的厂商通道
顺序、joint/actuator 名称、张开/闭合弧度端点、力限及预设，唯一来源是
`assembly_bringup/config/mvp3_connector_assembly.yaml` 的 `mujoco_contract.hand_control`；
生成器会从这份合同重建运行时 MJCF。该位置伺服只证明仿真姿态可复现，不代表真机
0--1000 角度标定或抓取力标定。

M3-08 的 `plug_frame` / `socket_frame` 是分别挂在对应 connector body 的 MuJoCo
site。当前 MVP 仿真基线是竖直并排：原始网格局部 `+Y` 是插接轴，connector body 绕
局部 `+X` 旋转 90 度后，网格 `+Y` 与世界 `+Z` 重合。site 自身绕局部 `-X` 旋转 90
度，使其局部 `+Z` 同样沿原始网格 `+Y`。后续插入判据应要求两 frame 的轴向和完整姿态
对齐。它们不是 STL 原点或包围盒中心，数值仍待 CAD/实物标定替换。

蓝色 socket 的 `0.05 kg` 和对角惯量仅用于让当前唯一 dynamic/free body 可加载；
棕色 plug 固定在桌面 fixture 上，作为 MVP-4 mating target。两者均不得采用旧版的悬空
布局。质量、惯量、摩擦、接触参数和 reset 状态仍属于后续标定任务。
