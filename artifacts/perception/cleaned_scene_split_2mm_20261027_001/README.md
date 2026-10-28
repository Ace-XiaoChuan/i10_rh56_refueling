# 环境 OctoMap 与独立盖体对象

已将同一份实测占据数据分成两个独立组件。位置继续使用既有单臂仿真临时变换。

| 组件 | 结果 |
| --- | --- |
| `environment.bt` | 78,796 个 2 mm 占据格，包含两个归属未定的混合格 |
| `cap_collision_object.json` | 一个 ID 为 `fuel_cap` 的 MoveIt CollisionObject，由 1,502 个原边长 box 组成 |
| `octomap_scene.json` | 两个组件的路径、摘要与统一坐标变换 |
| `voxel_components.npz` | 原始格号和完整点归属，0=环境、1=纯盖体、2=混合格 |
| `mixed_voxels.json` | 两个混合格的所有原始点及语义未定说明 |

两部分不重叠，并集保持原来的 80,298 格，没有重新体素化、填充或删掉观测格。5,742 个盖体裁剪点中，5,740 点对应独立盖体；另外两点位于混合格，连同对应裁剪外点保留为环境。原裁剪标签没有被改写。

[预览图](placement_preview.png)中，青色为环境、红色为独立盖体、黄色小格为混合格；盖轴也以黄色标出。显示网格仅用于预览，实际盖体碰撞几何在 CollisionObject 的 box 数组中。

当前没有修改 MTC 场景入口，没有放行任何接触或附着对象。后续加载应使用 `environment.bt` 与 `world_from_octomap`，并添加独立盖体；仅替换实测区域对应的旧环境几何。两臂安装变换继续未知，当前摆放不用于真机。

从项目根目录检查与查看：

```bash
source /opt/ros/humble/setup.bash
.venv/bin/python tools/check_split_voxel_scene.py \
  --scene-dir artifacts/perception/cleaned_scene_split_2mm_20261027_001
MUJOCO_GL=glfw .venv/bin/python tools/view_voxel_map_placement.py \
  --scene-dir artifacts/perception/cleaned_scene_split_2mm_20261027_001 --view
```

关键实现、输入输出与复现方法见[盖体与环境分开管理](../../../doc/point_cloud_voxelization.md#盖体与环境分开管理)。实际检查保存在 `validation_summary.json`、`validation.json`、`ros_message_validation.json` 与 `preview_validation.json`，源码和设置在 `source_snapshot/`。
