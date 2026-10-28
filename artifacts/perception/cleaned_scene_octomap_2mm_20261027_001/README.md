# 实测表面体素的 OctoMap 与临时摆放

已保存 2 mm 原生 OctoMap 和单臂仿真摆放变换。[预览图](placement_preview.png)展示实测地图与完整臂手，红色是盖体，黄色是外向盖轴；原占位油箱、盖体和门板已从预览移除。

- `scene.bt`：80,298 个原始占据格，含盖体；复读展开后格号完全一致，没有新增自由空间或占据格。
- `octomap_scene.json`：文件摘要、坐标系、`world_from_camera`、`world_from_octomap` 和两方向盖体候选。加载时必须同时使用地图与整图变换。
- `voxel_labels.npz`：逐格盖体标记，共 1,504 格。OctoMap 本身没有语义标签。
- `validation_summary.json`、`validation.json`、`preview_validation.json`：转换、测试和预览的实际检查结果。
- `source_snapshot/`：本次实现及设置副本；`archive.json`：归档文件摘要。

摆放将 `direction_000` 实测盖体系对齐到现有单臂仿真的 `cap_grip_center`：世界系位置 `[0,1.007,0.600]` m，外向盖轴 `[0,-1,0]`。这是临时仿真位置。两臂安装变换仍未知，不能用于真机右臂目标；MTC 尚未加载这份地图，预览没有执行碰撞检查或动力学步进。

从项目根目录打开：

```bash
MUJOCO_GL=glfw .venv/bin/python tools/view_voxel_map_placement.py \
  --scene-dir artifacts/perception/cleaned_scene_octomap_2mm_20261027_001 --view
```

生成新结果和关键计算见[OctoMap 与地图摆放](../../../doc/point_cloud_voxelization.md#octomap-与地图摆放)，参数见[项目说明](../../../doc/assembly_contract.md#相机)。
