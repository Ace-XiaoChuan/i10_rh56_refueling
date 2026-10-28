# 实测清理点云的 2 mm MuJoCo 体素场景

本场景来自 `doc/scene_cleaned.pcd`。247,103 个观测点按相机系固定零原点归入边长 2 mm 的格子，共 80,298 格；全部点均保留逐点归属。每格是一个独立 MuJoCo box，完整边长 0.002 m，XML 半尺寸为 0.001 m。

从工作区根目录打开：

```bash
.venv/bin/python tools/view_point_cloud_voxels.py \
  --scene-dir artifacts/perception/cleaned_scene_voxels_2mm_20261027_001
```

鼠标旋转，滚轮缩放；F 查看完整场景，G 查看盖体区域，D 查看格子细节。默认看盖体区域。全部格子都在模型中，窗口和三张渲染图均不抽样。盖体观测点所在格子标红，其余按相机 Z 伪彩色着色；红色只是显示分组，仍保留碰撞掩码。

首次编译约八万个独立 box 需要数分钟。同版本 MuJoCo 的查看入口优先读取 `scene.mjb` 编译缓存，并逐格核对其中心和大小；直接打开 `scene.xml` 会重新编译。当前版本见 `result.json`。

文件说明：

- `preview.png`：并排展示三种取景。
- `full_scene.png`、`cap_region.png`、`voxel_detail.png`：MuJoCo 实际渲染，全模型均进入渲染场景。
- `scene.xml`：独立环境模型，不依赖占位油箱资产；`scene.mjb` 为编译缓存。
- `voxels.npz`：全部格号、相机系米制中心、逐格点数、逐点归属、盖体标记及显示颜色。
- `voxel_centers_camera_mm.ply`：毫米制格子中心，可与原始 PCD 直接叠加；此文件只显示中心，实际体积以 box 模型为准。
- `result.json`、`validation.json`：输入来源、坐标、尺寸及实际数据核对。
- `source_snapshot/`、`manifest.json`：实现快照和文件摘要。

MuJoCo world 定义为 SDK 原生相机系，输入仅进行毫米到米换算；相机图像上方对应 -Y。网格原点为 `[0, 0, 0]` m，后续不同边长结果另存新目录。

表面未命中的格子保持未知，不填充物体内部或遮挡区域，不对环境整体取凸包，不进行膨胀。当前模型用于查看环境体素；完整机械臂的规划与碰撞检查、左右臂坐标传递另行接入。查看器仅同步几何，不推进物理时间，也不连接硬件。

参数统一维护在 `config/active_point_cloud_scene.json` 的 `surface_voxelization`。重新生成到新目录：

```bash
.venv/bin/python tools/voxelize_cleaned_scene.py \
  --output-dir artifacts/perception/my_cleaned_scene_voxels
```
