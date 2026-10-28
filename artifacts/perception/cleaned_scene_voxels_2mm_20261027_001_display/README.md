# 可交互的 2 mm 体素显示场景

完整 80,298 个体素按原材质组织成 33 个显示网格，改善原版逐格绘制时鼠标无响应的问题。2 mm 边长、原点、逐点归属及盖体标记均保留；`voxels.npz` 与原版逐字节一致。

从工作区根目录打开，原查看命令会自动选择本目录的显示缓存：

```bash
.venv/bin/python tools/view_point_cloud_voxels.py \
  --scene-dir artifacts/perception/cleaned_scene_voxels_2mm_20261027_001
```

在三维画面内操作：左键按住拖动旋转，右键按住拖动平移，滚轮缩放；主键盘 7/8/9 分别看完整场景、盖体区域和格子细节，Esc 返回自由相机。旧的 F/G/D 与 MuJoCo 原生功能冲突，当前取景操作改用数字键。查看器仅在切换取景时设相机，其余时间保留鼠标操作结果，不推进物理时间。

只删除两个相邻占据格之间的内部公共面。保留的 257,896 个方形面仍逐格对应 2 mm 网格，编译后逐三角形反算格角核对；没有补洞、平滑或对环境整体取凸包。33 个 mesh 沿用原来的深度伪彩色和盖体红色。

本目录 `scene.xml`、`scene.mjb`、`meshes/` 用于显示，显示网格的碰撞掩码关闭。碰撞数据继续保留在原版 `../cleaned_scene_voxels_2mm_20261027_001/` 的独立 box 模型中；后续碰撞计算应使用该 box 模型或占据格数据，不能使用本目录显示网格的凸包。

`preview.png` 为合批模型的 MuJoCo 三视图，`validation.json` 记录完整三角形与原始格角的对应检查；`interaction_validation.json` 和原始日志记录鼠标、取景键的实测；`source_snapshot/` 和 `manifest.json` 保留实现及文件摘要。

重新构建显示缓存到新目录：

```bash
.venv/bin/python tools/build_voxel_display.py \
  --scene-dir artifacts/perception/cleaned_scene_voxels_2mm_20261027_001 \
  --output-dir artifacts/perception/my_voxel_display
```

原始逐格显示可通过原版目录配合 `--box-geoms` 显式打开。诊断交互时增加 `--diagnostics`，终端会输出相机朝向、观察中心、距离及同步耗时。
