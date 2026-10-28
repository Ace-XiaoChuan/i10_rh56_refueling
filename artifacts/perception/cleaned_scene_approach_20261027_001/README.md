# 两方向手模型的直线接近回放

双击 `index.html`，用浏览器离线查看。默认显示 0° 方向的外退起点；下拉菜单可选 180°、两个方向同时显示或只看点云。两个方向是同一只手的候选姿态。

- “播放接近 / 暂停”：观察完整接近过程。
- 滑块：停在任意进度旋转查看；0% 为距抓取位 100 mm 的起点，100% 为原抓取位。
- “起点 / 终点”：直接对比两端。
- “抓取区域 / 完整场景”：调整观察范围。切换方向或拖动进度会暂停播放。

场景使用用户指定的 `doc/scene_cleaned.pcd`，保留全部 247,103 点，其中 5,742 个盖体点标为红色。两方向均保持 `[0, 0, 650, 650, 900, 0]` 手指预型和固定朝向；不演示闭合。动画速度仅用于观察，与机器人执行速度无关。

接近线表示手根的运动，与实测盖体外向轴平行。起点为原手根抓取位加 `100 * axis_camera` 毫米，终点回到原位；每一帧对整个网格加相同的绝对平移。当前外退向量为 `[-18.924321509233245, 17.944625913891386, -96.53942436241097]` 毫米。点云保持不动，朝向及手型不变，不加载占位油箱场景。

`result.json` 保存原抓取候选和 `approach_replay`；点云、PLY 及外退向量使用毫米，起终点位姿矩阵的平移使用米。同目录的 `hand_000.ply`、`hand_180.ply` 保留终点网格，与用户已确认的静态叠加版本逐字节一致，可与 `scene_cleaned.pcd` 同时加载到 CloudCompare。

`validation.json` 记录点云、网格和 100 mm 位移的数值核对；`ui_logic_validation.json` 记录 JavaScript 在模拟 DOM、Plotly 和定时器下的控件测试，未运行真实浏览器或 WebGL。接近过程仍需目视查看，本结果没有进行定量碰撞检查、整臂规划或硬件运动。

在工作区根目录可重新生成到新目录：

```bash
.venv/bin/python tools/overlay_grasps_on_scene.py \
  --approach-distance-m 0.10 --no-preview \
  --output-dir artifacts/perception/my_cleaned_scene_approach
```

`source_snapshot/` 保留本次工具及输入配置，`manifest.json` 记录文件摘要。历史静态叠加保留在 `../cleaned_scene_grasps_20261027_650_001/`。
