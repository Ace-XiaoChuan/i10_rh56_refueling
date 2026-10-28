# 检查真实 PCD 点云

本文行数：84行，预期阅读时长：4分钟

PCD 已经包含三维坐标，无需再做深度反投影。先确认文件可读、几何形状与现场一致，再核对单位、坐标轴和图像对应。当前实测文件及待核对项统一见[项目说明](assembly_contract.md#rgb-d-与三维接口)。

## 运行与查看

在项目根目录运行，输出目录须为新目录：

```bash
.venv/bin/python tools/inspect_pcd.py \
  --pcd doc/fuel_cap_01_ascii.pcd --pcd doc/fuel_cap_02_ascii.pcd \
  --reference-image fuel_cap_01.jpg \
  --reference-image fuel_cap_02.jpg \
  --length-unit mm --frame-id camera \
  --output-dir artifacts/perception/my_real_pcd

python3 -m http.server 8768 --bind 127.0.0.1 \
  --directory artifacts/perception/my_real_pcd
```

浏览器打开 `http://localhost:8768/index.html`。分别进入两份三维点云，拖动旋转、滚轮缩放；菜单可切换 Z 伪彩色和按 PCL 解出的 RGB。三视图的颜色来自原坐标 Z，不是照片颜色。参考照片单独列出，顺序不表示已配对。

本批次已保存的[点云查看页](../artifacts/perception/neisai_pcd_20261022/index.html)、[Open3D 独立复核](../artifacts/perception/neisai_pcd_20261022/validation_open3d.json)与[照片自动分割结果](../artifacts/perception/neisai_rgb_20261022/index.html)可直接查看。也可将服务命令中的目录替换成对应结果目录。

汇报时可直接打开[独立三维点云网页](../artifacts/perception/neisai_pcd_20261022/point_cloud_viewer.html)。该 HTML 内嵌两份点云及绘图脚本，可单独复制、离线打开；在页面切换点云，保留旋转、缩放、颜色切换、重置视角和全屏展示，单位为毫米。页面展示两份独立观测，不将其叠加或融合。

需要再次导出时，指定新的输出文件：

```bash
python3 tools/export_pcd_viewer.py \
  --inspection-dir artifacts/perception/neisai_pcd_20261022 \
  --output /tmp/fuel_cap_point_cloud.html
```

每份点云目录保存 `point_cloud_raw.npz`、`result.json`、`preview.png` 和 `index.html`。NPZ 的 `xyz_raw`、`records` 保留全部原行，`finite_mask` 和 `finite_source_indices` 记录哪些行可用于显示；存在颜色字段时保存 `rgb_pcl_decoded`。不丢弃原字段、不平滑、不填洞。报告保存输入及实现摘要；没有用文件修改时间代替采集时间。

通用工具默认 `--length-unit unknown --frame-id unknown`。本批 PCD 已由用户确认是相机系毫米，故上述命令显式使用 `--length-unit mm --frame-id camera`，额外保存的 `xyz_m` 仍在相机系中；相机到法兰外参及基座系转换条件见[末端相机外参](assembly_contract.md#末端相机外参)。已有查看页和报告已按确认后的单位重新生成，坐标轴、悬停读数及预览图统一标注 `mm`；原始检查记录保存在结果目录的 `original_inspection.tar.gz`。当前读取器支持 PCD 0.7 的 `DATA ascii` 和未压缩 `DATA binary`，压缩格式明确报错。二进制默认按常见 x86 导出的小端布局解释，`--byte-order big` 可显式指定大端；ASCII 文件不涉及字节序，报告相应字段记为 `null`。

项目当前只保留上述两份 ASCII 点云输入，已与转换前的二进制逐点核对 XYZ、RGB 和点序一致。原文件名及内容摘要保存在[输入名称与格式映射](../config/fuel_cap_input_names.json)，历史结果继续保留生成时的来源记录。

## 当前清理场景

当前环境输入统一为用户清理后的 [scene_cleaned.pcd](scene_cleaned.pcd)，包含 247,103 点，仍使用 SDK 相机系毫米坐标，见[当前场景配置](../config/active_point_cloud_scene.json)。当前两个方向的手模型采用 `[0, 0, 650, 650, 900, 0]` 接近预型，可打开[清理场景叠加页](../artifacts/perception/cleaned_scene_grasps_20261027_650_001/index.html)旋转查看和切换方向；结果目录也提供清理点云及两个手网格的副本，便于 CloudCompare 查看。原始扫描及旧预型查看页继续作为历史数据保留。

## 26号原始扫描全量查看

直接双击[原始扫描与裁剪对比查看页](../artifacts/perception/vizum_scene_review_20261026_001/point_cloud_viewer.html)即可离线打开，绘图脚本和点云全部内嵌，无需启动服务器。下拉菜单可切换原始扫描的全部 268,852 点、盖体周围 600×600×500 mm 观察窗口（187,817 点）、整盖裁剪（5,742 点）。拖动旋转、滚轮缩放、右键拖动平移；红色仅标记用于定位的实测整盖点，其余按 Z 伪彩色显示，单位为相机系毫米。

文件位于 `artifacts/perception/vizum_scene_review_20261026_001/`；同目录的 `scene_raw.pcd` 是原始扫描的字节一致副本，可直接用 Windows CloudCompare 打开。[全量与局部三视图](../artifacts/perception/vizum_scene_review_20261026_001/preview.png)用于快速对照，`coverage.json` 记录来源、坐标范围和窗口边界。观察窗口只改变查看范围，不代表新的采集或额外覆盖。

原始扫描文件实际保存在 `artifacts/hardware/vizum_i10_capture_20261026T055151Z/cloud_sdk_native.pcd`，此前抓姿叠加主要展示其中的整盖裁剪。本次全部返回点均已保留，用户已查看并确认关键油箱门板清晰可辨。后续接近检查进度见[项目说明](assembly_contract.md#相机)；单视角未观测区域保持未知。

## 裁剪后的盖体几何

完整原始输入保留为 `doc/fuel_cap_01_ascii.pcd`；用户在 Windows CloudCompare 中确认的整盖裁剪另存为 `doc/fuel_cap_01_cap_region_ascii.pcd`，含 5,378 点。裁剪只选择点，不改变坐标系和毫米单位。

已可从这份整盖点云分离握梁、估计盖轴与朝向，并按占位模型换算握梁参考中心。运行、结果查看和参考点含义见[盖体几何估计](cap_geometry.md)。输出的着色 PCD 可继续用 CloudCompare 打开，无需 RGB 映射。

## 关键计算

实现位于 [pcd_point_cloud.py](../src/fuel_cap_task/pcd_point_cloud.py) 的 `load_pcd()`。头部的 FIELDS、SIZE、TYPE、COUNT 共同描述每个点的字段布局。ASCII 按点逐行检查列数和数据类型，整数颜色直接解析为整数，避免浮点中转；二进制按字节布局读取：

```python
# dtype 来自文件头；载荷长度须等于点数乘以每点字节数。
records = np.frombuffer(content, dtype=dtype, count=count, offset=offset)
# 每一行仍对应文件中的同一个点；这里不改变长度单位或坐标系。
xyz = np.column_stack([records[name] for name in ("x", "y", "z")])
```

颜色字段为浮点类型时，必须重解释其 32 位位模式，不能把浮点数值直接转成整数；再依次取第 16～23、8～15、0～7 位作为 R、G、B。依据为 [PCL 颜色定义](https://pointclouds.org/documentation/structpcl_1_1_point_x_y_z_r_g_b.html)。

`HEIGHT=1` 表示无组织点云，不能把点序直接重排成照片。需要设备保留的像素映射，或该次采集的相机内参、畸变和点云到图像相机的外参；投影时还要处理遮挡与一像素多点。PCD 的 VIEWPOINT 不等于相机到机器人基座的外参。格式含义见 [PCL PCD 文档](https://pointclouds.org/documentation/tutorials/pcd_file_format.html)。

确认逐点像素对应后，才可用 SAM 掩膜选择盖体点；没有映射时也可人工圈选或进行点云几何分割，并记录选区来源。接入现有[体素工具](point_cloud_voxelization.md)及规划时，须使用米制坐标，并结合采集时的机器人位姿计算目标坐标变换。

## 验证

```bash
.venv/bin/python -m pytest tools/tests/test_pcd_point_cloud.py -q
```

独立构造字节和文本样本检查已知坐标、整数及浮点颜色、非有限点保留、载荷截断、ASCII 点数与列数冲突、数值越界、多元素字段、大整数保真和压缩格式拒绝。实测文件还可用已有 `.venv-spahybgen` 环境的 Open3D 独立读取，逐点核对 XYZ 和颜色。
