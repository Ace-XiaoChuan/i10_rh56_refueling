# 检查真实 PCD 点云

PCD 已经包含三维坐标，无需再做深度反投影。先确认文件可读、几何形状与现场一致，再核对单位、坐标轴和图像对应。当前实测文件及待核对项统一见[项目说明](assembly_contract.md#rgb-d-与三维接口)。

## 运行与查看

在项目根目录运行，输出目录须为新目录：

```bash
.venv/bin/python tools/inspect_pcd.py \
  --pcd fuel_cap_01.pcd --pcd fuel_cap_02.pcd \
  --reference-image fuel_cap_01.jpg \
  --reference-image fuel_cap_02.jpg \
  --length-unit mm --frame-id camera \
  --output-dir artifacts/perception/my_real_pcd

python3 -m http.server 8768 --bind 127.0.0.1 \
  --directory artifacts/perception/my_real_pcd
```

浏览器打开 `http://localhost:8768/index.html`。分别进入两份三维点云，拖动旋转、滚轮缩放；菜单可切换 Z 伪彩色和按 PCL 解出的 RGB。三视图的颜色来自原坐标 Z，不是照片颜色。参考照片单独列出，顺序不表示已配对。

本批次已保存的[点云查看页](../artifacts/perception/neisai_pcd_20261022/index.html)、[Open3D 独立复核](../artifacts/perception/neisai_pcd_20261022/validation_open3d.json)与[照片自动分割结果](../artifacts/perception/neisai_rgb_20261022/index.html)可直接查看。也可将服务命令中的目录替换成对应结果目录。

每份点云目录保存 `point_cloud_raw.npz`、`result.json`、`preview.png` 和 `index.html`。NPZ 的 `xyz_raw`、`records` 保留全部原行，`finite_mask` 和 `finite_source_indices` 记录哪些行可用于显示；存在颜色字段时保存 `rgb_pcl_decoded`。不丢弃原字段、不平滑、不填洞。报告保存输入及实现摘要；没有用文件修改时间代替采集时间。

通用工具默认 `--length-unit unknown --frame-id unknown`。本批 PCD 已由用户确认是相机系毫米，故上述命令显式使用 `--length-unit mm --frame-id camera`，额外保存的 `xyz_m` 仍在相机系中；相机到法兰外参及基座系转换条件见[末端相机外参](assembly_contract.md#末端相机外参)。已有归档报告保留生成时的未知单位和坐标系。读取器默认按常见 x86 导出的小端布局解释，`--byte-order big` 可显式指定大端。当前仅支持 PCD 0.7 的 `DATA binary`，其他格式明确报错。

## 关键计算

实现位于 [pcd_point_cloud.py](../src/fuel_cap_task/pcd_point_cloud.py) 的 `load_binary_pcd()`。头部的 FIELDS、SIZE、TYPE、COUNT 共同描述每个点的字节布局：

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

独立构造字节检查已知坐标、整数及浮点颜色、非有限点保留、载荷截断、点数冲突和压缩格式拒绝。实测文件还可用已有 `.venv-spahybgen` 环境的 Open3D 独立读取，逐点核对 XYZ 和颜色。
