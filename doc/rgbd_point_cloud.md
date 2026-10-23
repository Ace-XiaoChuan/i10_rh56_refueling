# 从 RGB-D 观测生成点云

先采集一张 RGB 和对应深度，再计算相机可见表面的三维点。这个入口输出完整彩色点云和可旋转查看的页面，供后续盖体分割、体素生成和抓姿模块使用。当前进度与参数见[项目说明](assembly_contract.md#rgb-d-与三维接口)。

已有真实相机导出的 PCD 时，先用[真实 PCD 检查工具](pcd_inspection.md)核对数据；该输入已经是三维点，不需要重复反投影。

## 采集、重建与查看

从项目根目录执行，两个输出目录都应使用新名字：

```bash
.venv/bin/python tools/capture_cap_rgbd.py \
  --output-dir artifacts/perception/my_observation

.venv/bin/python tools/reconstruct_cap_point_cloud.py \
  --observation-dir artifacts/perception/my_observation \
  --output-dir artifacts/perception/my_point_cloud

python3 -m http.server 8765 --bind 127.0.0.1 \
  --directory artifacts/perception/my_point_cloud
```

终端保持运行，在已有浏览器打开 `http://localhost:8765/index.html`；结束时按 Ctrl+C。不需要安装 `xdg`。网页内拖动旋转、滚轮缩放、右键拖动平移。网页只抽样显示以保持流畅，导出的文件保留全部有效点。

重建运行使用共享 MuJoCo Python 环境中的 NumPy、Pillow 和 Matplotlib。离线页面复用当前环境或 `.venv-spahybgen` 中的 Plotly JavaScript；其他安装位置可用 `--plotly-js /path/to/plotly.min.js` 指定，无需联网打开页面。

需要把完整门板拍进画面时，可以指定取景与分辨率；示例参数见[项目说明](assembly_contract.md#rgb-d-与三维接口)：

```bash
.venv/bin/python tools/capture_cap_rgbd.py \
  --width 1280 --height 960 --lookat-world-m 0 .94 .62 \
  --distance-m .9 --azimuth-deg 90 --elevation-deg -20 \
  --offscreen-samples 0 --primitive-slices 128 \
  --output-dir artifacts/perception/my_wide_observation
```

每份新观测会保存实际取景参数、对应内参和相机位姿。完整门板版本与较细体素的对照入口见[精细观察](point_cloud_voxelization.md#精细观察先分清取景网格和显示)。

## 已保存的点云结果

完整示例保存在 `artifacts/perception/rgbd_point_cloud_20261016/`：

- [原始 RGB-D 与标定](../artifacts/perception/rgbd_point_cloud_20261016/observation/observation.json)。
- [彩色点云交互页面](../artifacts/perception/rgbd_point_cloud_20261016/point_cloud/index.html)与[静态预览](../artifacts/perception/rgbd_point_cloud_20261016/point_cloud/preview.png)。
- [重建结果与来源摘要](../artifacts/perception/rgbd_point_cloud_20261016/point_cloud/result.json)。
- [独立射线位置复核](../artifacts/perception/rgbd_point_cloud_20261016/point_cloud/evaluation_rays.json)。

该例恢复了盖体、油箱正面、门板和地面的可见表面。原始观测、NPZ、两种坐标系的 PLY 及页面均已归档，复制摘要见 [archive.json](../artifacts/perception/rgbd_point_cloud_20261016/archive.json)。报告保留生成时的原路径；复制未改写原结果。

初次取景中门板顶部超出画面。完整门板的新观测保存在 `artifacts/perception/rgbd_voxel_detail_20261016/`，可查看[原始 RGB](../artifacts/perception/rgbd_voxel_detail_20261016/observation/rgb.png)、[点云页面](../artifacts/perception/rgbd_voxel_detail_20261016/point_cloud/index.html)与[独立位置复核](../artifacts/perception/rgbd_voxel_detail_20261016/point_cloud/evaluation_rays.json)。对应两种体素的查看方法见[精细观察](point_cloud_voxelization.md#精细观察先分清取景网格和显示)。

查看此示例：

```bash
python3 -m http.server 8766 --bind 127.0.0.1 \
  --directory artifacts/perception/rgbd_point_cloud_20261016
```

浏览器打开 `http://localhost:8766/point_cloud/index.html`。该端口与前面临时点云页面的 8765 分开；结束后按 Ctrl+C。

## 输入与输出

重建只读取观测目录中的三个文件：

| 文件 | 用途 |
| --- | --- |
| `rgb.png` | 与深度逐像素对齐的 RGB |
| `depth.npz` | `depth_m` 为光轴 Z 深度，单位米；`valid` 为布尔有效掩膜 |
| `observation.json` | 内参、分辨率、相机到世界系的变换、时间与来源 |

当前入口要求图像符合无斜切针孔模型且已去畸变。RGB、深度和标定分辨率须一致；来源和时钟域须明确。元数据若包含输入摘要，则核对文件是否仍匹配。真实相机需要先将设备输出适配到这些约定。

| 输出 | 用途 |
| --- | --- |
| `index.html` | 可离线旋转的世界系彩色点云，附原始 RGB |
| `preview.png` | 静态三维侧视图 |
| `cloud_camera.ply` | 相机光学系彩色点云，坐标为 float32 米 |
| `cloud_world.ply` | 世界系彩色点云，坐标为 float32 米 |
| `point_cloud.npz` | float64 相机点、世界点、uint8 RGB 和原图 `pixel_uv` 索引 |
| `result.json` | 来源、时间、点数、范围、坐标变换、输入和实现摘要 |

相机光学系 X 向右、Y 向下、Z 向前；世界系由 `world_from_optical` 定义。当前世界系与机器人 `base_link` 不是同一个坐标系，接入机械臂前还需显式变换。

## 像素怎样变成三维点

核心实现位于 `src/fuel_cap_task/rgbd_point_cloud.py` 的 `backproject_depth()`：

```python
# u、v 是有效像素的列、行；z 是该像素的光轴 Z 深度，单位米。
x = (u - k[0, 2]) * z / k[0, 0]
y = (v - k[1, 2]) * z / k[1, 1]
points = np.column_stack((x, y, z))
```

`k[0,0]`、`k[1,1]` 是以像素表示的焦距，`k[0,2]`、`k[1,2]` 是主点位置。像素先给出观察方向，深度再确定该方向上表面点的位置。Z 深度不同于点到相机原点的斜距，因此不能直接把 Z 当作射线长度。

`transform_points()` 使用同一次观测的旋转与平移，把相机点转为世界点：

```python
# 行向量批量表示：等价于对每个列向量计算 R @ p_camera + t。
world_points = camera_points @ rotation.T + translation
```

每个点都保留原像素 `(u,v)`，颜色从该像素取。后续获得 SAM 掩膜后，可以根据这些索引选出盖体点，无需通过三维均值猜测盖中心。

重建不读取场景几何和 `evaluation_only.npz`，不会补全背面或遮挡区域。无点区域不能直接作为自由空间。本入口输出点云，后续表面体素化见[分步教程](point_cloud_voxelization.md)；自体过滤、盖轴估计与抓姿生成另行接入。

## 检查方法

```bash
PYTHONPATH=src .venv/bin/python -m pytest tests/test_rgbd_point_cloud.py -q
```

数值检查使用已知平面验证 Z 深度、已知旋转与平移验证变换方向，并检查无效值过滤、颜色与像素对应、输入不匹配及二进制 PLY 格式。查看实际结果时，应确认尺度、上下方向和门板位置，再进行体素化。

计算公式参考 [Open3D 深度反投影接口](https://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.create_from_depth_image)。
