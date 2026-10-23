# 跟着做：点云转表面体素

这一步把相机观测点归到三维格子中，得到可见表面的体素表示。先理解一个可手算的小例子，再运行整份点云，最后比较转换前后的形状。

当前采用的边长、原点与显示设置统一见[项目说明](assembly_contract.md#rgb-d-与三维接口)。

## 第一步：理解点与格子的区别

一个点只有位置。一个体素则是有位置和边长的立方格，里面可以落入多个点。

用边长 5 mm、原点为零的网格举例：

| 点 | 世界坐标 XYZ（mm） | 所属格号 XYZ |
| --- | --- | --- |
| A | `[12,23,41]` | `[2,4,8]` |
| B | `[14,24,44]` | `[2,4,8]` |
| C | `[16,23,41]` | `[3,4,8]` |

A 和 B 属于同一个格子，C 属于相邻格子。第 2 个 X 格占 `[10,15)` mm；左端包含在格子内，右端属于下一个格子。负坐标也按向下取整计算，例如 X=-1 mm 属于第 -1 格。

核心实现位于 `src/fuel_cap_task/voxelization.py` 的 `voxelize_points()`：

```python
# 每个坐标先减去网格原点，再除以边长，向下取整得到 XYZ 格号。
point_indices = np.floor((points - origin) / size_m).astype(np.int64)
```

`points`、`origin` 和 `size_m` 都以米为单位，所以运算后的格号没有长度单位。原点固定在世界系，而不是每次改成点云的最小坐标，后续不同观测才有相同的格子编号。

可以先在项目根目录手动运行小例子：

```bash
.venv/bin/python - <<'PY'
import numpy as np

# 毫米先换成米，三行分别为 A、B、C。
points = np.array([[12,23,41], [14,24,44], [16,23,41]]) / 1000
indices = np.floor(points / .005).astype(int)
print(indices)
PY
```

应打印表格里的三组格号。

## 第二步：把相同格号合并

```python
# 相同格号只保存一次；inverse 保留每个原始点归属哪一格。
indices, inverse, counts = np.unique(
    point_indices, axis=0, return_inverse=True, return_counts=True)
```

在小例子中只剩两个格子，点数分别是 2 和 1；`inverse` 为 `[0,0,1]`。每格还记录其中所有点的平均 RGB。保留这份对应关系后，后续可从原图像素追踪到点，再追踪到体素。

显示用的格子中心计算为：

```python
# 格号指向格子的下边界，加半格才得到中心。
centers = origin + (indices + .5) * size_m
```

第 `[2,4,8]` 格的中心是 `[12.5,22.5,42.5]` mm。这个中心不一定是实际物体表面点，也不是自动求得的抓取点。

## 第三步：处理整份观测点云

输入来自[已归档的 RGB-D 点云](rgbd_point_cloud.md#已保存的点云结果)，不读取 MuJoCo 场景几何或盖体真值。使用新目录保存新结果：

```bash
.venv/bin/python tools/voxelize_cap_point_cloud.py \
  --point-cloud-dir artifacts/perception/rgbd_point_cloud_20261016/point_cloud \
  --voxel-size-m .005 \
  --origin-world-m 0 0 0 \
  --output-dir artifacts/perception/my_voxels_5mm
```

| 输出 | 内容 |
| --- | --- |
| `voxels.npz` | 全部格号、中心、点数、平均颜色、原点与边长；`point_to_voxel` 对应输入点顺序 |
| `index.html` | 点云与体素中心的全景比较，点击后显示局部真实立方格 |
| `preview.png` | 全景与初始局部的静态预览 |
| `voxel_centers.ply` | 中心点，供外部软件查看；立方格体积需结合边长解释 |
| `result.json` | 数据来源、点数、体素数量、设置、摘要和显示范围 |

NPZ 使用稀疏索引，只保存被观测点命中的格子。未列出的格子保持未知；没有执行自由空间建图、内部填充或 TSDF 融合。当前整场景表示也还需要选取局部范围、核对尺度与输入约定，才能交给 SpaHybGen。

## 第四步：看首次零原点结果

已保存的体素示例在 `artifacts/perception/rgbd_point_cloud_20261016/voxels_5mm/`。不需要 `xdg`，从项目根目录启动：

```bash
python3 -m http.server 8766 --bind 127.0.0.1 \
  --directory artifacts/perception/rgbd_point_cloud_20261016
```

在浏览器打开 `http://localhost:8766/voxels_5mm/index.html`。同一个服务也能打开 `http://localhost:8766/point_cloud/index.html`。

[结果报告](../artifacts/perception/rgbd_point_cloud_20261016/voxels_5mm/result.json)记录实际点数与体素数；[逐点归属复核](../artifacts/perception/rgbd_point_cloud_20261016/voxels_5mm/validation.json)确认所有输入点都在其所属格子内，点数和对应关系保持一致。测试输出与实现快照一并保存在该结果目录中。

1. 左侧先切到“原始点云”，观察盖体、门板和油箱表面。
2. 再切到“体素中心”，比较哪些位置合并成了同一格。
3. 点击盖体或门板，右侧查看附近按实际边长画出的立方体，旋转并放大观察。

全景显示中心点以便交互，右侧才是格子的真实体积。局部显示限制不会删除存储数据。物体表面恰在网格边界上时，深度的微小波动也可能命中相邻两层格子；本工具保留原观测，没有把这种波动偷偷归零。

第一次先检查形状、尺度和像素到格子的对应是否正确，再观察握梁、凹槽边缘是否被 5 mm 网格合并。边长越大，细节越容易合并；边长越小，格子数量和计算量通常越多。

## 精细观察：先分清取景、网格和显示

首次页面有两类现象：门板顶部在原始 RGB 中已经出画，默认局部窗口又落在油箱中部；另一方面，近似平面刚好跨在网格边界上，微小起伏被表示成相邻两层格子。这些原因需要分别处理，不能只把格子改小。

精细观察使用扩大取景的新 RGB-D；在同一份新点云上比较 5 mm、2 mm 两种网格。每种网格的原点取各轴负半格，将世界原点置于格子中心。观测点本身不移动、不平滑；这能减少本例轴对齐平面恰在边界上的分层，但斜面仍会产生正常的离散台阶。

新查看器顶部有原始 RGB。点击门板或盖体，按原始像素对应直接查出体素编号；无有效深度的像素会明确提示。局部观察半宽按毫米设置，改变体素分辨率时不再跟着改变物理观察范围。超过局部显示上限时页面会提示，不能据此判断数据缺失。

新的观测与两种网格保存在 `artifacts/perception/rgbd_voxel_detail_20261016/`，旧结果保留。从项目根目录查看：

```bash
python3 -m http.server 8767 --bind 127.0.0.1 \
  --directory artifacts/perception/rgbd_voxel_detail_20261016
```

- `http://localhost:8767/voxels_2mm/index.html`：较细网格。
- `http://localhost:8767/voxels_5mm/index.html`：同一份观测的较粗网格。
- `http://localhost:8767/point_cloud/index.html`：原始重建点云。

[像素与体素对应检查](../artifacts/perception/rgbd_voxel_detail_20261016/validation.json)核对全部有效像素的格子编号、无效像素、逐点归属和点数守恒。[边界诊断](../artifacts/perception/rgbd_voxel_detail_20261016/diagnostics/original_boundary_diagnostic.json)保留同一份旧点云改变网格原点的对照；[归档摘要](../artifacts/perception/rgbd_voxel_detail_20261016/archive.json)记录输入、源码快照及保存文件的摘要。

先点击照片里的门板，再点击盖体，比较两种边长保留了哪些细节。半宽默认 40 mm；可调大观察范围，页面会说明当前是否达到显示数量上限。固定原点、范围和输入来源都要一起记录，不能只记录“用了几毫米体素”。

重新运行较细网格时，使用新输出目录：

```bash
.venv/bin/python tools/voxelize_cap_point_cloud.py \
  --point-cloud-dir artifacts/perception/rgbd_voxel_detail_20261016/point_cloud \
  --voxel-size-m .002 --origin-world-m -.001 -.001 -.001 \
  --local-radius-m .04 \
  --output-dir artifacts/perception/my_voxels_2mm
```

## 这部分需要掌握到什么程度

当前目标是能读懂核心计算、说明输入输出、选择参数并定位错误：

1. 说清 RGB、深度、点云、体素的数据关系和单位。
2. 看懂深度反投影、坐标变换、点到格号三步计算。
3. 用小例子手算一个点属于哪个格子，理解边长与原点分别改变什么。
4. 区分没采到、没转换成功、没在页面显示；会检查像素对应与数据摘要。
5. 知道未观测区域保持未知，网络还需要约定局部范围、体素尺度和输入格式。

先读 `rgbd_point_cloud.py` 的 `backproject_depth()`、`transform_points()`，以及 `voxelization.py` 的 `voxelize_points()`，重点看注释、计算和返回值。`pixel_to_voxel_lookup()` 则展示照片中的点怎样追踪到格子。暂时不要求深入渲染引擎和网页库内部；真正采用 TSDF、多视角融合或修改网络时，再补相应方法。

## 数值检查

```bash
PYTHONPATH=src .venv/bin/python -m pytest \
  tests/test_rgbd_point_cloud.py tests/test_voxelization.py -q
```

检查小例子、负坐标、边界、固定原点、点数与对应关系、颜色平均以及显示立方体的边长。实际样本再核对每个点都落在其所属格子内，所有格子的点数之和等于输入点数。

体素化含义参考 [Open3D 点云体素化说明](https://www.open3d.org/docs/release/tutorial/geometry/voxelization.html#from-point-cloud)。
