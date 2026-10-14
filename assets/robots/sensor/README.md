# 传感器模型

原始文件为 [传感器.STEP](source/传感器.STEP) 和 [传感器.STL](source/传感器.STL)，摘要见 [SHA256SUMS](source/SHA256SUMS)。

## 网格坐标

STEP 使用 mm。STL 包围盒为 `[0, 0, 0]` 到 `[75, 75, 33.5]` mm，转换为：

```text
p_sensor = (p_STL - [37.5, 37.5, 0]) × 0.001
```

米制网格原点在底面中心，+Z 朝顶部，保留 6018 个三角面。82 个 STEP 拓扑顶点到转换后 STL 表面的最大距离约 0.0323 mm，用于核对坐标偏移。

`urdf/sensor.urdf` 定义 `sensor_link` 及 CAD 顶面的 `sensor_output_frame`。尺寸和装配关系见[项目说明](../../../doc/assembly_contract.md#当前装配变换)。

当前 MuJoCo 使用无碰撞、零密度显示网格。质量惯量、实物安装方向与测量接口待补。

## 重建与查看

在项目根目录运行：

```bash
.venv/bin/python tools/build_sensor_urdf.py
.venv/bin/python tools/view_mjcf.py --sensor-only --check
.venv/bin/python tools/view_mjcf.py --sensor-only
```
