# adapter 连接件

原始文件为 [连接件.STEP](source/连接件.STEP) 和 [连接件.STL](source/连接件.STL)，摘要见 [SHA256SUMS](source/SHA256SUMS)。原始 CAD 保留，当前项目模型按目标尺寸生成。

## 网格坐标

STEP 使用 mm，底面中心为原点，轴线沿 Z。STL 包围盒为 `[0, 0, 0]` 到 `[63, 63, 25]` mm，转换为：

```text
z_target = 2 z_STL                    (z_STL ≤ 10 mm)
z_target = 20 + 1.8 (z_STL - 10)      (z_STL > 10 mm)
p_adapter = (x_STL - 31.5, y_STL - 31.5, z_target) × 0.001
```

`meshes/adapter.stl` 使用米，保留 6124 个三角面和孔洞。当前底盘外径 63 mm、厚 20 mm，凸台外径 38 mm、高 27 mm，总高 47 mm；输出包围盒为 `[-31.5, -31.5, 0]` 到 `[31.5, 31.5, 47]` mm。仅分段调整 Z，XY 与孔径保留。原始 STEP/STL 仍为修改前尺寸，制造前需同步修改 CAD。

尺寸、凸台插入和安装变换见[项目说明](../../../doc/assembly_contract.md#当前装配变换)。当前 MuJoCo 使用显示网格；实物配合、质量惯量及接触模型待核对。

## 重建与查看

在项目根目录运行：

```bash
.venv/bin/python tools/build_adapter_mesh.py
.venv/bin/python tools/build_i10_hand_preview.py
.venv/bin/python tools/view_mjcf.py --with-hand --check
```
