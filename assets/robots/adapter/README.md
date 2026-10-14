# adapter 连接件

原始文件为 [连接件.STEP](source/连接件.STEP) 和 [连接件.STL](source/连接件.STL)，摘要见 [SHA256SUMS](source/SHA256SUMS)。

## 网格坐标

STEP 使用 mm，底面中心为原点，轴线沿 Z。STL 包围盒为 `[0, 0, 0]` 到 `[63, 63, 25]` mm，转换为：

```text
p_adapter = (p_STL - [31.5, 31.5, 0]) × 0.001
```

`meshes/adapter.stl` 使用米，保留 6124 个三角面和孔洞。38 个 STEP 拓扑顶点到转换后 STL 表面的最大距离约 0.001652 mm，用于核对坐标偏移。

尺寸、凸台插入和安装变换见[项目说明](../../../doc/assembly_contract.md#当前装配变换)。当前 MuJoCo 使用显示网格；实物配合、质量惯量及接触模型待核对。

## 重建与查看

在项目根目录运行：

```bash
.venv/bin/python tools/build_adapter_mesh.py
.venv/bin/python tools/build_i10_hand_preview.py
.venv/bin/python tools/view_mjcf.py --with-hand --check
```
