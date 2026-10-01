# 资产目录

| 路径 | 内容 |
| --- | --- |
| `robots/sensor/source/` | 用户提供的传感器 STEP/STL 原件及 SHA-256 校验记录；派生米制网格位于相邻 `meshes/` |
| `robots/adapter/source/` | 用户提供的连接件 STEP/STL 原件及 SHA-256 校验记录；派生米制网格位于相邻 `meshes/` |
| `robots/aubo_i10/urdf/` | 当前预览使用的 i10 URDF 及配套 SRDF，迁自工作空间根目录 |
| `robots/aubo_i10/meshes/` | i10 DAE、STL、3DS 网格，迁自 `aaa/aubo_description-main/meshes/` |
| `robots/aubo_i10/source/urdf/` | 原下载目录中的 URDF/SRDF，保留原件，不作为预览入口 |
| `objects/fuel_cap/urdf/fuel_cap.urdf` | 当前油箱盖模型，含灰色握条、灰色中部圆柱、直径 40 mm 的白色下部圆柱及三个参考坐标系 |
| `objects/fuel_tank/urdf/fuel_tank.urdf` | 油箱占位粗模，当前尺寸见项目合同，含收缩凹槽与环形油箱口；假设与生成方式见[说明](objects/fuel_tank/README.md) |
| `robots/aubo_i10/mjcf/` | 由当前 URDF 生成的 i10 MJCF 和外观 OBJ |
| `objects/fuel_cap/mjcf/fuel_cap.xml` | 油箱盖 MJCF，含三个参考 site |
| `scenes/i10_fuel_cap.xml` | MuJoCo 组合展示场景 |
| `scenes/i10_rh56_fuel_cap_preview.xml` | i10 + 传感器 + adapter + RH56 右手 + 油箱盖 + 门板 + 油箱预览；安装和摆放参数未标定 |
| `hardware_photos/` | 硬件照片与来源记录 |
| `reference/standards/` | 用户提供的 `SAE J1114-2025.pdf`，迁自工作空间根目录 |
| `reference/aviation/` | 已迁入的航插历史参考资产 |

2026-09-16 归档时逐文件核对 SHA-256，模型、网格和 PDF 内容未改变。
当前 i10 URDF 的底座固定关节名为 `base_joint`；下载原件为 `world_joint`。
两份 SRDF 仅换行格式不同，均保留。后续编辑请使用当前预览目录中的版本。
下载资产的确切仓库版本及许可证尚待补充核实，目录名不代表来源已验证。

URDF 中保留 `package://aubo_description/` 网格引用；
`tools/view_i10_fuel_cap_rviz.py` 在加载时将其映射到 `robots/aubo_i10/`。
此目录尚未注册为 ROS 包；直接通过其他工具打开时需配置相同的包路径映射。

在工作空间中运行：

```bash
bash tools/view_i10_fuel_cap_rviz.sh
bash tools/view_i10_fuel_cap_rviz.sh --cap-only
```

尺寸、模型版本和安装参数的确认边界见[项目合同](../doc/assembly_contract.md)；本次归档不代表
动力学、规划或实物验证通过。

MJCF 的生成、预览与验证边界见 [MJCF 使用说明](../doc/mjcf_inspection.md)。
