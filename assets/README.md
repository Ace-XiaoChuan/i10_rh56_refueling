# 项目资产

当前模型、照片和参考资料按下面的目录组织。尺寸与装配参数见[项目说明](../doc/assembly_contract.md#装配与场景参数)。

| 路径 | 内容 |
| --- | --- |
| `robots/aubo_i10/urdf/` | 当前 i10 URDF/SRDF |
| `robots/aubo_i10/meshes/`、`mjcf/` | 网格、派生 MJCF 与 OBJ |
| `robots/aubo_i10/source/` | 下载原件 |
| [robots/sensor/](robots/sensor/README.md) | 传感器 STEP/STL、米制网格与 URDF |
| [robots/adapter/](robots/adapter/README.md) | adapter STEP/STL 与米制网格 |
| `objects/fuel_cap/` | 盖体 URDF/MJCF 与参考坐标系 |
| [objects/fuel_tank/](objects/fuel_tank/README.md) | 油箱粗模、凹槽与环口 |
| `scenes/i10_rh56_fuel_cap_preview.xml` | 完整臂手与油箱场景 |
| `scenes/i10_fuel_cap.xml` | 臂盖单独展示场景 |
| [hardware_photos/](hardware_photos/README.md) | 硬件与安装场景照片 |
| `reference/standards/` | 用户提供的 SAE J1114-2025 |
| [reference/aviation/](reference/aviation/README.md) | RH56 与航插工程来源副本 |

当前 i10 URDF 的底座固定关节名为 `base_joint`，下载原件为 `world_joint`。编辑使用当前 URDF；原件用于查询来源。

URDF 保留 `package://aubo_description/` 网格引用，RViz 工具将其映射到 `robots/aubo_i10/`。其他加载器需要相同包路径映射。下载资产的确切提交和许可证仍需补充核对。

预览与重建见[模型工具](../doc/mjcf_inspection.md)。
