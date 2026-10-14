# 油箱粗模

模型入口为 [fuel_tank.urdf](urdf/fuel_tank.urdf)，箱体固定，底面落地。尺寸与组合场景位置见[项目说明](../../../doc/assembly_contract.md#场景参数)。

## 几何与坐标

原模型绕 X 轴立起 90°，开口朝世界 -Y。开孔组件整体上移，箱体向下延伸至地面，保留凹槽和环口的相对几何。

圆周使用 64 段，碰撞由 192 个凸网格及一个底部延伸 box 组成，保留凹腔和开口。凹槽深度、贯通内腔属于建模假设；模型作为固定环境使用。

| 坐标系 | 定义 |
| --- | --- |
| `tank_opening_center` | 凹槽入口中心，箱体局部 `(0, 0, 0.4)` m |
| `tank_recess_floor` | 槽底中心 |
| `tank_filler_top` | 环口顶面中心 |
| `tank_body_center` | 完整箱体几何中心，用于偏航采样 |

开孔坐标轴指向世界 +X/+Z/-Y。独立 URDF 与组合场景使用各自的放置位置，组合位置由项目说明定义。

## 重建与查看

在项目根目录运行：

```bash
python3 tools/build_fuel_tank_urdf.py
.venv/bin/python tools/view_mjcf.py --tank-only
.venv/bin/python tools/build_i10_hand_preview.py
.venv/bin/python tools/view_mjcf.py --with-hand
```

网格路径相对 URDF，加载器需按文件目录解析。`--check` 可检查模型加载。
