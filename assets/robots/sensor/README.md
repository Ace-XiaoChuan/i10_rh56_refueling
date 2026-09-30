# 传感器模型

当前装配关系以 [文档版合同](../../../doc/assembly_contract.md#当前装配变换) 为主记录；本文保留资产来源、转换依据与使用说明。

来源：用户提供的 [传感器.STEP](source/传感器.STEP) 和 [传感器.STL](source/传感器.STL)，已从根目录归档到 `source/`，保留原名与内容。校验记录见 [SHA256SUMS](source/SHA256SUMS)。
STEP 声明毫米单位；STL 包围盒为 `[0, 0, 0]` 到 `[75, 75, 33.5]` mm。
转换采用 `p_sensor = (p_STL - [37.5, 37.5, 0]) × 0.001`，输出网格以米为单位，
原点位于底面中心、轴线沿 Z，保留 6018 个三角面。模型外径 75 mm，总高 33.5 mm。
STEP 的 82 个拓扑顶点到平移后 STL 表面的最大距离约 0.0323 mm，支持此坐标转换；
这不是完整曲面等价性验证。

`urdf/sensor.urdf` 包含显示模型 `sensor_link`，以及位于 Z=0.0335 m 的
`sensor_output_frame` 固定参考坐标系。后者仅表示 CAD 顶面，实际哪一面接机械臂、
绕轴安装角度和连接件配合关系待用户确认。
未提供质量惯量和接触参数，因此当前仅用于静态外观检查，没有 collision、
inertial 或测量输出模型，不能据此验证载荷或传感器测量。

```bash
# 在工作区根目录执行
.venv/bin/python tools/build_sensor_urdf.py
.venv/bin/python tools/view_mjcf.py --sensor-only --check
.venv/bin/python tools/view_mjcf.py --sensor-only
```

MuJoCo 3.13.0 已直接加载此 URDF 并通过 `mj_forward` 检查。

## 组合预览

用户确认独立模型外观并同意继续后，已接入 `i10 → 传感器 → adapter → RH56`。
预览暂按 CAD 底面接 `wrist3_Link`、顶面接 adapter、同轴零旋转放置。
adapter 底面距法兰 33.5 mm，手安装坐标系距法兰 43.5 mm（连接件凸台已按用户要求插入 15 mm，手根部落在连接件 Z=10 mm 肩面）。
这仅是端面放置假设，不代表实物安装方向、孔位或角度已标定。
组合场景中的传感器仅有无碰撞、零密度显示网格。

```bash
.venv/bin/python tools/build_i10_hand_preview.py
.venv/bin/python tools/view_mjcf.py --with-hand
```
