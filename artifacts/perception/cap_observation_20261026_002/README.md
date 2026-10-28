# 当前 SDK 原生点云的盖体定位换算

使用已采集的同一份点云与 5,742 点整盖裁剪，按 SDK 原生 XYZ 重新生成当前定位结果。相机到左臂法兰外参和采集时左臂位姿沿用原始观测中的版本。长度从毫米换算成米，没有额外翻转 Y 或其他坐标轴。

用户说明已有拍摄点云也来自 SDK；采集源码调用 `VzNL_PointCloudSnapShort` 时明确使用 `keFlipType_None`。此前额外引入的旧文件导出镜像猜测不作为当前定位的前置条件。旧实验记录保持原样。

`geometry/result.json` 保存相机系和左臂基座系的盖体参考位置、盖轴及两个相差 180° 的握梁方向候选。所有几何数值与上一份结果的 SDK 原生坐标分支完全一致。左臂基座系模型参考位置为 `[1.1274052757200694, 0.3482653761414757, -0.4499735360298671] m`。

原始观测为 `artifacts/hardware/vizum_i10_capture_20261026T055151Z/observation.json`，裁剪为 `artifacts/perception/cap_observation_20261026_001/crop/`。当前预览见 `geometry/preview.png`。`validation.json` 记录本轮实际测试和新旧数值核对，`source_snapshot/` 保存相关实现；原始点云与旧实验未改写。

本轮只完成离线定位换算。外参精度、模型参考点和实际工具安装仍需实物复核，离线工具继续输出 `motion_target_valid=false`；该标记不表示仍需判断 Y 镜像。本轮没有连接设备或发送运动指令。
