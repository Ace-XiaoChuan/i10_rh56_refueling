# 伟景相机接入

当前使用伟景 `VZ-AR-1920-130N3K`。铭牌标注工作距离 300–1500 mm，镜头为 `5mm-1/2.3-450nm`。产品网页命名与铭牌存在差异，设备身份及实际 RGB 输出以[项目说明](../assembly_contract.md#相机)中的采集记录为准。

## 资料与 SDK

[VzNLSDK 使用说明](../伟景智能--VzNLSDK使用说明(激光线).pdf)。

用户提供的 SDK 目录：

```text
C:\Users\86159\Downloads\7eb218e20133372f59f9c90c77028098\DetectLaser_3.6.20.119_Windows&Linux&ARM
```

Linux x64 库在 `SDK/VzNLSDK/Linux/x64/`，头文件在 `SDK/VzNLSDK/Inc/`。已完成 Linux 库依赖、头文件编译、链接、运行时加载及真实静态点云采集。采集原型和原始调用记录随观测归档，当前数据与查看入口见[项目说明](../assembly_contract.md#相机)。油枪采用[上位机三维框过滤](../assembly_contract.md#油枪点云自动过滤)，保留完整原始扫描。

| 用途 | 接口 |
| --- | --- |
| 设备身份、瞳距、版本 | `VzNL_GetDeviceInfo`、`VzNL_GetPupilDistance`、`VzNL_GetVersion` |
| 静态扫描、结果释放 | `VzNL_PointCloudSnapShort`、`VzNL_ReleasePointCloudSnapShortResult` |
| 图像与点云映射 | `VzNL_GetMappedRGBImageAndPointCloud`、`VzNL_GetMappedLRImageAndPointCloud` |
| RGB 标定参数 | `VZNL_GetRGBCalibParam` |

## 设备验证步骤

1. 按设备说明接供电和网络，记录 IP、型号、序列号、固件及能力信息。
2. 完成一次静态扫描，保存图像、点云和标定信息，核对单位、轴方向、无效值及图像对应关系。
3. 重复采集盖体，检查凹腔遮挡、反光缺点、边缘质量和位置重复性。
4. 按[停稳采集约定](../assembly_contract.md#停稳采集与机器人位姿记录)保存点云和采集前后机器人位姿，再接入感知模块；已有相机到法兰标定可用于转换，其实物精度复核另行完成，参数见[末端相机外参](../assembly_contract.md#末端相机外参)。

参数和待办见[项目说明](../assembly_contract.md#相机)。
