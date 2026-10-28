# 固定 RH56 抓姿与实测盖体点云叠加

本文行数：17行，预期阅读时长：1分钟

在 Windows 资源管理器地址栏打开：

```text
\\wsl.localhost\Ubuntu-22.04\home\ace\i10_rh56_refueling_ws\artifacts\perception\cap_hand_overlay_initial
```

将 `cap_cloud.pcd` 和 `rh56_fixed_pregrasp.ply` 同时拖入 CloudCompare。二者均为相机坐标系，数值单位是毫米，已处于同一位置，无需再配准。检查时旋转观察视角即可。

浅灰是 RH56，蓝色是盖面，橙色是握梁。手根采用固定抓姿，手指采用接近预型；当前使用第一个盖体方向候选，未消除 180° 歧义。`preview.png` 是整体和局部预览，`overview.png`、`grip_detail.png` 为单独图片。

`overlay.json` 中变换、指尖位置使用米，关节角使用 rad，四元数沿用项目 XYZW 约定。`cap_from_hand` 与固定候选一致；网格来自当前手模型，没有调整候选来贴合点云。静态叠加不表示抓稳、接触动力学或机械臂可达。

重现、坐标链与实现说明见[盖体几何估计](../../../doc/cap_geometry.md#固定抓姿叠加)。
