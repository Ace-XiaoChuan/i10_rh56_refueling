# 真实 PCD 检查结果

两份点云的长度单位已确认是毫米（mm），坐标系为相机坐标系（camera）。
index.html 为离线入口，cloud_000、cloud_001 分别展示两份点云；页面坐标轴、悬停读数和三视图均以 mm 标注。

当前输入为 doc/fuel_cap_01_ascii.pcd、doc/fuel_cap_02_ascii.pcd，分别有 28897、28914 个点。
重新生成时已逐项核对原始坐标、字段、颜色与点序，与原检查归档完全一致。
point_cloud_raw.npz 中 xyz_raw 保留原始毫米坐标，xyz_m 为乘 0.001 后的相机系米制坐标。
点云未执行位姿估计、融合或机器人坐标变换；相机各轴物理方向和逐像素映射仍待核实。

Z 伪彩色用于显示几何；按 PCL 解码的 R 通道变化，G/B 为零，真实颜色及字段来源仍需核对。
参考照片单独展示；文件与照片的配对状态以 doc/assembly_contract.md 为准。

validation_summary.json 保存本次单位、数组、视图和摘要核对结果。
source_snapshot 保存当前读取器、展示工具和现有读取测试。
original_inspection.tar.gz 完整保留更新前的页面、数据、报告和 Open3D 独立读取记录。
当前单位、坐标及相机外参见 config/fuel_cap_camera_calibration.json。

validation_open3d.json 保留原始 Open3D 独立逐点读取结果；本次已确认原始数组保持一致。
