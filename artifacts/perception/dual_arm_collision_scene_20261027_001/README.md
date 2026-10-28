# 双臂与 T 型架世界系中的实测场景

复用已验证的 2 mm 环境 OctoMap 与 1,502 个独立盖体 box，使用本次点云采集姿态、相机到左法兰外参及已记录的两基座安装关系恢复地图位置。

`world_from_camera = world_from_right_base @ right_base_from_left_base @ left_base_from_camera_at_capture`。原占据格、原点和分辨率不变，环境树逐字节相同，盖体 box 的位置与朝向整体转换。原 ROS 消息复读、组件不重叠与占据并集守恒检查已通过。

`octomap_scene.json` 绑定 `dual_arm/` 模型的 URDF、SRDF、KDL 与元数据摘要；不能与旧单臂坐标混用。左臂固定为该次采集前记录的姿态，尚不是让位姿态。当前模型没有相机外形。此归档准备碰撞数据，不连接硬件。

字段与源码见 `source_snapshot/`。规划验证另存于项目 `artifacts/planning/dual_arm_mtc_20261027_001/`。
