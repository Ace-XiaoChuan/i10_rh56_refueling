# RViz 交互与轨迹播放修复

查看配置原来缺少鼠标工具，且轨迹只发送一次。已补齐 Move Camera / Interact，默认启用轨迹显示，并每 5 秒补发轻量轨迹。静态地图仍仅在新查看器连接时发送。

编译成功后已重新打开窗口，用户确认“可以拖动缩放，机械臂也在动”。继续使用原来的 observed_scene.launch.py 命令即可。项目操作说明见 ../../../doc/point_cloud_voxelization.md。未改变碰撞几何、规划结果或硬件执行。
