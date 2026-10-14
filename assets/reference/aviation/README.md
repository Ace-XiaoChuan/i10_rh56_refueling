# 航插工程参考资产

本目录保存 `/home/ace/bimanual_dexterous_mvp_ws` 的来源副本。逐文件路径和 SHA-256 见 [manifest.json](manifest.json)，原件用于查询模型和接口依据。

当前项目主要复用 RH56 资料，i10 装配与任务参数见[项目说明](../../../doc/assembly_contract.md)。

## 可用资料

| 资源 | 入口 |
| --- | --- |
| RH56 左右单手模型 | [左手](third_party/correlllab_rh56_controller_h12/mujoco_inspire/inspire_left.xml)、[右手](third_party/correlllab_rh56_controller_h12/mujoco_inspire/inspire_right.xml) |
| 手部串口说明 | [控制说明](doc/hand/README.md)、[角度脚本](doc/hand/rh56_angle_cmd.py)、[状态脚本](doc/hand/rh56_watch.py) |
| 手部力矩与模型资料 | [力矩说明](doc/hardware/rh56_mujoco_position_torque.md)、[模型清单](doc/hardware/fr3_rh56_model_resource_inventory.md) |
| FR3 与 RH56 组合参考 | [MJCF](derived/fr3_rh56_right/fr3_rh56_right.xml) |
| 来源与许可 | [模型来源](src/assembly_mujoco/mjcf/MODEL_PROVENANCE.md)及各资源目录的许可文件 |

`derived/extract_robot.py` 从来源场景提取机器人子树和所需网格。单手及组合 MJCF 在本目录中自包含；Xacro 和描述包是来源节选。

本目录使用 `COLCON_IGNORE` 隔离。副本正文、命令和路径按来源保留，当前项目入口使用根目录 README。许可证和 NOTICE 随原资源保存。
