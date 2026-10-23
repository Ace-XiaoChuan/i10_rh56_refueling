# SDK 与预抓取输入检查

本次检查使用离线 SDK 和已保存的数据，保存当前代码、点云工具、固定抓姿接入、真机只读记录及用户确认的相机参数。完整结果见 [validation.json](validation.json)。

- SDK 三个 Release 目标构建成功，离线初始化和 26 个接口符号检查通过。
- MoveIt 来源检查通过：237 项输入、233 项生成文件；相机外参的旋转、米制平移和逆变换复核通过。
- 原始输入名称映射、两批真机采集清单全部通过 SHA-256 核对。
- 场景、规划、点云、体素、任务流程与工具共 173 项回归中，125 项通过，17 项失败、31 项初始化错误。全部 48 项均被旧实验的场景来源哈希失配阻止，详情见 [日志](legacy_planning_failures.txt) 与 [JUnit](legacy_planning_failures.xml)。旧结果保持原样，当前回归套件尚未全部通过，需另行生成匹配当前模型的规划测试输入。
- 当前模型的 E2 baseline、mixed 路径重新回放通过，位置残差分别约 0.0579 mm、0.0604 mm，机器人最大穿透均为零；[复核结果](current_path_validation.json)。

回归命令：

```bash
PYTHONPATH=src MUJOCO_GL=egl .venv/bin/python -m pytest   tests/scene tests/planning tests/test_rgbd_point_cloud.py tests/test_voxelization.py   tests/test_cap_task.py tests/test_feedback_task.py tools/tests   -q -o cache_dir=/tmp/refueling-pytest-cache
```

实时状态、采集时法兰位姿、工具与场景几何、时间化轨迹及停止反馈的接入仍见[当前真机目标](../../../doc/assembly_contract.md#近期真机目标)。SDK 原始开发包保留在本机，仓库保存下载入口与逐文件摘要。
