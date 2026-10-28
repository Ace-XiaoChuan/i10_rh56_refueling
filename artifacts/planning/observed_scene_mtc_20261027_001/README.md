# 实测场景接入 MTC 与碰撞、路径验证

环境 OctoMap 和独立盖体已加载到 MTC 的实际 PlanningScene，替换 197 个旧油箱、盖体与门板对象；保留地面、完整单臂、传感器、adapter 和 RH56。该入口继续使用之前确定的临时单臂摆放。

| 候选 | 当前结果 |
| --- | --- |
| `direction_000` | 标准安装入口规划成功，含绕障碍到预抓取位和 0.100 m 轴向接近；1,950 个加密路点通过完整模型 FCL 和关节限位检查 |
| `direction_180` | 预抓取位存在无碰撞 IK；最终目标在检查到的各 IK 分支中均存在拇指中节与环境格相交，MTC 拒绝完整接近 |

成功轨迹的终点位置误差约 0.0693 mm，姿态误差约 0.00576°。[起点和终点预览](trajectory_preview.png)与 [RViz 原生渲染](rviz_scene.png)使用同一份实测场景。[汇总](summary.json)记录实际结果。只有 `direction_000/trajectory.json` 可作为已验证的几何路径回放，仍未生成可执行的控制器时序。

环境与盖体分别通过相交探针对照，明确分离位置通过无碰撞对照；完整模型 22 个含碰撞几何的 link 均通过表面相交探针检查。源覆盖模型的 19 组 FK、KDL 与碰撞对照通过，最大 FK 矩阵元素差约 2.11e-15；规划覆盖新增的地面使两个原本没有地面检查的参考 IK 构型被判为臂与地面相交，该差异如实保留在模型检查文件中。

文件入口：

- `direction_000/`、`direction_180/`：请求、场景检查、规划日志与成功或失败报告。方向 0 采用标准安装 launch 入口得到的轨迹。
- `installation_checks/`：两种模型覆盖检查、标准 launch 检查、错误场景摘要的拒绝测试。
- `rviz_check/planning_scene.cdr`：从标准 `get_planning_scene` 服务实际取回的原生 PlanningScene 序列化快照，含机器人状态、环境与盖体。
- `rviz_check/service_validation.json`：服务响应确认地面、1,502 个盖体 box 和 2 mm OcTree。
- `rviz_standard_check/`：正式 RViz 启动、订阅与关闭日志。本机 WSLg 环境中，显示正常，退出阶段 rviz2 发生段错误；规划和路径检查运行于独立进程。实验用原生截图程序也在退出阶段发生同类异常，图片已成功保存。此问题没有记为通过。
- `native_first_success/`：直接可执行入口得到的另一条已验证方向 0 路径；当前可回放结果以 `direction_000/trajectory.json` 为准。
- `setup_diagnostics/`：DDS 配置及探针位置调整过程的原始失败日志，保留为诊断资料。
- `source_snapshot/`、`environment.json`、`archive.json`：实现、实际配置、二进制摘要和逐文件归档摘要。

从项目根目录回放：

```bash
MUJOCO_GL=glfw .venv/bin/python tools/view_voxel_map_placement.py \
  --scene-dir artifacts/perception/cleaned_scene_split_2mm_20261027_001 \
  --trajectory artifacts/planning/observed_scene_mtc_20261027_001/direction_000/trajectory.json --view
```

空格暂停、R 重播；只做前向运动学，未运行物理步进或连接设备。相邻路点的加密为离散检查，不是连续碰撞证明。该入口尚未切换到当前记录的双臂安装链或完整现场障碍，不作为真机执行结果。

生成新请求、标准 MTC 与 RViz 启动方法见[实测场景接入 MTC](../../../doc/point_cloud_voxelization.md#实测场景接入-mtc)。
