# 模型预览与调抓姿

完整场景包含 i10、传感器、adapter、RH56 右手、油箱盖、打开的门板和油箱。尺寸及装配关系见[项目说明](assembly_contract.md#装配与场景参数)。

所有命令从项目根目录执行，使用共享仿真环境。

## 打开模型

```bash
.venv/bin/python tools/view_mjcf.py --with-hand
.venv/bin/python tools/view_mjcf.py --with-hand --check
.venv/bin/python tools/view_mjcf.py --cap-only
.venv/bin/python tools/view_mjcf.py --sensor-only
.venv/bin/python tools/view_mjcf.py --tank-only
```

`--check` 做无窗口加载检查。窗口使用 `mj_forward` 更新几何，右侧 Joints 面板可调整关节。鼠标拖动旋转，滚轮缩放；外观为 group 1，碰撞几何为 group 0。

RViz 预览入口：

```bash
bash tools/view_i10_fuel_cap_rviz.sh
bash tools/view_i10_fuel_cap_rviz.sh --cap-only
```

RViz 默认 ROS domain 56，关闭窗口后结束配套节点。

## 重建模型

按修改的来源重建对应资产：

```bash
.venv/bin/python tools/build_mjcf.py
.venv/bin/python tools/build_sensor_urdf.py
.venv/bin/python tools/build_adapter_mesh.py
python3 tools/build_fuel_tank_urdf.py
.venv/bin/python tools/build_i10_hand_preview.py
```

`build_mjcf.py` 从 URDF 生成 i10、盖体 MJCF 和臂盖场景；`build_i10_hand_preview.py` 生成完整臂手场景。油箱或安装件变化后，再生成完整场景。规划模型的生成和构建见[MoveIt 说明](../src/i10_rh56_moveit_config/README.md)。

网格保持相对路径。DAE 节点变换烘焙为 OBJ；MuJoCo 网格碰撞使用凸包，油箱凹腔拆成多个凸片保留开口。

## 手动调抓姿

```bash
.venv/bin/python tools/tune_cap_grasp.py \
  --load config/fixed_grasp.json
```

不带 `--load` 时从默认种子开始。界面位置为盖体系 mm，姿态为固定轴 XYZ RPY 度；六通道编辑接近预型，闭合目标只读。

1. 编辑位姿和预型，点击“应用调姿”。
2. 点击“闭合”，观察实际运动、接触点和三指接触力。
3. “暂停”保留当前状态，“重置”返回已应用的预型。
4. “保存已应用候选”保存位姿、预型、模型信息和实验采样。

保存使用米和 XYZW。载入恢复初态，点击闭合才能进行新的接触实验。手根与盖体固定，这个工具用于检查局部闭合；自由持稳和旋拧在后续执行模型中验证。

## 完整装配 IK

```bash
.venv/bin/python tools/preview_cap_ik.py --view
```

工具由盖体观测和手盖相对候选生成手根目标，再求六轴臂 IK。N/P 切换解，O/G 切换预型与保存的实际闭合姿态，F 切换视角，C 显示碰撞几何。

## 随机场景

```bash
.venv/bin/python tools/sample_cap_scenes.py \
  --load-report artifacts/random_scenes/report.json --view
```

N/P 切换样本，[/] 切换 IK，F 切换视角，C 显示碰撞几何。新采样时去掉 `--load-report`，可通过 `--seed`、`--per-group`、`--output` 指定参数。范围见[项目说明](assembly_contract.md#随机场景范围)。

## 批量局部闭合探索

```bash
.venv/bin/python tools/search_cap_grasp.py \
  --output artifacts/grasp_search/my_contact_report.json

.venv/bin/python tools/search_cap_grasp.py \
  --load-report artifacts/grasp_search/my_contact_report.json --view
```

`--trials` 设置实验数量，`--keep` 设置详细保存的候选数量。N/P 切换候选，O/G 查看预型/末态，[/] 查看采样。`--render-dir` 生成末态三视图和网页。

候选报告保存 `cap_from_hand`、实际关节角、三指接触、穿透、其他接触和速度。`triple_contact_observed` 表示记录到了三指接触。`--static-diagnostic` 直接摆出空载目标形状，用于几何对照。

## 门板占位模型

门板由 `tools/fuel_door.py` 加入完整场景。默认打开姿态及占位参数见[场景参数](assembly_contract.md#场景参数)。当前工具只预览几何，门板保持和避让空间需要后续执行检查。

## 实测点云体素场景

用户清理的实测点云可转换为完整体素 box 模型，再在 MuJoCo 查看。入口与文件说明见[实测体素查看](point_cloud_voxelization.md#当前实测清理点云的-mujoco-查看)，当前边长见[相机与实测场景设置](assembly_contract.md#相机)。这份模型以相机系为 world，环境完全来自点云；默认使用外露面合批显示，7/8/9 切换全景、盖体及格子细节，Esc 返回自由相机。鼠标左键拖动旋转、右键拖动平移、滚轮缩放。

## WSLg 窗口排障

窗口不可见时先保存工作，在 Windows PowerShell 中执行：

```powershell
wsl --shutdown
```

这会停止全部 WSL 会话。重新打开 Ubuntu 后再次启动预览。如果仍有问题，检查 `DISPLAY`、`/mnt/wslg/weston.log` 和 WSLg 状态；可执行 `wsl --update` 后重启 WSL。

日常编辑使用项目的 `refueling-lite.code-workspace`，减少对环境、资产和结果目录的后台扫描。

[MuJoCo 建模文档](https://mujoco.readthedocs.io/en/stable/modeling.html)
