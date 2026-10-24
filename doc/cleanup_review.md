# 项目清理清单（A01～A04 及点云格式整理已完成）

盘点日期：2026-10-24。以下路径均相对于 `/home/ace/i10_rh56_refueling_ws`。

**A01～A04 已按用户批准执行：20 个目标路径已移除，共删除 815 个文件，文件内容合计约 6.92 MiB。点云输入已按后续决定只保留两份 ASCII PCD，删除四份二进制 PCD 和两份 XYZ 文本，合计约 5.00 MiB。A05、B02 和 C 组仍待审核。**

## 我的建议

重复构建目录、Python 缓存、旧窗口错误日志、空占位文件和点云格式整理已完成。以下保留各项范围及理由供核对。尚未处理的 B02 训练结果副本合计约 5.11 MiB；另有约 1.85 GiB 的 VS Code 索引缓存，可以单独选择是否清理。

脚本数量较多，但大多数有明确用途。检查了 `tools/` 顶层的 49 个 Python 脚本、1 个 Shell 入口、1 个 HTML 模板，以及 `doc/` 中的 16 份 Markdown。结合源码导入、测试、配置和文档引用，目前只有 `tools/inspect_cap_scene.py` 没找到直接引用；它仍有独立功能，列为待确认项。现有 Markdown 都有入口或相互引用，暂不建议整篇删除。

检查包括静态引用搜索、文件 SHA-256 比较和点云逐点比较。ASCII 读取适配已通过 11 项格式测试，并用两份实测数据完成报告、数组和查看页生成验证。没有连接机器人、运行训练或重建模型。没有直接引用不等于你没有手动使用；因此下面区分确定性较高的删除候选、需同步更新引用的候选，以及需要进一步确认的文件。

## A. 生成文件和缓存

### A01：五处重复生成的 MoveIt 构建目录（已删除）

```text
src/build/
tools/build/
config/build/
doc/build/
tests/build/
```

- 合计约 6.09 MiB，共 690 个生成文件，没有 Git 跟踪的文件。
- 五处 `CMakeCache.txt` 的项目名均为 `i10_rh56_moveit_config`，源码目录都指向 `src/i10_rh56_moveit_config`，内容以重复配置的 CMake/CTest 文件为主。
- 保留项目根目录的 `build/`、`install/`，以及 `tools/aubo_sdk_probe/build/`。这些位置包含现用规划或真机查询程序。
- 删除这五处不会删除 C++ 源码。若编辑器再次自动生成这些目录，需要另行统一 CMake 工作区的构建目录设置。

### A02：项目自身的 Python 缓存和 pytest 缓存（已删除）

```text
src/fuel_cap_task/__pycache__/
src/i10_rh56_moveit_config/launch/__pycache__/
tools/__pycache__/
tools/cloud_cap_dino/__pycache__/
tools/tests/__pycache__/
tests/__pycache__/
tests/scene/__pycache__/
tests/planning/__pycache__/
tests/spahybgen/__pycache__/
.pytest_cache/
```

合计约 0.83 MiB，盘点时共 120 个文件，均未被 Git 跟踪。它们会在后续运行时重新生成；删除 pytest 缓存也会清掉上次失败等测试选择记录。此项只包括上面列出的目录，保留虚拟环境、厂家 SDK 和第三方源码内部的文件。

### A03：一份旧的窗口初始化错误日志（已删除）

```text
src/MUJOCO_LOG.TXT
```

只有 59 字节，内容是 2026-10-20 的 `could not initialize GLFW`，没有实验数据或程序实现，可删除。

### A04：已非空目录中的占位文件（已删除）

```text
src/.gitkeep
tools/.gitkeep
config/.gitkeep
doc/hardware/.gitkeep
```

这些目录已有实际文件，空占位文件可删除。此项仅减少零碎文件，几乎不释放空间。

### A05：VS Code C++ 索引缓存（可选）

```text
.vscode/browse.vc.db
.vscode/browse.vc.db-wal
.vscode/browse.vc.db-shm
.vscode/browse.vc.db.lock
.vscode/browse.vc.2.db
.vscode/browse.vc.2.db-wal
.vscode/browse.vc.2.db-shm
.vscode/browse.vc.2.db.lock
src/.vscode/browse.vc.db
src/.vscode/browse.vc.db-wal
src/.vscode/browse.vc.db-shm
src/.vscode/browse.vc.db.lock
```

- 盘点时共约 1.85 GiB，是本次发现占用最大的缓存，大小可能继续变化。
- 关闭相关 VS Code 窗口、确认索引进程停止后，再处理这些数据库及配套文件。
- 后续 C++ 索引会重新建立，首次补全和跳转可能需要等待。
- 保留各目录的 `settings.json`、`c_cpp_properties.json` 和项目的 `.code-workspace` 文件；此项不删除整个 `.vscode/`。

## B. 结果副本与点云输入

### B01：二进制 PCD 输入及副本（按新决定已删除）

| 已删除的二进制文件 | 保留的 ASCII 输入 | 核对结果 |
| --- | --- | --- |
| `fuel_cap_01.pcd`、`doc/fuel_cap_01.pcd` | `doc/fuel_cap_01_ascii.pcd` | 28897 点，XYZ、RGB、点序逐字段一致 |
| `fuel_cap_02.pcd`、`doc/fuel_cap_02.pcd` | `doc/fuel_cap_02_ascii.pcd` | 28914 点，XYZ、RGB、点序逐字段一致 |

四份二进制文件合计约 1.77 MiB，均已删除。保留的 ASCII 文件没有改写；当前点云检查工具已支持 ASCII，单位和坐标系沿用相机系毫米。

**配套修改已完成：** 输入名称映射、相机配置、项目说明和点云操作命令均指向保留的 ASCII。名称映射同时保存原二进制摘要与当前 ASCII 摘要，历史实验记录保持原文。

### B02：散落在源码和文档目录中的训练对照结果副本

拟删除：

```text
src/comparison.html
doc/comparison.html
src/prediction_diagnostics.json
```

完整保留：

```text
artifacts/perception/cap_dino_cloud_20261010_02/
  cloud_review_received_20261010/comparison.html
  cloud_review_received_20261010/prediction_diagnostics.json
  cloud_review_received_20261010/review_report.md
  cloud_review_received_20261010/review_findings.json
```

- 两份拟删除 HTML 与上述归档 HTML 的 SHA-256 完全一致，都是 `cbc3d9623e28…`。
- 拟删除 JSON 与上述归档 JSON 完全一致，SHA-256 为 `d8f59b92b966…`。
- 在 `artifacts/perception/cloud_model_20261010/unpacked/outputs/review/` 中也核对到了相同文件，本轮保留该模型包内容。
- 可减少约 5.11 MiB，保留下来的归档仍包含原始对照结果。

**配套处理：** 当前导航指向归档位置。历史检查报告中记载的 `src/...` 是当时收到文件的位置，保留历史原文及摘要，不改写既有实验记录。

### B03：XYZ 文本已删除，ASCII PCD 保留

```text
doc/fuel_cap_01_xyz.txt
doc/fuel_cap_02_xyz.txt
```

- 两份 XYZ 文本合计约 3.23 MiB，已按用户要求删除。
- 已逐点核对：XYZ 文本与保留 ASCII 的 XYZ 一致，点数和顺序相同。
- 两份 ASCII PCD 保留于原位置，保存全部坐标和 RGB 字段，现用读取工具、配置与说明已适配。

后续以 B01 表中两份 ASCII 作为点云输入，无需再维护同坐标的 XYZ 文本。

## C. 需要你确认用途或版本的文件

### C01：一个没有直接引用的旧查看脚本

```text
tools/inspect_cap_scene.py
```

这是早期场景与手指通道查看入口，支持 `--check`、`--channels` 和 G/O/P 按键，主要通过通道指令摆放手指并打印场景报告。现有文档主要使用 `view_mjcf.py`、`tune_cap_grasp.py` 和 `read_grasp_targets.py`。

虽然部分用途重叠，但这些工具的功能并不完全相同。**只有你确认不再使用旧入口时，才建议删除该脚本；默认先保留。** 其他脚本暂未找到足够的删除依据。

### C02：两份独立点云查看页面，建议归档

```text
doc/fuel_cap_01_viewer.html
doc/fuel_cap_02_viewer.html
```

合计约 13.38 MiB，内嵌 Plotly 和全部点，可作为单文件离线打开。项目另有 `artifacts/perception/neisai_pcd_20261022/` 下的点云检查页面，但两种页面不是同一份文件，不能仅凭用途相似认定完全重复。

这两页还显示生成时的“长度单位尚未确认”，应作为历史输出保存。**建议从 `doc/` 移到 `artifacts/perception/standalone_pcd_viewers/`，保留页面内容。** 如你明确不再需要单文件查看器，再考虑删除。

### C03：旧的 MVP 状态清单，建议归档

```text
加油操作_MVP_状态清单.docx
```

文中状态停留在 2026-09-28，例如仍写“尚无已选定可用的手盖位姿”，与现有固定抓姿和后续规划进度不同。当前需求与进度已由 `doc/assembly_contract.md` 统一维护。

建议将这份 Word 作为旧稿归档，避免被当成当前状态；它可能包含你想保留的批注或旧讨论，默认不删除。

### C04：报告目录存在不同版本，不能按同名去重

```text
加油作业油箱盖旋拧系统研究报告_目录初稿.docx
doc/加油作业油箱盖旋拧系统研究报告_目录初稿.docx
doc/refueling_twisting_report_outline.md
```

已读取 Word 正文：根目录 Word 为较早的五章结构；`doc/` 下 Word 为六章结构，而且章节措辞与 Markdown 也不完全相同。它们不是重复文件。

需要你确定今后以哪份为主，再归档其他版本。暂不删除，也不自动用 Markdown 覆盖 Word。

另有 `双臂机器人装配系统研究报告(1).docx`，属于不同主题的参考材料，应保留；如需整理位置，可另行放入参考资料目录。

## D. 建议保留的现用文件

| 文件或目录 | 保留原因 |
| --- | --- |
| `tools/aubo_sdk_probe/` 及其 `build/` | 已验证的真机状态查询、采集、离线分析和可直接运行程序 |
| `tools/sdk/` | 本地官方 SDK、下载说明和逐文件摘要 |
| 根目录 `build/`、`install/` | 当前 MoveIt/ROS 运行与安装产物，暂不为整理目录而要求重建 |
| `src/fuel_cap_task/` | 项目主体模块，检查到命名导入关系 |
| `src/i10_rh56_moveit_config/` | 规划源码、启动文件、模型与网格；其中生成模型仍有来源绑定 |
| `tests/` 和 `tools/tests/` 内的测试源码 | 两处测试各有内容，不能把 `tools/tests/` 当缓存删除 |
| `doc/` 下两份 ASCII PCD 和根目录两张实拍 JPG | 当前实测输入，ASCII 数据与转换前逐字段一致 |
| `artifacts/hardware/` | 真机原始采集记录、统计、快照及打包资料 |
| 其他 `artifacts/` 结果、模型权重和源码快照 | 复现与来源证据，本轮没有逐项判定其可删除性 |
| `assets/`、`dependencies/`、各虚拟环境 | 模型资产、第三方资料、源码或运行依赖 |
| `doc/research_route_reference.md` | 合作方提供的参考原文 |

几组看似重复的脚本也建议保留：

- `segment_cap_local.py`：本机微调定位与分割入口。
- `infer_finetuned_cap.py`：被 `src/fuel_cap_task/finetuned_perception.py` 启动的独立检测进程。
- `segment_cap_image.py`：原预训练模型的基线对照入口。
- `segment_cap_cloud_predictions.py`：对已有云端自动框运行本机分割。
- `check_cap_rotation.py` / `preview_cap_rotation.py`：规划文档明确保留的单分支旋转对照。
- `build_*`：用于重新生成模型、安装件和规划资产，并非可按文件名删除的构建缓存。
- `view_i10_fuel_cap_rviz.py` / `.sh`：仍在模型查看文档中提供独立 RViz 操作入口。

## 审核方式

剩余项目可以按编号批准或保留，例如：

```text
同意 B02。
A05 等我关闭 VS Code 后再做。
C01 删除；C02、C03 只归档；C04 先保留所有版本。
```

A01～A04、B01、B03 已完成，其中 B01、B03 按用户后续决定保留 ASCII、删除二进制与 XYZ。B02 和 A05 仍待审核，C 组逐项确认。执行清理时，只处理批准的编号，并同步检查相关路径引用。
