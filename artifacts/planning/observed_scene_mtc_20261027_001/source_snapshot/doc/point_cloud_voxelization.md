# 跟着做：点云转表面体素

这一步把相机观测点归到三维格子中，得到可见表面的体素表示。先理解一个可手算的小例子，再运行整份点云，最后比较转换前后的形状。

当前采用的边长、原点与显示设置统一见[项目说明](assembly_contract.md#rgb-d-与三维接口)。

## 当前实测清理点云的 MuJoCo 查看

当前实测环境输入为 `doc/scene_cleaned.pcd`，边长及固定原点见[相机与实测场景设置](assembly_contract.md#相机)。入口读取 `config/active_point_cloud_scene.json`，将 SDK 相机系毫米坐标换算成米，复用 `voxelize_points()`，保留所有输入点的体素归属。这里的 MuJoCo world 定义为 SDK 相机系，不需要先完成左右臂基座换算。

[当前交互查看结果](../artifacts/perception/cleaned_scene_voxels_2mm_20261027_001_display/README.md)已保存。可直接运行下面的查看命令，将 `--scene-dir` 换成 `artifacts/perception/cleaned_scene_voxels_2mm_20261027_001`；工具会核对体素摘要并自动使用旁边的 `_display` 显示缓存。三种取景见[MuJoCo 预览](../artifacts/perception/cleaned_scene_voxels_2mm_20261027_001_display/preview.png)。

```bash
.venv/bin/python tools/voxelize_cleaned_scene.py \
  --output-dir artifacts/perception/my_cleaned_scene_voxels

.venv/bin/python tools/view_point_cloud_voxels.py \
  --scene-dir artifacts/perception/my_cleaned_scene_voxels
```

查看器默认聚焦油箱盖周围。左键拖动旋转、右键拖动平移、滚轮缩放，按 7 看完整场景、8 看盖体区域、9 看格子细节，Esc 返回自由相机。场景只更新显示，不推进物理时间。盖体所在格子为红色，其余按相机 Z 伪彩色着色。原始 box 场景中所有格子保留碰撞掩码。

完整 80,298 个独立 box 在当前图形环境中出现鼠标无响应，因此交互查看改为按原材质合并外露面，共 33 个显示 mesh；只移除两个相邻占据格之间的内部公共面。每个外露四边形仍为 2 mm 格面，体素 NPZ 与原版逐字节一致，不平滑、不补洞。显示 mesh 的碰撞掩码关闭，碰撞数据继续使用原始独立 box，避免整片环境的凸包填平凹槽。`--box-geoms` 可显式查看原始 box 模型。

原始 `scene.xml` 可直接加载到 MuJoCo。`voxels.npz` 保存 `indices_xyz`、`centers_camera_m`、`point_counts` 和 `point_to_voxel`；同目录 `voxel_centers_camera_mm.ply` 为毫米制中心点，可与原始点云叠加。`full_scene.png`、`cap_region.png`、`voxel_detail.png` 为 MuJoCo 实际渲染，`preview.png` 将三张图并排展示。显示版另存 `_display` 目录，构建入口为 `tools/build_voxel_display.py --scene-dir 原始结果目录 --output-dir 新显示目录`，逐三角形核对编译后的网格与原始整数格角一致。

关键实现见 [voxel_mujoco.py](../src/fuel_cap_task/voxel_mujoco.py)。MuJoCo 的 box `size` 是半边长，因此 2 mm 格子写作：

```xml
<!-- pos 为所属格子的米制几何中心；三个半尺寸均为 1 mm。 -->
<geom type="box" pos="0.101 0.051 0.651" size="0.001 0.001 0.001"/>
```

只生成观测表面占据格，不推断门板厚度、遮挡区域或自由空间，也不对整片环境取凸包。体素与 MuJoCo 模型的数值核对保存在 `validation.json`。OctoMap 导出和单臂旁的临时摆放见下节；真实右臂坐标换算、MoveIt 碰撞场景加载与路径规划继续待接入。

## OctoMap 与地图摆放

已将当前实测体素保存成原生 `OcTree` 的 `.bt` 文件，并为地图设置单臂仿真的临时位置。[结果目录](../artifacts/perception/cleaned_scene_octomap_2mm_20261027_001/)包含地图、变换、盖体标记和复读检查；[同屏预览](../artifacts/perception/cleaned_scene_octomap_2mm_20261027_001/placement_preview.png)展示完整臂手与实测地图，红色为盖体，黄色为外向盖轴。原占位油箱、盖体和门板已从此预览移除。

当前位置将实测 `direction_000` 盖体系对齐到原仿真的 `cap_grip_center`。数值和用途集中维护在[项目设置](assembly_contract.md#相机)及 [point_cloud_map_placement.json](../config/point_cloud_map_placement.json)。这是单臂仿真的地图摆放；两臂安装变换仍为未知，不能用于真机右臂目标。

从项目根目录生成新结果并查看：

```bash
.venv/bin/python tools/export_voxel_octomap.py \
  --output-dir artifacts/perception/my_scene_octomap

MUJOCO_GL=glfw .venv/bin/python tools/view_voxel_map_placement.py \
  --scene-dir artifacts/perception/my_scene_octomap --render

MUJOCO_GL=glfw .venv/bin/python tools/view_voxel_map_placement.py \
  --scene-dir artifacts/perception/my_scene_octomap --view
```

查看已有结果时，将 `--scene-dir` 替换为 `artifacts/perception/cleaned_scene_octomap_2mm_20261027_001`。鼠标旋转、平移和缩放；窗口只运行 `mj_forward`，不推进动力学或控制机器人。省略 `--view` 和 `--render` 时只核对来源与模型。

导出脚本用系统 `g++`、`pkg-config`、OctoMap 开发库及 `nlohmann/json.hpp` 临时编译原生转换器；本机已具备这些依赖，无需启动 ROS。也可用 `--converter` 复用已编译的可执行文件。

| 文件 | 用途 |
| --- | --- |
| `scene.bt` | 全部观测表面占据格，含盖体；文件本身不带 frame 或摆放变换 |
| `octomap_scene.json` | 分辨率、来源摘要、相机系和规划系、整图变换、两个盖体方向候选 |
| `voxel_labels.npz` | 原始 XYZ 格号和逐格盖体标记，供后续把盖体独立管理 |
| `validation.json` | 文件复读、原分辨率格号集合相等、无新增空闲格、摆放对齐误差 |
| `placement_config.json`、`scene_config.json` | 本次实际使用的配置副本 |
| `placement_preview.png`、`placement_preview.xml` | 机器人与地图同屏预览，不作为 MoveIt 碰撞检查结果 |

关键实现分两处。原生转换器 [voxel_octomap_export.cpp](../tools/voxel_octomap_export.cpp) 将整数格号转换成 OctoMap 键，不经过点云射线更新：

```cpp
// key 是整数格号加上 OctoMap 的键偏置；lazy_eval 最后统一更新内部节点。
tree.setNodeValue(key, tree.getClampingThresMaxLog(), true);
tree.updateInnerOccupancy();
tree.writeBinary(path);  // 只合并完全同值的八个子格，不填凹槽。
```

保存采用 [OctoMap 原生序列化接口](https://octomap.github.io/octomap/doc/classoctomap_1_1OcTree.html)。2 mm 是最细层分辨率，剪枝后的部分叶子可能更大；复读时把每个叶子展开回 2 mm 格号，检查与原输入集合完全相等，同时拒绝任何空闲叶子。当前 80,298 格对应 80,172 个占据叶子，1,504 个盖体格标记独立保留。

坐标计算在 [voxel_octomap.py](../src/fuel_cap_task/voxel_octomap.py) 的 `align_map()`：

```python
# 实测盖体系与目标盖体系对齐，移动整张地图，避免旋转后重新分格。
world_from_camera = world_from_cap @ np.linalg.inv(camera_from_cap)
camera_from_octomap = np.eye(4)
camera_from_octomap[:3, 3] = grid_origin_camera_m
world_from_octomap = world_from_camera @ camera_from_octomap
```

地图在局部网格中保存，原网格原点由 `camera_from_octomap` 补回，因此以后网格原点非零时也不会丢失平移。[MoveIt 的整图加载接口](https://moveit.picknik.ai/humble/api/html/classplanning__scene_1_1PlanningScene.html)可以接收地图与刚体位姿。后续应将 `.bt` 和 `world_from_octomap` 一起加载到 MTC 实际使用的 `PlanningScene`，并同步处理实测区域的旧占位障碍。

完整地图保留为转换与摆放基线；可分别加载的环境和盖体组件见下一节。MTC 的实测入口已加载拆分组件；当前模型为单臂，另一臂、相机和 T 型架仍待加入。

相关数值检查：

```bash
.venv/bin/python -m unittest discover -s tools/tests -p test_voxel_octomap.py -v
```

覆盖负格号、完整八格剪枝、带缺角的格子、无效输入、非零网格原点、混合格处理、旋转 box 的角点及原生 ROS 消息读写；实际数据另做全部格号的文件复读核对。

## 盖体与环境分开管理

当前已准备好环境 OctoMap 与独立盖体 `CollisionObject`，保存于[拆分结果](../artifacts/perception/cleaned_scene_split_2mm_20261027_001/README.md)。[拆分预览](../artifacts/perception/cleaned_scene_split_2mm_20261027_001/placement_preview.png)中，青色为环境，红色为独立盖体，黄色小格为归属未定的混合格。盖轴也用黄色标出。地图位置继续使用[项目设置](assembly_contract.md#相机)中的临时仿真变换。

| 组件 | 内容与用途 |
| --- | --- |
| `environment.bt` | 环境占据格，含保守保留的混合格 |
| `cap_collision_object.json` | ID 为 `fuel_cap` 的一个标准 MoveIt 碰撞对象，内部使用原边长的多个 box |
| `voxel_components.npz` | 全部原格号、完整点归属和组件类别：0 为环境、1 为纯盖体、2 为留在环境的混合格 |
| `mixed_voxels.json` | 混合格的原始点行号、坐标、盖体裁剪归属及最近盖体点距离 |
| `octomap_scene.json` | 环境地图、盖体对象、整图变换及摘要的统一入口 |
| `ros_message_validation.json` | 实际盖体文件的原生 ROS 序列化复读、全部 box 位置和朝向检查 |

原 1,504 个盖体标记格中，有两个同时包含盖体裁剪点和裁剪外点。这些点相距接近一个格宽，不能由相邻关系确定语义，因此混合格保留在环境。拆分后为 1,502 个独立盖体格和 78,796 个环境格，两部分不重叠，并集保持原 80,298 格；全部原始点归属仍保留。混合格不会随 `fuel_cap` 的接触放行而被忽略，后续若规划受其影响，需复核原始裁剪或补观测。这里的盖体几何仍为观测表面，没有推断不可见背面或内部实体。

从项目根目录生成与检查：

```bash
.venv/bin/python tools/split_voxel_collision_scene.py \
  --output-dir artifacts/perception/my_split_scene

.venv/bin/python tools/check_split_voxel_scene.py \
  --scene-dir artifacts/perception/my_split_scene

MUJOCO_GL=glfw .venv/bin/python tools/view_voxel_map_placement.py \
  --scene-dir artifacts/perception/my_split_scene --view
```

分离入口从当前配置读取已归档的完整地图及其原始体素，拒绝覆盖旧结果。保存预览图片时将 `--view` 换为 `--render`。检查盖体文件需要本机 ROS 2 Humble 的 Python 消息包，系统 ROS 环境已提供这些包。

关键计算在 [voxel_octomap.py](../src/fuel_cap_task/voxel_octomap.py) 的 `split_voxel_membership()`：

```python
# 原点归属恢复每格的盖体点数；全部点来自盖体裁剪时才归入独立对象。
cap_counts = np.bincount(point_to_voxel[cap_point_rows], minlength=voxel_count)
pure_cap = cap_counts == point_counts
mixed = (cap_counts > 0) & ~pure_cap
environment = ~pure_cap  # 混合格仍是障碍，不会误随目标放行。
```

`cap_collision_object()` 将纯盖体格中心与朝向一起变换到规划系，保持每个格子的原始体积；所有 box 共用一个对象 ID。对象 `pose` 为单位变换，每个 `primitive_pose` 都在消息 `header.frame_id` 中表达，避开不同版本对整体 pose 的处理差异。JSON 中的 `operation: [0]` 可由本机 `rosidl_runtime_py.set_message_fields()` 转成 `CollisionObject.ADD` 单字节值。[MoveIt 场景接口](https://moveit.picknik.ai/humble/api/html/classplanning__scene_1_1PlanningScene.html)加载的是该对象和环境八叉树，点云本身不直接充当碰撞对象。

两个组件已接入下面的 MTC 实测入口，未增加接触放行、附着对象或轨迹执行。

## 实测场景接入 MTC

已将环境 OctoMap 和独立盖体加载到 MTC `FixedState` 实际持有的 PlanningScene。加载时移除旧 `fuel_tank_body`、`cap_body`、`fuel_door` 对应对象，保留地面与完整臂手链、传感器和 adapter。两个候选共用同一地图变换；不传 `--observed-scene` 时，原仿真场景入口继续可用。

[本次结果](../artifacts/planning/observed_scene_mtc_20261027_001/README.md)包含场景检查、模型核对、成功轨迹及被拒绝方向的原因。`direction_000` 完成绕障碍到预抓取位，再沿盖轴接近；`direction_180` 的预抓取位可达，但其目标处拇指中节与环境格相交，MTC 未生成完整接近轨迹。失败文件为 `mtc_failed_no_replay`，不会作为成功路径显示。

先生成绑定实测场景的请求，再调用已安装的标准入口：

```bash
.venv/bin/python tools/prepare_cap_approach.py \
  --start 0 -.5 1.2 0 0 0 --distance .1 --direction direction_000 \
  --observed-scene artifacts/perception/cleaned_scene_split_2mm_20261027_001/octomap_scene.json \
  --output artifacts/planning/my_observed/request.json

source /opt/ros/humble/setup.bash
source install/setup.bash
env -u CYCLONEDDS_URI ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=47 \
  ros2 launch i10_rh56_moveit_config approach.launch.py \
  request_path:="$PWD/artifacts/planning/my_observed/request.json" \
  output_path:="$PWD/artifacts/planning/my_observed/trajectory.json"
```

这里使用[项目设置](assembly_contract.md#当前进度)中的离线起点与临时地图摆放。DDS 设置仅对该进程生效，使离线演示不依赖现场网卡配置。起点一旦存在碰撞或关节超限，规划停止并保存原因，不移动起点来规避失败。

查看同一份实际规划场景与成功轨迹：

```bash
env -u CYCLONEDDS_URI ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=47 \
  ros2 launch i10_rh56_moveit_config observed_scene.launch.py \
  request_path:="$PWD/artifacts/planning/my_observed/request.json" \
  output_path:="$PWD/artifacts/planning/my_observed/scene_audit.json" \
  trajectory_path:="$PWD/artifacts/planning/my_observed/trajectory.json" \
  publish_seconds:=600.0 rviz:=true
```

场景由 `audit_observed_scene` 经 `/i10_observed_planning_scene` 和 `get_planning_scene` 服务提供。轨迹通过 `/display_planned_path` 显示；显示消息中的十秒动画时长不是执行时序。可不传 `trajectory_path`，只核对场景。该入口重新运行相交/分离和完整模型几何参与检查，默认不执行任何运动。

也可以在原 MuJoCo 查看器中回放已验证路径：

```bash
MUJOCO_GL=glfw .venv/bin/python tools/view_voxel_map_placement.py \
  --scene-dir artifacts/perception/cleaned_scene_split_2mm_20261027_001 \
  --trajectory artifacts/planning/my_observed/trajectory.json --view
```

空格暂停，R 重播；只运行 `mj_forward`。原 `replay_cap_approach.py` 使用占位油箱，遇到实测请求会拒绝，避免显示错误环境。需要重新运行两方向离线验证时，使用 `tools/run_observed_mtc_validation.py --output-dir 新目录`。

关键加载逻辑位于 [observed_scene_loader.hpp](../src/i10_rh56_moveit_config/src/observed_scene_loader.hpp)，MTC 入口在 [plan_approach.cpp](../src/i10_rh56_moveit_config/src/plan_approach.cpp)：

```cpp
// 整张局部八叉树使用已保存的刚体变换，盖体作为独立对象加入同一场景。
scene.processOctomapPtr(octree, world_from_octomap);
scene.processCollisionObjectMsg(cap);
fixed->setState(scene);  // MTC 与碰撞验证共用此场景。
```

加载前后均核对清单和几何文件的 SHA-256；两方向目标由 [observed_mtc.py](../src/fuel_cap_task/observed_mtc.py) 从同一份实测候选计算。检查对环境、盖体及所有含碰撞几何的 link 分别构造相交对照，同时检查明确分离的探针；规划路径按[项目采样设置](assembly_contract.md#当前进度)加密，并检查整个单臂模型的关节限位和碰撞，传感器、adapter、掌部和所有手指均参与。源覆盖模型的 FK 与碰撞对照一致；规划覆盖新增地面，故原 MuJoCo 没有地面碰撞的两个参考构型会额外报告臂与地面相交，详见保存的模型诊断。

当前结果验证临时单臂摆放下的几何路径；该入口尚未切换到已记录的双臂安装链，完整现场障碍及真机工具安装仍须核对，接触控制和硬件轨迹执行尚未接入。本机 RViz 显示与场景服务核对正常，窗口退出阶段出现段错误；已将退出日志保存在结果目录，数值规划与路径检查在独立进程中完成。

## 第一步：理解点与格子的区别

一个点只有位置。一个体素则是有位置和边长的立方格，里面可以落入多个点。

用边长 5 mm、原点为零的网格举例：

| 点 | 世界坐标 XYZ（mm） | 所属格号 XYZ |
| --- | --- | --- |
| A | `[12,23,41]` | `[2,4,8]` |
| B | `[14,24,44]` | `[2,4,8]` |
| C | `[16,23,41]` | `[3,4,8]` |

A 和 B 属于同一个格子，C 属于相邻格子。第 2 个 X 格占 `[10,15)` mm；左端包含在格子内，右端属于下一个格子。负坐标也按向下取整计算，例如 X=-1 mm 属于第 -1 格。

核心实现位于 `src/fuel_cap_task/voxelization.py` 的 `voxelize_points()`：

```python
# 每个坐标先减去网格原点，再除以边长，向下取整得到 XYZ 格号。
point_indices = np.floor((points - origin) / size_m).astype(np.int64)
```

`points`、`origin` 和 `size_m` 都以米为单位，所以运算后的格号没有长度单位。原点固定在世界系，而不是每次改成点云的最小坐标，后续不同观测才有相同的格子编号。

可以先在项目根目录手动运行小例子：

```bash
.venv/bin/python - <<'PY'
import numpy as np

# 毫米先换成米，三行分别为 A、B、C。
points = np.array([[12,23,41], [14,24,44], [16,23,41]]) / 1000
indices = np.floor(points / .005).astype(int)
print(indices)
PY
```

应打印表格里的三组格号。

## 第二步：把相同格号合并

```python
# 相同格号只保存一次；inverse 保留每个原始点归属哪一格。
indices, inverse, counts = np.unique(
    point_indices, axis=0, return_inverse=True, return_counts=True)
```

在小例子中只剩两个格子，点数分别是 2 和 1；`inverse` 为 `[0,0,1]`。每格还记录其中所有点的平均 RGB。保留这份对应关系后，后续可从原图像素追踪到点，再追踪到体素。

显示用的格子中心计算为：

```python
# 格号指向格子的下边界，加半格才得到中心。
centers = origin + (indices + .5) * size_m
```

第 `[2,4,8]` 格的中心是 `[12.5,22.5,42.5]` mm。这个中心不一定是实际物体表面点，也不是自动求得的抓取点。

## 第三步：处理整份观测点云

输入来自[已归档的 RGB-D 点云](rgbd_point_cloud.md#已保存的点云结果)，不读取 MuJoCo 场景几何或盖体真值。使用新目录保存新结果：

```bash
.venv/bin/python tools/voxelize_cap_point_cloud.py \
  --point-cloud-dir artifacts/perception/rgbd_point_cloud_20261016/point_cloud \
  --voxel-size-m .005 \
  --origin-world-m 0 0 0 \
  --output-dir artifacts/perception/my_voxels_5mm
```

| 输出 | 内容 |
| --- | --- |
| `voxels.npz` | 全部格号、中心、点数、平均颜色、原点与边长；`point_to_voxel` 对应输入点顺序 |
| `index.html` | 点云与体素中心的全景比较，点击后显示局部真实立方格 |
| `preview.png` | 全景与初始局部的静态预览 |
| `voxel_centers.ply` | 中心点，供外部软件查看；立方格体积需结合边长解释 |
| `result.json` | 数据来源、点数、体素数量、设置、摘要和显示范围 |

NPZ 使用稀疏索引，只保存被观测点命中的格子。未列出的格子保持未知；没有执行自由空间建图、内部填充或 TSDF 融合。当前整场景表示也还需要选取局部范围、核对尺度与输入约定，才能交给 SpaHybGen。

## 第四步：看首次零原点结果

已保存的体素示例在 `artifacts/perception/rgbd_point_cloud_20261016/voxels_5mm/`。不需要 `xdg`，从项目根目录启动：

```bash
python3 -m http.server 8766 --bind 127.0.0.1 \
  --directory artifacts/perception/rgbd_point_cloud_20261016
```

在浏览器打开 `http://localhost:8766/voxels_5mm/index.html`。同一个服务也能打开 `http://localhost:8766/point_cloud/index.html`。

[结果报告](../artifacts/perception/rgbd_point_cloud_20261016/voxels_5mm/result.json)记录实际点数与体素数；[逐点归属复核](../artifacts/perception/rgbd_point_cloud_20261016/voxels_5mm/validation.json)确认所有输入点都在其所属格子内，点数和对应关系保持一致。测试输出与实现快照一并保存在该结果目录中。

1. 左侧先切到“原始点云”，观察盖体、门板和油箱表面。
2. 再切到“体素中心”，比较哪些位置合并成了同一格。
3. 点击盖体或门板，右侧查看附近按实际边长画出的立方体，旋转并放大观察。

全景显示中心点以便交互，右侧才是格子的真实体积。局部显示限制不会删除存储数据。物体表面恰在网格边界上时，深度的微小波动也可能命中相邻两层格子；本工具保留原观测，没有把这种波动偷偷归零。

第一次先检查形状、尺度和像素到格子的对应是否正确，再观察握梁、凹槽边缘是否被 5 mm 网格合并。边长越大，细节越容易合并；边长越小，格子数量和计算量通常越多。

## 精细观察：先分清取景、网格和显示

首次页面有两类现象：门板顶部在原始 RGB 中已经出画，默认局部窗口又落在油箱中部；另一方面，近似平面刚好跨在网格边界上，微小起伏被表示成相邻两层格子。这些原因需要分别处理，不能只把格子改小。

精细观察使用扩大取景的新 RGB-D；在同一份新点云上比较 5 mm、2 mm 两种网格。每种网格的原点取各轴负半格，将世界原点置于格子中心。观测点本身不移动、不平滑；这能减少本例轴对齐平面恰在边界上的分层，但斜面仍会产生正常的离散台阶。

新查看器顶部有原始 RGB。点击门板或盖体，按原始像素对应直接查出体素编号；无有效深度的像素会明确提示。局部观察半宽按毫米设置，改变体素分辨率时不再跟着改变物理观察范围。超过局部显示上限时页面会提示，不能据此判断数据缺失。

新的观测与两种网格保存在 `artifacts/perception/rgbd_voxel_detail_20261016/`，旧结果保留。从项目根目录查看：

```bash
python3 -m http.server 8767 --bind 127.0.0.1 \
  --directory artifacts/perception/rgbd_voxel_detail_20261016
```

- `http://localhost:8767/voxels_2mm/index.html`：较细网格。
- `http://localhost:8767/voxels_5mm/index.html`：同一份观测的较粗网格。
- `http://localhost:8767/point_cloud/index.html`：原始重建点云。

[像素与体素对应检查](../artifacts/perception/rgbd_voxel_detail_20261016/validation.json)核对全部有效像素的格子编号、无效像素、逐点归属和点数守恒。[边界诊断](../artifacts/perception/rgbd_voxel_detail_20261016/diagnostics/original_boundary_diagnostic.json)保留同一份旧点云改变网格原点的对照；[归档摘要](../artifacts/perception/rgbd_voxel_detail_20261016/archive.json)记录输入、源码快照及保存文件的摘要。

先点击照片里的门板，再点击盖体，比较两种边长保留了哪些细节。半宽默认 40 mm；可调大观察范围，页面会说明当前是否达到显示数量上限。固定原点、范围和输入来源都要一起记录，不能只记录“用了几毫米体素”。

重新运行较细网格时，使用新输出目录：

```bash
.venv/bin/python tools/voxelize_cap_point_cloud.py \
  --point-cloud-dir artifacts/perception/rgbd_voxel_detail_20261016/point_cloud \
  --voxel-size-m .002 --origin-world-m -.001 -.001 -.001 \
  --local-radius-m .04 \
  --output-dir artifacts/perception/my_voxels_2mm
```

## 这部分需要掌握到什么程度

当前目标是能读懂核心计算、说明输入输出、选择参数并定位错误：

1. 说清 RGB、深度、点云、体素的数据关系和单位。
2. 看懂深度反投影、坐标变换、点到格号三步计算。
3. 用小例子手算一个点属于哪个格子，理解边长与原点分别改变什么。
4. 区分没采到、没转换成功、没在页面显示；会检查像素对应与数据摘要。
5. 知道未观测区域保持未知，网络还需要约定局部范围、体素尺度和输入格式。

先读 `rgbd_point_cloud.py` 的 `backproject_depth()`、`transform_points()`，以及 `voxelization.py` 的 `voxelize_points()`，重点看注释、计算和返回值。`pixel_to_voxel_lookup()` 则展示照片中的点怎样追踪到格子。暂时不要求深入渲染引擎和网页库内部；真正采用 TSDF、多视角融合或修改网络时，再补相应方法。

## 数值检查

```bash
PYTHONPATH=src .venv/bin/python -m pytest \
  tests/test_rgbd_point_cloud.py tests/test_voxelization.py -q
```

检查小例子、负坐标、边界、固定原点、点数与对应关系、颜色平均以及显示立方体的边长。实际样本再核对每个点都落在其所属格子内，所有格子的点数之和等于输入点数。

体素化含义参考 [Open3D 点云体素化说明](https://www.open3d.org/docs/release/tutorial/geometry/voxelization.html#from-point-cloud)。
