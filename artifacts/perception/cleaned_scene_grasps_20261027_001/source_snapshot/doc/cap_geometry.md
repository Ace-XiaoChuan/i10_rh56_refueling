# 从裁剪点云估计盖体位置与朝向

本文行数：187行，预期阅读时长：11分钟

输入是整盖点云，输出是相机系中的盖轴、握梁长轴、观测轮廓近似圆，以及结合占位模型得到的握梁参考中心。选区可以由 CloudCompare 人工裁剪，也可以用显式 ROI 与三维种子辅助提取。有配对的停稳观测时，可核对来源后换算到基座系；当前输出为诊断候选，坐标约定和实物精度仍需复核。

## 先看结果与用法

[首份结果三视图](../artifacts/perception/cap_geometry_initial/preview.png)中，蓝色是盖面，橙色是握梁，紫色虚线是观测外轮廓的近似圆。红叉表示握梁可见表面的参考点，绿加号表示按占位模型换算的 `cap_grip_center`。两点沿盖体外向轴相差 8.5 mm。

可在 Windows CloudCompare 中打开结果目录的 `classified_cap.pcd`，旋转检查分组；`face.pcd`、`bridge.pcd`、`boundary.pcd` 是对应子集。它们只包含真实观测点，仍使用输入文件的毫米单位。`derived_reference_points.pcd` 单独保存计算得到的三个参考点，紫色为轮廓圆心、红色为表面参考点、绿色为模型中心，不把它们混入观测点云。

在项目根目录运行，输出目录选新名字：

```bash
.venv/bin/python tools/estimate_cap_geometry.py \
  --pcd doc/fuel_cap_01_cap_region_ascii.pcd \
  --length-unit mm \
  --output-dir artifacts/perception/my_cap_geometry
```

依赖为本项目 `.venv` 已有的 NumPy、SciPy、Matplotlib；本次运行版本为 NumPy 2.2.6、SciPy 1.15.2。程序不启动机器人或修改模型。

`result.json` 中长度统一为米，方向向量无量纲，姿态四元数为 XYZW。`groups.npz` 保存米制相机点和输入点顺序对应的分组索引。采集时间保持未知，`motion_target_valid=false`；两个姿态候选都在 `camera` 中表达。

## 绑定停稳观测并换算到基座系

本节已有观测中的机器人状态来自左侧拍照臂，`base_from_flange` 和 `pose_candidates_base_model_based` 均对应左臂基座。用于右侧旋拧臂时，还需两基座固定变换；公式与设备对应见[双臂分工与坐标传递](assembly_contract.md#双臂分工与坐标传递)。左臂让位后继续使用采集时保存的法兰位姿。当前工具尚未接入跨臂转换，现有输出继续作为诊断候选。

已配对观测来自 `artifacts/hardware/vizum_i10_capture_20261026T055151Z/`，原始场景有 268,852 点。当前辅助选区含 5,742 点：盖面 4,446 点、握梁 1,296 点。选区由当前点云的宽松 ROI 与两个实际观测点作为种子得到，保留原始行号；没有按预设圆半径切边，也没有补点。

[当前拟合三视图](../artifacts/perception/cap_observation_20261026_001/geometry/preview.png)与[手模型叠加](../artifacts/perception/cap_observation_20261026_001/overlay/preview.png)用于复核选区、表面参考点及固定抓姿。蓝色是盖面，橙色是握梁，绿色参考点沿用模型的 8.5 mm 高度偏移。观测轮廓拟合直径为 76.36 mm，轮廓平面及圆 RMS 分别约 0.95 mm、1.10 mm；这些是对观测点的拟合残差，不是实物定位精度。此前观测的约 72 mm 结果保留，应核对两次是否使用相同物理边缘。

完整命令在项目根目录运行，输出目录必须是新目录：

```bash
.venv/bin/python tools/crop_cap_observation.py \
  --pcd artifacts/hardware/vizum_i10_capture_20261026T055151Z/cloud_sdk_native.pcd \
  --length-unit mm --bounds-mm 65 195 -55 80 620 700 \
  --seed-mm 131.0390625 7.15625 655.6328125 \
  --seed-mm 127.140625 16.76953125 634.86328125 \
  --connection-radius-mm 1.8 --output-dir artifacts/perception/my_observation/crop

.venv/bin/python tools/estimate_cap_geometry.py \
  --pcd artifacts/perception/my_observation/crop/cap_crop.pcd --length-unit mm \
  --observation artifacts/hardware/vizum_i10_capture_20261026T055151Z/observation.json \
  --crop-manifest artifacts/perception/my_observation/crop/crop.json \
  --output-dir artifacts/perception/my_observation/geometry
```

ROI、种子和 1.8 mm 连通距离只对应本次观测。激光扫描线之间存在采样间隔，选定的盖面与握梁分别是两个三维连通区域；扩大连接距离可能把凹腔或背景连进来。换观测时先看点云再选参数。

[观测适配实现](../src/fuel_cap_task/cap_observation.py)检查原始点云、裁剪、外参的 SHA-256；检查裁剪仍是该份源点云的子集、前后状态为真机静止、主机调用时间顺序正确，以及法兰矩阵符合保存的米制固定轴 XYZ RPY。外参使用采集目录中的副本，后续改动工作区外参不会悄悄替换历史版本。

所有 JSON 平移使用米。`base_from_flange` 将法兰点映射到机器人基座系，`flange_from_camera` 将相机点映射到法兰系，`camera_from_cap` 表达盖体参考帧：

```python
# 原始毫米点先转米，几何估计和两级装配变换随后都在米制下计算。
p_camera_m = 0.001 * p_sdk_mm
base_from_camera = base_from_flange_at_capture @ flange_from_camera
base_from_cap = base_from_camera @ camera_from_cap
```

`result.json` 保留原相机系字段，新增 `pose_candidates_base_model_based`、基座系盖轴及 `observation_binding`。绑定保存采集前后六轴角、法兰矩阵、外参摘要和主机时间窗口，使用采集时状态，不使用后来查询的当前姿态。硬件曝光时间仍未知，`capture_timestamp_s` 不伪造为主机时间。

### SDK 原生坐标与握梁的 180° 歧义

用户说明已有拍摄点云也来自 SDK；当前采集调用 `VzNL_PointCloudSnapShort` 明确传入 `keFlipType_None`，保存的点云声明 `mirror=none`、`frame=camera_sdk_native`。当前定位直接使用 SDK 原始 XYZ，长度从毫米换算到米，再使用已有相机到左臂法兰外参；没有额外的 Y 翻转步骤。

此前镜像对照仅表示人为改变坐标后的数值差异，没有提供实际发生 Y 镜像的证据；不再作为当前定位的待解决项。旧对照结果保留，当前[原生坐标换算结果](../artifacts/perception/cap_observation_20261026_002/geometry/result.json)沿用同一份原始点云、裁剪和采集时外参，数值与旧结果的原生坐标分支一致。

握梁长轴仍有两个绕盖轴相差 180° 的方向候选，它们具有相同位置和盖轴。后续检查抓姿适用性或多视角观测时按无向轴比较。外参实物精度、8.5 mm 模型偏移及真实工具安装仍需复核；当前离线工具输出保持 `motion_target_valid=false`。

## 本次观测得到什么

本次输入为用户确认的整盖裁剪，5,378 个有效点。裁剪结果已与原始 28,897 点匹配，最大最近点差约 `7.69e-6 mm`，只反映导出舍入，不表示相机或位姿精度。

| 量 | 本次几何结果 |
| --- | --- |
| 观测外轮廓拟合直径 | 约 72.00 mm |
| 盖面 / 握梁分组点数 | 3,995 / 1,383 |
| 观测轮廓支持点数 | 65 |
| 轮廓平面 / 平面内圆的 RMS 残差 | 约 0.792 / 0.620 mm |
| 握梁表面二次高度拟合 RMS | 约 0.365 mm |
| 外向轴在相机系中的分量 | `[-0.01166, 0.05689, -0.99831]` |
| 握梁长轴的一个符号候选 | `[0.53760, 0.84217, 0.04172]` |

随机保留 80% 点重复 12 次，均能得到结果；相对全量拟合，轮廓圆心最大变化约 0.090 mm、表面参考点约 0.180 mm、外向轴约 0.264°、握梁长轴约 0.614°。这只衡量移除部分点后的敏感性，不能反映共同的裁剪偏差、标定偏差或实物真值误差。

外轮廓来自投影凸包。人工剪切边界、遮挡或离群点都可能改变它；因此近似圆不能直接视为实物圆沿的精确测量。盖面本身有曲率，程序使用外轮廓支持点估计方向，不把全部盖面点强行拟合成同一个平面。

## 握梁尺寸与参考点

当前尺寸和来源统一见[项目说明](assembly_contract.md#盖体几何估计)。[URDF](../assets/objects/fuel_cap/urdf/fuel_cap.urdf)仍用长方体包络表示握梁：沿项目盖体系 X 长 60 mm、Y 根部宽 19 mm、Z 高 17 mm。用户确认实际描述为梯形截面，顶面总宽较根部少 2 mm，因此顶面宽为 17 mm，每侧向内收 1 mm。

宽度收窄发生在 Y 方向，顶面到参考中心的偏移发生在 Z 方向。现有 `cap_grip_center` 位于 `cap_body` 的 Z=30 mm，`cap_grip_top_center` 位于 Z=38.5 mm，故当前抓姿参考点到顶面为 8.5 mm。这里沿用已保存抓姿的高度中点，不将参考点改成梯形体的体积质心。程序只读取这些模型关系，没有重建或改写 URDF。

设外向单位轴为 `n`，握梁可见表面参考点为 `p_surface`，单位均按 JSON 的米制解释：

```python
# 顶面沿 +Z 位于参考中心外侧，求中心时沿 -Z 回退。
p_grip = p_surface - 0.0085 * n
```

实际代码从 URDF 解析偏移，不硬编码 8.5 mm。该位置属于“观测表面 + 占位模型尺寸”的估计，实物高度与模型一致程度仍需核对；采用模型尺寸的来源随结果保存。

## 算法怎样工作

实现见 [cap_geometry.py](../src/fuel_cap_task/cap_geometry.py)，运行与导出见 [estimate_cap_geometry.py](../tools/estimate_cap_geometry.py)。

1. 整体 PCA 得到粗法向。假设相机看到盖外侧，用从盖体指向相机原点的方向选择法向正负。
2. 将点投影到该法向，对一维高度做确定性二聚类，分出靠外的握梁与盖面；两组没有清楚分层时拒绝估计。
3. 盖面投影凸包提供观测外轮廓。用 SVD 拟合其平面，再在平面内稳健拟合圆；分组与法向交替更新三次。
4. 将握梁支持点投影到估计盖面，以最大主方向求长轴 X；Z 为外向轴，Y=`Z×X`，组成右手旋转矩阵 `R=[X Y Z]`。
5. 拟合握梁表面的局部二次高度，在观测圆心轴线上求表面参考点，再按模型偏移换算现有抓姿原点。

分组至少各 30 点；轮廓至少 8 点且角度覆盖不少于 270°，平面和圆 RMS 分别不超过半径的 4% 和 6%；握梁投影协方差的长短特征值比至少为 4。这些是首版几何拒绝条件，不是相机精度、抓取成功率或现场到位容差。

握梁长轴只有一条无向直线。程序保存两个绕盖轴相差 180° 的姿态候选，同时反转 X、Y，保持 Z。坐标符号的确定性约定不等于已经识别握梁的物理正端。

## 固定抓姿叠加

已将 RH56 按现有固定抓姿放到当前实测盖体点云的 0°、180° 两个方向上。[两方向查看页](../artifacts/perception/cap_grasp_candidates_20261026_001/index.html)和[对比图](../artifacts/perception/cap_grasp_candidates_20261026_001/comparison.png)显示整体及握梁局部：浅灰为 RH56，蓝色为盖面，橙色为握梁；红、绿、蓝箭头对应盖体系 X、Y、Z。两方向使用相同的取景中心与距离，观测点显示球半径为 0.45 mm，不改变保存点云。

手根使用固定抓取位置，手指使用候选中的接近预型。手指预型与手根的预抓取位置是两个概念：此次只放置固定抓姿，没有沿盖轴外退到另一个位置。两个方向均保留为候选，尚未选定执行方向。新结果中的 0° 手根抓取位与此前已保存结果一致。

在 Windows CloudCompare 中隐藏已有其他点云，把结果目录的以下两个文件一起拖入即可自动叠加，二者坐标均为相机系毫米，无需执行配准：

- `cap_cloud.pcd`：原有着色观测点，文件字节与几何估计输出一致。
- `rh56_fixed_grasp.ply`：位于抓取位置的 RH56 显示网格，包含 13 个手部部件，带表面法向和颜色。

当前目录为 `artifacts/perception/cap_grasp_candidates_20261026_001/`，`direction_000/` 和 `direction_180/` 分别保存两方向的点云、手网格、预览和 `overlay.json`；后者包含相机系及左臂基座系手根抓取位、手指通道、十二关节角（rad）、指尖位置（m）及来源摘要。根目录的 `grasp_candidates.json` 汇总两方向位姿。JSON 位移和网格内部计算均使用米，导出给 CloudCompare 时才转换为毫米。历史归档保持原样，旧版手网格文件名为 `rh56_fixed_pregrasp.ply`。

重现命令：

```bash
.venv/bin/python tools/preview_cap_cloud_grasp.py \
  --geometry-dir artifacts/perception/cap_observation_20261026_002/geometry \
  --pose-index 0 --output-dir artifacts/perception/my_cap_grasp_000

.venv/bin/python tools/preview_cap_cloud_grasp.py \
  --geometry-dir artifacts/perception/cap_observation_20261026_002/geometry \
  --pose-index 1 --output-dir artifacts/perception/my_cap_grasp_180
```

`--pose-index 0` 为当前预览候选，`--pose-index 1` 可显示另一个候选；选择索引只是在显示两个可能方向，不表示已辨别物理正端。输出目录必须是新目录。默认使用 EGL 离屏渲染，`--no-render` 可只导出 PLY、PCD 和变换数据；无需在 WSL 打开 GUI。

设 C 为点云相机系、O 为 `cap_grip_center`、H 为手根 `base`，变换将后一个坐标系的点映射到前一个坐标系：

```python
# camera_from_cap 来自点云位姿，cap_from_hand 来自已有固定候选。
camera_from_hand = camera_from_cap @ cap_from_hand
```

实现见 [cap_hand_overlay.py](../src/fuel_cap_task/cap_hand_overlay.py) 和[运行入口](../tools/preview_cap_cloud_grasp.py)。`HandOnlyScene` 从已校验的历史来源 XML 中只提取 RH56 的手部网格、关节、耦合和驱动映射，再编译独立手模型；编译模型中不含占位油箱、油箱盖、机械臂、传感器或地面。目标位置和朝向完全来自点云几何结果，固定抓姿配置保持原值。每方向保存派生的 `hand_only.xml`，只更新 mocap/qpos 并执行 `mj_forward`，不推进物理时间或调用机器人接口。

导出的顶点使用 MuJoCo 编译后网格与 `geom_xpos/geom_xmat` 的完整变换，不重复施加网格居中补偿。核对了手盖相对位姿、实际模型手根、每个网格和指尖随刚体变换的一致性，并通过独立 PLY 读取器逐点核对毫米/米单位和三角形索引。细节见 [MuJoCo 3.13.0 mesh 文档](https://mujoco.readthedocs.io/en/3.13.0/XMLreference.html#asset-mesh)。

这一步检查静态几何关系；手根的数值变换吻合不等于手与实物已经抓稳，尚未评价接触力、承载或机械臂可达性。盖体参考中心仍使用前述占位模型的 8.5 mm 偏移。

### 当前清理场景中的两方向手模型

当前环境统一使用用户清理的 `doc/scene_cleaned.pcd`（247,103 点），入口为 `config/active_point_cloud_scene.json`。全部 5,742 个盖体观测点仍保留，清理点云坐标与原始扫描仅有 ASCII 保存精度差异，因此复用已有两个方向的手网格和手根抓取位，不再变换或重新对齐。

[清理场景叠加查看页](../artifacts/perception/cleaned_scene_grasps_20261027_001/index.html)可离线旋转查看，切换 0°、180°、同时显示或只看点云；“完整场景”和“抓取区域”控制取景范围。网页包含全部清理点云和完整手网格。结果目录同时保存 `scene_cleaned.pcd`、`hand_000.ply`、`hand_180.ply`，可直接一起加载到 CloudCompare，使用相机系毫米坐标，无需配准。

```bash
.venv/bin/python tools/overlay_grasps_on_scene.py \
  --scene-config config/active_point_cloud_scene.json \
  --output-dir artifacts/perception/my_cleaned_scene_grasps
```

该入口核对场景摘要、清理点与原始扫描的最近点距离、盖体点保留情况及手网格摘要。相机系和左臂基座系手根位姿沿用原候选，保存在 `result.json`；`validation.json` 记录实际数据核对结果。本次为静态场景叠加，接近段和碰撞检查尚待完成。

## 验证与接入

```bash
.venv/bin/python -B -m pytest tests/test_cap_geometry.py -q
```

用已知曲面盖、握梁和圆沿检查方向与参考位置，验证刚体变换、点序变化和小噪声；拒绝无握梁、半盖、非有限与退化输入。另用不同宽高的独立 URDF 样例检查宽度和轴向偏移不会混用。

接入固定抓姿时使用 `pose_candidates_camera_model_based`，不能将 `visible_bridge_surface_reference_m` 直接当作 `cap_grip_center`。变换到基座还需对应采集时的基座到法兰变换，且 180° 候选需要继续辨别或分别检查抓姿适用性、可达性与碰撞。

原始完整点云保留为 `doc/fuel_cap_01_ascii.pcd`，本次裁剪另存为 `doc/fuel_cap_01_cap_region_ascii.pcd`。输入和实现摘要随每次结果保存，已有归档保持不变。

相关 API 见 [SciPy 1.15.2 ConvexHull](https://docs.scipy.org/doc/scipy-1.15.2/reference/generated/scipy.spatial.ConvexHull.html) 与 [least_squares](https://docs.scipy.org/doc/scipy-1.15.2/reference/generated/scipy.optimize.least_squares.html)。
