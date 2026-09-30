# FR3-RH56 转接件 RViz 预览

本包提供 FR3、两件转接件与右 RH56 灵巧手的相对朝向预览，以及 MVP-3 MoveIt
使用的固定张开手型 collision model。它不修改官方 FR3 描述，也不向 MuJoCo 提供
控制接口。

所有固定变换的默认值由
`config/fr3_rh56_mount_provisional.yaml` 提供。这是一份 R1 provisional
model-collision 合同：RViz 允许逐段检查其模型和 TF；MVP-3 MTC 使用同一组固定
变换和同一版 visual/collision mesh 作无接触避障。它不是实物安装标定，不能用于
接触、真机或精确碰撞结论。
launch 参数仍可临时覆盖任一变换以比较姿态；确认后的值必须回写该 YAML，而不是留在命令行。

预览机械臂固定在官方 `fr3_arm` 的 `ready` 关节构型：
`[0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4]`。该值来自
`franka_description/robots/common/group_definition.xacro`，与项目任务中
`home -> ready` 的映射一致，不再使用 `joint_state_publisher` 自行选择的默认关节值。
预览节点使用 `/adapter_preview` 命名空间，其关节状态为
`/adapter_preview/joint_states`，不会与 MoveIt 或其他预览抢占全局 `/joint_states`。

## 待机械确认的 RH56 安装变换

本轮软件验证可先不使用两个转接件。为使后续机械数据能替换进来而不改动手部或任务接口，
预留安装变换：

```text
T_fr3_link8_rh56_base: rh56_right_base_link -> fr3_link8
```

它满足 `p_fr3_link8 = T_fr3_link8_rh56_base * p_rh56_base`，即描述 RH56 基座坐标系在
FR3 末端 `fr3_link8` 坐标系中的位姿。机械设计完成后应提供：法兰中心定义、
`rh56_right_base_link` 基准定义、平移（m）、旋转（约定为 RPY 或四元数）、以及对应的
CAD/实物测量依据。

当前状态：该变换尚未获得机械确认，不能用于真实硬件、碰撞检查、抓取接触或 MuJoCo
插接验收；当前 adapter 网格预览中的任何 `xyz/rpy` 都不是它的替代值。

替换规则：机械数据到位后，仅实现或更新这一固定变换；需要展示机械结构时，可在其内部
展开为 `adapter_1`、`adapter_2` 的固定链。`rh56_right_base_link`、`rh56_grasp_tcp`、
MoveIt 规划组和任务/控制接口保持不变。

默认预览链为：

```text
fr3_link8 -> adapter_1_link
```

确认 `adapter_1` 后，可通过 `show_adapter_2:=true show_rh56:=true` 显示完整预览链：

```text
fr3_link8 -> adapter_1_link -> adapter_2_link -> rh56_right_base_link
```

`adapter_1` 的 visual mesh 来自 SolidWorks 的“坐标系1”导出：该坐标系位于法兰配合面，
网格以米为单位，URDF 不再额外施加 visual offset 或 `0.001` 缩放。其 link 初始时与
`fr3_link8` 的原点重合，姿态绕 X 轴正向旋转 90 度、
绕 Y 轴正向旋转 180 度、再绕 Z 轴正向旋转 45 度：`xyz="0 0 0"`、
`rpy="0.7853981633974483 1.5707963267948966 0"`。该姿态等价于此前姿态在
adapter 的**局部**正 Z（蓝轴）上按右手系正向旋转 `+90°`，因此此前的局部 X（红轴）
转至此前的局部 Y（绿轴）位置。其余
固定变换来自 STL 的包围尺寸；所有固定变换都通过 launch 参数暴露，RViz 核对后再逐项校准。

## adapter_1 校准记录

- 当前预览姿态：`adapter_1_rpy="0.7853981633974483 1.5707963267948966 0"`
  （依次为 X `+45°`、Y `+90°`、Z `0°`）。它是此前姿态右乘局部
  `Rz(+90°)` 的等价 RPY 表达，即蓝轴不变、红轴旋转到此前绿轴位置。这是 `fr3_link8` 到安装坐标系的
  暂定姿态，不是对 mesh 原点的补偿。
- 待确认位置：`adapter_1_xyz` 当前为 `0 0 0`，仅表示相对 `fr3_link8` 原点暂不偏移，
  不是最终机械安装位置。后续只调节此平移参数，不重新讨论已确认的姿态。

运行：

```bash
ROS_LOG_DIR=/tmp/fr3_rh56_preview_logs \
ros2 launch fr3_rh56_adapter_description fr3_rh56_visual_preview.launch.py
```

若要先只检查 `fr3_link8 -> adapter_1_link`，可运行：

```bash
ros2 launch fr3_rh56_adapter_description fr3_rh56_visual_preview.launch.py \
  show_adapter_2:=false show_rh56:=false
```

若只想检查两个转接件而暂不显示 RH56：

```bash
ros2 launch fr3_rh56_adapter_description fr3_rh56_visual_preview.launch.py \
  show_adapter_2:=true show_rh56:=false
```
