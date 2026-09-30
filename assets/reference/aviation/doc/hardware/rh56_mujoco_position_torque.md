# RH56 在 MuJoCo 中的关节目标与驱动力矩

更新时间：2026-09-08。本文说明当前项目的 RH56 仿真控制：六通道目标如何转换
为关节角度，位置执行器如何计算力矩，以及它对夹持母头的影响。参数属于当前
仿真基线，不代表 RH56 真机内部控制算法、额定力矩或已标定的通道映射。

## 1. 配置与代码入口

- [共享 YAML](../../src/assembly_bringup/config/mvp3_connector_assembly.yaml)：
  `fixed_grasp_candidate.hand_grasp.channel_values` 是任务抓取目标；
  `mujoco_contract.hand_control` 定义通道映射、执行器增益和输出限制。
- [通道映射代码](../../src/assembly_mujoco/assembly_mujoco/rh56_hand_control.py)：
  `RH56HandTargetMapper.target_for_command` 将通道值转换为驱动关节目标。
- [抓取执行代码](../../src/assembly_mujoco/assembly_mujoco/simulation.py)：
  `execute_grasp_and_lift` 读取任务的 `hand_grasp.channel_values`，不是通用
  `command_channel_values.close` 预设。
- [生成后的 MJCF](../../src/assembly_mujoco/mjcf/mvp3_connector_assembly.xml)：
  `<actuator>` 中的六个 `rh56_right_*_position` 是实际位置执行器。
- [场景生成器](../../src/assembly_mujoco/tools/generate_fr3_rh56_scene.py)：
  将配置写入场景；更新参数时需核对生成后模型，避免只改生成文件后被覆盖。

## 2. 从通道值到目标角度

通道顺序为：小指、无名指、中指、食指、拇指弯曲、拇指旋转。通道值
`1000` 对应配置的张开端点，`0` 对应闭合端点。当前抓取目标为
`[400, 400, 400, 400, 400, 0]`；本轮抓握讨论固定最后一个通道为 `0`。

每个通道按以下公式映射，角度单位为 rad：

```text
closing_fraction = 1 - channel_value / 1000
q_target = q_open + closing_fraction × (q_close - q_open)
```

四指驱动关节当前均为 `q_open = 0.13430029546073955`、`q_close = 1.30`。
因此食指通道 `400` 对应 `q_target ≈ 0.83372 rad`，约 `47.77°`。
拇指弯曲端点为 `0～0.40 rad`，不能套用四指的角度映射。
其余从动关节由模型中的 equality 传动约束求解。

通道值描述目标位置，不描述握力百分比；`400` 表示配置闭合行程的 60%。

## 3. 从目标角度到执行器力矩

当前食指执行器配置如下，其他五个通道采用相同的 `kp` 与输出限制：

```xml
<position name="rh56_right_index_position"
          joint="fr3_rh56dfx_right_index_1_joint"
          kp="5"
          ctrllimited="true" ctrlrange="0.13430029546073954 1.3"
          forcelimited="true" forcerange="-1 1" />
```

对于这里直接连接转动关节、传动比为默认值 1 的位置执行器，力矩关系为：

```text
tau_raw = kp × (q_target - q_actual) - kv × q_velocity
tau_actuator = clip(tau_raw, -1, +1)
```

`clip` 表示截断到指定范围。当前 RH56 没有设置执行器 `kv`，默认值为 0，
所以可简化为：

```text
tau_actuator = clip(5 × (q_target - q_actual), -1, +1)  [N·m]
```

| 参数 | 当前含义 |
| --- | --- |
| `kp = 5` | 位置反馈增益，当前单位为 N·m/rad；角度误差越大，驱动力矩越大 |
| `ctrlrange` | 允许输入的目标角度范围，不是实际关节运动的硬性锁定 |
| `forcerange = [-1, 1]` | 当前直接转动关节执行器的输出力矩限制，单位 N·m |
| 关节 `damping = 0.1` | 另行计算的被动阻尼，产生反向于速度的力矩；不包含在上述执行器输出内 |

位置执行器的增益、速度项和传动定义见
[MuJoCo 位置执行器参考](https://mujoco.readthedocs.io/en/3.4.0/XMLreference.html#actuator-position)。
上述简化公式不能不加修改地用于不同 gear、tendon 传动或其他类型执行器。

## 4. 数值示例

取食指目标的近似值 `q_target = 0.834 rad`：

| 假设实际角度 | 角度误差 | 未限幅力矩 | 最终执行器力矩 |
| --- | --- | --- | --- |
| 0.134 rad | 0.700 rad | 3.50 N·m | 1.00 N·m，达到上限 |
| 0.700 rad | 0.134 rad | 0.67 N·m | 0.67 N·m |
| 0.820 rad | 0.014 rad | 0.07 N·m | 0.07 N·m |
| 0.834 rad | 0 | 0 | 0 |

这些是假设状态下的计算示例，不是抓取运行测量。恰好到达目标时，此位置
执行器的输出为零，不表示重力、接触力或其他关节作用也为零。

## 5. 从力矩到关节运动

每个仿真步都会根据当时的位置和速度重新计算执行器输出。随后 MuJoCo 将
驱动力矩、重力、惯性效应、被动阻尼、接触反力和关节传动约束一起求解，
得到加速度，再由积分器更新速度与位置。模型当前时间步为 `0.002 s`。
因此固定目标角度不意味着固定输出力矩，也不意味着实际角度会立即等于目标。
相关机制见 [MuJoCo 动力学计算](https://mujoco.readthedocs.io/en/3.8.0/computation/)。

手指碰到母头后，可能被阻挡而停在目标之前。只要位置误差仍然存在，执行器
就会持续输出闭合力矩；接触反力等作用与之平衡时，可以形成静止夹持。

## 6. 对抓握和打滑的影响

- 如果手指刚接触母头就已经很接近目标，剩余位置误差小，持续闭合力矩可能不足。
- 适当减小通道值会使目标更闭合；在实际角度暂时不变时，误差与驱动力矩增大。
- 当前误差达到 `1 / 5 = 0.2 rad`（约 `11.46°`）时，执行器达到力矩上限。
  超过该误差继续减小目标通道值，不会继续增加执行器输出上限。
- `1 N·m` 是驱动关节的力矩，不是指腹的 `1 N` 接触压力。接触压力还取决于
  力臂、手指姿态、从动关节约束和接触分布，不能直接把两者等同。
- 摩擦抗滑能力依赖接触压力；位置误差小或接触位置不合适时，仅提高摩擦系数
  不能保证抓稳。是否属于上述原因，需要用实际角度、执行器输出和接触力验证。

当前 `kp=5` 与 `±1 N·m` 是仿真设置，不能据此推断真机能够输出同样的夹持力。
