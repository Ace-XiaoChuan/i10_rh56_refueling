# RH56 的 SpaHybGen 适配

适配器用六个独立驱动表示 RH56 十二关节，提供可微正运动学和指面接触采样，在 SpaHybGen 的空间接触预测上优化手根位姿与手型。

当前使用原示例场景，油箱盖空间输入及碰撞筛选按[项目说明](assembly_contract.md#spahybgen-与-rh56)继续接入。

## 查看结果

结果目录为 `artifacts/rh56_spahybgen/original_20261008/`：

| 文件 | 内容 |
| --- | --- |
| `candidates.html`、`preview.png` | 候选交互图和三视图 |
| `contact_regions.html` | 指面与法向 |
| `candidates.json` | 位姿和关节数组 |
| `optimization.npz` | 迭代与末态损失 |
| `model_audit.json` | FK、采样和梯度对照 |
| `result.json` | 运行设置和来源 |

网页中灰色为手，彩色点为场景体素，红点为指面采样，青线为外法向。红色三角形表示选中的实际网格面。HTML 内嵌 Plotly，可离线查看。

## 模块与坐标

| 文件 | 作用 |
| --- | --- |
| `tools/build_rh56_spahybgen_model.py` | 提取手部子树、网格、驱动关系和指面 |
| `src/fuel_cap_task/rh56_drive_map.py` | 六驱动到完整关节的仿射映射与共同限位 |
| `src/fuel_cap_task/rh56_spahybgen.py` | 可微 FK、接触对齐、硬投影和末态重算 |
| `tools/generate_rh56_grasps.py` | 优化、保存与结果检查 |

优化变量为 `[场景 xyz（m）, R6d, 六驱动角（rad）]`。`scene_from_hand` 是手根 `base` 在输入体素场景中的位姿。`driver_names` 和 `full_joint_names` 给出数组顺序。

`driver_angles_rad` 和 `full_joint_angles_rad` 是模型角；硬件 0～1000 通道需另外标定。

## 核心计算

```python
# 六驱动通过仿射耦合展开为十二关节，梯度沿这次变换传播。
return drivers @ self.drive_matrix.T + self.drive_offset

# 按运动学库的关节顺序计算 FK，得到指面位置与法向。
full = self.expand_drivers(q[:, 9:])
self.current_status = self.robot.forward_kinematics(full[:, self.chain_rows])
```

驱动区间取主动关节、执行器和从动关节的共同允许范围。扰动与 Adam 更新后做硬投影，末态重算损失再排序。候选还需经过自碰、环境和任务能力检查。

## 生成与检查

在项目根目录运行，生成结果使用新目录：

```bash
.venv/bin/python tools/build_rh56_spahybgen_model.py \
  --output-dir artifacts/rh56_spahybgen/model_new

.venv-spahybgen/bin/python tools/generate_rh56_grasps.py \
  --model-dir artifacts/rh56_spahybgen/model_new \
  --output-dir artifacts/rh56_spahybgen/candidates_new

.venv-spahybgen/bin/python tools/generate_rh56_grasps.py \
  --verify-only --output-dir artifacts/rh56_spahybgen/original_20261008
```

模型派生使用共享仿真环境，优化使用 SpaHybGen 独立环境；默认读取已保存的原示例预测。

```bash
MPLCONFIGDIR=/tmp/rh56-matplotlib PYTHONPATH=src:dependencies/SpaHybGen/src \
  .venv-spahybgen/bin/python -m unittest discover -s tests/spahybgen -v
```
