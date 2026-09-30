# FR3—adapter1—adapter2 接口坐标系待确认项

## 当前结论

- **adapter1 的局部原点位置已正确。** 当前使用的网格由 SolidWorks“坐标系1”导出；该原点位于
  FR3 法兰与 adapter1 的配合面，网格单位为米。
- **adapter1 的姿态尚未得到机械确认。** 目前 URDF 中 `fr3_link8 -> adapter_1_link` 的旋转是
  根据 RViz 视觉观察猜测得到，只用于预览。它必须参考实机安装方向校准，使仿真模型与真机对准。
- adapter2 的当前网格原点位于零件内部的中心附近，不是其与 adapter1 的配合面，不能直接作为
  `adapter_1_link -> adapter_2_link` 固定关节的子坐标系。
- 当前预览中 adapter1 到 adapter2 的平移/旋转是网格尺寸估计，不能用于 MoveIt 碰撞、MuJoCo
  接触或真机动作。

## 需要机械侧提供的定义

请在 CAD 中为以下两个配合面建立并导出命名坐标系：

| 坐标系 | 原点 | 必须定义的方向 |
| --- | --- | --- |
| `adapter1_out_mate_frame` | adapter1 与 adapter2 配合面的中心（或定位轴/孔阵中心） | 配合面法向，以及由定位销、螺孔或平边确定的 X 轴钟向 |
| `adapter2_in_mate_frame` | adapter2 与 adapter1 配合面的中心（或定位轴/孔阵中心） | 与 `adapter1_out_mate_frame` 一致的法向/钟向约定 |

同时请明确装配后两坐标系的相对关系：原点是否重合、法向是否同向，及是否存在垫片厚度或固定的
绕法向转角。若双方按各自零件“外法线”为正 Z，贴合后 Z 轴会相反，URDF 固定变换中需要一个
`180°` 翻转；这必须由机械定义，而不能由视觉猜测。

## 软件侧交付格式

推荐提供以下任一项：

1. 含上述命名坐标系的 CAD/STEP；或
2. `T_adapter1_out_adapter2_in` 的 4×4 齐次变换（平移单位 m，旋转注明 RPY/四元数及坐标约定）；或
3. 在正确坐标系下导出的 STL + 对应坐标系说明。

收到后，软件侧只更新统一安装合同
`src/fr3_rh56_adapter_description/config/fr3_rh56_mount_provisional.yaml`，再同步生成 MoveIt 和
MuJoCo 所用模型；不在 RViz、URDF、MJCF 中分别手写不同变换。

