# AUBO 只读采集资料

采集时间为 2026-10-23T09:50:37.800972Z 至 2026-10-23T09:50:52.888301Z（UTC）。

控制柜 `192.168.1.32`，机器人 `rob1`，SDK `0.23.1`。

RPC 查询 71 项，成功 71 项。RTDE 保存 785 帧，解析异常 0 帧。

订阅请求为 50.0 Hz、15.0 秒；按主机收包时间计算的平均频率为 52.444 Hz。这是主机收包统计，不是控制周期或无丢包证明。

## 主要配置

| 项目 | 读取值 |
| --- | --- |
| 型号 | `aubo_i10_JY` |
| 控制器版本码 / 接口版本码 | `29004` / `23001` |
| 控制器版本 hash | `02301bb` |
| 控制周期 | `0.005` s |
| 当前 TCP 偏移 | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` |
| 当前负载配置 | `{"aom": [0.0, 0.0, 0.0], "cog": [0.0, -0.06, 0.24], "inertia": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "mass": 8.0}` |
| 当前重力配置 | `[-0.10527343775827445, 9.799205557066411, -0.06699219104753168]` |
| TCP 力传感器 | `['kw_ftsensor']` |

## 结果用途与限制

- 控制器时间戳有效帧为 0；无效或缺失帧为 785，不能用于相机时钟对齐。主机收包时间保留。
- 空数组和空字典按原值保存；它们不表示已获得有效标定或传感数据。
- 负载与重力是控制器配置，TCP 力值不是经独立验证的接触力；没有执行传感器清零或载荷标定。
- 本批为短时近静止状态数据，不是手眼标定或运动轨迹跟踪试验。
- `queries.jsonl` 和 `rtde_samples.jsonl` 为原始记录；CSV 与 analysis 为离线派生结果。
- `metadata.json` 的 parse_errors 只表示解析异常；字段有效性以 analysis.json 为准。

返回空值的 RPC 项：

- `RobotConfig.getBaseForceOffset`
- `RobotConfig.getBaseForceSensorNames`
- `RobotConfig.getKinematicsCompensate(20)`
- `RobotConfig.getRobotSubType`
- `RobotState.getJointTorqueSensors`

本批未采集数学探针；完整模型核对与来源快照见第二批资料。

复用命令与项目状态见 `doc/hardware/aubo_sdk_connection.md` 和 `doc/assembly_contract.md`。
