# AUBO 真机状态查询与数据采集

本机通过 AUBO SDK 查询控制柜，保存配置与状态，为 MoveIt 模型核对、实时关节反馈和轨迹执行适配提供资料。工具仅查询状态和配置、订阅 RTDE 输出；不发送运动、上电、使能、模式切换或配置写入指令。

设备地址、SDK 版本、已完成工作和待办统一见[项目说明](../assembly_contract.md#近期真机目标)。以下命令使用其中当前控制柜 IP，在 WSL 终端运行。

## 本次保存的资料

- [第一批资料说明](../../artifacts/hardware/aubo_sdk_readonly_20261023_001/README.md)：配置快照与第一段 RTDE 记录。
- [第二批资料说明](../../artifacts/hardware/aubo_sdk_readonly_20261023_002/README.md)：配置、第二段 RTDE、正运动学与坐标变换探针、来源快照。
- [第二批状态 CSV](../../artifacts/hardware/aubo_sdk_readonly_20261023_002/rtde_samples.csv)和[数据质量与模型比对结果](../../artifacts/hardware/aubo_sdk_readonly_20261023_002/analysis.json)。
- [两批完整压缩包](../../artifacts/hardware/aubo_sdk_readonly_20261023_bundle_001.tar.gz)：包含原始记录、CSV、统计、说明和来源快照，SHA-256 为 `bae922b6f31b97a460a226e2e953b97b922d710e49163bacad090a584d9dad96`。

两批采集已结束，查看、整理和复算这些文件不需要保持机械臂连接。再次运行在线查询或采集前，重新接好网线并确认控制柜地址。

## 查询一次当前状态

```bash
cd /home/ace/i10_rh56_refueling_ws
tools/aubo_sdk_probe/build/aubo_sdk_probe --host 192.168.1.32
```

输出机器人型号、六个关节角、TCP/法兰位姿、TCP 偏移、运行与安全模式、当前静止状态及仿真开关。关节角单位为 rad；示教器若显示度，按 `度 = rad × 180 / π` 换算。数组顺序仍需与规划模型逐项核对。

程序默认使用该版本官方示例的 RPC 认证参数，无需另行注册账户。若现场改过认证参数，可追加 `--user 实际RPC用户名 --ask-password`，隐藏输入密码。协议与源码入口见[工具说明](../../tools/aubo_sdk_probe/README.md)。

## 多采集并保存

```bash
cd /home/ace/i10_rh56_refueling_ws
tools/aubo_sdk_probe/build/aubo_sdk_collect \
  --host 192.168.1.32 \
  --output-dir "artifacts/hardware/aubo_sdk_$(date -u +%Y%m%dT%H%M%SZ)" \
  --duration 15 --hz 50
```

目录名使用 UTC 时间，已有目录会被拒绝覆盖。程序先保存配置，再请求 15 秒、50 Hz 的 RTDE 输出；实际接收频率和有效字段以保存结果为准。采集工具目前使用本次已验证的默认 RPC/RTDE 认证参数。

配置包含控制器与接口版本、机器人型号、控制周期、DH 理论与实际参数、20℃补偿查询、关节和 TCP 限位、当前 TCP、安装姿态、重力、负载、力传感器信息、硬件版本和一次状态快照。单项失败会记录方法名、错误和时长，其余资料继续保存。

另存的正运动学和坐标变换探针只计算假设输入的结果，没有要求机械臂移动到那些构型；它们可用于离线核对模型，不能用作多姿态手眼标定样本。

## 查看保存结果

| 文件 | 内容 |
| --- | --- |
| `metadata.json` | 地址、版本、主机时间、采集参数、RPC/RTDE 结果 |
| `configuration.json` | 每项 RPC 查询的值、成功标志和起止时间 |
| `queries.jsonl` | 逐项即时保存的查询记录 |
| `rtde_output_menu.json` | 控制柜报告的 RTDE 输出菜单 |
| `rtde_registered_recipes.json` | 本连接注册的 RTDE 字段与频率 |
| `rtde_samples.jsonl` | 原始状态序列，每行一帧，包含主机收包时间 |
| `analysis.json` | 派生的数据有效性、接收频率与数值范围统计 |
| `rtde_samples.csv` | 状态序列的展开表格，缺失值保留为空 |

已有采集目录可生成统计和 CSV：

```bash
python3 tools/aubo_sdk_probe/summarize_capture.py 采集目录
```

统计程序保留原始 JSON/JSONL，只更新派生的统计与 CSV。原始记录是历史快照，规划前应重新读取当前状态。

统计脚本使用本机已有的 NumPy；其中模型比对仅比较控制器模型计算和本地 URDF，不验证实物定位误差。

## 使用这些资料时需要核对

主机时间与控制器时间尚未同步。若控制器时间戳经 SDK 解析为非有限值，会以 `null` 和无效字段标记保存；不能用它对齐相机曝光时间。原始关节力矩传感器等接口可能返回空数组，接口调用成功不等于有有效测量。

负载、重力和 TCP 是控制器当前配置，不等于实际装配的独立测量。TCP 力读数不能直接当成已完成零偏与重力补偿的接触力。DH 实际参数与理论参数相同、补偿查询为空时，仍需确认厂家校准资料。

原始六维位姿保留原格式。本次 `Math.poseTrans` 探针与 13 组正运动学结果支持后三项使用固定轴 XYZ 的 RPY，矩阵为 `Rz(yaw) Ry(pitch) Rx(roll)`，见第二批 `analysis.json`。随包 RTDE 表的“旋转向量”描述不用于本次数据；接 MoveIt 时按已核对的 RPY 转四元数。

## 本地重新构建

```bash
cd /home/ace/i10_rh56_refueling_ws
cmake -S tools/aubo_sdk_probe -B tools/aubo_sdk_probe/build -DCMAKE_BUILD_TYPE=Release
cmake --build tools/aubo_sdk_probe/build --parallel 2
tools/aubo_sdk_probe/build/aubo_sdk_probe --offline
tools/aubo_sdk_probe/build/aubo_sdk_api_check
```

厂家开发包不需要复制到系统库目录。工程使用旁边的 `tools/sdk/` 开发包；本次工具源码保存在 `tools/aubo_sdk_probe/`。
