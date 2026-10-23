# AUBO SDK 只读检查

先验证 WSL 能使用本地 SDK，再连接控制柜获取一次机器人状态，为 MoveIt 实时状态接入做准备。默认仅执行本地加载检查；在线模式查询名称、六个关节角、TCP/法兰位姿、TCP 偏移、机器人型号、仿真开关、运行模式、安全模式与当前静止状态。程序不包含运动、上电、使能或模式切换调用。

项目目标、设备状态和待办统一见[项目说明](../../doc/assembly_contract.md#近期真机目标)。

当前设备的可直接运行命令、完整资料采集和字段解释见[真机状态查询与数据采集](../../doc/hardware/aubo_sdk_connection.md)。

## 构建与离线检查

在项目根目录运行：

```bash
cmake -S tools/aubo_sdk_probe -B tools/aubo_sdk_probe/build -DCMAKE_BUILD_TYPE=Release
cmake --build tools/aubo_sdk_probe/build --parallel 2
tools/aubo_sdk_probe/build/aubo_sdk_probe --offline
tools/aubo_sdk_probe/build/aubo_sdk_api_check
```

默认开发包路径为旁边的 `tools/sdk/aubo_sdk-0.23.1-Linux_x86_64+71831e1/`，新检出仓库时先按[开发包下载说明](../sdk/README.md)准备。修改位置时通过 CMake 的 `-DAUBO_SDK_ROOT=/实际目录` 指定，不需要复制库到系统目录。

## 到现场后只读连接

在控制柜界面确认实际网络 IP；将下面的中文占位符替换为该 IP：

```bash
tools/aubo_sdk_probe/build/aubo_sdk_probe --host 控制柜IPv4
```

在线模式采用随包 `share/example/c++/example_state.cpp` 的 `login("aubo", "123456")` 默认认证参数。这是控制器 RPC 通信认证，无需另行申请“SDK 账户”。若现场修改了用户名或密码，在交互终端追加 `--user 实际RPC用户名 --ask-password`，按提示隐藏输入密码；自定义密码不写入日志或文件。

RPC 端口 `30004` 来自该版本的 `include/aubo_sdk/rpc.h`，与照片里的界面连接端口不同。若有多个机器人，需要用 `--robot` 指定控制器返回的名称。当前设备 IP 等现场参数见项目说明。

关节角保持 SDK 原顺序，单位为 rad；TCP/法兰六维位姿保留原值，本次 RPY 编码与 URDF 模型的核对结果见上述项目说明。转换到 MoveIt 时还需核对实际工具点与目标坐标系。各项查询按顺序完成，适用于连接诊断；相机观测配对需要单独实现时间关联。当前静止状态仅作为诊断输出。

## 接入 MoveIt 前的接口依据

以本地 0.23.1 头文件为准：`RobotState` 提供关节和末端状态；`MotionControl` 提供 `servoJoint`、路径缓存与 `movePathBuffer`，本包没有声明较新文档中的 `trackJoint`。先核对现成驱动的适配，再决定如何发送完整轨迹；控制周期、插值、跟踪误差、取消和通信中断处理都需要现场验证。

编程接口核对使用 `include/aubo_sdk/rpc.h`、`include/aubo/robot/robot_state.h`、`include/aubo/robot/motion_control.h` 和随包 `share/example/c++/example_state.cpp`。原 SDK 文件保持原样。

## 官方驱动适配核对

[官方驱动的构建配置](https://github.com/AuboRobot/aubo_ros2_driver/blob/main/aubo_ros2_driver/CMakeLists.txt)目前默认下载 `0.24.1-rc.3`。接入本项目时应先调整为使用本地 `0.23.1` 并完成构建，不能仅因开发包已下载就认定驱动已经适配。

`aubo_sdk_api_check` 仅取 [硬件驱动](https://github.com/AuboRobot/aubo_ros2_driver/blob/main/aubo_ros2_driver/src/aubo_hardware_interface.cpp)所需的主要成员函数地址，验证本地头文件和共享库是否提供对应符号。检查涵盖 RPC、RTDE、机器人接口、伺服和状态解析；它不构造客户端，不调用列出的运动或配置方法。驱动源码 blob 为 `97711b1ebf7d42e7981b8f43b7f2bd6c6f637af8`，构建配置 blob 为 `07ed1d2554b8243c5efffc795d570b28fdac8085`。

官方驱动在硬件激活时配置控制器参数并进入伺服模式。首次只读通信使用前述 `aubo_sdk_probe`；完整驱动留到执行适配阶段验证。独立工具已采集控制周期、RTDE 菜单和状态，完整 ROS2 驱动构建及驱动中的周期、状态与真机执行仍待验证。
