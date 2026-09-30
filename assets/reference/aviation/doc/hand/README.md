# RH56 灵巧手串口控制与实时监控说明

本目录保存 RH56DFX 灵巧手的串口脚本和人工测试资料。项目级集成总结、
FR3 API 分层、MoveIt 仿真/真机解耦说明，以及当前资料缺口清单见：

```text
doc/hardware/fr3_rh56_api_integration.md
```

FR3 + RH56DFX 右手在 `franka_description` / RViz 中的模型验证记录见：

```text
doc/hand/fr3_rh56dfx_rviz_visualization.md
```

## 1. 功能概述

本目录包含两个主要脚本：

```text
rh56_angle_cmd.py   # 控制灵巧手六个自由度的目标角度
rh56_watch.py       # 实时读取灵巧手角度、受力、温度、错误码等状态
```

当前配置：

```text
左手 ID = 1
右手 ID = 2
通信口 = /dev/ttyUSB_robot_485
波特率 = 115200
协议 = RH56 RS485 协议
```

两只手接在同一条 485 总线上时，必须保证 ID 不同。当前左手为 1，右手为 2，可以同时挂在同一条总线上使用。

---

## 2. 手指顺序说明

两个脚本中，六个数值的顺序固定为：

```text
小拇指 无名指 中指 食指 大拇指弯曲 大拇指旋转
```

英文输出中对应为：

```text
little ring middle index thumb_bend thumb_rotate
```

也就是：

| 序号 | 名称           | 含义    |
| -- | ------------ | ----- |
| 0  | little       | 小拇指   |
| 1  | ring         | 无名指   |
| 2  | middle       | 中指    |
| 3  | index        | 食指    |
| 4  | thumb_bend   | 大拇指弯曲 |
| 5  | thumb_rotate | 大拇指旋转 |

---

# 3. 角度控制脚本：rh56_angle_cmd.py

## 3.1 脚本作用

`rh56_angle_cmd.py` 用来给指定 ID 的 RH56 灵巧手发送角度控制命令。

它可以：

1. 控制左手或右手。
2. 指定 6 个自由度的目标角度。
3. 支持某些手指不动。
4. 发送命令前读取当前角度、受力、温度、错误码。
5. 发送命令后继续读取一段时间状态。

---

## 3.2 角度数值含义

角度命令范围：

```text
0 ~ 1000
```

一般理解为：

```text
1000 = 张开
0    = 最大弯曲
```

特殊值：

```text
-1 = 该自由度不动作，保持当前状态
```

例如：

```text
-1 -1 -1 700 -1 -1
```

表示：

```text
小拇指不动
无名指不动
中指不动
食指运动到 700
大拇指弯曲不动
大拇指旋转不动
```

---

## 3.3 基本用法

### 控制左手

左手 ID 为 1：

```bash
python3 rh56_angle_cmd.py 1000 1000 1000 1000 1000 1000 --id 1
```

### 控制右手

右手 ID 为 2：

```bash
python3 rh56_angle_cmd.py 1000 1000 1000 1000 1000 1000 --id 2
```

脚本默认会要求确认：

```text
Send this command? Type y to continue:
```

输入：

```text
y
```

才会真正发送命令。

---

## 3.4 跳过确认

如果已经确认安全，可以加 `--yes` 跳过确认：

```bash
python3 rh56_angle_cmd.py 1000 1000 1000 1000 1000 1000 --id 2 --yes
```

---

## 3.5 控制单个手指

### 左手只动食指

```bash
python3 rh56_angle_cmd.py -1 -1 -1 700 -1 -1 --id 1
```

### 右手只动食指

```bash
python3 rh56_angle_cmd.py -1 -1 -1 700 -1 -1 --id 2
```

### 右手食指继续弯曲

```bash
python3 rh56_angle_cmd.py -1 -1 -1 500 -1 -1 --id 2
```

### 右手食指恢复张开

```bash
python3 rh56_angle_cmd.py -1 -1 -1 1000 -1 -1 --id 2
```

---

## 3.6 常用姿态示例

### 全部张开

左手：

```bash
python3 rh56_angle_cmd.py 1000 1000 1000 1000 1000 1000 --id 1
```

右手：

```bash
python3 rh56_angle_cmd.py 1000 1000 1000 1000 1000 1000 --id 2
```

### 半握拳

```bash
python3 rh56_angle_cmd.py 600 600 600 600 700 900 --id 2
```

### 更明显握拳

```bash
python3 rh56_angle_cmd.py 400 400 400 400 500 800 --id 2
```

建议第一次测试不要直接发送：

```bash
python3 rh56_angle_cmd.py 0 0 0 0 0 0 --id 2
```

应先从 800、700、600、500 这类较安全的值逐步测试。

---

## 3.7 指定串口

默认串口为：

```text
/dev/ttyUSB_robot_485
```

如果需要手动指定：

```bash
python3 rh56_angle_cmd.py 1000 1000 1000 1000 1000 1000 --id 2 --port /dev/ttyUSB_robot_485
```

---

## 3.8 控制脚本输出说明

运行后会显示类似：

```text
Target hand id: 2
Target command:
  little       : 1000
  ring         : 1000
  middle       : 1000
  index        : 700
  thumb_bend   : 1000
  thumb_rotate : 1000
```

发送成功后会显示：

```text
write ack: 90 eb 02 ...
```

其中：

```text
90 eb = RH56 回复帧头
02    = 当前控制的是 ID 2，也就是右手
```

之后会显示命令前后的状态：

```text
finger           angle    force    temp
little            1000     0.00      32
ring              1000     0.00      32
middle            1000     0.00      32
index              700     0.25      32
thumb_bend        1000     0.00      29
thumb_rotate       984     0.00      30
errors: [0, 0, 0, 0, 0, 0]
```

---

# 4. 实时监控脚本：rh56_watch.py

## 4.1 脚本作用

`rh56_watch.py` 用于实时查看灵巧手状态。

它可以实时显示：

1. 当前角度 `angle`
2. 当前受力 `force(N)`
3. 相对受力变化 `dF(N)`
4. 温度 `temp`
5. 错误码 `err`

它适合用于：

1. 检查灵巧手是否在线。
2. 检查某个手指受力是否变化。
3. 用手按压某个手指，观察力传感器是否有响应。
4. 检查手指动作后角度是否变化。
5. 检查是否有错误码。

---

## 4.2 force(N) 是什么

`force(N)` 是当前读取到的受力值，脚本中已经换算成牛顿 N。

换算关系：

```text
1 gf ≈ 0.00980665 N
```

脚本内部使用：

```python
force_N = force_gf * 0.00980665
```

注意：这个值适合观察趋势，不建议当高精度力计使用。

---

## 4.3 dF(N) 是什么

`dF` 是 Delta Force，也就是“受力变化量”。

脚本启动时，会先记录一次当前 force，作为基线：

```text
baseline force
```

之后每次读取时计算：

```text
dF = 当前 force - baseline force
```

例如：

```text
启动脚本时食指 force = 0.20 N
按压食指后 force = 1.80 N
那么 dF = 1.60 N
```

所以：

```text
force(N) = 当前绝对读数
dF(N)    = 相对脚本启动时的变化量
```

实际测试“我按了一下手指，传感器有没有变化”时，重点看 `dF(N)`。

---

## 4.4 查看右手状态

右手 ID 为 2：

```bash
python3 rh56_watch.py --id 2 --rate 10
```

表示以 10Hz 左右频率读取右手状态。

---

## 4.5 查看左手状态

左手 ID 为 1：

```bash
python3 rh56_watch.py --id 1 --rate 10
```

---

## 4.6 同时查看左右手

```bash
python3 rh56_watch.py --both --rate 10
```

等价于同时读取：

```text
ID 1 左手
ID 2 右手
```

---

## 4.7 仪表盘刷新模式

普通模式会一直往终端下面滚动。

如果想固定在当前屏幕刷新，可以加：

```bash
python3 rh56_watch.py --both --rate 10 --clear
```

单独看右手仪表盘：

```bash
python3 rh56_watch.py --id 2 --rate 10 --clear
```

---

## 4.8 监控脚本输出说明

输出示例：

```text
14:22:31.125 | id=2 | force [  0.00   0.00   0.00   1.47   0.00   0.00] | dF [  0.00   0.00   0.00   1.18   0.00   0.00] | angle [ 1000  1000  1000   720  1000   980] | temp [32, 32, 32, 32, 29, 30] | err [0, 0, 0, 0, 0, 0]
```

字段含义：

| 字段    | 含义                 |
| ----- | ------------------ |
| time  | 当前时间               |
| id    | 灵巧手 ID             |
| force | 当前受力，单位 N          |
| dF    | 相对于脚本启动时的受力变化，单位 N |
| angle | 当前六个自由度角度          |
| temp  | 六个自由度对应驱动单元温度      |
| err   | 错误码                |

---

## 4.9 force、dF、angle 的顺序

所有数组顺序都一样：

```text
小拇指 无名指 中指 食指 大拇指弯曲 大拇指旋转
```

例如：

```text
force [0.00 0.00 0.00 1.47 0.00 0.00]
```

表示：

```text
小拇指受力       0.00 N
无名指受力       0.00 N
中指受力         0.00 N
食指受力         1.47 N
大拇指弯曲受力   0.00 N
大拇指旋转受力   0.00 N
```

---

# 5. 常见测试流程

## 5.1 测试右手食指是否可控

第一步，打开监控：

```bash
python3 rh56_watch.py --id 2 --rate 10
```

第二步，另一个终端发送食指角度命令：

```bash
python3 rh56_angle_cmd.py -1 -1 -1 700 -1 -1 --id 2
```

如果同一条串口不能被两个脚本同时打开，则先关闭监控，再发送控制命令。

第三步，看监控输出中的：

```text
angle
force
dF
```

食指对应第 4 个数值。

---

## 5.2 测试右手食指力传感器

运行：

```bash
python3 rh56_watch.py --id 2 --rate 10
```

然后用手轻轻按压右手食指指尖。

观察输出：

```text
dF
```

如果食指对应的第 4 个数值明显变化，说明力传感器有响应。

---

## 5.3 测试左右手是否 ID 正确

左手：

```bash
python3 rh56_watch.py --id 1 --rate 5
```

轻按左手某个手指，看 `dF` 是否变化。

右手：

```bash
python3 rh56_watch.py --id 2 --rate 5
```

轻按右手某个手指，看 `dF` 是否变化。

如果控制 ID 1 时右手动，或者控制 ID 2 时左手动，说明左右手 ID 或接线映射反了。

---

# 6. 注意事项

## 6.1 不要同时抢占同一个串口

如果两个脚本、ROS 节点、或者其他程序同时打开：

```text
/dev/ttyUSB_robot_485
```

可能出现：

```text
short response
bad header
read failed
device busy
```

因此同一时间建议只运行一个串口读写程序。

如果要确认串口是否被占用：

```bash
lsof /dev/ttyUSB_robot_485
```

如果有输出，说明串口正在被占用。

---

## 6.2 动作测试要从小幅度开始

建议先测试：

```bash
python3 rh56_angle_cmd.py -1 -1 -1 800 -1 -1 --id 2
python3 rh56_angle_cmd.py -1 -1 -1 700 -1 -1 --id 2
python3 rh56_angle_cmd.py -1 -1 -1 600 -1 -1 --id 2
```

不要一开始直接发送：

```bash
python3 rh56_angle_cmd.py 0 0 0 0 0 0 --id 2
```

避免夹到线缆、工装、桌面或机械臂本体。

---

## 6.3 错误码检查

如果输出：

```text
err [0, 0, 0, 0, 0, 0]
```

表示当前没有错误。

如果某一位不是 0，表示对应自由度存在故障或异常，需要停止动作并排查。

---

## 6.4 温度检查

`temp` 为六个自由度对应驱动单元温度。

如果温度持续升高，或者明显异常，应停止测试，检查是否存在堵转、过流、长时间夹紧等情况。

---

# 7. 快速命令汇总

## 左手全张开

```bash
python3 rh56_angle_cmd.py 1000 1000 1000 1000 1000 1000 --id 1
```

## 右手全张开

```bash
python3 rh56_angle_cmd.py 1000 1000 1000 1000 1000 1000 --id 2
```

## 左手食指弯曲到 700

```bash
python3 rh56_angle_cmd.py -1 -1 -1 700 -1 -1 --id 1
```

## 右手食指弯曲到 700

```bash
python3 rh56_angle_cmd.py -1 -1 -1 700 -1 -1 --id 2
```

## 监控左手

```bash
python3 rh56_watch.py --id 1 --rate 10
```

## 监控右手

```bash
python3 rh56_watch.py --id 2 --rate 10
```

## 同时监控左右手

```bash
python3 rh56_watch.py --both --rate 10
```

## 仪表盘模式监控左右手

```bash
python3 rh56_watch.py --both --rate 10 --clear
```

---

# 8. 当前推荐调试顺序

1. 先确认通信正常：

```bash
python3 rh56_watch.py --id 1 --rate 5
python3 rh56_watch.py --id 2 --rate 5
```

2. 再测试单指小幅动作：

```bash
python3 rh56_angle_cmd.py -1 -1 -1 800 -1 -1 --id 1
python3 rh56_angle_cmd.py -1 -1 -1 800 -1 -1 --id 2
```

3. 再逐步加大弯曲：

```bash
python3 rh56_angle_cmd.py -1 -1 -1 700 -1 -1 --id 2
python3 rh56_angle_cmd.py -1 -1 -1 600 -1 -1 --id 2
```

4. 最后再测试多指动作：

```bash
python3 rh56_angle_cmd.py 600 600 600 600 700 900 --id 2
```
