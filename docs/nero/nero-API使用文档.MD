# API使用文档

## 导入模块

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory
```

## 创建实例并连接

### 创建配置参数 create_agx_arm_config()

- 定义

```python
create_agx_arm_config(
        robot:Literal["nero","piper","piper_h","piper_l","piper_x"],
        comm:Literal["can"] = "can",
        firmeware_version: str = "default",
        **kwargs) -> dict
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|robot|str|["nero","piper","piper_h","piper_l","piper_x"]参数可选，选择预期创建的机械臂实例|
|comm|str|通讯类型，可选：`"can"`（默认：`"can"`）。注意：`comm` 不是 can 通道名；can 通道由下方 `channel` 指定|
|firmeware_version|str|默认为"default"，选择机械臂当前底层对应的主控固件版本|
|---可选参数---|||
|joint_limits|dict|默认可以不输入，程序内自动根据机械臂赋值指定的关节限位，如果要指定，需要手动输入dict类型的关节限位，单位：弧度，示例下文可见，暂时不会对手动输入的关节限位生效到实际控制中|
|当通讯方式为can时|||
|channel|str|默认为"can0"，选择预期连接的can channel|
|interface|str|默认为"socketcan"，选择连接can模块的接口种类，官方can模块连接到linux系统中必须为"socketcan"，如果想要连接macos，需要更换串口can模块然后将该参数设置为"slcan"|
|bitrate|int|默认为1000000，预期can端口的波特率，机械臂官方can模块默认为1Mbps|
|enable_check_can|bool|默认为True，功能为是否打开创建内部Comm实例时检查can模块，默认打开|
|auto_connect|bool|默认为True，功能为是否打开创建内部Comm实例时自动创建can bus实例，默认打开|
|timeout|float|默认为1.0，设置can bus读取和发送的超时时间|

- 返回值:

类型: dict

结构如下：

```json
{
    "robot": "nero",
    "comm":{
        "type":"can",
        "can":{
            "channel": "can0",
            "interface": "socketcan",
            "bitrate": 1000000,
            "enable_check_can": true,
            "auto_connect": true,
            "timeout": 1.0
        }
    },
    "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"], 
    "joint_limits":{
        "joint1":[-2.705261, 2.705261],
        "joint2":[-1.745330, 1.745330],
        "joint3":[-2.757621, 2.757621],
        "joint4":[-1.012291, 2.146755],
        "joint5":[-2.757621, 2.757621],
        "joint6":[-0.733039, 0.959932],
        "joint7":[-1.570797, 1.570797]
    },
}
```

关节运动限位单位为弧度

夹爪运动限位单位为米

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
print(cfg)
```

### 创建机械臂Driver实例 AgxArmFactory.create_arm()

- 功能说明:

根据传入的配置文件，使用工厂创建不同的机械臂实例

- 定义:

```python
create_arm(cls, config: dict, **kwargs) -> T
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|config|dict|读取配置参数字典|

- 返回值:

类型:Driver

不同臂、通讯类型、固件版本有不同的实例创建

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
```

### 创建连接 connect()

- 功能说明:

该指令用来创建连接，启动读取线程

- 定义:

```python
connect(self, start_read_thread: bool = True) -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|start_read_thread|bool|是否打开读取数据线程，默认为打开|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
```

### 初始化末端执行器Driver实例 init_effector()

- 功能说明:

初始化末端执行器 Driver，并返回对应的执行器实例（例如夹爪/灵巧手等）。

该接口为“一次性初始化”：同一个 `robot` 实例 **只能初始化一次** 执行器；如需切换到其它执行器类型，请创建新的机械臂实例。

- 定义:

```python
init_effector(self, effector: str) -> EffectorDriver
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|effector|str|执行器类型（建议使用 `robot.EFFECTOR.xxx` 常量）|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 初始化执行器（只能调用一次）
end_effector = robot.init_effector(robot.EFFECTOR.REVO2)
```

### 获取关节数量 joint_nums

- 功能说明:

获取机械臂关节数量（例如 Nero 为 7）。

- 定义:

```python
joint_nums: int
```

- 返回值:

类型：`int`

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

print("robotic arm joint_nums =", robot.joint_nums)

# 示例：依次使能所有关节（不触发运动）
for joint_index in range(1, robot.joint_nums + 1):
    start_t = time.monotonic()
    while True:
        if robot.enable(joint_index):
            print(f"enable joint {joint_index} success")
            break
        if time.monotonic() - start_t > 5.0:
            print(f"enable joint {joint_index} timeout (5s)")
            break
        time.sleep(0.01)
```

## 机械臂数据读取相关

### MessageAbstract 返回值通用说明（重要）

本 SDK 多数读取接口返回 `MessageAbstract[T] | None`。其中：

|字段|类型|说明|
|---|---|---|
|`ret.msg`|`T`|消息数据本体（例如 `list[float]` 或某个反馈消息结构体）|
|`ret.hz`|`float`|该消息类型的接收频率（由 SDK 统计），单位：Hz|
|`ret.timestamp`|`float`|消息时间戳（由 SDK 记录），单位：s|

### 读取机械臂状态 get_arm_status()

- 功能说明:

读取机械臂整体状态反馈（控制模式、运动模式、急停/异常状态、轨迹点编号等）。

- 定义:

```python
get_arm_status(self) -> MessageAbstract[ArmMsgFeedbackStatus] | None
```

- 返回值:

类型：`MessageAbstract[ArmMsgFeedbackStatus] | None`

- 消息字段说明（`MessageAbstract[ArmMsgFeedbackStatus].msg`）:

|字段|类型|说明|
|---|---|---|
|ctrl_mode|int|控制模式（待机/CAN/示教/以太网/WiFi/离线轨迹等）|
|arm_status|int|机械臂状态（正常/急停/奇异/超限/碰撞等）|
|mode_feedback|int|模式反馈（MOVE P/J 等）|
|teach_status|int|示教状态（开始记录/结束记录/执行/暂停/继续/终止等）|
|motion_status|int|运动状态：0 已到达；1 未到达|
|trajectory_num|int|轨迹点编号（离线轨迹模式下反馈）|
|err_status|object|错误状态位（关节角度超限/关节通信异常等）|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    arm_status = robot.get_arm_status()
    if arm_status is not None:
        print(arm_status.msg)
        print(arm_status.hz, arm_status.timestamp)
    time.sleep(0.02)
```

### 读取机械臂关节角度消息 get_joint_angles()

- 功能说明:

获取关节角度消息

- 定义:

```python
get_joint_angles(self) -> MessageAbstract[list[float]] | None
```

- 返回值:

类型：`MessageAbstract[list[float]] | None`

`MessageAbstract[list[float]].msg` 为长度 7 的 `list[float]`：`[j1, j2, j3, j4, j5, j6, j7]`，单位：rad。

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    ja = robot.get_joint_angles()
    if ja is not None:
        print(ja.msg) # list
        print(ja.hz, ja.timestamp)
    time.sleep(0.005)
```

### 读取机械臂法兰位姿 get_flange_pose()

- 功能说明:

获取末端位姿消息

说明（Terminology）：

`flange` 指机械臂最后一个连杆（末端连杆）的安装法兰/连接面，是工具/末端执行器的机械安装接口。

- 定义:

```python
get_flange_pose(self) -> MessageAbstract[list[float]] | None
```

- 返回值:

类型：`MessageAbstract[list[float]] | None`

`MessageAbstract[list[float]].msg` 为长度 6 的 `list[float]`：`[x, y, z, roll, pitch, yaw]`。

`x, y, z`: 位置坐标（单位：m）

`roll, pitch, yaw`: 姿态欧拉角（单位：rad，分别对应绕 X/Y/Z 轴旋转）

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    fp = robot.get_flange_pose()
    if fp is not None:
        print(fp.msg) # list
        print(fp.hz, fp.timestamp)
    time.sleep(0.005)
```

### 读取机械臂电机状态 get_motor_states()

- 功能说明:

读取指定关节的电机高速反馈（位置/速度/电流/扭矩）。

- 定义:

```python
get_motor_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 7]) -> MessageAbstract[ArmMsgFeedbackHighSpd] | None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号，范围：1~7|

- 返回值:

类型：`MessageAbstract[ArmMsgFeedbackHighSpd] | None`

- 消息字段说明（`MessageAbstract[ArmMsgFeedbackHighSpd].msg`）:

|字段|类型|说明|
|---|---|---|
|pos|float|电机位置，单位：rad|
|motor_speed|float|电机速度，单位：rad/s|
|current|float|电机电流，单位：A|
|torque|float|电机扭矩，单位：N·m|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

ms = robot.get_motor_states(1)
if ms is not None:
    print(ms.msg.pos, ms.msg.motor_speed, ms.msg.current, ms.msg.torque)
    print(ms.hz, ms.timestamp)
```

### 读取机械臂驱动器状态 get_driver_states()

- 功能说明:

读取指定关节的驱动器低速反馈（电压/温度/母线电流/驱动状态位等）。

- 定义:

```python
get_driver_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 7]) -> MessageAbstract[ArmMsgFeedbackLowSpd] | None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号，范围：1~7|

- 返回值:

类型：`MessageAbstract[ArmMsgFeedbackLowSpd] | None`

- 消息字段说明（`MessageAbstract[ArmMsgFeedbackLowSpd].msg`）:

|字段|类型|说明|
|---|---|---|
|vol|float|驱动电压|
|foc_temp|float|驱动温度，单位：°C|
|motor_temp|float|电机温度，单位：°C|
|bus_current|float|母线电流，单位：A|
|foc_status|object|驱动状态位（电压过低/过温/过流/碰撞/失能/堵转等）|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

ds = robot.get_driver_states(1)
if ds is not None:
    print(ds.msg.vol, ds.msg.foc_temp, ds.msg.motor_temp, ds.msg.bus_current)
    print(ds.msg.foc_status.driver_enable_status)
    print(ds.hz, ds.timestamp)
```

### 读取关节电机使能状态 get_joint_enable_status()

- 功能说明:

获取机械臂关节电机的使能状态

- 定义:

```python
get_joint_enable_status(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 7, 255]) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号：`1~7` 查询单关节；`255` 查询全部关节（内部使用 `all([...])` 汇总）|

- 返回值:

类型：`bool`（`True` 为使能，`False` 为未使能或当前无反馈）

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.get_joint_enable_status(1)
if success:
    print("关节1电机已使能")
```

### 读取全部关节使能状态 get_joints_enable_status_list()

- 功能说明:

读取全部关节电机的使能状态列表（按关节 1~7 顺序）。

- 定义:

```python
get_joints_enable_status_list(self) -> list[bool]
```

- 返回值:

类型：`list[bool]`

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

print(robot.get_joints_enable_status_list())
```

## 机械臂参数设定相关

### 设定机械臂运行速度 set_speed_percent()

- 功能说明:

该指令用来设定机械臂在位置速度模式(move_j)、末端控制模式(move_p)和直线控制模式(move_l)下的运行速度百分比

- 定义:

```python
set_speed_percent(self, percent: int = 100) -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|percent|int|设定机械臂运行速度，默认为100,范围[0,100]|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 设定机械臂运行速度为百分之百
robot.set_speed_percent(100)
```

### 设定运动模式 set_motion_mode()

- 功能说明:

设置运动模式（`move_p` 或 `move_j`）。该接口会下发模式设置到控制器。

- 定义:

```python
set_motion_mode(self, motion_mode: Literal["p", "j"] = "p") -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|motion_mode|str|运动模式：`"p"` 末端模式；`"j"` 关节模式（建议使用 `robot.OPTIONS.MOTION_MODE.xxx` 常量）|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.P)  # 或 robot.set_motion_mode("p")
```

## TCP相关

### 设置TCP偏移 set_tcp_offset()

- 功能说明:

设置 TCP（工具中心点）相对于法兰（`flange`）的偏移位姿（在 **法兰坐标系** 下）。

默认无偏移：`[0, 0, 0, 0, 0, 0]`。该偏移值仅保存在 SDK/Driver 实例内，不会下发到控制器。

- 定义:

```python
set_tcp_offset(self, pose: list[float]) -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|pose|list[float]|TCP 在法兰坐标系下的位姿偏移 `[x, y, z, roll, pitch, yaw]`：`x, y, z` 为位置坐标（m）；`roll, pitch, yaw` 为姿态欧拉角（rad，分别绕 X/Y/Z 轴旋转）。范围：`roll/yaw` 为 `[-pi, pi]`，`pitch` 为 `[-pi/2, pi/2]`|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 示例：TCP 相对法兰在 Z 方向偏移 10cm
robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])
```

### 获取TCP位姿 get_tcp_pose()

- 功能说明:

获取 TCP 位姿（工具中心点位姿）。该接口会先读取法兰位姿，然后根据 `set_tcp_offset()` 保存的偏移值做刚体变换得到 TCP 位姿。

若未设置偏移，则 TCP 位姿与法兰位姿相同。

- 定义:

```python
get_tcp_pose(self) -> MessageAbstract[list[float]] | None
```

- 返回值:

类型：`MessageAbstract[list[float]] | None`

`MessageAbstract[list[float]].msg` 为长度 6 的 `list[float]`：`[x, y, z, roll, pitch, yaw]`（m / rad）。

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])

while True:
    tcp = robot.get_tcp_pose()
    if tcp is not None:
        print(tcp.msg)
        print(tcp.hz, tcp.timestamp)
    time.sleep(0.02)
```

### 法兰位姿转TCP位姿 get_flange2tcp_pose()

- 功能说明:

输入法兰位姿（基座/世界坐标系下），根据 `set_tcp_offset()` 保存的偏移值算出对应的 TCP 位姿。

- 定义:

```python
get_flange2tcp_pose(self, flange_pose: list[float]) -> list[float]
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|flange_pose|list[float]|法兰位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。范围：`roll/yaw` 为 `[-pi, pi]`，`pitch` 为 `[-pi/2, pi/2]`|

- 返回值:

类型：`list[float]`

返回 TCP 位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 示例一：直接指定法兰位姿
tcp_pose = robot.get_flange2tcp_pose([-0.16, -0.043, 0.69, 1.118, 0.9272, 0.1482])
print("tcp_pose =", tcp_pose)

# 示例二：从当前位姿获取法兰位姿，与 `get_tcp_pose()` 得到的pose相同
flange_pose = robot.get_flange_pose()
if flange_pose is not None:
    tcp_pose = robot.get_flange2tcp_pose(flange_pose)
    print("tcp_pose =", tcp_pose)
```

### 目标TCP位姿转法兰位姿 get_tcp2flange_pose()

- 功能说明:

输入目标 TCP 位姿（基座/世界坐标系下），根据 `set_tcp_offset()` 保存的偏移值反算出对应的目标法兰位姿。

将返回的目标法兰位姿传给 `move_p()`，即可实现 **TCP 运动到目标 TCP 位姿**（取决于运动学与控制器实际执行结果）。

- 定义:

```python
get_tcp2flange_pose(self, tcp_pose: list[float]) -> list[float]
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|tcp_pose|list[float]|目标 TCP 位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。范围：`roll/yaw` 为 `[-pi, pi]`，`pitch` 为 `[-pi/2, pi/2]`|

- 返回值:

类型：`list[float]`

返回目标法兰位姿 `[x, y, z, roll, pitch, yaw]`（m / rad），可直接用于 `move_p()`。

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])

target_tcp_pose = [-0.16, -0.043, 0.69, 1.118, 0.9272, 0.1482]
target_flange_pose = robot.get_tcp2flange_pose(target_tcp_pose)
print("target_flange_pose =", target_flange_pose)

# robot.move_p(target_flange_pose)  # 注意：会触发运动
```

## 机械臂控制相关

### 使能机械臂 enable()

- 功能说明:

该指令会将机械臂使能上电

- 定义:

```python
enable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 7, 255] = 255) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号：`1~7` 使能单关节；`255` 使能全部关节|

- 返回值:

类型:bool

使能成功状态，True为使能成功

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 使能全部
while not robot.enable():
    time.sleep(0.01)
```

### 失能机械臂 disable()

- 功能说明:

该指令会将机械臂失电，如果执行指令时机械臂关节为抬起状态，会立刻掉下，需确保机械臂安全再使用

- 定义:

```python
disable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 7, 255] = 255) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号：`1~7` 失能单关节；`255` 失能全部关节|

- 返回值:

类型:bool

失能成功状态，True为失能成功

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 失能全部
while not robot.disable():
    time.sleep(0.01)
```

### 重置机械臂 reset()

- 功能说明:

该指令会将机械臂模式重置并令机械臂立刻失电，如果执行指令时机械臂关节为抬起状态，会立刻掉下，需确保机械臂安全再使用

- 定义:

```python
reset(self) -> None
```

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.reset()
```

### 电子急停机械臂 electronic_emergency_stop()

- 功能说明:

该指令会将机械臂设置为急停，如果执行指令时机械臂关节为抬起状态，机械臂会缓慢以一个恒定阻尼落下

- 定义:

```python
electronic_emergency_stop(self) -> None
```

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.electronic_emergency_stop()
```

### 切换为正常模式 set_normal_mode()

- 功能说明:

将机械臂设置为正常控制模式（单臂模式）。常用于从主从/联动模式切回普通模式。

- 定义:

```python
set_normal_mode(self) -> None
```

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_normal_mode()
```

### 关节运动 move_j()

- 功能说明:

该指令会将机械臂设置为关节位置速度控制模式

- 定义:

```python
move_j(self, joints: list[float]) -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joints|list[float]|长度 7 的各关节目标角度数组：`[j1, j2, j3, j4, j5, j6, j7]`（单位：rad，数值精度：1.74532925199e-5）|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_j([0, 0.4, -0.4, 0, -0.4, 0, 0])

# 等待运动结束（带 5s 超时，避免无限等待）
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("已到达目标位置")
        break
    if time.monotonic() - start_t > 5.0:
        print("等待运动结束超时（5s）")
        break
    time.sleep(0.1)
```

注意:

如果连续该指令会覆盖上一次的目标值

### 点到点运动 move_p()

- 功能说明:

该指令为发送一个目标法兰位姿，机械臂会根据当前关节位置和目标位姿进行关节角度解算

- 定义:

```python
move_p(self, pose: list[float]) -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|pose|list[float]|目标位姿 `[x, y, z, roll, pitch, yaw]`：`x, y, z` 为位置坐标（m，数值精度：1e-6）；`roll, pitch, yaw` 为姿态欧拉角（rad，分别绕 X/Y/Z 轴旋转，数值精度：1.74532925199e-5）|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_p([-0.4, -0.0, 0.4, 1.570823, 0.0, 0.0])

# 等待运动结束（带 5s 超时，避免无限等待）
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("已到达目标位置")
        break
    if time.monotonic() - start_t > 5.0:
        print("等待运动结束超时（5s）")
        break
    time.sleep(0.1)
```

注意:

如果连续执行该指令会覆盖上一次的目标值

### 直线运动 move_l()

- 功能说明:

该指令为发送一个目标法兰位姿，机械臂会根据当前位姿和目标位姿进行直线规划

- 定义:

```python
move_l(self, pose: list[float]) -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|pose|list[float]|目标位姿 `[x, y, z, roll, pitch, yaw]`：`x, y, z` 为位置坐标（m，数值精度：1e-6）；`roll, pitch, yaw` 为姿态欧拉角（rad，分别绕 X/Y/Z 轴旋转，数值精度：1.74532925199e-5）|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_l([-0.4, -0.2, 0.4, 1.570823, 0.0, 0.0])

# 等待运动结束（带 5s 超时，避免无限等待）
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("已到达目标位置")
        break
    if time.monotonic() - start_t > 5.0:
        print("等待运动结束超时（5s）")
        break
    time.sleep(0.1)
```

注意:

连续执行该指令虽然可以覆盖上一次的目标，但是由于底层每接收到新的点位，都需要重新进行直线规划，因此不能够使用该指令连续发目标点
