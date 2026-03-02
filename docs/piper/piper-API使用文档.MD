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
    "robot": "piper",
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
    "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"], 
    "joint_limits":{
        "joint1":[-2.617994, 2.617994],
        "joint2":[0.0, 3.141593],
        "joint3":[-2.967060, 0.0],
        "joint4":[-1.745330, 1.745330],
        "joint5":[-1.221730, 1.221730],
        "joint6":[-2.094396, 2.094396]
    },
}
```

关节运动限位单位为弧度

夹爪运动限位单位为米

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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
|effector|str|执行器类型（建议使用 `robot.OPTIONS.EFFECTOR.xxx` 常量）|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 初始化执行器（只能调用一次）
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
```

## 通用状态相关

### 获取关节数量 joint_nums

- 功能说明:

获取机械臂关节数量（例如 Piper 为 6）。

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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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

### 通信是否正常 is_ok()

- 功能说明:

用于判断机械臂数据接收是否正常。该值由 SDK 内部的数据监控逻辑根据“最近一段时间是否持续收不到数据”计算得出。

- 定义:

```python
is_ok(self) -> bool
```

- 返回值:

类型：`bool`

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

time.sleep(0.5)
print("robotic arm is_ok =", robot.is_ok())
```

### 获取机械臂数据接收频率 get_fps()

- 功能说明:

获取机械臂数据监控的接收频率（Hz）。该频率是 SDK 对机械臂解析器收到数据的统计值。

- 定义:

```python
get_fps(self) -> float
```

- 返回值:

类型：`float`（单位：Hz）

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

time.sleep(0.5)
print("robotic arm fps =", robot.get_fps(), "Hz")
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
|mode_feedback|int|模式反馈（MOVE P/J/L/C/MIT 等）|
|teach_status|int|示教状态（开始记录/结束记录/执行/暂停/继续/终止等）|
|motion_status|int|运动状态：0 已到达；1 未到达|
|trajectory_num|int|轨迹点编号（离线轨迹模式下反馈）|
|err_status|object|错误状态位（关节角度超限/关节通信异常等）|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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

`MessageAbstract[list[float]].msg` 为长度 6 的 `list[float]`：`[j1, j2, j3, j4, j5, j6]`，单位：rad。

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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
get_motor_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6]) -> MessageAbstract[ArmMsgFeedbackHighSpd] | None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号，范围：1~6|

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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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
get_driver_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6]) -> MessageAbstract[ArmMsgFeedbackLowSpd] | None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号，范围：1~6|

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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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
get_joint_enable_status(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255]) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号：`1~6` 查询单关节；`255` 查询全部关节（内部使用 `all([...])` 汇总）|

- 返回值:

类型：`bool`（`True` 为使能，`False` 为未使能或当前无反馈）

True为关节电机使能，False为关节电机未使能

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.get_joint_enable_status(1)
if success:
    print("关节1电机已使能")
```

### 读取全部关节使能状态 get_joints_enable_status_list()

- 功能说明:

读取全部关节电机的使能状态列表（按关节 1~6 顺序）。

- 定义:

```python
get_joints_enable_status_list(self) -> list[bool]
```

- 返回值:

类型：`list[bool]`

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

print(robot.get_joints_enable_status_list())
```

### 读取固件信息 get_firmware()

- 功能说明:

读取机械臂固件信息（软件版本/硬件版本/生产日期等）。该接口会下发查询帧并等待对应反馈。

- 定义:

```python
get_firmware(self, timeout: float = 1.0, min_interval: float = 1.0) -> dict | None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|timeout|float|等待反馈超时时间（秒），默认 1.0；`timeout=0.0` 表示非阻塞|
|min_interval|float|最小请求间隔（秒），默认 1.0|

- 返回值:

类型：`dict | None`

常见字段：

|key|类型|说明|
|---|---|---|
|software_version|str|软件版本（例如 `S-V1.8-2`）|
|hardware_version|str|硬件版本（例如 `H-V1.2-1`）|
|production_date|str|生产日期（例如 `250925`）|
|node_type|str|节点类型|
|node_number|int|节点编号|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

fw = robot.get_firmware()
if fw is not None:
    print(fw)
```

## 机械臂参数设定相关

### 设定机械臂运行速度 set_speed_percent()

- 功能说明:

该指令用来设定机械臂在位置速度模式下的关节运动(move_j)、点到点运动(move_p)、直线运动(move_l)和圆弧运动(move_c)的运行速度百分比

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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 设定机械臂运行速度为百分之百
robot.set_speed_percent(100)
```

### 设定机械臂安装位置 set_installation_pos()

- 功能说明:

该指令用来设定机械臂安装位置，有水平、朝左和朝右三个方向

- 定义:

```python
set_installation_pos(self, pos: Literal["horizontal", "left", "right"] = "horizontal") -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|pos|str|`["horizontal", "left", "right"]`（建议使用 `robot.OPTIONS.INSTALLATION_POS.xxx` 常量）|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 设定水平安装
robot.set_installation_pos(robot.OPTIONS.INSTALLATION_POS.HORIZONTAL)
```

### 设定运动模式 set_motion_mode()

- 功能说明:

设置运动模式（`move_p/move_j/move_l/move_c/move_mit/move_js`）。该接口会下发模式设置到控制器。

`move_p/move_j/move_l/move_c`为位置速度模式，底层会对接收到消息进行平滑处理。

`move_mit/move_js`为MIT电机透传模式，底层只负责消息转发，不会进行任何的平滑处理。

- 定义:

```python
set_motion_mode(self, motion_mode: Literal["p", "j", "l", "c", "mit", "js"] = "p") -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|motion_mode|str|运动模式：`p/j/l/c/mit/js`（建议使用 `robot.OPTIONS.MOTION_MODE.xxx` 常量）|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.J)
```

### 设定机械臂负载 set_payload()

- 功能说明:

该指令用来设定机械臂负载（Payload）

- 定义:

```python
set_payload(self, load: Literal['empty', 'half', 'full'] = 'empty', timeout: float = 1.0) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|load|str|范围为 `['empty', 'half', 'full']`；`'empty'` 为空载，`'half'` 为半载，`'full'` 为满载（建议使用 `robot.OPTIONS.PAYLOAD.xxx` 常量）|
|timeout|float|设定指令发送后，等待反馈的循环时间|

- 返回值:

类型:bool

True为指令应答返回，并不表明设定成功

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 设定满载
robot.set_payload(robot.OPTIONS.PAYLOAD.FULL)
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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])

# 示例一：直接指定法兰位姿
tcp_pose = robot.get_flange2tcp_pose([0.30, 0.0, 0.30, 0.0, 1.5707, 0.0])
print("tcp_pose =", tcp_pose)

# 示例二：从当前位姿获取法兰位姿，与 `get_tcp_pose()` 得到的pose相同
flange_pose = robot.get_flange_pose()
if flange_pose is not None:
    tcp_pose = robot.get_flange2tcp_pose(flange_pose)
    print("tcp_pose =", tcp_pose)
```

### 目标TCP位姿转法兰位姿 get_tcp2flange_pose()

- 功能说明:

输入目标 TCP 位姿（基座/世界坐标系下），根据 `set_tcp_offset()` 保存的偏移值算出对应的目标法兰位姿。

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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])

target_tcp_pose = [0.30, 0.0, 0.30, 0.0, 1.5707, 0.0]
target_flange_pose = robot.get_tcp2flange_pose(target_tcp_pose)
print("target_flange_pose =", target_flange_pose)

# robot.move_p(target_flange_pose)  # 注意：会触发运动
```

## 主从臂相关

### 设定机械臂为主臂模式 set_master_mode()

- 功能说明:

将机械臂设置为**主臂零力拖动模式**（主从协同场景下的“主臂”）。

该模式用于**主从臂联动/示教**等场景；若当前仅使用单臂，可忽略该接口。

进入该模式后，主臂通常处于可拖动/零力拖动状态；从臂的受控状态需通过 `set_slave_mode()` 配置。

- 定义:

```python
set_master_mode(self) -> None
```

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_master_mode()
```

### 设定机械臂为从臂模式 set_slave_mode()

- 功能说明:

将机械臂设置为**从臂受控模式**（主从协同场景下的“从臂”）。

该模式可用于主从臂联动：从臂跟随主臂控制/指令运行。

可与 `set_master_mode()` 配套使用。

- 定义:

```python
set_slave_mode(self) -> None
```

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_slave_mode()
```

### 主臂回 Home move_master_to_home()

- 功能说明:

让主臂回到 Home 位姿（主臂相关）。该操作完成后，建议调用 `restore_master_drag_mode()` 恢复主臂“零力拖动”状态。

- 定义:

```python
move_master_to_home(self) -> None
```

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_master_mode()
robot.move_master_to_home()
robot.restore_master_drag_mode()
```

### 恢复主臂零力拖动 restore_master_drag_mode()

- 功能说明:

将主臂恢复为“零力拖动”状态（主臂相关），通常用于 `move_master_to_home()` 之后。

- 定义:

```python
restore_master_drag_mode(self) -> None
```

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 常见用法：主臂回 Home 后恢复“零力拖动”
robot.set_master_mode()
robot.move_master_to_home()
robot.restore_master_drag_mode()
```

### 读取主臂关节角度消息 get_master_joint_angles()

- 功能说明:

获取主臂关节角度消息，用于控制从臂

- 定义:

```python
get_master_joint_angles(self) -> MessageAbstract[list[float]] | None
```

- 返回值:

类型：`MessageAbstract[list[float]] | None`

`MessageAbstract[list[float]].msg` 为长度 6 的 `list[float]`：`[j1, j2, j3, j4, j5, j6]`，单位：rad。

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_master_mode()

while True:
    mja = robot.get_master_joint_angles()
    if mja is not None:
        print(mja.msg) # list
        print(mja.hz, mja.timestamp)
    time.sleep(0.005)
```

## 机械臂控制相关

### 使能机械臂 enable()

- 功能说明:

该指令会将机械臂使能上电

- 定义:

```python
enable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号：`1~6` 使能单关节；`255` 使能全部关节|

- 返回值:

类型:bool

使能成功状态，True为使能成功

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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
disable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号：`1~6` 失能单关节；`255` 失能全部关节|

- 返回值:

类型:bool

失能成功状态，True为失能成功

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.electronic_emergency_stop()
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
|joints|list[float]|长度 6 的各关节目标角度数组：`[j1, j2, j3, j4, j5, j6]`（单位：rad，数值精度：1.74532925199e-5）|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_j([0, 0.4, -0.4, 0, -0.4, 0])

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

### 关节运动(从臂控制模式) move_js()

- 功能说明:

该指令会将机械臂切换到 **JS（从臂）模式**（MIT 透传模式），并下发关节目标角度。

与 `move_j` 相比，`move_js` 更偏向“快速响应”控制：**不做平滑处理、无轨迹规划**，控制器/驱动器会尽可能快（但并非无限快）地响应目标角度。

- 定义:

```python
move_js(self, joints: list[float]) -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joints|list[float]|长度 6 的各关节目标角度数组：`[j1, j2, j3, j4, j5, j6]`（单位：rad，数值精度：1.74532925199e-5）|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

# robot.set_speed_percent(100) # 该模式下设定的速度失效
robot.move_js([0, 0.4, -0.4, 0, -0.4, 0])
```

注意:

1. **风险等级：极高**。该模式可能导致**冲击、振荡、失稳**等风险，请仅在充分评估安全与控制稳定性的前提下使用，并确保随时可急停。

2. 该模式为“瞬时响应”风格：**无平滑过程、无轨迹规划**，控制器/驱动器尝试以最快响应（非无限快）到达目标，可能产生冲击、振荡。

3. 如果连续执行该指令会覆盖上一次的目标值

4. 由于响应变快，关节的控制力度相较于位置速度模式小，刚度也会变小

5. 在旧版本机械臂固件下（低于 `S-V1.8-5`），如果机械臂当前模式为从臂模式，如果想要切换到位置速度控制模式，需要先执行 `robot.reset()`，机械臂此时会重置掉电，然后再执行 `move_j` 才能正常控制

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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_p([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])

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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_l([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])

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

### 圆弧运动 move_c()

- 功能说明:

圆弧运动控制：通过“起点/中间点/终点”三个目标法兰位姿进行圆弧轨迹规划并执行。

- 定义:

```python
move_c(self, start_pose: list[float], mid_pose: list[float], end_pose: list[float]) -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|start_pose|list[float]|起点位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）|
|mid_pose|list[float]|中间点位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）|
|end_pose|list[float]|终点位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
sp = [0.2, 0.0, 0.3, 0.0, 1.5708, 0.0]
mp = [0.2, 0.05, 0.35, 0.0, 1.5708, 0.0]
ep = [0.2, 0.0, 0.4, 0.0, 1.5708, 0.0]
robot.move_c(sp, mp, ep)

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

### 单关节mit控制 move_mit()

- 功能说明:

该指令为使用关节驱动底层的mit控制接口，控制单个关节电机，可以实现电流模拟的力矩控制

该接口会发送 MIT 控制消息。

控制器概念上会计算参考力矩：

$$T_{\text{ref}} = k_p \cdot (p_{\text{des}} - p) + k_d \cdot (v_{\text{des}} - v) + T_{\text{ff}}$$

其中 \(p/v\) 为关节实测位置/速度。

典型用法建议：

**速度控制**：设置 `kp = 0`、`kd != 0`，主要通过 `v_des` 控制。

**力矩控制**：设置 `kp = 0`、`kd = 0`，主要通过 `t_ff` 控制。

**位置控制**：当 `kp != 0` 时，不建议将 `kd` 设为 0（可适当增大阻尼以降低振荡风险）。

风险提示：MIT 属于较底层控制接口，参数不当可能引发 **冲击/振荡/不稳定**；建议从小增益开始调试，并在安全工况下使用。

- 定义:

```python
move_mit(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
    p_des: float = 0.0,
    v_des: float = 0.0,
    kp: float = 10.0,
    kd: float = 0.8,
    t_ff: float = 0.0,
    ) -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|目标关节，有效值范围[1, 2, 3, 4, 5, 6]|
|p_des|float|目标位置，数值范围[-12.5, 12.5]，单位：弧度，默认值0.0，数值精度：3.8147554741741054e-4|
|v_des|float|目标速度，数值范围[-45.0, 45.0]，单位：弧度/秒，默认值0.0，数值精度：2.197802197802198e-2|
|kp|float|位置增益，数值范围[0.0, 500.0]，默认值10.0，数值精度：1.221001221001221e-1|
|kd|float|速度增益，数值范围[-5.0, 5.0]，默认值0.8。数值精度：2.442002442002442e-3|
|t_ff|float|前馈扭矩，数值范围[-8.0, 8.0]，单位N·m，默认值0.0，数值精度：6.274509803921569e-2|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

# robot.set_speed_percent(100) # 该模式下速度设置无效
for i in range(1, robot.joint_nums + 1):
    robot.move_mit(
        joint_index=i,
        p_des=0.0,
        v_des=0.0,
        kp=10.0,
        kd=0.8,
        t_ff=0.0,
    )
```

注意:

如果连续执行该指令会覆盖上一次的目标值

## 高级参数读取/配置相关

### 读取关节角度/速度限制 get_joint_angle_vel_limits()

- 功能说明:

查询指定关节的角度限制与速度限制（由控制器反馈）。

- 定义:

```python
get_joint_angle_vel_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd] | None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号，范围：1~6|
|timeout|float|等待反馈超时时间（秒），默认 1.0；`timeout=0.0` 表示非阻塞|
|min_interval|float|最小请求间隔（秒），默认 1.0|

- 返回值:

类型：`MessageAbstract[ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd] | None`

- 消息字段说明（`MessageAbstract[ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd].msg`）:

|字段|类型|说明|
|---|---|---|
|min_angle_limit|float|最小角度限制，单位：rad|
|max_angle_limit|float|最大角度限制，单位：rad|
|min_joint_spd|float|最小关节速度限制，单位：rad/s|
|max_joint_spd|float|最大关节速度限制，单位：rad/s|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

limit = robot.get_joint_angle_vel_limits(1)
if limit is not None:
    print(limit.msg.min_angle_limit, limit.msg.max_angle_limit)
    print(limit.msg.min_joint_spd, limit.msg.max_joint_spd)
```

### 读取关节加速度限制 get_joint_acc_limits()

- 功能说明:

查询指定关节的最大加速度限制。

- 定义:

```python
get_joint_acc_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[ArmMsgFeedbackCurrentMotorMaxAccLimit] | None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号，范围：1~6|
|timeout|float|等待反馈超时时间（秒），默认 1.0；`timeout=0.0` 表示非阻塞|
|min_interval|float|最小请求间隔（秒），默认 1.0|

- 返回值:

类型：`MessageAbstract[ArmMsgFeedbackCurrentMotorMaxAccLimit] | None`

- 消息字段说明（`MessageAbstract[ArmMsgFeedbackCurrentMotorMaxAccLimit].msg`）:

|字段|类型|说明|
|---|---|---|
|max_joint_acc|float|最大关节加速度限制，单位：rad/s²|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

limit = robot.get_joint_acc_limits(1)
if limit is not None:
    print(limit.msg.max_joint_acc)
    print(limit.hz, limit.timestamp)
```

### 读取法兰速度/加速度限制 get_flange_vel_acc_limits()

- 功能说明:

查询末端最大线速度/角速度与线加速度/角加速度限制。

- 定义:

```python
get_flange_vel_acc_limits(self, timeout: float = 1.0, min_interval: float = 1.0) -> MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam] | None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|timeout|float|等待反馈超时时间（秒），默认 1.0；`timeout=0.0` 表示非阻塞|
|min_interval|float|最小请求间隔（秒），默认 1.0|

- 返回值:

类型：`MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam] | None`

- 消息字段说明（`MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam].msg`）:

|字段|类型|说明|
|---|---|---|
|end_max_linear_vel|float|末端最大线速度，单位：m/s|
|end_max_angular_vel|float|末端最大角速度，单位：rad/s|
|end_max_linear_acc|float|末端最大线加速度，单位：m/s²|
|end_max_angular_acc|float|末端最大角加速度，单位：rad/s²|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

limit = robot.get_flange_vel_acc_limits()
if limit is not None:
    print(
        limit.msg.end_max_linear_vel,
        limit.msg.end_max_angular_vel,
        limit.msg.end_max_linear_acc,
        limit.msg.end_max_angular_acc,
    )
    print(limit.hz, limit.timestamp)
```

### 读取碰撞防护等级 get_crash_protection_rating()

- 功能说明:

查询各关节碰撞防护等级（控制器返回列表）。

- 定义:

```python
get_crash_protection_rating(
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[list[int]] | None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|timeout|float|等待反馈超时时间（秒），默认 1.0；`timeout=0.0` 表示非阻塞|
|min_interval|float|最小请求间隔（秒），默认 1.0|

- 返回值:

类型：`MessageAbstract[list[int]] | None`

`MessageAbstract[list[int]].msg` 为碰撞防护等级列表（按关节顺序），每项为 `int`（范围：0~8），**等级越高越敏感，越容易触发碰撞保护机制**（更保守）。

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

rating = robot.get_crash_protection_rating()
if rating is not None:
    print(rating.msg)  # list[int]
    print(rating.hz, rating.timestamp)
```

### 关节置零/标定 calibrate_joint()

- 功能说明:

对指定关节执行置零/标定流程（等待控制器 ACK/响应并返回结果）。

- 定义:

```python
calibrate_joint(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    timeout: float = 1.0,
) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|1~6 标定单关节；255 标定全部|
|timeout|float|等待响应超时时间（秒），默认 1.0|

- 返回值:

类型：`bool`（`True` 表示收到成功响应；`False` 表示超时或失败）

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 示例：标定单关节（请确保现场安全、必要时支撑/卸载，避免掉落）
joint_index = 1
robot.disable(joint_index)
time.sleep(0.2)
input("请手动将关节移动到零位位置后按回车继续...")

if robot.calibrate_joint(joint_index):
    print("calibrate_joint success")
```

### 配置关节角度/速度限制 set_joint_angle_vel_limits()

- 功能说明:

设置关节角度/速度限制，并可通过读回校验是否生效。

- 定义:

```python
set_joint_angle_vel_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    min_angle_limit: Optional[float] = None,
    max_angle_limit: Optional[float] = None,
    max_joint_spd: Optional[float] = None,
    timeout: float = 1.0,
) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号：`1~6` 配置单关节；`255` 配置全部关节|
|min_angle_limit|Optional[float]|最小角度限制（rad）；`None` 表示不配置/保持不变|
|max_angle_limit|Optional[float]|最大角度限制（rad）；`None` 表示不配置/保持不变|
|max_joint_spd|Optional[float]|最大关节速度限制（rad/s）；`None` 表示不配置/保持不变|
|timeout|float|等待 ACK/校验超时时间（秒），默认 1.0|

- 返回值:

类型：`bool`

`True` 表示已收到 ACK 且读回校验通过；`False` 表示超时/失败/校验未通过。

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_joint_angle_vel_limits(
    joint_index=1,
    min_angle_limit=-2.618,
    max_angle_limit=2.618,
    max_joint_spd=3.0,
)
print("set_joint_angle_vel_limits success =", success)

# 仅设置最大速度限制（不改角度限制）
success = robot.set_joint_angle_vel_limits(joint_index=1, max_joint_spd=3.0)
print("set_joint_angle_vel_limits(max_joint_spd) success =", success)
```

### 配置关节加速度限制 set_joint_acc_limits()

- 功能说明:

设置指定关节的最大加速度限制。

- 定义:

```python
set_joint_acc_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    max_joint_acc: Optional[float] = None,
    timeout: float = 1.0,
) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号：`1~6` 配置单关节；`255` 配置全部关节|
|max_joint_acc|Optional[float]|最大加速度（rad/s^2）；`None` 表示不配置/保持不变|
|timeout|float|等待 ACK/校验超时时间（秒），默认 1.0|

- 返回值:

类型：`bool`

`True` 表示已收到 ACK 且读回校验通过；`False` 表示超时/失败/校验未通过。

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_joint_acc_limits(joint_index=1, max_joint_acc=5.0)
print("set_joint_acc_limits success =", success)
```

### 配置法兰速度/加速度限制 set_flange_vel_acc_limits()

- 功能说明:

设置末端速度/加速度限制。

- 定义:

```python
set_flange_vel_acc_limits(
    self,
    max_linear_vel: Optional[float] = None,
    max_angular_vel: Optional[float] = None,
    max_linear_acc: Optional[float] = None,
    max_angular_acc: Optional[float] = None,
    timeout: float = 1.0,
) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|max_linear_vel|Optional[float]|最大线速度（m/s）；`None` 表示不配置/保持不变|
|max_angular_vel|Optional[float]|最大角速度（rad/s）；`None` 表示不配置/保持不变|
|max_linear_acc|Optional[float]|最大线加速度（m/s²）；`None` 表示不配置/保持不变|
|max_angular_acc|Optional[float]|最大角加速度（rad/s²）；`None` 表示不配置/保持不变|
|timeout|float|等待 ACK/校验超时时间（秒），默认 1.0|

- 返回值:

类型：`bool`

`True` 表示已收到 ACK 且读回校验通过；`False` 表示超时/失败/校验未通过。

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_flange_vel_acc_limits(
    max_linear_vel=0.5,
    max_angular_vel=0.13,
    max_linear_acc=0.8,
    max_angular_acc=0.2,
)
print("set_flange_vel_acc_limits success =", success)
```

### 配置碰撞防护等级 set_crash_protection_rating()

- 功能说明:

设置碰撞防护等级（可指定单关节或全部关节）。

- 定义:

```python
set_crash_protection_rating(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    rating: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8] = 0,
    timeout: float = 1.0,
) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|joint_index|int|关节序号：`1~6` 配置单关节；`255` 配置全部关节|
|rating|int|碰撞防护等级：`0~8`。**等级越高越敏感，越容易触发碰撞保护机制**（更保守）|
|timeout|float|等待 ACK/校验超时时间（秒），默认 1.0|

- 返回值:

类型：`bool`

`True` 表示已收到 ACK 且读回校验通过；`False` 表示超时/失败/校验未通过。

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_crash_protection_rating(joint_index=1, rating=1)
print("set_crash_protection_rating success =", success)
```

### 恢复法兰速度/加速度默认值 set_flange_vel_acc_limits_to_default()

- 功能说明:

将末端速度/加速度限制恢复为默认值。

- 定义:

```python
set_flange_vel_acc_limits_to_default(self, timeout: float = 1.0) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|timeout|float|等待 ACK/响应超时时间（秒），默认 1.0|

- 返回值:

类型：`bool`

`True` 表示在 `timeout` 内收到 ACK/响应；该接口**不提供读回校验**，如需确认可再调用 `get_flange_vel_acc_limits()` 查询。

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_flange_vel_acc_limits_to_default()
print("set_flange_vel_acc_limits_to_default success =", success)
```

### 恢复关节角度/速度/加速度默认值 set_joint_angle_vel_acc_limits_to_default()

- 功能说明:

将关节角度/速度/加速度限制恢复为默认值。

- 定义:

```python
set_joint_angle_vel_acc_limits_to_default(self, timeout: float = 1.0) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|timeout|float|等待 ACK/响应超时时间（秒），默认 1.0|

- 返回值:

类型：`bool`

`True` 表示在 `timeout` 内收到 ACK/响应；该接口**不提供读回校验**，如需确认可再调用 `get_joint_angle_vel_limits()` / `get_joint_acc_limits()` 查询。

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_joint_angle_vel_acc_limits_to_default()
print("set_joint_angle_vel_acc_limits_to_default success =", success)
```

### 设置各关节 link 的笛卡尔速度/加速度周期反馈 set_links_vel_acc_period_feedback()

- 功能说明:

设置各关节 link 的笛卡尔速度/加速度周期反馈开关（对应 CAN 周期帧 `0x481~0x486`）。

该功能在底层主控中 **已废弃**，但总线仍可能周期上报对应帧；且上报数据 **全为 0**，无实际意义。**建议默认关闭**（`enable=False`），避免占用带宽与造成误解。

该接口无直接“读回校验”方式，建议使用 `candump` 观察 `0x481~0x486` 周期帧是否出现来验证：

```bash
candump can0 | grep "48[1-6]"
```

- 定义:

```python
set_links_vel_acc_period_feedback(self, enable: bool = False, timeout: float = 1.0) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|enable|bool|是否开启周期反馈：`True` 开启（会周期上报 `0x481~0x486`）；`False` 关闭（**建议默认关闭**）|
|timeout|float|等待 ACK/响应超时时间（秒），默认 1.0|

- 返回值:

类型：`bool`

`True` 表示在 `timeout` 内收到 ACK/响应；`False` 表示超时/失败。该接口**不提供读回校验**，建议通过 `candump` 验证总线周期帧是否出现。

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 开启/关闭周期反馈（对应 CAN 周期帧 0x481~0x486）
success = robot.set_links_vel_acc_period_feedback(enable=True)
print("enable periodic feedback success =", success)

success = robot.set_links_vel_acc_period_feedback(enable=False)
print("disable periodic feedback success =", success)
```
