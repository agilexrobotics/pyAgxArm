# Nero First-Time User Guide (CAN)

> Step-by-step guide for first-time Nero robotic arm users using CAN communication.

## Table of Contents

- [Switch to 中文](#nero-首次使用指南can)
- [1. Environment Requirements](#1-environment-requirements)
- [2. Hardware Connection and Software Flow](#2-hardware-connection-and-software-flow)
- [3. Common Troubleshooting](#3-common-troubleshooting)
- [Safety Notes](#safety-notes)

## 1. Environment Requirements

- Ubuntu 20.04 / 22.04 (recommended) / 24.04
- Python >= 3.8
- Install `pyAgxArm` via pip
- Install `can-utils` on Ubuntu

## 2. Hardware Connection and Software Flow

**Preparation:**

- Nero robotic arm
- Nero power adapter
- CAN-to-USB module (with Type-C to USB-A cable)
- Ubuntu PC (with USB-A port)

### Method 1

1. Connect the USB-to-CAN module to the Nero CAN wires (aviation cable side; expose copper core). Yellow wire to H, blue wire to L, tighten terminal screws.
2. Connect CAN-to-USB module to PC and activate CAN (see [can_user](../can_user.md#can-module-manual)). Assume CAN interface is `can0`.
3. Connect the Nero power adapter (100-240V~50/60Hz), connect XT30 to the arm cable, then power on the arm.
4. Wait for green indicator, connect to Nero web UI, and enable CAN push.
5. Run `candump can0` on PC. If data appears, `pyAgxArm` can read data normally.

### Method 2

1. Connect USB-to-CAN module to Nero CAN wires.
2. Connect CAN-to-USB module to PC and activate CAN (see [can_user](../can_user.md#can-module-manual)), assume `can0`.
3. Power on the arm as above.
4. Wait for green indicator, then call `set_normal_mode()` in `pyAgxArm`:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW
robot_cfg = create_agx_arm_config(robot=ArmModel.NERO, firmeware_version=NeroFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(robot_cfg)
robot.connect()

while not robot.enable():
    robot.set_normal_mode()
    time.sleep(0.01)
```

This switches the arm to normal mode, enables CAN push, and the arm will auto-enable.

5. Run `candump can0` and verify data output.

### Summary

| Method | Description |
|---|---|
| Method 1 | Enable CAN push from web UI |
| Method 2 | Enter normal mode via pyAgxArm and enable active CAN reporting |

Example of normal `candump can0` data stream:

![alt text](../../assets/pictures/candump_can0.png)

Attached is a code snippet showing how to read joint data after enabling CAN mode. Note that `robot.set_normal_mode()` is commented out here; you can enable it as needed.:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW
import time
robot_cfg = create_agx_arm_config(robot=ArmModel.NERO, firmeware_version=NeroFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(robot_cfg)
robot.connect()

while not robot.enable():
    # robot.set_normal_mode()
    time.sleep(0.01)

while True:
    print(robot.get_joint_angles())
    time.sleep(0.01)
```

## 3. Common Troubleshooting

**No data from `candump`:**
- CAN push not enabled on web UI
- Normal mode not enabled
- Wrong bitrate when activating CAN
- H/L wires reversed
- CAN wire not properly stripped (no copper contact)

**`connect` failed:**
- `can0` does not exist

**Arm indicator is not green:**
- Arm initialization not finished
- Check power supply and 24V adapter output

## Safety Notes

> - Ensure there are no obstacles around the arm during first power-on.
> - Do not send motion commands when arm state is unknown.

---

# Nero 首次使用指南（CAN）

> Nero 机械臂首次使用 CAN 通信的分步指南。

## 目录

- [切换到 English](#nero-first-time-user-guide-can)
- [一、环境要求](#一环境要求)
- [二、硬件连接以及软件执行流程](#二硬件连接以及软件执行流程)
- [三、常见错误排查](#三常见错误排查)

## 一、环境要求

- Ubuntu 20.04 / 22.04（推荐）/ 24.04
- Python >= 3.8
- pip 安装 pyAgxArm
- Ubuntu 安装 can-utils

## 二、硬件连接以及软件执行流程

**准备：**

- Nero 机械臂
- Nero 机械臂适配器
- CAN 转 USB 模块（带 Type-C 转 USB-A 线材）
- Ubuntu 系统 PC 一台（需要有 USB-A 接口）

### 方法一

1. 将 USB 转 CAN 模块连接 Nero 机械臂的 CAN 线（在航插线上，注意要剥开线皮露出铜线），黄色出线对应模块的 H，蓝色出线对应模块的 L，使用一字螺丝刀拧紧接线端子；
2. 将 CAN 转 USB 模块连接 PC，在 PC 上激活 CAN 模块（详见：[can_user](../can_user.md#can-模块使用手册)），这里假设激活的 CAN 名称为 `can0`；
3. 使用 Nero 机械臂适配器，一端连接交流电（100-240v~50/60Hz），另一端对插适配器与航插线的 XT30 接口，然后将航插插入机械臂，将 Nero 机械臂上电；
4. 观察机械臂指示灯，待机械臂指示灯绿色后，连接 Nero 网页上位机，在上位机端打开 CAN 推送；
5. 在 PC 上执行 `candump can0`，看是否有数据，有的话 `pyAgxArm` 即可正常读取数据。

### 方法二

1. 将 USB 转 CAN 模块连接 Nero 机械臂的 CAN 线（在航插线上，注意要剥开线皮露出铜线），黄色出线对应模块的 H，蓝色出线对应模块的 L，使用一字螺丝刀拧紧接线端子；
2. 将 CAN 转 USB 模块连接 PC，在 PC 上激活 CAN 模块（详见：[can_user](../can_user.md#can-模块使用手册)），这里假设激活的 CAN 名称为 `can0`；
3. 使用 Nero 机械臂适配器，一端连接交流电（100-240v~50/60Hz），另一端对插适配器与航插线的 XT30 接口，然后将航插插入机械臂，将 Nero 机械臂上电；
4. 观察机械臂指示灯，待机械臂指示灯绿色后，在 PC 上编写代码，调用 `pyAgxArm` 的 `set_normal_mode()` 函数

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW
robot_cfg = create_agx_arm_config(robot=ArmModel.NERO, firmeware_version=NeroFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(robot_cfg)
robot.connect()

while not robot.enable():
    robot.set_normal_mode()
    time.sleep(0.01)
```

该函数作用是将机械臂切换到普通模式并开启 CAN 推送，执行该指令后，机械臂会自动使能。

5. 在 PC 上开启终端执行 `candump can0`，看是否有数据，有的话 `pyAgxArm` 即可正常读取数据。

### 总结

| 方法 | 说明 |
|---|---|
| 方法一 | 通过网页上位机开启 CAN 推送 |
| 方法二 | 通过 pyAgxArm 进入机械臂的 normal mode，并开启 CAN 主动上报 |

执行 `candump can0`，正常的数据流如下：

![alt text](../../assets/pictures/candump_can0.png)

附上一段开启 CAN 模式后，读取关节数据的代码，注意这里将 `robot.set_normal_mode()` 注释掉了，可以按需开启。

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW
import time
robot_cfg = create_agx_arm_config(robot=ArmModel.NERO, firmeware_version=NeroFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(robot_cfg)
robot.connect()

while not robot.enable():
    # robot.set_normal_mode()
    time.sleep(0.01)

while True:
    print(robot.get_joint_angles())
    time.sleep(0.01)
```

## 三、常见错误排查

**candump 没有数据：**
- 未开启网页 CAN 推送
- 未开启 normal mode
- 激活 CAN 模块时 bitrate 错误
- H/L 接反
- CAN 线连接时线头没有露出铜线就连接了 CAN 模块

**connect 失败：**
- can0 不存在

**机械臂灯非绿色：**
- 机械臂未初始化完成
- 检查电源是否正常，适配器 24V 输出是否有电压

> **安全警告：**
>
> - 首次上电请确保机械臂周围无障碍物
> - 不要在未知状态下发送运动指令
