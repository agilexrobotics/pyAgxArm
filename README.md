# pyAgxArm User Guide

> `pyAgxArm` is a Python SDK for AgileX robotic arms and end effectors. It supports CAN communication, status reading, motion control, and end-effector control for Piper, Nero, AgxGripper, and Revo2.

## Table of Contents

- [Switch to 中文](#pyagxarm-使用说明)
- [Introduction](#pyagxarm-user-guide)
- [Environment](#environment)
- [Documentation](#documentation)
- [Install](#install)
- [Communication Setup](#communication-setup)
- [Quick Start](#quick-start)
- [Notes](#notes)
- [Contact](#contact)


### Environment

- Ubuntu: `18.04 / 20.04 / 22.04 / 24.04`
- Windows: `10 / 11`
- macOS (Darwin)
- Python: `3.6` and above (compatible up to `3.14`)

### Documentation

| Topic | Link |
| --- | --- |
| GitHub Discussions | [All](https://github.com/agilexrobotics/pyAgxArm/discussions) · [Announcements](https://github.com/agilexrobotics/pyAgxArm/discussions/categories/announcements) · [General](https://github.com/agilexrobotics/pyAgxArm/discussions/categories/general) · [Q&A](https://github.com/agilexrobotics/pyAgxArm/discussions/categories/q-a) |
| ROS | [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) |
| Piper API | [docs/piper/piper_api.md](./docs/piper/piper_api.md#piper-api-documentation) |
| Nero API | [docs/nero/nero_api.md](./docs/nero/nero_api.md#nero-api-documentation) |
| AgxGripper API | [docs/effector/agx_gripper/agx_gripper_api.md](./docs/effector/agx_gripper/agx_gripper_api.md#agxgripper-api-documentation) |
| Revo2 API | [docs/effector/revo2/revo2_api.md](./docs/effector/revo2/revo2_api.md#revo2-api-documentation) |
| Revo2 Touch API | [docs/effector/revo2_touch/revo2_touch_api.md](./docs/effector/revo2_touch/revo2_touch_api.md#revo2-touch-api-documentation) |
| CAN module manual | [docs/can_user.md](./docs/can_user.md#can-module-manual) |
| Nero first-time CAN guide | [docs/nero/first_time_user_guide_can.md](./docs/nero/first_time_user_guide_can.md#nero-first-time-user-guide-can) |
| WSL2 USB-CAN guide | [docs/wsl2_usb_can_guide.md](./docs/wsl2_usb_can_guide.md#wsl2-ubuntu-2204-complete-usb-can-setup-guide) |
| Ubuntu 24.04 pip guide | [docs/ubuntu_24_04_pip_install.md](./docs/ubuntu_24_04_pip_install.md#ubuntu-2404-pip-installation-guide) |
| Changelog | [CHANGELOG.md](./CHANGELOG.md#changelog) |
| Demos | [pyAgxArm/demos](./pyAgxArm/demos) |

### Install

```shell
pip3 install python-can
```

`python-can` should be newer than `3.3.4`.

If you want to use this SDK on Windows, you must install the `python-can-agx-cando` plugin and use the `agx_cando` interface. The recommended way is:

```shell
pip3 install "git+https://github.com/agilexrobotics/python-can-agx-cando.git"
```

You can also clone the repository and install it locally:

```shell
git clone https://github.com/agilexrobotics/python-can-agx-cando.git
cd python-can-agx-cando
pip3 install .
```

Then install `pyAgxArm`. The recommended way is to install directly from GitHub:

```shell
pip3 install "git+https://github.com/agilexrobotics/pyAgxArm.git"
```

You can also clone the repository and install it locally:

```shell
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
pip3 install .
```

Ubuntu 24.04 users can also refer to:
[docs/ubuntu_24_04_pip_install.md](./docs/ubuntu_24_04_pip_install.md#ubuntu-2404-pip-installation-guide)

### Communication Setup

See:
[docs/can_user.md](./docs/can_user.md#can-module-manual)

### Quick Start

Assume default channel values in this quick-start example:

- Windows: `interface="agx_cando"`, `channel="0"`
- Linux: `interface="socketcan"`, `channel="can0"`
- macOS: `interface="slcan"`, `channel="/dev/ttyACM0"`

Prerequisites before running:

- Linux: activate CAN first (for example: `sudo ip link set can0 up type can bitrate 1000000`)
- Linux: you can also use our shell scripts in [CAN module manual - Activate a Single CAN Module](./docs/can_user.md#2-activate-a-single-can-module)
- macOS: grant serial permission first (`sudo chmod 777 /dev/ttyACM0`)

```python
import time
from platform import system
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW

# Nero firmware: <= 1.10 → NeroFW.DEFAULT; 1.11 → NeroFW.V111; 1.12 → NeroFW.V112; >= 1.20 → NeroFW.V120.
platform_system = system()
if platform_system == "Windows":
    interface = "agx_cando"
    channel = "0"
elif platform_system == "Linux":
    interface = "socketcan"
    channel = "can0"
elif platform_system == "Darwin":
    interface = "slcan"
    channel = "/dev/ttyACM0"
else:
    raise RuntimeError("pyAgxArm currently documents Linux `socketcan`, Windows `agx_cando`, and macOS `slcan`.")

cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=NeroFW.DEFAULT,
    interface=interface,
    channel=channel,
)
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    ja = robot.get_joint_angles()
    if ja is not None:
        print(ja.msg)
        print(ja.hz, ja.timestamp)
    time.sleep(0.005)
```

### Notes

- Activate CAN first and configure the correct bitrate before reading or controlling the arm.
- On Windows, `interface="agx_cando"` requires the separately installed `python-can-agx-cando` plugin.
- On macOS (`Darwin`), grant serial-port permission before using `interface="slcan"`.
- MIT single-joint control is an advanced feature; improper use may damage the robot.

### Contact

- [GitHub Discussions](https://github.com/agilexrobotics/pyAgxArm/discussions) (announcements, Q&A, ideas, and community discussion)
- [GitHub Issues](https://github.com/agilexrobotics/pyAgxArm/issues) (bugs, feature requests, documentation, and other tracked items)
- Email: [support@agilex.ai](mailto:support@agilex.ai) (firmware, host software, tool packages, and after-sales support)
- Discord: <https://discord.gg/wrKYTxwDBd> (real-time chat and community help)

---

# pyAgxArm 使用说明

> `pyAgxArm` 是 AgileX 机械臂与末端执行器的 Python SDK，支持 CAN 通信、状态读取、运动控制，以及 Piper、Nero、AgxGripper、Revo2 等设备的接口调用。

## 目录

- [切换到 English](#pyagxarm-user-guide)
- [简介](#pyagxarm-使用说明)
- [环境支持](#环境支持)
- [文档入口](#文档入口)
- [安装方法](#安装方法)
- [通信激活](#通信激活)
- [快速开始](#快速开始)
- [注意事项](#注意事项)
- [联系我们](#联系我们)

## 环境支持

- Ubuntu：`18.04 / 20.04 / 22.04 / 24.04`
- Windows：`10 / 11`
- macOS (Darwin)
- Python：`3.6` 及以上（目前适配至 `3.14`）

## 文档入口

| 说明 | 文档 |
| --- | --- |
| GitHub 讨论区 | [全部](https://github.com/agilexrobotics/pyAgxArm/discussions) · [公告](https://github.com/agilexrobotics/pyAgxArm/discussions/categories/announcements) · [综合](https://github.com/agilexrobotics/pyAgxArm/discussions/categories/general) · [问答](https://github.com/agilexrobotics/pyAgxArm/discussions/categories/q-a) |
| ROS | [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) |
| Piper API | [docs/piper/piper_api.md](./docs/piper/piper_api.md#piper-机械臂-api-使用文档) |
| Nero API | [docs/nero/nero_api.md](./docs/nero/nero_api.md#nero-机械臂-api-使用文档) |
| AgxGripper API | [docs/effector/agx_gripper/agx_gripper_api.md](./docs/effector/agx_gripper/agx_gripper_api.md#agxgripper-夹爪-api-使用文档) |
| Revo2 API | [docs/effector/revo2/revo2_api.md](./docs/effector/revo2/revo2_api.md#revo2-灵巧手-api-使用文档) |
| Revo2 Touch API | [docs/effector/revo2_touch/revo2_touch_api.md](./docs/effector/revo2_touch/revo2_touch_api.md#revo2-touch-灵巧手-api-使用文档) |
| CAN 模块手册 | [docs/can_user.md](./docs/can_user.md#can-模块使用手册) |
| Nero 首次使用 CAN 指南 | [docs/nero/first_time_user_guide_can.md](./docs/nero/first_time_user_guide_can.md#nero-首次使用指南can) |
| WSL2 USB-CAN 使用指南 | [docs/wsl2_usb_can_guide.md](./docs/wsl2_usb_can_guide.md#wsl2-ubuntu-2204-连接-usb-can-模块完整指南) |
| Ubuntu 24.04 pip 安装说明 | [docs/ubuntu_24_04_pip_install.md](./docs/ubuntu_24_04_pip_install.md#ubuntu-2404-安装第三方-pip-包的方法) |
| 更新日志 | [CHANGELOG.md](./CHANGELOG.md#更新日志) |
| 示例代码 | [pyAgxArm/demos](./pyAgxArm/demos) |

## 安装方法

```shell
pip3 install python-can
```

`python-can` 版本应高于 `3.3.4`。

如果你想在 Windows 上使用本 SDK，必须先安装 `python-can-agx-cando` 插件，并使用 `agx_cando` 接口。推荐直接安装：

```shell
pip3 install "git+https://github.com/agilexrobotics/python-can-agx-cando.git"
```

也可以克隆源码后在本地安装：

```shell
git clone https://github.com/agilexrobotics/python-can-agx-cando.git
cd python-can-agx-cando
pip3 install .
```

然后再安装 `pyAgxArm`。推荐直接通过 GitHub 安装：

```shell
pip3 install "git+https://github.com/agilexrobotics/pyAgxArm.git"
```

也可以克隆源码后在本地安装：

```shell
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
pip3 install .
```

Ubuntu 24.04 可参考：
[docs/ubuntu_24_04_pip_install.md](./docs/ubuntu_24_04_pip_install.md#ubuntu-2404-安装第三方-pip-包的方法)

## 通信激活

详见：
[docs/can_user.md](./docs/can_user.md#can-模块使用手册)

## 快速开始

本节快速示例按“默认通道”假设：

- Windows：`interface="agx_cando"`，`channel="0"`
- Linux：`interface="socketcan"`，`channel="can0"`
- macOS：`interface="slcan"`，`channel="/dev/ttyACM0"`

运行前前置条件：

- Linux：先激活 CAN（例如：`sudo ip link set can0 up type can bitrate 1000000`）
- Linux：也可使用我们提供的脚本，见 [CAN 模块手册 - 激活单个 CAN 模块](./docs/can_user.md#2-激活单个-can-模块)
- macOS：先给串口权限（`sudo chmod 777 /dev/ttyACM0`）

```python
import time
from platform import system
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW

# Nero 固件：≤ 1.10 选 NeroFW.DEFAULT；1.11 选 NeroFW.V111；1.12 选 NeroFW.V112；≥ 1.20 选 NeroFW.V120。
platform_system = system()
if platform_system == "Windows":
    interface = "agx_cando"
    channel = "0"
elif platform_system == "Linux":
    interface = "socketcan"
    channel = "can0"
elif platform_system == "Darwin":
    interface = "slcan"
    channel = "/dev/ttyACM0"
else:
    raise RuntimeError("pyAgxArm 当前公开说明包含 Linux `socketcan`、Windows `agx_cando` 与 macOS `slcan`。")

cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=NeroFW.DEFAULT,
    interface=interface,
    channel=channel,
)
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    ja = robot.get_joint_angles()
    if ja is not None:
        print(ja.msg)
        print(ja.hz, ja.timestamp)
    time.sleep(0.005)
```

## 注意事项

- 使用 CAN 协议时，需要先激活 CAN 设备并设置正确波特率。
- Windows 下使用 `interface="agx_cando"` 前，需要先单独安装 `python-can-agx-cando` 插件。
- macOS（`Darwin`）下使用 `interface="slcan"` 前，需要先给予串口权限。
- MIT 单关节控制属于高级功能，使用不当可能损坏机械臂。

## 联系我们

- [GitHub 讨论区](https://github.com/agilexrobotics/pyAgxArm/discussions)（公告、问答、想法交流与社区讨论）
- [GitHub Issues](https://github.com/agilexrobotics/pyAgxArm/issues)（缺陷、功能建议、文档等问题反馈与跟踪）
- 邮箱：[support@agilex.ai](mailto:support@agilex.ai)（固件、上位机、工具包及售后支持等）
- Discord：<https://discord.gg/wrKYTxwDBd>（实时聊天与社区互助）
