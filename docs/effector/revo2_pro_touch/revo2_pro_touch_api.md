# Revo2 Pro and Revo2 Touch API Documentation

> This document describes the shared **Revo2 Pro** and **Revo2 Touch** Python API (based on **`bc-stark-sdk`**). Revo2 Touch inherits the complete Revo2 Pro API and adds capacitive touch operations.

> **Note:** This driver bridges the official Revo2 hand SDK. For the full SDK API and usage, see [Advanced: run_sdk and client](#advanced-run_sdk-and-client).

- [Switch to 中文](#revo2-pro-与-revo2-touch-灵巧手-api-使用文档)
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Value Ranges and Finger Order](#value-ranges-and-finger-order)
- [Create Instance and Connect](#create-instance-and-connect)
- [Hand-Side Routing](#hand-side-routing)
- [SDK Types on the Driver](#sdk-types-on-the-driver)
- [Device Information](#device-information)
- [Device Configuration](#device-configuration)
- [Finger Motion Control](#finger-motion-control)
- [Motor Status and Settings](#motor-status-and-settings)
- [Capacitive Touch (Revo2 Touch only)](#capacitive-touch-revo2-touch-only)
- [LED / Buzzer / Vibration](#led--buzzer--vibration)
- [Device Discovery](#device-discovery)
- [Advanced: run_sdk and client](#advanced-run_sdk-and-client)
- [API Index](#api-index)

---

## Overview

Choose the model when initializing the end effector:

```python
effector = robot.OPTIONS.EFFECTOR.REVO2_PRO
# For Revo2 Touch instead: effector = robot.OPTIONS.EFFECTOR.REVO2_TOUCH
hand = robot.init_effector(effector)
```

> The driver wraps **bc-stark-sdk** and communicates with the hand through the conversion board.

| Capability | Revo2 Pro | Revo2 Touch |
| --- | --- | --- |
| Device, finger, motor, indicators and SDK bridge | Yes | Yes (inherited from Pro) |
| Capacitive touch APIs | No | Yes |

Read APIs often return `None` on failure; write APIs fail silently via `run_sdk` on timeout or error.

---

## Prerequisites

1. **Hardware (required):** **Revo2 Pro or Revo2 Touch** connected to the **arm bus through the Agilex conversion board**
2. **Connection:** Same bus / channel as the arm — see [CAN module manual](../../can_user.md)
3. **Python package:**

```bash
pip install bc-stark-sdk
```

4. **Read loop:** Call `robot.connect()` so the arm can send and receive hand data normally

---

## Quick Start

**Nero example:**

```python
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=NeroFW.V112,
    channel="can0",
)
robot = AgxArmFactory.create_arm(cfg)
hand = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2_PRO)
robot.connect()
```

**Piper example** (same effector API; only arm config differs):

```python
from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config

cfg = create_agx_arm_config(
    robot=ArmModel.PIPER,
    firmeware_version=PiperFW.V188,
    channel="can0",
)
robot = AgxArmFactory.create_arm(cfg)
hand = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2_TOUCH)
robot.connect()
```

Common usage after connect:

```python
hand.set_hand_side("right")  # optional; default auto-detects left/right

info = hand.get_device_info()
if info is not None:
    print(info.description)

hand.set_finger_positions([500] * 6)

robot.disconnect()
```

---

## Value Ranges and Finger Order

**Finger array order (length 6):** Thumb Flex, Thumb Aux, Index, Middle, Ring, Pinky — use `hand.FingerId.*`.

**Unified ranges (SDK):**

| Quantity | Range | Notes |
| --- | --- | --- |
| Position | 0 ~ 1000 | 0 = open, 1000 = closed |
| Speed | -1000 ~ +1000 | + close, − open, 0 = stop |
| Current | -1000 ~ +1000 | + close, − open |
| Duration (Revo2) | 1 ~ 2000 ms | Per-finger or batch APIs |

---

## Create Instance and Connect

```python
effector = robot.OPTIONS.EFFECTOR.REVO2_PRO  # or REVO2_TOUCH
hand = robot.init_effector(effector)
robot.connect()
```

**`EFFECTOR` constant:**

```python
REVO2_PRO: Final[Literal["revo2_pro"]] = "revo2_pro"
REVO2_TOUCH: Final[Literal["revo2_touch"]] = "revo2_touch"
```

> **Notes:**
> 1. Call `init_effector` only once per arm session.
> 2. Create the effector before `connect()` (recommended).
> 3. **Prerequisite:** Revo2 Pro or Revo2 Touch connected through the Agilex conversion board.

---

## Hand-Side Routing

### Properties

| Name | Type | Description |
| --- | --- | --- |
| `hand.hand_side` | `str \| None` | `"left"`, `"right"`, or `None` before auto-bind |
| `hand.slave_id` | `int \| None` | Bound hand device id, or `None` if not yet known |
| `hand.client` | `DeviceContext` | Raw bc-stark-sdk context (advanced) |

### `set_hand_side(side=None)`

| Parameter | Description |
| --- | --- |
| `"left"` / `"right"` | Pin routing; no failover |
| `None` | Auto mode: detect left/right hand, bind first responder, failover on error |

```python
hand.set_hand_side("right")   # pin right
hand.set_hand_side(None)      # back to auto
```

---

## SDK Types on the Driver

Re-exported on the driver class (no separate import needed):

| Attribute | Use |
| --- | --- |
| `hand.FingerId` | `Thumb`, `ThumbAux`, `Index`, `Middle`, `Ring`, `Pinky` |
| `hand.FingerUnitMode` | `Normalized`, `Physical` |
| `hand.LedColor` | `RGB`, `R`, `G`, `B`, … |
| `hand.LedMode` | `Blink`, `Blink2Hz`, `Keep`, … |
| `hand.LedInfo` | LED configuration object |
| `hand.MotorSettings` | Per-finger motor limits |

---

## Device Information

| Method | Returns | Description |
| --- | --- | --- |
| `get_device_info()` | `DeviceInfo \| None` | SN, firmware, hardware type — **call once after connect** |
| `get_device_sn()` | `str \| None` | Serial number |
| `get_device_fw_version()` | `str \| None` | Firmware string |
| `get_sku_type()` | `SkuType \| None` | SKU / hand side |
| `get_hand_type()` | `HandType \| None` | Left / right enum |
| `get_button_event()` | `ButtonPressEvent \| None` | Physical button event |

```python
info = hand.get_device_info()
if info is not None:
    print(info.serial_number, info.firmware_version, info.hardware_type)
```

---

## Device Configuration

| Method | Description |
| --- | --- |
| `get_auto_calibration_enabled()` → `bool \| None` | Query auto-calibration |
| `set_auto_calibration(enabled: bool)` | Enable / disable auto-calibration |
| `calibrate_position()` | Trigger manual position calibration |
| `reset_default_settings()` | Factory reset settings |
| `reboot()` | Reboot the hand |

---

## Finger Motion Control

### Single finger

| Method | Description |
| --- | --- |
| `set_finger_position(finger_id, position)` | Position 0~1000 |
| `set_finger_position_with_millis(finger_id, position, ms)` | Position + duration (Revo2) |
| `set_finger_position_with_speed(finger_id, position, speed)` | Position + speed (Revo2) |
| `set_finger_speed(finger_id, speed)` | Speed −1000~+1000 |
| `set_finger_current(finger_id, current)` | Current −1000~+1000 |

### All fingers (arrays length 6)

| Method | Description |
| --- | --- |
| `set_finger_positions(positions)` | Six positions |
| `set_finger_positions_and_durations(positions, durations)` | Six positions + six durations (Revo2) |
| `set_finger_positions_and_speeds(positions, speeds)` | Six positions + six speeds (Revo2) |
| `set_finger_speeds(speeds)` | Six speeds |
| `set_finger_currents(currents)` | Six currents |

### Readbacks

| Method | Returns |
| --- | --- |
| `get_finger_positions()` | `list[int] \| None` |
| `get_finger_speeds()` | `list[int] \| None` |
| `get_finger_currents()` | `list[int] \| None` |

```python
hand.set_finger_position(hand.FingerId.Index, 500)
hand.set_finger_positions_and_speeds([1000] * 6, [300] * 6)
pos = hand.get_finger_positions()
```

---

## Motor Status and Settings

| Method | Returns / action |
| --- | --- |
| `get_motor_status()` | `MotorStatusData \| None` (positions, speeds, currents, states) |
| `get_motor_state()` | `list[int] \| None` — running / idle / stalled codes |
| `get_finger_unit_mode()` / `set_finger_unit_mode(mode)` | Revo2 unit mode |
| `get_all_finger_settings()` | `list[MotorSettings] \| None` |
| `get_finger_settings(finger_id)` / `set_finger_settings(finger_id, settings)` | Per-finger settings |
| `get/set_finger_min_position(finger_id, …)` | Limit min position |
| `get/set_finger_max_position(finger_id, …)` | Limit max position |
| `get/set_finger_max_speed(finger_id, …)` | Limit max speed |
| `get/set_finger_max_current(finger_id, …)` | Limit max current |
| `get/set_finger_protected_current(finger_id, …)` | Stall-detection threshold (single) |
| `get_finger_protected_currents()` / `set_finger_protected_currents(currents)` | Stall threshold (all) |
| `get/set_thumb_aux_lock_current(lock_current)` | Thumb auxiliary lock current |

---

## Capacitive Touch (Revo2 Touch only)

These methods exist only on Revo2 Touch. Revo2 Pro intentionally does not expose touch APIs. Revo2 Touch provides **capacitive** tactile data (not Force3D / array pressure — use `run_sdk` for unwrapped SDK features if needed).

| Method | Description |
| --- | --- |
| `get_touch_sensor_enabled()` | Which fingertip sensors are enabled |
| `get_touch_sensor_fw_versions()` | Firmware version string per sensor |
| `get_touch_sensor_raw_data()` | `TouchRawData \| None` |
| `get_touch_sensor_status()` | `list[TouchFingerItem] \| None` — all fingers |
| `get_single_touch_sensor_status(index)` | One finger, `index` 0–4 |
| `touch_sensor_setup(enable_mask)` | Enable selected sensors |
| `touch_sensor_reset(reset_mask)` | Reset selected sensors |
| `touch_sensor_calibrate(calibrate_mask)` | Calibrate selected sensors |

```python
item = hand.get_single_touch_sensor_status(1)
```

---

## LED / Buzzer / Vibration

| Method | Description |
| --- | --- |
| `get_led_enabled()` / `set_led_enabled(enabled)` | LED switch |
| `get_led_info()` / `set_led_info(led_info)` | Color, mode, brightness |
| `get_buzzer_enabled()` / `set_buzzer_enabled(enabled)` | Buzzer switch |
| `get_vibration_enabled()` / `set_vibration_enabled(enabled)` | Vibration motor switch |

```python
led = hand.LedInfo(hand.LedColor.RGB, hand.LedMode.Blink2Hz)
hand.set_led_info(led)
```

---

## Device Discovery

### `scan_slave_ids(candidate_ids=None, *, timeout_ms=500) -> int | None`

Search for a responding hand on the conversion board (default: try left and right hand ids). Does **not** change `hand_side` binding.

```python
device_id = hand.scan_slave_ids()
print(device_id if device_id is not None else "no hand found")
```

---

## Advanced: run_sdk and client

### `run_sdk(fn, *args, **kwargs) -> T | None`

Runs any `hand.client.<method>` on the SDK asyncio thread with automatic left/right routing and auto/failover handling.

```python
# Read (same as hand.get_finger_positions())
pos = hand.run_sdk(hand.client.get_finger_positions)

# Write
hand.run_sdk(hand.client.set_finger_positions, [0, 500, 500, 500, 500, 500])

# Unwrapped SDK methods, e.g.:
# hand.run_sdk(hand.client.is_touch_hand)
```

Returns `None` on timeout or SDK error.

### `hand.client`

Direct access to **bc-stark-sdk** `DeviceContext`. Prefer wrapped driver methods; use `client` + `run_sdk` for APIs not exposed on the driver (DFU, action sequences, factory tools, etc.).

> For full **`hand.client`** API details, see the official documentation: [Revo2 Python SDK](https://www.brainco-hz.com/docs/revolimb-hand/revo2/python_sdk.html)

---

## API Index

Revo2 Pro exposes 59 public methods. Revo2 Touch exposes 67: all 59 inherited methods plus the 8 touch methods below.

| Category | Methods |
| --- | --- |
| Routing | `set_hand_side`, `scan_slave_ids`, `run_sdk` |
| Properties | `client`, `hand_side`, `slave_id` |
| Device info | `get_device_info`, `get_device_sn`, `get_device_fw_version`, `get_sku_type`, `get_hand_type`, `get_button_event` |
| Config | `get_auto_calibration_enabled`, `set_auto_calibration`, `calibrate_position`, `reset_default_settings`, `reboot` |
| Position | `set_finger_position`, `set_finger_position_with_millis`, `set_finger_position_with_speed`, `set_finger_positions`, `set_finger_positions_and_durations`, `set_finger_positions_and_speeds`, `get_finger_positions` |
| Speed | `set_finger_speed`, `set_finger_speeds`, `get_finger_speeds` |
| Current | `set_finger_current`, `set_finger_currents`, `get_finger_currents` |
| Status | `get_motor_status`, `get_motor_state` |
| Settings | `get/set_finger_unit_mode`, `get_all_finger_settings`, `get/set_finger_settings`, min/max position/speed/current, protected currents, thumb aux lock |
| Touch (Revo2 Touch only) | `get_touch_sensor_*`, `touch_sensor_setup/reset/calibrate` |
| Indicators | LED / buzzer / vibration get/set |

**Type aliases on class:** `FingerId`, `FingerUnitMode`, `LedColor`, `LedInfo`, `LedMode`, `MotorSettings`

---

# Revo2 Pro 与 Revo2 Touch 灵巧手 API 使用文档

> 本文档统一描述 **Revo2 Pro** 和 **Revo2 Touch** 的 Python API（基于 **bc-stark-sdk**）。Revo2 Touch 继承 Revo2 Pro 的全部接口，并扩展电容触觉操作。

> **提示：** 本 Driver 桥接了灵巧手官方 SDK。更多 SDK 能力与用法请参阅 [进阶：run_sdk 与 client](#进阶run_sdk-与-client)。

## 目录

- [切换到 English](#revo2-pro-and-revo2-touch-api-documentation)
- [概述](#概述)
- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [量程与手指顺序](#量程与手指顺序)
- [创建实例并连接](#创建实例并连接)
- [左右手路由](#左右手路由)
- [Driver 上的 SDK 类型](#driver-上的-sdk-类型)
- [设备信息](#设备信息)
- [设备配置](#设备配置)
- [手指运动控制](#手指运动控制)
- [电机状态与参数](#电机状态与参数)
- [电容触觉（仅-Revo2-Touch）](#电容触觉仅-revo2-touch)
- [LED / 蜂鸣器 / 振动](#led--蜂鸣器--振动)
- [设备发现](#设备发现)
- [进阶：run_sdk 与 client](#进阶run_sdk-与-client)
- [API 索引](#api-索引)

---

## 概述

初始化时选择对应型号：

```python
effector = robot.OPTIONS.EFFECTOR.REVO2_PRO
# Revo2 Touch 则使用：effector = robot.OPTIONS.EFFECTOR.REVO2_TOUCH
hand = robot.init_effector(effector)
```

> Driver 封装 **bc-stark-sdk**，经转换板与灵巧手通信。

| 能力 | Revo2 Pro | Revo2 Touch |
| --- | --- | --- |
| 设备、手指、电机、指示器和 SDK 桥接 | 支持 | 支持（继承 Pro） |
| 电容触觉接口 | 不支持 | 支持 |

读接口失败时多返回 `None`；写接口经 `run_sdk` 在超时/错误时静默失败。

---

## 环境要求

1. **硬件（必须）：** **Revo2 Pro 或 Revo2 Touch 经 Agilex 转换板接入机械臂总线**
2. **连接：** 与机械臂共用同一总线/通道，见 [CAN 模块手册](../../can_user.md)
3. **依赖：** `pip install bc-stark-sdk`
4. 必须 `robot.connect()` 启动读循环，才能正常收发灵巧手数据

---

## 快速开始

**Nero 示例：**

```python
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

cfg = create_agx_arm_config(robot=ArmModel.NERO, firmeware_version=NeroFW.V112, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
hand = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2_PRO)
robot.connect()
```

**Piper 示例**（末端 API 相同，仅机械臂配置不同）：

```python
from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.V188, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
hand = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2_TOUCH)
robot.connect()
```

连接后通用用法：

```python
hand.set_hand_side("right")  # 可选；默认自动探测左右手

info = hand.get_device_info()
if info is not None:
    print(info.description)

hand.set_finger_positions([500] * 6)

robot.disconnect()
```

---

## 量程与手指顺序

**六指数组顺序：** 大拇指Flex、大拇指Aux、食指、中指、无名指、小指 — 使用 `hand.FingerId.*`。

**统一量纲：**

| 量 | 范围 | 说明 |
| --- | --- | --- |
| 位置 | 0 ~ 1000 | 0 张开，1000 闭合 |
| 速度 | -1000 ~ +1000 | 正闭合，负张开 |
| 电流 | -1000 ~ +1000 | 正闭合，负张开 |
| 时长 | 1 ~ 2000 ms | Revo2 专用 API |

---

## 创建实例并连接

```python
effector = robot.OPTIONS.EFFECTOR.REVO2_PRO  # 或 REVO2_TOUCH
hand = robot.init_effector(effector)
robot.connect()
```

常量：`REVO2_PRO = "revo2_pro"`、`REVO2_TOUCH = "revo2_touch"`。

> 1. 每次连接会话只应调用一次 `init_effector`。  
> 2. 建议在 `connect()` 前创建末端。  
> 3. **前提：** Revo2 Pro 或 Revo2 Touch 经 Agilex 转换板接入机械臂总线。

---

## 左右手路由

| 属性 / 方法 | 说明 |
| --- | --- |
| `hand_side` | `"left"` / `"right"` / 未绑定时 `None` |
| `slave_id` | 当前绑定的手设备 id，未绑定时 `None` |
| `set_hand_side("left"\|"right")` | 固定左右手，不 failover |
| `set_hand_side(None)` | 自动探测并绑定，失败时切换另一侧 |

---

## Driver 上的 SDK 类型

| 属性 | 用途 |
| --- | --- |
| `FingerId` | 手指枚举 |
| `FingerUnitMode` | 归一化 / 物理单位 |
| `LedColor` / `LedMode` / `LedInfo` | LED |
| `MotorSettings` | 单指电机参数 |

---

## 设备信息

| 方法 | 说明 |
| --- | --- |
| `get_device_info()` | 设备信息（连接后建议先调一次） |
| `get_device_sn()` | 序列号 |
| `get_device_fw_version()` | 固件版本 |
| `get_sku_type()` / `get_hand_type()` | SKU / 左右手类型 |
| `get_button_event()` | 按键事件 |

---

## 设备配置

| 方法 | 说明 |
| --- | --- |
| `get_auto_calibration_enabled()` / `set_auto_calibration()` | 自动校准开关 |
| `calibrate_position()` | 手动位置校准 |
| `reset_default_settings()` | 恢复出厂参数 |
| `reboot()` | 重启 |

---

## 手指运动控制

**单指：** `set_finger_position`、`set_finger_position_with_millis`、`set_finger_position_with_speed`、`set_finger_speed`、`set_finger_current`。

**六指数组：** `set_finger_positions`、`set_finger_positions_and_durations`、`set_finger_positions_and_speeds`、`set_finger_speeds`、`set_finger_currents`。

**读取：** `get_finger_positions`、`get_finger_speeds`、`get_finger_currents`。

```python
hand.set_finger_positions_and_speeds([1000] * 6, [300] * 6)
```

---

## 电机状态与参数

| 方法 | 说明 |
| --- | --- |
| `get_motor_status()` | 位置/速度/电流/状态汇总 |
| `get_motor_state()` | 六指状态码 |
| `get/set_finger_unit_mode` | 单位模式 |
| `get_all_finger_settings` / `get/set_finger_settings` | 单指/全部参数 |
| `get/set_finger_min/max_position` | 位置限位 |
| `get/set_finger_max_speed` / `max_current` | 速度/电流上限 |
| `get/set_finger_protected_current(s)` | 堵转保护电流 |
| `get/set_thumb_aux_lock_current` | 拇指副指锁止电流 |

---

## 电容触觉（仅 Revo2 Touch）

以下方法仅 Revo2 Touch 提供，Revo2 Pro 不暴露触觉接口。Driver 封装的是 **电容触觉** 接口（不含 Force3D / 面阵压力；未封装能力可用 `run_sdk(hand.client.xxx)`）。

| 方法 | 说明 |
| --- | --- |
| `get_touch_sensor_enabled()` | 已启用的指尖传感器 |
| `get_touch_sensor_fw_versions()` | 各触觉传感器固件版本 |
| `get_touch_sensor_raw_data()` | 原始数据 |
| `get_touch_sensor_status()` | 五指状态列表 |
| `get_single_touch_sensor_status(index)` | 单指，`index` 为 0–4 |
| `touch_sensor_setup/reset/calibrate(mask)` | 启用 / 复位 / 校准指定传感器 |

```python
item = hand.get_single_touch_sensor_status(1)
```

---

## LED / 蜂鸣器 / 振动

| 方法 | 说明 |
| --- | --- |
| `get/set_led_enabled` | LED 开关 |
| `get/set_led_info` | 颜色、模式、亮度 |
| `get/set_buzzer_enabled` | 蜂鸣器 |
| `get/set_vibration_enabled` | 振动马达 |

---

## 设备发现

**`scan_slave_ids(candidate_ids=None, *, timeout_ms=500)`** — 在转换板上搜索有响应的灵巧手（默认尝试左右手），不改变当前 `hand_side` 绑定。

---

## 进阶：run_sdk 与 client

**`run_sdk(fn, *args, **kwargs)`** — 在 SDK 线程执行 `hand.client` 任意方法，自动处理左右手路由与 failover。

**`hand.client`** — 原始 `DeviceContext`；Driver 未封装的 SDK 能力（DFU、动作序列等）通过 `run_sdk` 调用。

```python
# 读（与 hand.get_finger_positions() 等价）
pos = hand.run_sdk(hand.client.get_finger_positions)

# 写
hand.run_sdk(hand.client.set_finger_positions, [0, 500, 500, 500, 500, 500])
```

失败或超时时返回 `None`。

> **`hand.client`** 的完整 API 说明请参阅官方文档：[Revo2 Python SDK](https://www.brainco-hz.com/docs/revolimb-hand/revo2/python_sdk.html)

---

## API 索引

Revo2 Pro 共 **59** 个公开方法；Revo2 Touch 共 **67** 个，即继承 59 个公共方法并扩展 8 个触觉方法。分类见 [English API Index](#api-index)。类属性：`FingerId`、`FingerUnitMode`、`LedColor`、`LedInfo`、`LedMode`、`MotorSettings`。
