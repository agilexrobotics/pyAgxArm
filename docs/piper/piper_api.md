# Piper API Documentation

> This document describes the `pyAgxArm` model-specific CAN APIs for Piper robotic arms, focusing on runtime data read/write, motion control, and advanced protocol parameters.

## Table of Contents

- [Switch to 中文](#piper-机械臂-api-使用文档)
- [Firmware Version](#firmware-version)
  - [Version List](#version-list)
  - [How to Choose](#how-to-choose)
- [Data Reading](#data-reading)
  - [MessageAbstract Return Value Overview](#messageabstract-return-value-overview)
  - [Get Arm Status — get_arm_status()](#get-arm-status--get_arm_status)
  - [Get Joint Angles — get_joint_angles()](#get-joint-angles--get_joint_angles)
  - [Get Flange Pose — get_flange_pose()](#get-flange-pose--get_flange_pose)
  - [Get Motor States — get_motor_states()](#get-motor-states--get_motor_states)
  - [Get Driver States — get_driver_states()](#get-driver-states--get_driver_states)
  - [Get Joint Enable Status — get_joint_enable_status()](#get-joint-enable-status--get_joint_enable_status)
  - [Get All Joint Enable Status List — get_joints_enable_status_list()](#get-all-joint-enable-status-list--get_joints_enable_status_list)
  - [Get Firmware Info — get_firmware()](#get-firmware-info--get_firmware)
- [Parameter Settings](#parameter-settings)
  - [Set Speed Percent — set_speed_percent()](#set-speed-percent--set_speed_percent)
  - [Set Installation Position — set_installation_pos()](#set-installation-position--set_installation_pos)
  - [Set Motion Mode — set_motion_mode()](#set-motion-mode--set_motion_mode)
  - [Set Payload — set_payload()](#set-payload--set_payload)
- [Leader-Follower Arm](#leader-follower-arm)
  - [Set Leader Mode — set_leader_mode()](#set-leader-mode--set_leader_mode)
  - [Set Follower Mode — set_follower_mode()](#set-follower-mode--set_follower_mode)
  - [Move Leader to Home — move_leader_to_home()](#move-leader-to-home--move_leader_to_home)
  - [Move Leader & Follower to Home — move_leader_follower_to_home()](#move-leader--follower-to-home--move_leader_follower_to_home)
  - [Restore Leader Drag Mode — restore_leader_drag_mode()](#restore-leader-drag-mode--restore_leader_drag_mode)
  - [Get Leader Joint Angles — get_leader_joint_angles()](#get-leader-joint-angles--get_leader_joint_angles)
- [Motion Control](#motion-control)
  - [Enable — enable()](#enable--enable)
  - [Disable — disable()](#disable--disable)
  - [Electronic Emergency Stop — electronic_emergency_stop()](#electronic-emergency-stop--electronic_emergency_stop)
  - [Reset — reset()](#reset--reset)
  - [Joint Motion — move_j()](#joint-motion--move_j)
  - [Joint Motion (Follower Mode) — move_js()](#joint-motion-follower-mode--move_js)
  - [Point-to-Point Motion — move_p()](#point-to-point-motion--move_p)
  - [Linear Motion — move_l()](#linear-motion--move_l)
  - [Arc Motion — move_c()](#arc-motion--move_c)
  - [Single Joint MIT Control — move_mit()](#single-joint-mit-control--move_mit)
- [CPV Motion and Parameters](#cpv-motion-and-parameters)
  - [CPV Command APIs](#cpv-command-apis)
  - [CPV Parameter Read APIs](#cpv-parameter-read-apis)
  - [CPV Parameter Write APIs](#cpv-parameter-write-apis)
- [Advanced Parameter Reading and Configuration](#advanced-parameter-reading-and-configuration)
  - [Get Joint Angle/Velocity Limits — get_joint_angle_vel_limits()](#get-joint-anglevelocity-limits--get_joint_angle_vel_limits)
  - [Get Joint Acceleration Limits — get_joint_acc_limits()](#get-joint-acceleration-limits--get_joint_acc_limits)
  - [Get Flange Velocity/Acceleration Limits — get_flange_vel_acc_limits()](#get-flange-velocityacceleration-limits--get_flange_vel_acc_limits)
  - [Get Crash Protection Rating — get_crash_protection_rating()](#get-crash-protection-rating--get_crash_protection_rating)
  - [Get Joint Assistance Rating — get_joint_assistance_rating()](#get-joint-assistance-rating--get_joint_assistance_rating)
  - [Calibrate Joint — calibrate_joint()](#calibrate-joint--calibrate_joint)
  - [Clear Joint Error — clear_joint_error()](#clear-joint-error--clear_joint_error)
  - [Set Joint Angle/Velocity Limits — set_joint_angle_vel_limits()](#set-joint-anglevelocity-limits--set_joint_angle_vel_limits)
  - [Set Joint Acceleration Limits — set_joint_acc_limits()](#set-joint-acceleration-limits--set_joint_acc_limits)
  - [Set Flange Velocity/Acceleration Limits — set_flange_vel_acc_limits()](#set-flange-velocityacceleration-limits--set_flange_vel_acc_limits)
  - [Set Crash Protection Rating — set_crash_protection_rating()](#set-crash-protection-rating--set_crash_protection_rating)
  - [Set Joint Assistance Rating — set_joint_assistance_rating()](#set-joint-assistance-rating--set_joint_assistance_rating)
  - [Reset Flange Limits to Default — set_flange_vel_acc_limits_to_default()](#reset-flange-limits-to-default--set_flange_vel_acc_limits_to_default)
  - [Reset Joint Limits to Default — set_joint_angle_vel_acc_limits_to_default()](#reset-joint-limits-to-default--set_joint_angle_vel_acc_limits_to_default)
  - [Set Link Velocity/Acceleration Periodic Feedback — set_links_vel_acc_period_feedback()](#set-link-velocityacceleration-periodic-feedback--set_links_vel_acc_period_feedback)

---

## Firmware Version

Piper series robotic arms may ship with different firmware versions. Some versions introduce protocol-level changes that affect parameter ranges, encoding precision, and internal scaling. The SDK uses the `firmeware_version` parameter in `create_agx_arm_config()` to select the matching driver, ensuring API behavior is consistent with the firmware running on the arm. The firmware version number used here is the software version number, such as `"S-V1.8-8"`.

### Version List

| SDK Version | Constant | Firmware Range | Key Differences |
| --- | --- | --- | --- |
| `"default"` | `PiperFW.DEFAULT` | ≤ S-V1.8-2 | MIT torque: joints 1-3 input range ±32 N·m, joints 4-6 range ±8 N·m; 8-bit encoding |
| `"v183"` | `PiperFW.V183` | S-V1.8-3 ~ S-V1.8-7 | MIT torque: all joints range ±8 N·m; 8-bit encoding |
| `"v188"` | `PiperFW.V188` | ≥ S-V1.8-8 | MIT torque: all joints range ±16 N·m; 12-bit encoding; CRC checksum removed; motion mode code changed |

### How to Choose

Check the firmware version on the arm's main controller, you can use the [get_firmware()](#get-firmware-info--get_firmware) method (format: **S-VX.X-X**), then pick the corresponding SDK version:

| Your Firmware | `firmeware_version` to Use | Constant |
| --- | --- | --- |
| S-V1.8-2 or earlier | `"default"` (or omit the parameter) | `PiperFW.DEFAULT` |
| S-V1.8-3 ~ S-V1.8-7 | `"v183"` | `PiperFW.V183` |
| S-V1.8-8 or later | `"v188"` | `PiperFW.V188` |

> **⚠️ Safety Warning:** Using the wrong firmware version may cause the SDK to send incorrectly encoded torque commands. In particular, sending v188 protocol data to an older firmware arm may result in **dangerous unexpected motion**. Always verify your firmware version before choosing the SDK version.

**Usage Example (recommended — use constants for IDE auto-complete):**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

# Firmware is S-V1.8-5 → falls in v183 ~ v187, use PiperFW.V183
cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.V183, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
```

Raw strings are also accepted (backward-compatible):

```python
cfg = create_agx_arm_config(robot="piper", firmeware_version="v183", channel="can0")
```

---

## Data Reading

### MessageAbstract Return Value Overview

Most read interfaces in this SDK return `MessageAbstract[T] | None`, with the following common fields:

| Field | Type | Description |
| --- | --- | --- |
| `ret.msg` | `T` | Message data body (e.g., `list[float]` or a feedback message struct) |
| `ret.hz` | `float` | Receive frequency of this message type (SDK statistics), unit: Hz |
| `ret.timestamp` | `float` | Message timestamp (SDK recorded), unit: s |

---

### Get Arm Status — `get_arm_status()`

**Description:** Read the overall status feedback of the robotic arm (control mode, motion mode, emergency stop / error status, trajectory point number, etc.).

**Function Definition:**

```python
get_arm_status(self) -> MessageAbstract[ArmMsgFeedbackStatus] | None
```

**Return Value:** `MessageAbstract[ArmMsgFeedbackStatus] | None`

**Message Fields (`.msg`):**

| Field | Type | Description |
| --- | --- | --- |
| `ctrl_mode` | `int` | Control mode enum (see meanings below) |
| `arm_status` | `int` | Robotic arm status enum (see meanings below) |
| `mode_feedback` | `int` | Mode feedback enum (see meanings below) |
| `teach_status` | `int` | Teaching state enum (see meanings below) |
| `motion_status` | `int` | Motion status enum (see meanings below) |
| `trajectory_num` | `int` | Trajectory point number (feedback in offline trajectory mode) |
| `err_status` | `object` | Error status bitfield converted to boolean flags (see meanings below) |

**Enum meanings for `ArmMsgFeedbackStatus.msg`:**

`ctrl_mode` (control mode):
- `0x00` STANDBY: Standby mode
- `0x01` CAN_CTRL: CAN instruction control
- `0x02` TEACHING_MODE: Teaching mode
- `0x03` ETHERNET_CONTROL_MODE: Ethernet control mode
- `0x04` WIFI_CONTROL_MODE: Wi-Fi control mode
- `0x05` REMOTE_CONTROL_MODE: Remote control mode
- `0x06` LINKAGE_TEACHING_INPUT_MODE: Linkage teaching input mode
- `0x07` OFFLINE_TRAJECTORY_MODE: Offline trajectory mode
- `0xFF` UNKNOWN

`arm_status` (robot arm status):
- `0x00` NORMAL
- `0x01` EMERGENCY_STOP
- `0x02` NO_SOLUTION
- `0x03` SINGULARITY_POINT
- `0x04` TARGET_POS_EXCEEDS_LIMIT
- `0x05` JOINT_COMMUNICATION_ERR
- `0x06` JOINT_BRAKE_NOT_RELEASED
- `0x07` COLLISION_OCCURRED
- `0x08` OVERSPEED_DURING_TEACHING_DRAG
- `0x09` JOINT_STATUS_ERR
- `0x0A` OTHER_ERR
- `0x0B` TEACHING_RECORD
- `0x0C` TEACHING_EXECUTION
- `0x0D` TEACHING_PAUSE
- `0x0E` MAIN_CONTROLLER_NTC_OVER_TEMPERATURE
- `0x0F` RELEASE_RESISTOR_NTC_OVER_TEMPERATURE
- `0xFF` UNKNOWN

`mode_feedback` (current motion mode feedback):
- `0x00` MOVE_P
- `0x01` MOVE_J
- `0x02` MOVE_L
- `0x03` MOVE_C
- `0x04` MOVE_MIT (Piper firmware < v188; use `PiperFW.DEFAULT` / `PiperFW.V183`)
- `0x05` MOVE_CPV
- `0x06` MOVE_MIT (Piper firmware >= v188; use `PiperFW.V188`)
- `0xFF` UNKNOWN

`teach_status` (teaching state):
- `0x00` DISABLED
- `0x01` START_RECORDING
- `0x02` STOP_RECORDING
- `0x03` EXECUTE_TRAJECTORY
- `0x04` PAUSE_EXECUTION
- `0x05` RESUME_EXECUTION
- `0x06` TERMINATE_EXECUTION
- `0x07` MOVE_TO_START
- `0xFF` UNKNOWN

`motion_status`:
- `0x00` REACH_TARGET_POS_SUCCESSFULLY
- `0x01` REACH_TARGET_POS_FAILED
- `0xFF` UNKNOWN

`err_status` (16-bit error code -> boolean flags):
- `msg.err_code`: original 16-bit error code integer (0~65535).
- `msg.err_status.joint_i_angle_limit` (`i=1..6`): `True` means joint i angle limit exceeded.
- `msg.err_status.communication_status_joint_i` (`i=1..6`): `True` means communication exception on joint i.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    arm_status = robot.get_arm_status()
    if arm_status is not None:
        print(arm_status.msg)
        print(arm_status.hz, arm_status.timestamp)
    time.sleep(0.02)
```

---

### Get Joint Angles — `get_joint_angles()`

**Description:** Get the current angle of each joint.

**Function Definition:**

```python
get_joint_angles(self) -> MessageAbstract[list[float]] | None
```

**Return Value:** `MessageAbstract[list[float]] | None`

`.msg` is a `list[float]` of length 6: `[j1, j2, j3, j4, j5, j6]`, unit: **rad**.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    ja = robot.get_joint_angles()
    if ja is not None:
        print(ja.msg)
        print(ja.hz, ja.timestamp)
    time.sleep(0.005)
```

---

### Get Flange Pose — `get_flange_pose()`

**Description:** Get the end flange pose.

> **Terminology:** `flange` refers to the mounting flange / connection face of the last link (end link) of the robotic arm. It is the mechanical mounting interface for tools / end effectors.

**Function Definition:**

```python
get_flange_pose(self) -> MessageAbstract[list[float]] | None
```

**Return Value:** `MessageAbstract[list[float]] | None`

`.msg` is a `list[float]` of length 6: `[x, y, z, roll, pitch, yaw]`

- `x, y, z`: Position coordinates (unit: m)
- `roll, pitch, yaw`: Euler angles (unit: rad, corresponding to rotation around X/Y/Z axes respectively)

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    fp = robot.get_flange_pose()
    if fp is not None:
        print(fp.msg)
        print(fp.hz, fp.timestamp)
    time.sleep(0.005)
```

### Get Motor States — `get_motor_states()`

**Description:** Read the high-speed motor feedback (position / velocity / current / torque) for a specified joint.

**Function Definition:**

```python
get_motor_states(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
) -> MessageAbstract[ArmMsgFeedbackHighSpd] | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index, range: `1~6` |

**Return Value:** `MessageAbstract[ArmMsgFeedbackHighSpd] | None`

**Message Fields (`.msg`):**

| Field | Type | Description |
| --- | --- | --- |
| `position` | `float` | Motor position (rad) |
| `velocity` | `float` | Motor velocity (rad/s) |
| `current` | `float` | Motor current (A) |
| `torque` | `float` | Motor torque (N·m) |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

ms = robot.get_motor_states(1)
if ms is not None:
    print(ms.msg.position, ms.msg.velocity, ms.msg.current, ms.msg.torque)
    print(ms.hz, ms.timestamp)
```

---

### Get Driver States — `get_driver_states()`

**Description:** Read the low-speed driver feedback (voltage / temperature / bus current / driver status bits, etc.) for a specified joint.

**Function Definition:**

```python
get_driver_states(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
) -> MessageAbstract[ArmMsgFeedbackLowSpd] | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index, range: `1~6` |

**Return Value:** `MessageAbstract[ArmMsgFeedbackLowSpd] | None`

**Message Fields (`.msg`):**

| Field | Type | Description |
| --- | --- | --- |
| `vol` | `float` | Driver voltage |
| `foc_temp` | `float` | Driver temperature (°C) |
| `motor_temp` | `float` | Motor temperature (°C) |
| `bus_current` | `float` | Bus current (A) |
| `foc_status` | `object` | Driver status bits (under-voltage / over-temperature / over-current / collision / disabled / stall, etc.) |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

ds = robot.get_driver_states(1)
if ds is not None:
    print(ds.msg.vol, ds.msg.foc_temp, ds.msg.motor_temp, ds.msg.bus_current)
    print(ds.msg.foc_status.driver_enable_status)
    print(ds.hz, ds.timestamp)
```

---

### Get Joint Enable Status — `get_joint_enable_status()`

**Description:** Get the enable status of a specified joint motor.

**Function Definition:**

```python
get_joint_enable_status(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255],
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index: `1~6` queries a single joint; `255` queries all joints (internally aggregated using `all([...])`) |

**Return Value:** `bool` — `True` means enabled, `False` means not enabled or no feedback available.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

if robot.get_joint_enable_status(1):
    print("Joint 1 motor is enabled")
```

---

### Get All Joint Enable Status List — `get_joints_enable_status_list()`

**Description:** Read the enable status list of all joint motors (in order of joints 1~6).

**Function Definition:**

```python
get_joints_enable_status_list(self) -> list[bool]
```

**Return Value:** `list[bool]`

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

print(robot.get_joints_enable_status_list())
```

---

### Get Firmware Info — `get_firmware()`

**Description:** Read the robotic arm firmware information (software version / hardware version / production date, etc.). This interface sends a query frame and waits for the corresponding feedback.

**Function Definition:**

```python
get_firmware(
    self,
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> dict | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `timeout` | `float` | Timeout for waiting for feedback (seconds), default `1.0`; `0.0` means non-blocking |
| `min_interval` | `float` | Minimum request interval (seconds), default `1.0` |

**Return Value:** `dict | None`

Common fields:

| Key | Type | Description |
| --- | --- | --- |
| `software_version` | `str` | Software version (e.g., `S-V1.8-2`) |
| `hardware_version` | `str` | Hardware version (e.g., `H-V1.2-1`) |
| `production_date` | `str` | Production date (e.g., `250925`) |
| `node_type` | `str` | Node type |
| `node_number` | `int` | Node number |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

fw = robot.get_firmware()
if fw is not None:
    print(fw)
```

---

## Parameter Settings

### Set Speed Percent — `set_speed_percent()`

**Description:** Set the running speed percentage of the robotic arm in position-velocity mode, applicable to `move_j` / `move_p` / `move_l` / `move_c`.

**Function Definition:**

```python
set_speed_percent(self, percent: int = 100) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `percent` | `int` | Running speed percentage, range `[0, 100]`, default `100` |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_speed_percent(100)
```

---

### Set Installation Position — `set_installation_pos()`

**Description:** Set the installation position of the robotic arm. Supports horizontal, left-facing, and right-facing orientations.

**Function Definition:**

```python
set_installation_pos(
    self,
    pos: Literal["horizontal", "left", "right"] = "horizontal",
) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `pos` | `str` | Installation orientation, valid values: `'horizontal'` / `'left'` / `'right'`, default: `'horizontal'` (recommended to use `robot.OPTIONS.INSTALLATION_POS.xxx` constants) |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_installation_pos(robot.OPTIONS.INSTALLATION_POS.HORIZONTAL)
```

---

### Set Motion Mode — `set_motion_mode()`

**Description:** Set the motion mode.

| Mode | Type | Description |
| --- | --- | --- |
| `move_p` / `move_j` / `move_l` / `move_c` | **Position-velocity mode** | The lower layer smooths received messages to ensure continuous and stable motion |
| `move_mit` / `move_js` | **MIT motor pass-through mode** | The lower layer only forwards messages with **no smoothing**, suitable for direct motor control scenarios |

> **Tip:** When calling any `move_*` motion command, the system **automatically switches to the corresponding motion mode**, so there is usually **no need to manually call `set_motion_mode()`**.

**Function Definition:**

```python
set_motion_mode(
    self,
    motion_mode: Literal["p", "j", "l", "c", "mit", "js"] = "p",
) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `motion_mode` | `str` | Motion mode, valid values: `'p'` / `'j'` / `'l'` / `'c'` / `'mit'` / `'js'`, default: `'p'` (recommended to use `robot.OPTIONS.MOTION_MODE.xxx` constants) |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.J)
```

---

### Set Payload — `set_payload()`

**Description:** Set the robotic arm payload.

**Function Definition:**

```python
set_payload(
    self,
    load: Literal['empty', 'half', 'full'] = 'empty',
    timeout: float = 1.0,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `load` | `str` | Payload level, valid values: `'empty'` (no load) / `'half'` (half load) / `'full'` (full load), default: `'empty'` (recommended to use `robot.OPTIONS.PAYLOAD.xxx` constants) |
| `timeout` | `float` | Timeout for waiting for feedback (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates that the command acknowledgement was received, but does not guarantee the setting was successful.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_payload(robot.OPTIONS.PAYLOAD.FULL)
```

---

## Leader-Follower Arm

### Set Leader Mode — `set_leader_mode()`

**Description:** Set the robotic arm to **leader zero-force drag mode** (the "leader" in a leader-follower coordination scenario). After entering this mode, the leader arm is typically in a draggable / zero-force drag state.

> **Tip:** This mode is used for leader-follower arm linkage / teaching scenarios. If using a single arm only, this interface can be ignored.

**Function Definition:**

```python
set_leader_mode(self) -> None
```

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_leader_mode()
```

---

### Set Follower Mode — `set_follower_mode()`

**Description:** Set the robotic arm to **follower controlled mode** (the "follower" in a leader-follower coordination scenario). The follower arm follows the leader arm's control / commands. Can be used together with `set_leader_mode()`.

**Function Definition:**

```python
set_follower_mode(self) -> None
```

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_follower_mode()
```

---

### Move Leader to Home — `move_leader_to_home()`

**Description:** Move the leader arm back to the Home pose. After completion, it is recommended to call `restore_leader_drag_mode()` to restore the leader arm to the "zero-force drag" state.

**Function Definition:**

```python
move_leader_to_home(self) -> None
```

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# robot.set_leader_mode()
robot.move_leader_to_home()
# robot.restore_leader_drag_mode()
```

---

### Move Leader & Follower to Home — `move_leader_follower_to_home()`

**Description:** Move the leader and follower arms back to the Home pose together. After completion, it is recommended to call `restore_leader_drag_mode()` to restore the leader arm to the "zero-force drag" state.

**Function Definition:**

```python
move_leader_follower_to_home(self) -> None
```

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# robot.set_leader_mode()
robot.move_leader_follower_to_home()
# robot.restore_leader_drag_mode()
```

---

### Restore Leader Drag Mode — `restore_leader_drag_mode()`

**Description:** Restore the leader arm to the "zero-force drag" state, typically used after `move_leader_to_home()` or `move_leader_follower_to_home()`.

**Function Definition:**

```python
restore_leader_drag_mode(self) -> None
```

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# robot.set_leader_mode()
# robot.move_leader_to_home()
robot.restore_leader_drag_mode()
```

---

### Get Leader Joint Angles — `get_leader_joint_angles()`

**Description:** Get the leader arm joint angle message, used for controlling the follower arm.

**Function Definition:**

```python
get_leader_joint_angles(self) -> MessageAbstract[list[float]] | None
```

**Return Value:** `MessageAbstract[list[float]] | None`

`.msg` is a `list[float]` of length 6: `[j1, j2, j3, j4, j5, j6]`, unit: **rad**.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_leader_mode()

while True:
    mja = robot.get_leader_joint_angles()
    if mja is not None:
        print(mja.msg)
        print(mja.hz, mja.timestamp)
    time.sleep(0.005)
```

---

## Motion Control

### Enable — `enable()`

**Description:** Power on and enable the robotic arm.

**Function Definition:**

```python
enable(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index: `1~6` enables a single joint; `255` enables all joints, default: `255` |

**Return Value:** `bool` — `True` means enable was successful.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)
```

---

### Disable — `disable()`

**Description:** Power off the robotic arm.

> **Warning:** When executing this command, if the robotic arm joints are in a raised position, they will **fall immediately**. Make sure the robotic arm is in a safe state before using this command.

**Function Definition:**

```python
disable(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index: `1~6` disables a single joint; `255` disables all joints, default: `255` |

**Return Value:** `bool` — `True` means disable was successful.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.disable():
    time.sleep(0.01)
```

---

### Electronic Emergency Stop — `electronic_emergency_stop()`

**Description:** Set the robotic arm to emergency stop state. If the joints are in a raised position when executing, the arm will **slowly descend with constant damping** (will not fall immediately). After emergency stop, you can use `reset()` to reset the arm.

**Function Definition:**

```python
electronic_emergency_stop(self) -> None
```

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.electronic_emergency_stop()
```

---

### Reset — `reset()`

**Description:** Reset the robotic arm mode and immediately power off the arm.

> **Warning:** When executing this command, if the robotic arm joints are in a raised position, they will **fall immediately**. Make sure the robotic arm is in a safe state before using this command.

**Function Definition:**

```python
reset(self) -> None
```

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.reset()
```

---

### Joint Motion — `move_j()`

**Description:** Joint position-velocity control mode, set target angles for each joint.

**Function Definition:**

```python
move_j(self, joints: list[float]) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joints` | `list[float]` | Target angle array of length 6 `[j1, j2, j3, j4, j5, j6]` (unit: rad, precision: 1.74532925199e-5). Joint limits depend on robot variant configuration |

> **Note:** Consecutive execution of this command will overwrite the previous target value.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_j([0, 0.4, -0.4, 0, -0.4, 0])

# Wait for motion to finish (with 5s timeout)
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("Reached target position")
        break
    if time.monotonic() - start_t > 5.0:
        print("Wait for motion timeout (5s)")
        break
    time.sleep(0.1)
```

---

### Joint Motion (Follower Mode) — `move_js()`

**Description:** Switch the robotic arm to **JS (follower) mode** (MIT pass-through mode) and send joint target angles. Compared to `move_j`, `move_js` is more oriented toward "fast response" control: **no smoothing, no trajectory planning**; the controller / driver will respond to the target angle as fast as possible.

**Function Definition:**

```python
move_js(self, joints: list[float]) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joints` | `list[float]` | Target angle array of length 6 `[j1, j2, j3, j4, j5, j6]` (unit: rad, precision: 1.74532925199e-5). Joint limits depend on robot variant configuration |

> **Warning: Extremely high risk**
>
> 1. This mode may cause **impact, oscillation, instability**, and other risks. Use only after fully evaluating safety and control stability, and ensure emergency stop is available at all times.
> 2. **No smoothing, no trajectory planning** — the controller / driver attempts to reach the target as fast as possible, which may cause impact and oscillation.
> 3. Consecutive execution of this command will overwrite the previous target value.
> 4. Due to the faster response, joint control force is lower compared to position-velocity mode, and stiffness will also be lower.
> 5. On older firmware versions (below `S-V1.8-5`), if the robotic arm is currently in follower mode and you want to switch to position-velocity control mode, you need to first execute `robot.reset()` (the arm will reset and power off), then execute `move_j` for normal control.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.move_js([0, 0.4, -0.4, 0, -0.4, 0])
```

---

### Point-to-Point Motion — `move_p()`

**Description:** Send a target flange pose. The robotic arm performs joint angle inverse kinematics based on the current joint positions and target pose, then moves accordingly.

**Function Definition:**

```python
move_p(self, pose: list[float]) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `pose` | `list[float]` | Target pose `[x, y, z, roll, pitch, yaw]`: `x, y, z` are position (m, precision: 1e-6); `roll, pitch, yaw` are Euler angles (rad, precision: 1.74532925199e-5). Orientation range: `roll` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]`, `yaw` ∈ `[-π, π]` |

> **Note:** Consecutive execution of this command will overwrite the previous target value.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_p([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])

# Wait for motion to finish (with 5s timeout)
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("Reached target position")
        break
    if time.monotonic() - start_t > 5.0:
        print("Wait for motion timeout (5s)")
        break
    time.sleep(0.1)
```

---

### Linear Motion — `move_l()`

**Description:** Send a target flange pose. The robotic arm performs linear trajectory planning based on the current pose and target pose.

**Function Definition:**

```python
move_l(self, pose: list[float]) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `pose` | `list[float]` | Target pose `[x, y, z, roll, pitch, yaw]`: `x, y, z` are position (m, precision: 1e-6); `roll, pitch, yaw` are Euler angles (rad, precision: 1.74532925199e-5). Orientation range: `roll` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]`, `yaw` ∈ `[-π, π]` |

> **Note:** Although consecutive execution of this command can overwrite the previous target, since the lower layer needs to re-plan a linear trajectory each time a new point is received, **this command cannot be used to continuously send target points**.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_l([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])

# Wait for motion to finish (with 5s timeout)
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("Reached target position")
        break
    if time.monotonic() - start_t > 5.0:
        print("Wait for motion timeout (5s)")
        break
    time.sleep(0.1)
```

---

### Arc Motion — `move_c()`

**Description:** Perform arc trajectory planning and execution using three target flange poses: "start / midpoint / end".

**Function Definition:**

```python
move_c(
    self,
    start_pose: list[float],
    mid_pose: list[float],
    end_pose: list[float],
) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `start_pose` | `list[float]` | Start pose `[x, y, z, roll, pitch, yaw]` (m / rad). Orientation range: `roll` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]`, `yaw` ∈ `[-π, π]` |
| `mid_pose` | `list[float]` | Midpoint pose `[x, y, z, roll, pitch, yaw]` (m / rad). Orientation range: `roll` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]`, `yaw` ∈ `[-π, π]` |
| `end_pose` | `list[float]` | End pose `[x, y, z, roll, pitch, yaw]` (m / rad). Orientation range: `roll` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]`, `yaw` ∈ `[-π, π]` |

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
sp = [0.2, 0.0, 0.3, 0.0, 1.5708, 0.0]
mp = [0.2, 0.05, 0.35, 0.0, 1.5708, 0.0]
ep = [0.2, 0.0, 0.4, 0.0, 1.5708, 0.0]
robot.move_c(sp, mp, ep)

# Wait for motion to finish (with 5s timeout)
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("Reached target position")
        break
    if time.monotonic() - start_t > 5.0:
        print("Wait for motion timeout (5s)")
        break
    time.sleep(0.1)
```

---

### Single Joint MIT Control — `move_mit()`

**Description:** Use the joint driver's low-level MIT control interface to control a single joint motor. This enables current-simulated torque control.

The controller conceptually computes a reference torque:

$$T_{\text{ref}} = k_p \cdot (p_{\text{des}} - p) + k_d \cdot (v_{\text{des}} - v) + T_{\text{ff}}$$

where \(p/v\) are the measured joint position / velocity.

**Typical usage recommendations:**

| Control Method | Parameter Settings | Description |
| --- | --- | --- |
| **Velocity control** | `kp = 0`, `kd ≠ 0` | Primarily controlled via `v_des` |
| **Torque control** | `kp = 0`, `kd = 0` | Primarily controlled via `t_ff` |
| **Position control** | `kp ≠ 0`, `kd ≠ 0` | Not recommended to set `kd` to 0; increasing damping appropriately can reduce oscillation risk |

> **Warning:** MIT is a relatively low-level control interface. Improper parameters may cause **impact / oscillation / instability**. It is recommended to start with small gains for tuning and use under safe conditions.

**Function Definition:**

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

**Parameters (common to all versions):**

| Name | Type | Range | Unit | Default | Precision |
| --- | --- | --- | --- | --- | --- |
| `joint_index` | `int` | `1~6` | — | — | — |
| `p_des` | `float` | `[-12.5, 12.5]` | rad | `0.0` | 3.815e-4 |
| `v_des` | `float` | `[-45.0, 45.0]` | rad/s | `0.0` | 2.198e-2 |
| `kp` | `float` | `[0.0, 500.0]` | — | `10.0` | 1.221e-1 |
| `kd` | `float` | `[-5.0, 5.0]` | — | `0.8` | 2.442e-3 |

**`t_ff` parameter differs by firmware version:**

| Version | Joint | `t_ff` Range (N·m) | Encoding Bits | Precision (N·m) |
| --- | --- | --- | --- | --- |
| `default` (≤ v182) | 1-3 | `[-32.0, 32.0]` | 8 | 2.510e-1 |
| `default` (≤ v182) | 4-6 | `[-8.0, 8.0]` | 8 | 6.275e-2 |
| `v183` (v183 ~ v187) | 1-6 | `[-8.0, 8.0]` | 8 | 6.275e-2 |
| `v188` (≥ v188) | 1-6 | `[-16.0, 16.0]` | 12 | 7.813e-3 |

> **Note:** Consecutive execution of this command will overwrite the previous target value.
>
> The correct firmware version must be set via `create_agx_arm_config(firmeware_version=...)`. See [Firmware Version](#firmware-version) for details.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

# For firmware >= S-V1.8-8, use "v188"
cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.V188, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

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

---

## CPV Motion and Parameters

CPV mode provides direct joint **position / velocity command** and parameter read/write APIs.  
Calling CPV APIs will internally switch to CPV motion mode when needed (`set_motion_mode(MOVE_CPV)`).

### CPV Command APIs

| API | Signature | Description |
| --- | --- | --- |
| `move_cpv_pos` | `move_cpv_pos(self, joint_index: Literal[1, 2, 3, 4, 5, 6], pos: float) -> None` | Send CPV position command (rad). If outside joint limit, SDK clamps and logs warning. |
| `move_cpv_vel` | `move_cpv_vel(self, joint_index: Literal[1, 2, 3, 4, 5, 6], vel: float) -> None` | Send CPV velocity command (rad/s). |

### CPV Parameter Read APIs

All read APIs support `timeout` and `min_interval`, and return `float | None`.

| API | Unit / Meaning |
| --- | --- |
| `get_cpv_pos(joint_index, timeout=1.0, min_interval=1.0)` | Joint position (rad) |
| `get_cpv_vel(joint_index, timeout=1.0, min_interval=1.0)` | Joint velocity (rad/s) |
| `get_cpv_acc(joint_index, timeout=1.0, min_interval=1.0)` | Acceleration (rad/s^2) |
| `get_cpv_dcc(joint_index, timeout=1.0, min_interval=1.0)` | Deceleration (rad/s^2) |
| `get_cpv_cv(joint_index, timeout=1.0, min_interval=1.0)` | Contour/profile velocity (rad/s) |
| `get_cpv_pp(joint_index, timeout=1.0, min_interval=1.0)` | Position-loop proportional gain |
| `get_cpv_kp(joint_index, timeout=1.0, min_interval=1.0)` | Velocity-loop proportional gain |
| `get_cpv_ki(joint_index, timeout=1.0, min_interval=1.0)` | Velocity-loop integral gain |

### CPV Parameter Write APIs

Write APIs are **ACK + read-back verified** and return `bool`.

| API | Description |
| --- | --- |
| `set_cpv_acc(joint_index, acc, timeout=1.0)` | Set CPV acceleration parameter |
| `set_cpv_dcc(joint_index, dcc, timeout=1.0)` | Set CPV deceleration parameter |
| `set_cpv_cv(joint_index, cv, timeout=1.0)` | Set CPV contour/profile velocity parameter |
| `set_cpv_pp(joint_index, pp, timeout=1.0)` | Set CPV position-loop proportional gain |
| `set_cpv_kp(joint_index, kp, timeout=1.0)` | Set CPV velocity-loop proportional gain |
| `set_cpv_ki(joint_index, ki, timeout=1.0)` | Set CPV velocity-loop integral gain |

**Quick Example:**

```python
ok = robot.set_cpv_acc(joint_index=1, acc=2.0)
print("set_cpv_acc:", ok)
print("cpv_acc =", robot.get_cpv_acc(joint_index=1))
robot.move_cpv_vel(joint_index=1, vel=0.2)
```

---

## Advanced Parameter Reading and Configuration

### Get Joint Angle/Velocity Limits — `get_joint_angle_vel_limits()`

**Description:** Query the angle limits and velocity limits of a specified joint (feedback from the controller).

**Function Definition:**

```python
get_joint_angle_vel_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd] | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index, range: `1~6` |
| `timeout` | `float` | Timeout for waiting for feedback (seconds), default `1.0`; `0.0` means non-blocking |
| `min_interval` | `float` | Minimum request interval (seconds), default `1.0` |

**Return Value:** `MessageAbstract[ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd] | None`

**Message Fields (`.msg`):**

| Field | Type | Description |
| --- | --- | --- |
| `min_angle_limit` | `float` | Minimum angle limit (rad) |
| `max_angle_limit` | `float` | Maximum angle limit (rad) |
| `min_joint_spd` | `float` | Minimum joint velocity limit (rad/s) |
| `max_joint_spd` | `float` | Maximum joint velocity limit (rad/s) |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

limit = robot.get_joint_angle_vel_limits(1)
if limit is not None:
    print(limit.msg.min_angle_limit, limit.msg.max_angle_limit)
    print(limit.msg.min_joint_spd, limit.msg.max_joint_spd)
```

---

### Get Joint Acceleration Limits — `get_joint_acc_limits()`

**Description:** Query the maximum acceleration limit of a specified joint.

**Function Definition:**

```python
get_joint_acc_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[ArmMsgFeedbackCurrentMotorMaxAccLimit] | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index, range: `1~6` |
| `timeout` | `float` | Timeout for waiting for feedback (seconds), default `1.0`; `0.0` means non-blocking |
| `min_interval` | `float` | Minimum request interval (seconds), default `1.0` |

**Return Value:** `MessageAbstract[ArmMsgFeedbackCurrentMotorMaxAccLimit] | None`

**Message Fields (`.msg`):**

| Field | Type | Description |
| --- | --- | --- |
| `max_joint_acc` | `float` | Maximum joint acceleration limit (rad/s²) |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

limit = robot.get_joint_acc_limits(1)
if limit is not None:
    print(limit.msg.max_joint_acc)
    print(limit.hz, limit.timestamp)
```

---

### Get Flange Velocity/Acceleration Limits — `get_flange_vel_acc_limits()`

**Description:** Query the end-effector maximum linear velocity / angular velocity and linear acceleration / angular acceleration limits.

**Function Definition:**

```python
get_flange_vel_acc_limits(
    self,
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam] | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `timeout` | `float` | Timeout for waiting for feedback (seconds), default `1.0`; `0.0` means non-blocking |
| `min_interval` | `float` | Minimum request interval (seconds), default `1.0` |

**Return Value:** `MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam] | None`

**Message Fields (`.msg`):**

| Field | Type | Description |
| --- | --- | --- |
| `end_max_linear_vel` | `float` | Maximum end-effector linear velocity (m/s) |
| `end_max_angular_vel` | `float` | Maximum end-effector angular velocity (rad/s) |
| `end_max_linear_acc` | `float` | Maximum end-effector linear acceleration (m/s²) |
| `end_max_angular_acc` | `float` | Maximum end-effector angular acceleration (rad/s²) |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
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

---

### Get Crash Protection Rating — `get_crash_protection_rating()`

**Description:** Query the crash protection rating of each joint (list returned by the controller).

**Function Definition:**

```python
get_crash_protection_rating(
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[list[int]] | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `timeout` | `float` | Timeout for waiting for feedback (seconds), default `1.0`; `0.0` means non-blocking |
| `min_interval` | `float` | Minimum request interval (seconds), default `1.0` |

**Return Value:** `MessageAbstract[list[int]] | None`

`.msg` is a crash protection rating list (in joint order), where each item is an `int` (range: `0~8`). **The higher the rating, the more sensitive it is, and the more easily the crash protection mechanism is triggered** (more conservative).

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

rating = robot.get_crash_protection_rating()
if rating is not None:
    print(rating.msg)
    print(rating.hz, rating.timestamp)
```

---

### Get Joint Assistance Rating — `get_joint_assistance_rating()`

**Description:** Read the assistance rating list of all joints.

**Function Definition:**

```python
get_joint_assistance_rating(
    self,
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[list[int]] | None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `timeout` | `float` | Wait timeout in seconds, default `1.0`; `0.0` means non-blocking |
| `min_interval` | `float` | Minimum request interval in seconds, default `1.0` |

**Return Value:** `MessageAbstract[list[int]] | None`  
`.msg` is a `list[int]` (length 6), each value range: `0~10`.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

rating = robot.get_joint_assistance_rating()
if rating is not None:
    print(rating.msg)
    print(rating.hz, rating.timestamp)
```

---

### Calibrate Joint — `calibrate_joint()`

**Description:** Perform the zeroing / calibration process for a specified joint (waits for controller ACK / response and returns the result).

**Function Definition:**

```python
calibrate_joint(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    timeout: float = 1.0,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | `1~6` calibrates a single joint; `255` calibrates all joints |
| `timeout` | `float` | Response wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates a success response was received; `False` indicates timeout or failure.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

joint_index = 1
robot.disable(joint_index)
time.sleep(0.2)
input("Please move the joint to the zero position, then press Enter to continue...")

if robot.calibrate_joint(joint_index):
    print("calibrate_joint success")
```

---

### Clear Joint Error — `clear_joint_error()`

**Description:** Clear joint error code on one joint or all joints.

**Function Definition:**

```python
clear_joint_error(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    timeout: float = 1.0,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index: `1~6` for one joint; `255` for all joints |
| `timeout` | `float` | ACK timeout in seconds, default `1.0` |

**Return Value:** `bool` — ACK-only API (`True` means response received within timeout).

> **Tip:** This API only confirms that ACK/response is received; it does not include automatic read-back verification.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# Clear joint-2 error
ok = robot.clear_joint_error(joint_index=2)
print("clear_joint_error(j2) =", ok)

# Clear all-joint errors
ok = robot.clear_joint_error(joint_index=255)
print("clear_joint_error(all) =", ok)
```

---

### Set Joint Angle/Velocity Limits — `set_joint_angle_vel_limits()`

**Description:** Set joint angle / velocity limits, with optional read-back verification to check if the settings took effect.

**Function Definition:**

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

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index: `1~6` configures a single joint; `255` configures all joints |
| `min_angle_limit` | `Optional[float]` | Minimum angle limit (rad); `None` means no configuration |
| `max_angle_limit` | `Optional[float]` | Maximum angle limit (rad); `None` means no configuration |
| `max_joint_spd` | `Optional[float]` | Maximum joint velocity limit (rad/s); `None` means no configuration |
| `timeout` | `float` | ACK / verification wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates ACK received and read-back verification passed; `False` indicates timeout / failure / verification failed.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# Set angle and speed limits together
success = robot.set_joint_angle_vel_limits(
    joint_index=1,
    min_angle_limit=-2.618,
    max_angle_limit=2.618,
    max_joint_spd=3.0,
)
print("set_joint_angle_vel_limits success =", success)

# Set max speed limit only (keep angle limits unchanged)
success = robot.set_joint_angle_vel_limits(joint_index=1, max_joint_spd=3.0)
print("set_joint_angle_vel_limits(max_joint_spd) success =", success)
```

---

### Set Joint Acceleration Limits — `set_joint_acc_limits()`

**Description:** Set the maximum acceleration limit for a specified joint.

**Function Definition:**

```python
set_joint_acc_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    max_joint_acc: Optional[float] = None,
    timeout: float = 1.0,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index: `1~6` configures a single joint; `255` configures all joints |
| `max_joint_acc` | `Optional[float]` | Maximum acceleration (rad/s²); `None` means no configuration |
| `timeout` | `float` | ACK / verification wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates ACK received and read-back verification passed.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_joint_acc_limits(joint_index=1, max_joint_acc=5.0)
print("set_joint_acc_limits success =", success)
```

---

### Set Flange Velocity/Acceleration Limits — `set_flange_vel_acc_limits()`

**Description:** Set the end-effector velocity / acceleration limits.

**Function Definition:**

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

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `max_linear_vel` | `Optional[float]` | Maximum linear velocity (m/s); `None` means no configuration |
| `max_angular_vel` | `Optional[float]` | Maximum angular velocity (rad/s); `None` means no configuration |
| `max_linear_acc` | `Optional[float]` | Maximum linear acceleration (m/s²); `None` means no configuration |
| `max_angular_acc` | `Optional[float]` | Maximum angular acceleration (rad/s²); `None` means no configuration |
| `timeout` | `float` | ACK / verification wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates ACK received and read-back verification passed.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
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

---

### Set Crash Protection Rating — `set_crash_protection_rating()`

**Description:** Set the crash protection rating (can specify a single joint or all joints).

**Function Definition:**

```python
set_crash_protection_rating(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    rating: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8] = 0,
    timeout: float = 1.0,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index: `1~6` configures a single joint; `255` configures all joints, default: `255` |
| `rating` | `int` | Crash protection rating, range: `[0, 8]` (`0` = no detection), default: `0`. **The higher the rating, the more sensitive it is, and the more easily crash protection is triggered** (more conservative) |
| `timeout` | `float` | ACK / verification wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates ACK received and read-back verification passed.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_crash_protection_rating(joint_index=1, rating=1)
print("set_crash_protection_rating success =", success)
```

---

### Set Joint Assistance Rating — `set_joint_assistance_rating()`

**Description:** Set assistance rating for one joint or all joints.

**Function Definition:**

```python
set_joint_assistance_rating(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    rating: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10] = 0,
    timeout: float = 1.0,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_index` | `int` | Joint index: `1~6` for one joint; `255` for all joints |
| `rating` | `int` | Assistance level, range `[0, 10]` |
| `timeout` | `float` | ACK / verification timeout in seconds, default `1.0` |

**Return Value:** `bool` — `True` indicates ACK received and read-back verification passed.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# Set joint-1 assistance rating
ok = robot.set_joint_assistance_rating(joint_index=1, rating=3)
print("set_joint_assistance_rating(j1) =", ok)

# Set all-joint assistance rating
ok = robot.set_joint_assistance_rating(joint_index=255, rating=2)
print("set_joint_assistance_rating(all) =", ok)
```

---

### Reset Flange Limits to Default — `set_flange_vel_acc_limits_to_default()`

**Description:** Reset the end-effector velocity / acceleration limits to default values.

**Function Definition:**

```python
set_flange_vel_acc_limits_to_default(
    self,
    timeout: float = 1.0,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `timeout` | `float` | ACK / response wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates that ACK / response was received within the timeout.

> **Tip:** This interface does not provide read-back verification. To confirm, you can call `get_flange_vel_acc_limits()` to query.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_flange_vel_acc_limits_to_default()
print("set_flange_vel_acc_limits_to_default success =", success)
```

---

### Reset Joint Limits to Default — `set_joint_angle_vel_acc_limits_to_default()`

**Description:** Reset the joint angle / velocity / acceleration limits to default values.

**Function Definition:**

```python
set_joint_angle_vel_acc_limits_to_default(
    self,
    timeout: float = 1.0,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `timeout` | `float` | ACK / response wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates that ACK / response was received within the timeout.

> **Tip:** This interface does not provide read-back verification. To confirm, you can call `get_joint_angle_vel_limits()` / `get_joint_acc_limits()` to query.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_joint_angle_vel_acc_limits_to_default()
print("set_joint_angle_vel_acc_limits_to_default success =", success)
```

---

### Set Link Velocity/Acceleration Periodic Feedback — `set_links_vel_acc_period_feedback()`

**Description:** Set the Cartesian velocity / acceleration periodic feedback switch for each joint link (corresponding to CAN periodic frames `0x481~0x486`).

> **Warning:** This feature has been **deprecated** in the lower-level main controller, but the bus may still periodically report the corresponding frames, and the reported data is **all zeros** with no practical meaning. **It is recommended to keep this disabled by default** (`enable=False`) to avoid wasting bandwidth.
>
> There is no direct read-back verification method for this interface. It is recommended to use `candump` to observe whether periodic frames appear for verification:
>
> ```bash
> candump can0 | grep "48[1-6]"
> ```

**Function Definition:**

```python
set_links_vel_acc_period_feedback(
    self,
    enable: bool = False,
    timeout: float = 1.0,
) -> bool
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `enable` | `bool` | Whether to enable periodic feedback: `True` to enable; `False` to disable (**recommended to keep disabled by default**) |
| `timeout` | `float` | ACK / response wait timeout (seconds), default `1.0` |

**Return Value:** `bool` — `True` indicates that ACK / response was received within the timeout.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_links_vel_acc_period_feedback(enable=True)
print("enable periodic feedback success =", success)

success = robot.set_links_vel_acc_period_feedback(enable=False)
print("disable periodic feedback success =", success)
```

---

# Piper 机械臂 API 使用文档

> 本文档描述 `pyAgxArm` SDK 中 Piper 机型相关的 CAN 收发 API，聚焦运行时数据读写、运动控制与协议参数能力。

## 目录

- [切换到 English](#piper-api-documentation)
- [固件版本选择](#固件版本选择)
  - [版本列表](#版本列表)
  - [如何选择](#如何选择)
- [数据读取](#数据读取)
  - [MessageAbstract 返回值通用说明](#messageabstract-返回值通用说明)
  - [读取机械臂状态 — get_arm_status()](#读取机械臂状态--get_arm_status)
  - [读取关节角度 — get_joint_angles()](#读取关节角度--get_joint_angles)
  - [读取法兰位姿 — get_flange_pose()](#读取法兰位姿--get_flange_pose)
  - [读取电机状态 — get_motor_states()](#读取电机状态--get_motor_states)
  - [读取驱动器状态 — get_driver_states()](#读取驱动器状态--get_driver_states)
  - [读取关节使能状态 — get_joint_enable_status()](#读取关节使能状态--get_joint_enable_status)
  - [读取全部关节使能状态 — get_joints_enable_status_list()](#读取全部关节使能状态--get_joints_enable_status_list)
  - [读取固件信息 — get_firmware()](#读取固件信息--get_firmware)
- [参数设定](#参数设定)
  - [设定运行速度 — set_speed_percent()](#设定运行速度--set_speed_percent)
  - [设定安装位置 — set_installation_pos()](#设定安装位置--set_installation_pos)
  - [设定运动模式 — set_motion_mode()](#设定运动模式--set_motion_mode)
  - [设定负载 — set_payload()](#设定负载--set_payload)
- [Leader-Follower 臂](#leader-follower-臂)
  - [设定主导臂（Leader）模式 — set_leader_mode()](#设定主导臂leader模式--set_leader_mode)
  - [设定跟随臂（Follower）模式 — set_follower_mode()](#设定跟随臂follower模式--set_follower_mode)
  - [主导臂（Leader）回 Home — move_leader_to_home()](#主导臂leader回-home--move_leader_to_home)
  - [主从臂（Leader-Follower）一起回 Home — move_leader_follower_to_home()](#主从臂leader-follower一起回-home--move_leader_follower_to_home)
  - [恢复主导臂（Leader）零力拖动 — restore_leader_drag_mode()](#恢复主导臂leader零力拖动--restore_leader_drag_mode)
  - [读取主导臂（Leader）关节角度 — get_leader_joint_angles()](#读取主导臂leader关节角度--get_leader_joint_angles)
- [运动控制](#运动控制)
  - [使能 — enable()](#使能--enable)
  - [失能 — disable()](#失能--disable)
  - [电子急停 — electronic_emergency_stop()](#电子急停--electronic_emergency_stop)
  - [重置 — reset()](#重置--reset)
  - [关节运动 — move_j()](#关节运动--move_j)
  - [关节运动 (Follower 模式) — move_js()](#关节运动-follower-模式--move_js)
  - [点到点运动 — move_p()](#点到点运动--move_p)
  - [直线运动 — move_l()](#直线运动--move_l)
  - [圆弧运动 — move_c()](#圆弧运动--move_c)
  - [单关节 MIT 控制 — move_mit()](#单关节-mit-控制--move_mit)
- [CPV 运动与参数](#cpv-运动与参数)
  - [CPV 指令接口](#cpv-指令接口)
  - [CPV 参数读取接口](#cpv-参数读取接口)
  - [CPV 参数写入接口](#cpv-参数写入接口)
- [高级参数读取与配置](#高级参数读取与配置)
  - [读取关节角度/速度限制 — get_joint_angle_vel_limits()](#读取关节角度速度限制--get_joint_angle_vel_limits)
  - [读取关节加速度限制 — get_joint_acc_limits()](#读取关节加速度限制--get_joint_acc_limits)
  - [读取法兰速度/加速度限制 — get_flange_vel_acc_limits()](#读取法兰速度加速度限制--get_flange_vel_acc_limits)
  - [读取碰撞防护等级 — get_crash_protection_rating()](#读取碰撞防护等级--get_crash_protection_rating)
  - [读取关节助力等级 — get_joint_assistance_rating()](#读取关节助力等级--get_joint_assistance_rating)
  - [关节置零/标定 — calibrate_joint()](#关节置零标定--calibrate_joint)
  - [清除关节错误码 — clear_joint_error()](#清除关节错误码--clear_joint_error)
  - [配置关节角度/速度限制 — set_joint_angle_vel_limits()](#配置关节角度速度限制--set_joint_angle_vel_limits)
  - [配置关节加速度限制 — set_joint_acc_limits()](#配置关节加速度限制--set_joint_acc_limits)
  - [配置法兰速度/加速度限制 — set_flange_vel_acc_limits()](#配置法兰速度加速度限制--set_flange_vel_acc_limits)
  - [配置碰撞防护等级 — set_crash_protection_rating()](#配置碰撞防护等级--set_crash_protection_rating)
  - [配置关节助力等级 — set_joint_assistance_rating()](#配置关节助力等级--set_joint_assistance_rating)
  - [恢复法兰限制默认值 — set_flange_vel_acc_limits_to_default()](#恢复法兰限制默认值--set_flange_vel_acc_limits_to_default)
  - [恢复关节限制默认值 — set_joint_angle_vel_acc_limits_to_default()](#恢复关节限制默认值--set_joint_angle_vel_acc_limits_to_default)
  - [设置 Link 速度/加速度周期反馈 — set_links_vel_acc_period_feedback()](#设置-link-速度加速度周期反馈--set_links_vel_acc_period_feedback)

---

## 固件版本选择

Piper 系列机械臂可能搭载不同固件版本，部分版本引入了协议级别的改动，会影响参数范围、编码精度和内部缩放逻辑。SDK 通过 `create_agx_arm_config()` 的 `firmeware_version` 参数选择匹配的驱动，确保 API 行为与机械臂实际运行的固件一致，这里的固件版本采用的是软件的版本号，例如 `"S-V1.8-8"`。

### 版本列表

| SDK 版本 | 常量 | 固件范围 | 主要差异 |
| --- | --- | --- | --- |
| `"default"` | `PiperFW.DEFAULT` | ≤ S-V1.8-2 | MIT 力矩：关节 1-3 输入范围 ±32 N·m，关节 4-6 范围 ±8 N·m；8-bit 编码 |
| `"v183"` | `PiperFW.V183` | S-V1.8-3 ~ S-V1.8-7 | MIT 力矩：全关节范围 ±8 N·m；8-bit 编码 |
| `"v188"` | `PiperFW.V188` | ≥ S-V1.8-8 | MIT 力矩：全关节范围 ±16 N·m；12-bit 编码；去除 CRC 校验位；motion mode 编码变更 |

### 如何选择

查看机械臂主控上的固件版本号，可通过[get_firmware()](#读取固件信息--get_firmware)方法获取（格式：**S-VX.X-X**），根据下表选择对应的 SDK 版本：

| 您的固件版本 | 应填写的 `firmeware_version` | 常量 |
| --- | --- | --- |
| S-V1.8-2 及更早 | `"default"`（或不填，默认值） | `PiperFW.DEFAULT` |
| S-V1.8-3 ~ S-V1.8-7 | `"v183"` | `PiperFW.V183` |
| S-V1.8-8 及更新 | `"v188"` | `PiperFW.V188` |

> **⚠️ 安全警告：** 选错固件版本可能导致 SDK 发送编码错误的力矩指令。特别是将 v188 协议数据发送给旧固件机械臂，可能造成 **危险的非预期运动**。使用前请务必确认您的固件版本。

**使用示例（推荐 — 使用常量类获得 IDE 自动补全）：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

# 固件为 S-V1.8-5 → 处于 v183 ~ v187 之间，选择 PiperFW.V183
cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.V183, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
```

也兼容原始字符串写法：

```python
cfg = create_agx_arm_config(robot="piper", firmeware_version="v183", channel="can0")
```

---

## 数据读取

### MessageAbstract 返回值通用说明

本 SDK 多数读取接口返回 `MessageAbstract[T] | None`，其通用字段如下：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `ret.msg` | `T` | 消息数据本体（例如 `list[float]` 或某个反馈消息结构体） |
| `ret.hz` | `float` | 该消息类型的接收频率（SDK 统计），单位：Hz |
| `ret.timestamp` | `float` | 消息时间戳（SDK 记录），单位：s |

---

### 读取机械臂状态 — `get_arm_status()`

**功能说明：** 读取机械臂整体状态反馈（控制模式、运动模式、急停/异常状态、轨迹点编号等）。

**函数定义：**

```python
get_arm_status(self) -> MessageAbstract[ArmMsgFeedbackStatus] | None
```

**返回值：** `MessageAbstract[ArmMsgFeedbackStatus] | None`

**消息字段（`.msg`）：**

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `ctrl_mode` | `int` | 控制模式枚举（见下方含义） |
| `arm_status` | `int` | 机械臂状态枚举（见下方含义） |
| `mode_feedback` | `int` | 模式反馈枚举（见下方含义） |
| `teach_status` | `int` | 示教状态枚举（见下方含义） |
| `motion_status` | `int` | 运动状态枚举（见下方含义） |
| `trajectory_num` | `int` | 轨迹点编号（离线轨迹模式下反馈） |
| `err_status` | `object` | 故障状态位域（已转换为布尔标志，见下方含义） |

**枚举含义（`ArmMsgFeedbackStatus.msg`）：**

`ctrl_mode`（控制模式）：
- `0x00` 待机模式
- `0x01` CAN 指令控制
- `0x02` 示教模式
- `0x03` 以太网控制模式
- `0x04` Wi-Fi 控制模式
- `0x05` 遥控器控制模式
- `0x06` 联动示教输入模式
- `0x07` 离线轨迹模式
- `0xFF` 未知

`arm_status`（机械臂状态）：
- `0x00` 正常
- `0x01` 急停
- `0x02` 无解
- `0x03` 奇异点
- `0x04` 目标角度超过限
- `0x05` 关节通信异常
- `0x06` 关节抱闸未打开
- `0x07` 发生碰撞
- `0x08` 拖动示教时超速
- `0x09` 关节状态异常
- `0x0A` 其它异常
- `0x0B` 示教记录
- `0x0C` 示教执行
- `0x0D` 示教暂停
- `0x0E` 主控 NTC 过温
- `0x0F` 释放电阻 NTC 过温
- `0xFF` 未知

`mode_feedback`（模式反馈）：
- `0x00` MOVE P
- `0x01` MOVE J
- `0x02` MOVE L
- `0x03` MOVE C
- `0x04` MOVE MIT（Piper固件 < v188；使用 `PiperFW.DEFAULT` / `PiperFW.V183`）
- `0x05` MOVE_CPV
- `0x06` MOVE MIT（Piper固件 >= v188；使用 `PiperFW.V188`）
- `0xFF` 未知

`teach_status`（示教状态）：
- `0x00` 关闭
- `0x01` 开始示教记录（进入拖动示教模式）
- `0x02` 结束示教记录（退出拖动示教模式）
- `0x03` 执行示教轨迹（拖动示教轨迹复现）
- `0x04` 暂停执行
- `0x05` 继续执行（轨迹复现继续）
- `0x06` 终止执行
- `0x07` 运动到轨迹起点
- `0xFF` 未知

`motion_status`（运动状态）：
- `0x00` 到达指定点位
- `0x01` 未到达指定点位
- `0xFF` 未知

`err_status`（16-bit 故障码 -> 布尔标志，Piper 6 轴）：
- `msg.err_code`: 原始 16-bit 故障码整数（0~65535）。
- `msg.err_status.joint_i_angle_limit`（`i=1..6`）：`True` 表示关节 i 角度超限。
- `msg.err_status.communication_status_joint_i`（`i=1..6`）：`True` 表示关节 i 通信异常。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    arm_status = robot.get_arm_status()
    if arm_status is not None:
        print(arm_status.msg)
        print(arm_status.hz, arm_status.timestamp)
    time.sleep(0.02)
```

---

### 读取关节角度 — `get_joint_angles()`

**功能说明：** 获取当前各关节角度。

**函数定义：**

```python
get_joint_angles(self) -> MessageAbstract[list[float]] | None
```

**返回值：** `MessageAbstract[list[float]] | None`

`.msg` 为长度 6 的 `list[float]`：`[j1, j2, j3, j4, j5, j6]`，单位：**rad**。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    ja = robot.get_joint_angles()
    if ja is not None:
        print(ja.msg)
        print(ja.hz, ja.timestamp)
    time.sleep(0.005)
```

---

### 读取法兰位姿 — `get_flange_pose()`

**功能说明：** 获取末端法兰位姿。

> **术语说明：** `flange` 指机械臂最后一个连杆（末端连杆）的安装法兰/连接面，是工具/末端执行器的机械安装接口。

**函数定义：**

```python
get_flange_pose(self) -> MessageAbstract[list[float]] | None
```

**返回值：** `MessageAbstract[list[float]] | None`

`.msg` 为长度 6 的 `list[float]`：`[x, y, z, roll, pitch, yaw]`

- `x, y, z`：位置坐标（单位：m）
- `roll, pitch, yaw`：姿态欧拉角（单位：rad，分别对应绕 X/Y/Z 轴旋转）

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    fp = robot.get_flange_pose()
    if fp is not None:
        print(fp.msg)
        print(fp.hz, fp.timestamp)
    time.sleep(0.005)
```

---

### 读取电机状态 — `get_motor_states()`

**功能说明：** 读取指定关节的电机高速反馈（位置 / 速度 / 电流 / 扭矩）。

**函数定义：**

```python
get_motor_states(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
) -> MessageAbstract[ArmMsgFeedbackHighSpd] | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号，范围：`1~6` |

**返回值：** `MessageAbstract[ArmMsgFeedbackHighSpd] | None`

**消息字段（`.msg`）：**

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `position` | `float` | 电机位置（rad） |
| `velocity` | `float` | 电机速度（rad/s） |
| `current` | `float` | 电机电流（A） |
| `torque` | `float` | 电机扭矩（N·m） |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

ms = robot.get_motor_states(1)
if ms is not None:
    print(ms.msg.position, ms.msg.velocity, ms.msg.current, ms.msg.torque)
    print(ms.hz, ms.timestamp)
```

---

### 读取驱动器状态 — `get_driver_states()`

**功能说明：** 读取指定关节的驱动器低速反馈（电压 / 温度 / 母线电流 / 驱动状态位等）。

**函数定义：**

```python
get_driver_states(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
) -> MessageAbstract[ArmMsgFeedbackLowSpd] | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号，范围：`1~6` |

**返回值：** `MessageAbstract[ArmMsgFeedbackLowSpd] | None`

**消息字段（`.msg`）：**

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `vol` | `float` | 驱动电压 |
| `foc_temp` | `float` | 驱动温度（°C） |
| `motor_temp` | `float` | 电机温度（°C） |
| `bus_current` | `float` | 母线电流（A） |
| `foc_status` | `object` | 驱动状态位（电压过低 / 过温 / 过流 / 碰撞 / 失能 / 堵转等） |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

ds = robot.get_driver_states(1)
if ds is not None:
    print(ds.msg.vol, ds.msg.foc_temp, ds.msg.motor_temp, ds.msg.bus_current)
    print(ds.msg.foc_status.driver_enable_status)
    print(ds.hz, ds.timestamp)
```

---

### 读取关节使能状态 — `get_joint_enable_status()`

**功能说明：** 获取指定关节电机的使能状态。

**函数定义：**

```python
get_joint_enable_status(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255],
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号：`1~6` 查询单关节；`255` 查询全部关节（内部使用 `all([...])` 汇总） |

**返回值：** `bool` — `True` 为已使能，`False` 为未使能或当前无反馈。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

if robot.get_joint_enable_status(1):
    print("关节 1 电机已使能")
```

---

### 读取全部关节使能状态 — `get_joints_enable_status_list()`

**功能说明：** 读取全部关节电机的使能状态列表（按关节 1~6 顺序）。

**函数定义：**

```python
get_joints_enable_status_list(self) -> list[bool]
```

**返回值：** `list[bool]`

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

print(robot.get_joints_enable_status_list())
```

---

### 读取固件信息 — `get_firmware()`

**功能说明：** 读取机械臂固件信息（软件版本 / 硬件版本 / 生产日期等）。该接口会下发查询帧并等待对应反馈。

**函数定义：**

```python
get_firmware(
    self,
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> dict | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `timeout` | `float` | 等待反馈超时时间（秒），默认 `1.0`；`0.0` 表示非阻塞 |
| `min_interval` | `float` | 最小请求间隔（秒），默认 `1.0` |

**返回值：** `dict | None`

常见字段：

| Key | 类型 | 说明 |
| --- | --- | --- |
| `software_version` | `str` | 软件版本（例如 `S-V1.8-2`） |
| `hardware_version` | `str` | 硬件版本（例如 `H-V1.2-1`） |
| `production_date` | `str` | 生产日期（例如 `250925`） |
| `node_type` | `str` | 节点类型 |
| `node_number` | `int` | 节点编号 |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

fw = robot.get_firmware()
if fw is not None:
    print(fw)
```

---

## 参数设定

### 设定运行速度 — `set_speed_percent()`

**功能说明：** 设定机械臂在位置速度模式下的运行速度百分比，适用于 `move_j` / `move_p` / `move_l` / `move_c`。

**函数定义：**

```python
set_speed_percent(self, percent: int = 100) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `percent` | `int` | 运行速度百分比，范围 `[0, 100]`，默认 `100` |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_speed_percent(100)
```

---

### 设定安装位置 — `set_installation_pos()`

**功能说明：** 设定机械臂安装位置，支持水平、朝左和朝右三个方向。

**函数定义：**

```python
set_installation_pos(
    self,
    pos: Literal["horizontal", "left", "right"] = "horizontal",
) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `pos` | `str` | 安装方向，可选值：`'horizontal'` / `'left'` / `'right'`，默认：`'horizontal'`（建议使用 `robot.OPTIONS.INSTALLATION_POS.xxx` 常量） |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_installation_pos(robot.OPTIONS.INSTALLATION_POS.HORIZONTAL)
```

---

### 设定运动模式 — `set_motion_mode()`

**功能说明：** 设置运动模式。

| 模式 | 类型 | 说明 |
| --- | --- | --- |
| `move_p` / `move_j` / `move_l` / `move_c` | **位置速度模式** | 底层会对接收到的消息进行平滑处理，保证运动连续稳定 |
| `move_mit` / `move_js` | **MIT 电机透传模式** | 底层仅负责消息转发，**不进行任何平滑处理**，适用于直接控制电机的场景 |

> **提示：** 调用任一 `move_*` 运动指令时，系统 **会自动切换至对应的运动模式**，因此通常 **无需手动调用 `set_motion_mode()`**。

**函数定义：**

```python
set_motion_mode(
    self,
    motion_mode: Literal["p", "j", "l", "c", "mit", "js"] = "p",
) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `motion_mode` | `str` | 运动模式，可选值：`'p'` / `'j'` / `'l'` / `'c'` / `'mit'` / `'js'`，默认：`'p'`（建议使用 `robot.OPTIONS.MOTION_MODE.xxx` 常量） |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.J)
```

---

### 设定负载 — `set_payload()`

**功能说明：** 设定机械臂负载（Payload）。

**函数定义：**

```python
set_payload(
    self,
    load: Literal['empty', 'half', 'full'] = 'empty',
    timeout: float = 1.0,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `load` | `str` | 负载等级，可选值：`'empty'`（空载）/ `'half'`（半载）/ `'full'`（满载），默认：`'empty'`（建议使用 `robot.OPTIONS.PAYLOAD.xxx` 常量） |
| `timeout` | `float` | 等待反馈的超时时间（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示收到指令应答，但并不表示设定一定成功。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_payload(robot.OPTIONS.PAYLOAD.FULL)
```

---

## Leader-Follower 臂

### 设定主导臂（Leader）模式 — `set_leader_mode()`

**功能说明：** 将机械臂设置为 **主导臂（Leader）零力拖动模式**（Leader-Follower 协同场景下的"主导臂（Leader Arm）"）。进入该模式后，主导臂（Leader Arm）通常处于可拖动/零力拖动状态。

> **提示：** 该模式用于Leader-Follower 臂联动/示教等场景。若仅使用单臂，可忽略该接口。

**函数定义：**

```python
set_leader_mode(self) -> None
```

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_leader_mode()
```

---

### 设定跟随臂（Follower）模式 — `set_follower_mode()`

**功能说明：** 将机械臂设置为 **跟随臂（Follower）受控模式**（Leader-Follower 协同场景下的"跟随臂（Follower Arm）"），跟随臂（Follower Arm）跟随主导臂（Leader Arm）控制/指令运行。可与 `set_leader_mode()` 配套使用。

**函数定义：**

```python
set_follower_mode(self) -> None
```

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_follower_mode()
```

---

### 主导臂（Leader）回 Home — `move_leader_to_home()`

**功能说明：** 让主导臂（Leader Arm）回到 Home 位姿。完成后建议调用 `restore_leader_drag_mode()` 恢复主导臂（Leader Arm）"零力拖动"状态。

**函数定义：**

```python
move_leader_to_home(self) -> None
```

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# robot.set_leader_mode()
robot.move_leader_to_home()
# robot.restore_leader_drag_mode()
```

---

### 主从臂（Leader-Follower）一起回 Home — `move_leader_follower_to_home()`

**功能说明：** 让主导臂（Leader Arm）与跟随臂（Follower Arm）**一起**回到 Home 位姿。完成后建议调用 `restore_leader_drag_mode()` 恢复主导臂（Leader Arm）"零力拖动"状态。

**函数定义：**

```python
move_leader_follower_to_home(self) -> None
```

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# robot.set_leader_mode()
robot.move_leader_follower_to_home()
# robot.restore_leader_drag_mode()
```

---

### 恢复主导臂（Leader）零力拖动 — `restore_leader_drag_mode()`

**功能说明：** 将主导臂（Leader Arm）恢复为"零力拖动"状态，通常用于 `move_leader_to_home()` 或 `move_leader_follower_to_home()` 之后。

**函数定义：**

```python
restore_leader_drag_mode(self) -> None
```

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# robot.set_leader_mode()
# robot.move_leader_to_home()
robot.restore_leader_drag_mode()
```

---

### 读取主导臂（Leader）关节角度 — `get_leader_joint_angles()`

**功能说明：** 获取主导臂（Leader Arm）关节角度消息，用于控制跟随臂（Follower Arm）。

**函数定义：**

```python
get_leader_joint_angles(self) -> MessageAbstract[list[float]] | None
```

**返回值：** `MessageAbstract[list[float]] | None`

`.msg` 为长度 6 的 `list[float]`：`[j1, j2, j3, j4, j5, j6]`，单位：**rad**。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_leader_mode()

while True:
    mja = robot.get_leader_joint_angles()
    if mja is not None:
        print(mja.msg)
        print(mja.hz, mja.timestamp)
    time.sleep(0.005)
```

---

## 运动控制

### 使能 — `enable()`

**功能说明：** 将机械臂使能上电。

**函数定义：**

```python
enable(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号：`1~6` 使能单关节；`255` 使能全部关节，默认：`255` |

**返回值：** `bool` — `True` 为使能成功。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)
```

---

### 失能 — `disable()`

**功能说明：** 将机械臂失电。

> **⚠️ 安全警告：** 执行该指令时，如果机械臂关节处于抬起状态，会 **立刻掉落**。请确保机械臂处于安全状态后再使用。

**函数定义：**

```python
disable(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号：`1~6` 失能单关节；`255` 失能全部关节，默认：`255` |

**返回值：** `bool` — `True` 为失能成功。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.disable():
    time.sleep(0.01)
```

---

### 电子急停 — `electronic_emergency_stop()`

**功能说明：** 将机械臂设置为急停状态。如果执行时机械臂关节处于抬起状态，机械臂会 **缓慢以恒定阻尼落下**（不会立刻掉落），急停后可使用 `reset()` 进行重置。

**函数定义：**

```python
electronic_emergency_stop(self) -> None
```

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.electronic_emergency_stop()
```

---

### 重置 — `reset()`

**功能说明：** 将机械臂模式重置并令机械臂立刻失电。

> **⚠️ 安全警告：** 执行该指令时，如果机械臂关节处于抬起状态，会 **立刻掉落**。请确保机械臂处于安全状态后再使用。

**函数定义：**

```python
reset(self) -> None
```

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.reset()
```

---

### 关节运动 — `move_j()`

**功能说明：** 关节位置速度控制模式，设定各关节目标角度。

**函数定义：**

```python
move_j(self, joints: list[float]) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joints` | `list[float]` | 长度 6 的目标角度数组 `[j1, j2, j3, j4, j5, j6]`（单位：rad，精度：1.74532925199e-5）。关节限位取决于机械臂型号配置 |

> **注意：** 连续执行该指令会覆盖上一次的目标值。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_j([0, 0.4, -0.4, 0, -0.4, 0])

# 等待运动结束（带 5s 超时）
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

---

### 关节运动 (Follower 模式) — `move_js()`

**功能说明：** 将机械臂切换到 **JS（Follower）模式**（MIT 透传模式），并下发关节目标角度。与 `move_j` 相比，`move_js` 更偏向"快速响应"控制：**不做平滑处理、无轨迹规划**，控制器/驱动器会尽可能快地响应目标角度。

**函数定义：**

```python
move_js(self, joints: list[float]) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joints` | `list[float]` | 长度 6 的目标角度数组 `[j1, j2, j3, j4, j5, j6]`（单位：rad，精度：1.74532925199e-5）。关节限位取决于机械臂型号配置 |

> **⚠️ 风险等级：极高**
>
> 1. 该模式可能导致 **冲击、振荡、失稳** 等风险，请仅在充分评估安全与控制稳定性的前提下使用，并确保随时可急停。
> 2. **无平滑过程、无轨迹规划**，控制器/驱动器尝试以最快响应到达目标，可能产生冲击和振荡。
> 3. 连续执行该指令会覆盖上一次的目标值。
> 4. 由于响应变快，关节的控制力度相较于位置速度模式小，刚度也会变小。
> 5. 在旧版本固件（低于 `S-V1.8-5`）下，如果机械臂当前为Follower 模式，想切换到位置速度控制模式需要先执行 `robot.reset()`（机械臂会重置掉电），然后再执行 `move_j` 才能正常控制。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.move_js([0, 0.4, -0.4, 0, -0.4, 0])
```

---

### 点到点运动 — `move_p()`

**功能说明：** 发送目标法兰位姿，机械臂根据当前关节位置和目标位姿进行关节角度解算并运动。

**函数定义：**

```python
move_p(self, pose: list[float]) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `pose` | `list[float]` | 目标位姿 `[x, y, z, roll, pitch, yaw]`：`x, y, z` 为位置（m，精度：1e-6）；`roll, pitch, yaw` 为欧拉角（rad，精度：1.74532925199e-5）。姿态范围：`roll` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]`，`yaw` ∈ `[-π, π]` |

> **注意：** 连续执行该指令会覆盖上一次的目标值。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_p([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])

# 等待运动结束（带 5s 超时）
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

---

### 直线运动 — `move_l()`

**功能说明：** 发送目标法兰位姿，机械臂根据当前位姿和目标位姿进行直线轨迹规划。

**函数定义：**

```python
move_l(self, pose: list[float]) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `pose` | `list[float]` | 目标位姿 `[x, y, z, roll, pitch, yaw]`：`x, y, z` 为位置（m，精度：1e-6）；`roll, pitch, yaw` 为欧拉角（rad，精度：1.74532925199e-5）。姿态范围：`roll` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]`，`yaw` ∈ `[-π, π]` |

> **注意：** 连续执行该指令虽然可以覆盖上一次的目标，但由于底层每接收到新点位都需要重新进行直线规划，因此 **不能使用该指令连续发送目标点**。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
robot.move_l([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])

# 等待运动结束（带 5s 超时）
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

---

### 圆弧运动 — `move_c()`

**功能说明：** 通过"起点 / 中间点 / 终点"三个目标法兰位姿进行圆弧轨迹规划并执行。

**函数定义：**

```python
move_c(
    self,
    start_pose: list[float],
    mid_pose: list[float],
    end_pose: list[float],
) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `start_pose` | `list[float]` | 起点位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。姿态范围：`roll` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]`，`yaw` ∈ `[-π, π]` |
| `mid_pose` | `list[float]` | 中间点位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。姿态范围：`roll` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]`，`yaw` ∈ `[-π, π]` |
| `end_pose` | `list[float]` | 终点位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。姿态范围：`roll` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]`，`yaw` ∈ `[-π, π]` |

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

robot.set_speed_percent(100)
sp = [0.2, 0.0, 0.3, 0.0, 1.5708, 0.0]
mp = [0.2, 0.05, 0.35, 0.0, 1.5708, 0.0]
ep = [0.2, 0.0, 0.4, 0.0, 1.5708, 0.0]
robot.move_c(sp, mp, ep)

# 等待运动结束（带 5s 超时）
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

---

### 单关节 MIT 控制 — `move_mit()`

**功能说明：** 使用关节驱动底层的 MIT 控制接口，控制单个关节电机，可实现电流模拟的力矩控制。

控制器概念上会计算参考力矩：

$$T_{\text{ref}} = k_p \cdot (p_{\text{des}} - p) + k_d \cdot (v_{\text{des}} - v) + T_{\text{ff}}$$

其中 \(p/v\) 为关节实测位置/速度。

**典型用法建议：**

| 控制方式 | 参数设置 | 说明 |
| --- | --- | --- |
| **速度控制** | `kp = 0`, `kd ≠ 0` | 主要通过 `v_des` 控制 |
| **力矩控制** | `kp = 0`, `kd = 0` | 主要通过 `t_ff` 控制 |
| **位置控制** | `kp ≠ 0`, `kd ≠ 0` | 不建议将 `kd` 设为 0，适当增大阻尼可降低振荡风险 |

> **⚠️ 风险提示：** MIT 属于较底层控制接口，参数不当可能引发 **冲击 / 振荡 / 不稳定**。建议从小增益开始调试，并在安全工况下使用。

**函数定义：**

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

**参数说明（各版本通用）：**

| 名称 | 类型 | 范围 | 单位 | 默认值 | 精度 |
| --- | --- | --- | --- | --- | --- |
| `joint_index` | `int` | `1~6` | — | — | — |
| `p_des` | `float` | `[-12.5, 12.5]` | rad | `0.0` | 3.815e-4 |
| `v_des` | `float` | `[-45.0, 45.0]` | rad/s | `0.0` | 2.198e-2 |
| `kp` | `float` | `[0.0, 500.0]` | — | `10.0` | 1.221e-1 |
| `kd` | `float` | `[-5.0, 5.0]` | — | `0.8` | 2.442e-3 |

**`t_ff` 参数因固件版本而异：**

| 版本 | 关节 | `t_ff` 范围 (N·m) | 编码位数 | 精度 (N·m) |
| --- | --- | --- | --- | --- |
| `default`（≤ v182） | 1-3 | `[-32.0, 32.0]` | 8 | 2.510e-1 |
| `default`（≤ v182） | 4-6 | `[-8.0, 8.0]` | 8 | 6.275e-2 |
| `v183`（v183 ~ v187） | 1-6 | `[-8.0, 8.0]` | 8 | 6.275e-2 |
| `v188`（≥ v188） | 1-6 | `[-16.0, 16.0]` | 12 | 7.813e-3 |

> **注意：** 连续执行该指令会覆盖上一次的目标值。
>
> 必须通过 `create_agx_arm_config(firmeware_version=...)` 正确设置固件版本。详见[固件版本选择](#固件版本选择)。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

# 固件 >= S-V1.8-8，使用 "v188"
cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.V188, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while not robot.enable():
    time.sleep(0.01)

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

---

## CPV 运动与参数

CPV 模式提供了关节 **位置/速度指令** 与参数读写接口。  
调用 CPV 接口时，SDK 会在需要时自动切换到 `MOVE_CPV` 运动模式。

### CPV 指令接口

| 接口 | 签名 | 说明 |
| --- | --- | --- |
| `move_cpv_pos` | `move_cpv_pos(self, joint_index: Literal[1, 2, 3, 4, 5, 6], pos: float) -> None` | 下发 CPV 位置指令（rad）。若超出关节限位，SDK 会夹紧并输出告警日志。 |
| `move_cpv_vel` | `move_cpv_vel(self, joint_index: Literal[1, 2, 3, 4, 5, 6], vel: float) -> None` | 下发 CPV 速度指令（rad/s）。 |

### CPV 参数读取接口

所有读取接口都支持 `timeout` 与 `min_interval` 参数，返回 `float | None`。

| 接口 | 单位/含义 |
| --- | --- |
| `get_cpv_pos(joint_index, timeout=1.0, min_interval=1.0)` | 关节位置（rad） |
| `get_cpv_vel(joint_index, timeout=1.0, min_interval=1.0)` | 关节速度（rad/s） |
| `get_cpv_acc(joint_index, timeout=1.0, min_interval=1.0)` | 加速度（rad/s^2） |
| `get_cpv_dcc(joint_index, timeout=1.0, min_interval=1.0)` | 减速度（rad/s^2） |
| `get_cpv_cv(joint_index, timeout=1.0, min_interval=1.0)` | 轮廓/轨迹速度（rad/s） |
| `get_cpv_pp(joint_index, timeout=1.0, min_interval=1.0)` | 位置环比例增益 |
| `get_cpv_kp(joint_index, timeout=1.0, min_interval=1.0)` | 速度环比例增益 |
| `get_cpv_ki(joint_index, timeout=1.0, min_interval=1.0)` | 速度环积分增益 |

### CPV 参数写入接口

写接口为 **ACK + 读回校验**，返回 `bool`。

| 接口 | 说明 |
| --- | --- |
| `set_cpv_acc(joint_index, acc, timeout=1.0)` | 设置 CPV 加速度参数 |
| `set_cpv_dcc(joint_index, dcc, timeout=1.0)` | 设置 CPV 减速度参数 |
| `set_cpv_cv(joint_index, cv, timeout=1.0)` | 设置 CPV 轮廓/轨迹速度参数 |
| `set_cpv_pp(joint_index, pp, timeout=1.0)` | 设置 CPV 位置环比例增益 |
| `set_cpv_kp(joint_index, kp, timeout=1.0)` | 设置 CPV 速度环比例增益 |
| `set_cpv_ki(joint_index, ki, timeout=1.0)` | 设置 CPV 速度环积分增益 |

**快速示例：**

```python
ok = robot.set_cpv_acc(joint_index=1, acc=2.0)
print("set_cpv_acc:", ok)
print("cpv_acc =", robot.get_cpv_acc(joint_index=1))
robot.move_cpv_vel(joint_index=1, vel=0.2)
```

---

## 高级参数读取与配置

### 读取关节角度/速度限制 — `get_joint_angle_vel_limits()`

**功能说明：** 查询指定关节的角度限制与速度限制（由控制器反馈）。

**函数定义：**

```python
get_joint_angle_vel_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd] | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号，范围：`1~6` |
| `timeout` | `float` | 等待反馈超时（秒），默认 `1.0`；`0.0` 表示非阻塞 |
| `min_interval` | `float` | 最小请求间隔（秒），默认 `1.0` |

**返回值：** `MessageAbstract[ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd] | None`

**消息字段（`.msg`）：**

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `min_angle_limit` | `float` | 最小角度限制（rad） |
| `max_angle_limit` | `float` | 最大角度限制（rad） |
| `min_joint_spd` | `float` | 最小关节速度限制（rad/s） |
| `max_joint_spd` | `float` | 最大关节速度限制（rad/s） |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

limit = robot.get_joint_angle_vel_limits(1)
if limit is not None:
    print(limit.msg.min_angle_limit, limit.msg.max_angle_limit)
    print(limit.msg.min_joint_spd, limit.msg.max_joint_spd)
```

---

### 读取关节加速度限制 — `get_joint_acc_limits()`

**功能说明：** 查询指定关节的最大加速度限制。

**函数定义：**

```python
get_joint_acc_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6],
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[ArmMsgFeedbackCurrentMotorMaxAccLimit] | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号，范围：`1~6` |
| `timeout` | `float` | 等待反馈超时（秒），默认 `1.0`；`0.0` 表示非阻塞 |
| `min_interval` | `float` | 最小请求间隔（秒），默认 `1.0` |

**返回值：** `MessageAbstract[ArmMsgFeedbackCurrentMotorMaxAccLimit] | None`

**消息字段（`.msg`）：**

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `max_joint_acc` | `float` | 最大关节加速度限制（rad/s²） |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

limit = robot.get_joint_acc_limits(1)
if limit is not None:
    print(limit.msg.max_joint_acc)
    print(limit.hz, limit.timestamp)
```

---

### 读取法兰速度/加速度限制 — `get_flange_vel_acc_limits()`

**功能说明：** 查询末端最大线速度/角速度与线加速度/角加速度限制。

**函数定义：**

```python
get_flange_vel_acc_limits(
    self,
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam] | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `timeout` | `float` | 等待反馈超时（秒），默认 `1.0`；`0.0` 表示非阻塞 |
| `min_interval` | `float` | 最小请求间隔（秒），默认 `1.0` |

**返回值：** `MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam] | None`

**消息字段（`.msg`）：**

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `end_max_linear_vel` | `float` | 末端最大线速度（m/s） |
| `end_max_angular_vel` | `float` | 末端最大角速度（rad/s） |
| `end_max_linear_acc` | `float` | 末端最大线加速度（m/s²） |
| `end_max_angular_acc` | `float` | 末端最大角加速度（rad/s²） |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
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

---

### 读取碰撞防护等级 — `get_crash_protection_rating()`

**功能说明：** 查询各关节碰撞防护等级（控制器返回列表）。

**函数定义：**

```python
get_crash_protection_rating(
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[list[int]] | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `timeout` | `float` | 等待反馈超时（秒），默认 `1.0`；`0.0` 表示非阻塞 |
| `min_interval` | `float` | 最小请求间隔（秒），默认 `1.0` |

**返回值：** `MessageAbstract[list[int]] | None`

`.msg` 为碰撞防护等级列表（按关节顺序），每项为 `int`（范围：`0~8`）。**等级越高越敏感，越容易触发碰撞保护机制**（更保守）。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

rating = robot.get_crash_protection_rating()
if rating is not None:
    print(rating.msg)
    print(rating.hz, rating.timestamp)
```

---

### 读取关节助力等级 — `get_joint_assistance_rating()`

**功能说明：** 读取全部关节的助力等级。

**函数定义：**

```python
get_joint_assistance_rating(
    self,
    timeout: float = 1.0,
    min_interval: float = 1.0,
) -> MessageAbstract[list[int]] | None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `timeout` | `float` | 等待反馈超时（秒），默认 `1.0`；`0.0` 表示非阻塞 |
| `min_interval` | `float` | 最小请求间隔（秒），默认 `1.0` |

**返回值：** `MessageAbstract[list[int]] | None`  
其中 `.msg` 为 `list[int]`（长度 6），每个元素取值范围 `0~10`。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

rating = robot.get_joint_assistance_rating()
if rating is not None:
    print(rating.msg)
    print(rating.hz, rating.timestamp)
```

---

### 关节置零/标定 — `calibrate_joint()`

**功能说明：** 对指定关节执行置零/标定流程（等待控制器 ACK/响应并返回结果）。

**函数定义：**

```python
calibrate_joint(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    timeout: float = 1.0,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | `1~6` 标定单关节；`255` 标定全部 |
| `timeout` | `float` | 等待响应超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示收到成功响应；`False` 表示超时或失败。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

joint_index = 1
robot.disable(joint_index)
time.sleep(0.2)
input("请手动将关节移动到零位位置后按回车继续...")

if robot.calibrate_joint(joint_index):
    print("calibrate_joint success")
```

---

### 清除关节错误码 — `clear_joint_error()`

**功能说明：** 清除单关节或全部关节错误码。

**函数定义：**

```python
clear_joint_error(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    timeout: float = 1.0,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号：`1~6` 清除单关节；`255` 清除全部 |
| `timeout` | `float` | 等待 ACK 超时（秒），默认 `1.0` |

**返回值：** `bool` — 该接口仅做 ACK 校验（`True` 表示超时内收到了响应）。

> **提示：** 该接口仅确认收到 ACK/响应，不包含自动读回校验。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 清除 2 号关节错误
ok = robot.clear_joint_error(joint_index=2)
print("clear_joint_error(j2) =", ok)

# 清除全部关节错误
ok = robot.clear_joint_error(joint_index=255)
print("clear_joint_error(all) =", ok)
```

---

### 配置关节角度/速度限制 — `set_joint_angle_vel_limits()`

**功能说明：** 设置关节角度/速度限制，并可通过读回校验是否生效。

**函数定义：**

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

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号：`1~6` 配置单关节；`255` 配置全部 |
| `min_angle_limit` | `Optional[float]` | 最小角度限制（rad）；`None` 表示不配置 |
| `max_angle_limit` | `Optional[float]` | 最大角度限制（rad）；`None` 表示不配置 |
| `max_joint_spd` | `Optional[float]` | 最大关节速度限制（rad/s）；`None` 表示不配置 |
| `timeout` | `float` | 等待 ACK/校验超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示已收到 ACK 且读回校验通过；`False` 表示超时/失败/校验未通过。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 同时设置角度和速度限制
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

---

### 配置关节加速度限制 — `set_joint_acc_limits()`

**功能说明：** 设置指定关节的最大加速度限制。

**函数定义：**

```python
set_joint_acc_limits(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    max_joint_acc: Optional[float] = None,
    timeout: float = 1.0,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号：`1~6` 配置单关节；`255` 配置全部 |
| `max_joint_acc` | `Optional[float]` | 最大加速度（rad/s²）；`None` 表示不配置 |
| `timeout` | `float` | 等待 ACK/校验超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示已收到 ACK 且读回校验通过。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_joint_acc_limits(joint_index=1, max_joint_acc=5.0)
print("set_joint_acc_limits success =", success)
```

---

### 配置法兰速度/加速度限制 — `set_flange_vel_acc_limits()`

**功能说明：** 设置末端速度/加速度限制。

**函数定义：**

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

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `max_linear_vel` | `Optional[float]` | 最大线速度（m/s）；`None` 表示不配置 |
| `max_angular_vel` | `Optional[float]` | 最大角速度（rad/s）；`None` 表示不配置 |
| `max_linear_acc` | `Optional[float]` | 最大线加速度（m/s²）；`None` 表示不配置 |
| `max_angular_acc` | `Optional[float]` | 最大角加速度（rad/s²）；`None` 表示不配置 |
| `timeout` | `float` | 等待 ACK/校验超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示已收到 ACK 且读回校验通过。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
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

---

### 配置碰撞防护等级 — `set_crash_protection_rating()`

**功能说明：** 设置碰撞防护等级（可指定单关节或全部关节）。

**函数定义：**

```python
set_crash_protection_rating(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    rating: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8] = 0,
    timeout: float = 1.0,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号：`1~6` 配置单关节；`255` 配置全部，默认：`255` |
| `rating` | `int` | 碰撞防护等级，范围：`[0, 8]`（`0` = 不检测），默认：`0`。**等级越高越敏感，越容易触发碰撞保护**（更保守） |
| `timeout` | `float` | 等待 ACK/校验超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示已收到 ACK 且读回校验通过。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_crash_protection_rating(joint_index=1, rating=1)
print("set_crash_protection_rating success =", success)
```

---

### 配置关节助力等级 — `set_joint_assistance_rating()`

**功能说明：** 设置单关节或全部关节的助力等级。

**函数定义：**

```python
set_joint_assistance_rating(
    self,
    joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255,
    rating: Literal[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10] = 0,
    timeout: float = 1.0,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_index` | `int` | 关节序号：`1~6` 配置单关节；`255` 配置全部 |
| `rating` | `int` | 助力等级，范围 `[0, 10]` |
| `timeout` | `float` | 等待 ACK/校验超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示收到 ACK 且读回校验通过。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# 设置 1 号关节助力等级
ok = robot.set_joint_assistance_rating(joint_index=1, rating=3)
print("set_joint_assistance_rating(j1) =", ok)

# 设置全部关节助力等级
ok = robot.set_joint_assistance_rating(joint_index=255, rating=2)
print("set_joint_assistance_rating(all) =", ok)
```

---

### 恢复法兰限制默认值 — `set_flange_vel_acc_limits_to_default()`

**功能说明：** 将末端速度/加速度限制恢复为默认值。

**函数定义：**

```python
set_flange_vel_acc_limits_to_default(
    self,
    timeout: float = 1.0,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `timeout` | `float` | 等待 ACK/响应超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示在超时内收到 ACK/响应。

> **提示：** 该接口不提供读回校验。如需确认，可调用 `get_flange_vel_acc_limits()` 查询。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_flange_vel_acc_limits_to_default()
print("set_flange_vel_acc_limits_to_default success =", success)
```

---

### 恢复关节限制默认值 — `set_joint_angle_vel_acc_limits_to_default()`

**功能说明：** 将关节角度/速度/加速度限制恢复为默认值。

**函数定义：**

```python
set_joint_angle_vel_acc_limits_to_default(
    self,
    timeout: float = 1.0,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `timeout` | `float` | 等待 ACK/响应超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示在超时内收到 ACK/响应。

> **提示：** 该接口不提供读回校验。如需确认，可调用 `get_joint_angle_vel_limits()` / `get_joint_acc_limits()` 查询。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_joint_angle_vel_acc_limits_to_default()
print("set_joint_angle_vel_acc_limits_to_default success =", success)
```

---

### 设置 Link 速度/加速度周期反馈 — `set_links_vel_acc_period_feedback()`

**功能说明：** 设置各关节 link 的笛卡尔速度/加速度周期反馈开关（对应 CAN 周期帧 `0x481~0x486`）。

> **⚠️ 注意：** 该功能在底层主控中 **已废弃**，但总线仍可能周期上报对应帧，且上报数据 **全为 0**，无实际意义。**建议默认关闭**（`enable=False`），避免占用带宽。
>
> 该接口无直接读回校验方式，建议使用 `candump` 观察周期帧是否出现来验证：
>
> ```bash
> candump can0 | grep "48[1-6]"
> ```

**函数定义：**

```python
set_links_vel_acc_period_feedback(
    self,
    enable: bool = False,
    timeout: float = 1.0,
) -> bool
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `enable` | `bool` | 是否开启周期反馈：`True` 开启；`False` 关闭（**建议默认关闭**） |
| `timeout` | `float` | 等待 ACK/响应超时（秒），默认 `1.0` |

**返回值：** `bool` — `True` 表示在超时内收到 ACK/响应。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

success = robot.set_links_vel_acc_period_feedback(enable=True)
print("enable periodic feedback success =", success)

success = robot.set_links_vel_acc_period_feedback(enable=False)
print("disable periodic feedback success =", success)
```
