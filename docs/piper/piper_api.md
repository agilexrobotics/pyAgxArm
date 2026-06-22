# Piper API Documentation

> This document describes the `pyAgxArm` API for Piper robotic arms, covering instance creation, status reading, motion control, and advanced parameter configuration.

## Table of Contents

- [Switch to 中文](#piper-机械臂-api-使用文档)
- [Import Module](#import-module)
- [Firmware Version](#firmware-version)
- [Create Instance and Connect](#create-instance-and-connect)
  - [Create Configuration — create_agx_arm_config()](#create-configuration--create_agx_arm_config)
  - [Create Arm Driver Instance — AgxArmFactory.create_arm()](#create-arm-driver-instance--agxarmfactorycreate_arm)
  - [Connect — connect()](#connect--connect)
  - [Disconnect — disconnect()](#disconnect--disconnect)
  - [Check Communication Error State — has_comm_error()](#check-communication-error-state--has_comm_error)
  - [Get Communication Error — get_comm_error()](#get-communication-error--get_comm_error)
  - [Initialize End Effector — init_effector()](#initialize-end-effector--init_effector)
- [General Status](#general-status)
  - [Get Joint Count — joint_nums](#get-joint-count--joint_nums)
  - [Check Communication — is_ok()](#check-communication--is_ok)
  - [Get Data Receive Frequency — get_fps()](#get-data-receive-frequency--get_fps)
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
- [TCP Related](#tcp-related)
  - [Set TCP Offset — set_tcp_offset()](#set-tcp-offset--set_tcp_offset)
  - [Get TCP Pose — get_tcp_pose()](#get-tcp-pose--get_tcp_pose)
  - [Flange Pose to TCP Pose — get_flange2tcp_pose()](#flange-pose-to-tcp-pose--get_flange2tcp_pose)
  - [TCP Pose to Flange Pose — get_tcp2flange_pose()](#tcp-pose-to-flange-pose--get_tcp2flange_pose)
- [Kinematics Related](#kinematics-related)
  - [Get IK Joint Angles — get_ik_joint_angles()](#get-ik-joint-angles--get_ik_joint_angles)
  - [Forward Kinematics — fk()](#forward-kinematics--fk)
- [SDK Config Related](#sdk-config-related)
  - [Set Auto Motion Mode Switching — set_auto_set_motion_mode_enabled()](#set-auto-motion-mode-switching--set_auto_set_motion_mode_enabled)
  - [Get Auto Motion Mode Switching State — get_auto_set_motion_mode_enabled()](#get-auto-motion-mode-switching-state--get_auto_set_motion_mode_enabled)
  - [Set Joint Limits Enabled — set_joint_limits_enabled()](#set-joint-limits-enabled--set_joint_limits_enabled)
  - [Get Joint Limits Enabled State — get_joint_limits_enabled()](#get-joint-limits-enabled-state--get_joint_limits_enabled)
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

## Import Module

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW
```

---

## Firmware Version

Piper firmware versioning is independent from Nero. The SDK uses `firmeware_version` in `create_agx_arm_config()` to select the matching driver (software version strings such as `"S-V1.8-8"`).

Check the arm firmware with [get_firmware()](#get-firmware-info--get_firmware) (format **S-VX.X-X**), then:

| Your firmware | `firmeware_version` | Constant |
| --- | --- | --- |
| S-V1.8-9 or later | `"v189"` | `PiperFW.V189` |
| S-V1.8-8 | `"v188"` | `PiperFW.V188` |
| S-V1.8-3 ~ S-V1.8-7 | `"v183"` | `PiperFW.V183` |
| S-V1.8-2 or earlier | `"default"` (or omit) | `PiperFW.DEFAULT` |

> Version evolution, API availability, and behavioral differences: [Piper Firmware Reference](firmware_reference.md#piper-firmware-reference).

**Usage Example (recommended — use constants for IDE auto-complete):**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

# Firmware is S-V1.8-2, use PiperFW.DEFAULT
cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
```

Raw strings are also accepted (backward-compatible):

```python
cfg = create_agx_arm_config(robot="piper", firmeware_version="default", channel="can0")
```

---

## Create Instance and Connect

### Create Configuration — `create_agx_arm_config()`

**Description:** Generate the configuration dictionary required by the robotic arm, used to create a Driver instance.

**Function Definition:**

```python
create_agx_arm_config(
    robot: Literal["nero", "piper", "piper_h", "piper_l", "piper_x"],
    comm: Literal["can"] = "can",
    firmeware_version: str = "default",
    **kwargs,
) -> dict
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `robot` | `str` | Robotic arm model. Use `ArmModel` constants: `ArmModel.PIPER` / `ArmModel.PIPER_H` / `ArmModel.PIPER_L` / `ArmModel.PIPER_X` / `ArmModel.NERO` (raw strings `"piper"` etc. also accepted) |
| `comm` | `str` | Communication type. Options: `"can"` (default). Note: `comm` is not the CAN channel name; the CAN channel is specified by `channel` |
| `firmeware_version` | `str` | Main controller firmware version. Use per-robot constants: Piper series → `PiperFW.DEFAULT` / `PiperFW.V183` / `PiperFW.V188` / `PiperFW.V189`. See [Firmware Version](#firmware-version) for details. Default `"default"` |

**Optional Keyword Arguments (`**kwargs`):**

| Name | Type | Description |
| --- | --- | --- |
| `joint_limits` | `dict` | Custom joint limits (unit: rad). Automatically assigned by default; manually entered limits are not currently applied to actual control. See example below |
| `channel` | `str` | CAN channel identifier. Default `"can0"`. The documented and verified combinations are: with `"agx_cando"` use device index strings such as `"0"`, `"1"`, `"2"`; with `"socketcan"` use Linux CAN netdev names such as `"can0"` or your renamed interface; with `"slcan"` use serial device paths such as `"/dev/ttyACM0"` on macOS (`Darwin`). |
| `interface` | `str` | CAN interface type, default `"socketcan"`. The documented and verified values are `"socketcan"` on Linux, `"agx_cando"` on Windows with the Agilex CANDO backend, and `"slcan"` on macOS (`Darwin`). |
| `bitrate` | `int` | CAN baud rate, default `1000000` (1 Mbps) |
| `enable_check_can` | `bool` | Whether to check the CAN module when creating a Comm instance, default `True`. This pre-check is currently only effective for Linux `socketcan`; for other backends (for example, Windows `agx_cando` and macOS `slcan`) the actual availability check happens when the CAN bus is opened. |
| `auto_connect` | `bool` | Whether to automatically create a CAN Bus instance, default `True` |
| `timeout` | `float` | CAN Bus read/write timeout (seconds), default `1.0` |
| `receive_own_messages` | `bool` | Whether the local CAN backend should receive frames sent by the same process/device. Default `False`. This is useful for debugging, loopback tests, or virtual/single-node verification, but is usually not recommended for normal arm control. Backend support depends on the selected `interface`. The `slcan` backend on macOS generally does not support this; **do not pass** it when using `interface="slcan"`. |
| `local_loopback` | `bool` | Whether to enable CAN **local loopback**. Default is `False` (loopback disabled), so your local terminal/process will **not** receive the CAN frames it sends itself. You may enable it for debugging, but it is **not recommended** for normal SDK usage because it may consume bus receive resources and impact reading performance. The `slcan` backend on macOS generally does not support this; **do not pass** it when using `interface="slcan"`. |

**Return Value:** `dict`

Example return structure:

```json
{
    "robot": "piper",
    "firmeware_version": "default",
    "log": {
        "level": "INFO",
        "path": ""
    },
    "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    "joint_limits": {
        "joint1": [-2.617994, 2.617994],
        "joint2": [0.0, 3.141593],
        "joint3": [-2.967060, 0.0],
        "joint4": [-1.745330, 1.745330],
        "joint5": [-1.221730, 1.221730],
        "joint6": [-2.094396, 2.094396]
    },
    "comm": {
        "type": "can",
        "can": {
            "channel": "can0",
            "interface": "socketcan",
            "bitrate": 1000000,
            "enable_check_can": true,
            "auto_connect": true,
            "timeout": 1.0,
            "receive_own_messages": false,
            "local_loopback": false
        }
    }
}
```

Verified interface/channel examples:

- Linux `socketcan`: `create_agx_arm_config(..., interface="socketcan", channel="can0")`
- Windows `agx_cando`: `create_agx_arm_config(..., interface="agx_cando", channel="0")`
- macOS `slcan`: `create_agx_arm_config(..., interface="slcan", channel="/dev/ttyACM0")`

On Windows, `interface="agx_cando"` requires the separately installed `python-can-agx-cando` plugin. Install it from `https://github.com/agilexrobotics/python-can-agx-cando.git`, then run `pip3 install .` in that repository before using `pyAgxArm`.
On macOS (`Darwin`), when using `interface="slcan"` with the default channel, grant serial permission first.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
print(cfg)
```

---

### Create Arm Driver Instance — `AgxArmFactory.create_arm()`

**Description:** Create the corresponding robotic arm Driver instance via factory method based on the configuration dictionary.

**Function Definition:**

```python
create_arm(cls, config: dict, **kwargs) -> T
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `config` | `dict` | Configuration dictionary generated by `create_agx_arm_config()` |

**Return Value:** `Driver` — Different arm models, communication methods, and firmware versions correspond to different instances.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
```

---

### Connect — `connect()`

**Description:** Establish the connection and start the data reading thread.

**Function Definition:**

```python
connect(self, start_read_thread: bool = True) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `start_read_thread` | `bool` | Whether to start the data reading thread, default `True` |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
```

---

### Disconnect — `disconnect()`

**Description:** Disconnect from the arm and release underlying threads and CAN resources.

This method is **idempotent** and can be safely called when the arm instance is no longer needed, e.g. after reading firmware version and before creating a new instance.

> **Note:** After `disconnect()`, the internal communication handle may be released. Calling `robot.is_connected()` will return `False`.

**Function Definition:**

```python
disconnect(self, join_timeout: float = 1.0) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `join_timeout` | `float` | Timeout (seconds) for joining background threads during shutdown, default `1.0` |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
print(robot.is_connected())

robot.disconnect()
print(robot.is_connected())
```

---

### Check Communication Error State — `has_comm_error()`

**Description:** Check whether the communication layer is currently in an error state.

**Function Definition:**

```python
has_comm_error(self) -> bool
```

**Return Value:** `bool` — `True` if a communication error has been recorded; otherwise `False`.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

if robot.has_comm_error():
    print("Communication error detected.")
```

---

### Get Communication Error — `get_comm_error()`

**Description:** Get the most recent communication error information.

**Function Definition:**

```python
get_comm_error(self)
```

**Return Value:** `Any` — The error object stored in the communication context. Usually returns `None` when there is no current error.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

err = robot.get_comm_error()
if err is not None:
    print("Last communication error:", err)
```

---

### Initialize End Effector — `init_effector()`

**Description:** Initialize the end effector Driver and return the corresponding effector instance (e.g., gripper / dexterous hand, etc.).

> **Note:** A single `robot` instance can only initialize an end effector **once**. To switch to a different effector type, create a new robotic arm instance.

**Function Definition:**

```python
init_effector(self, effector: str) -> EffectorDriver
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `effector` | `str` | Effector type (recommended to use `robot.OPTIONS.EFFECTOR.xxx` constants) |

**Return Value:** `EffectorDriver`

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
```

---

## General Status

### Get Joint Count — `joint_nums`

**Description:** Get the number of joints of the robotic arm (e.g., Piper has 6).

**Attribute Definition:**

```python
joint_nums: int
```

**Return Value:** `int`

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

print("robotic arm joint_nums =", robot.joint_nums)

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

---

### Check Communication — `is_ok()`

**Description:** Check whether the robotic arm data reception is normal. This value is computed by the SDK's internal data monitoring logic based on whether data has not been received for a period of time.

**Function Definition:**

```python
is_ok(self) -> bool
```

**Return Value:** `bool`

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

time.sleep(0.5)
print("robotic arm is_ok =", robot.is_ok())
```

---

### Get Data Receive Frequency — `get_fps()`

**Description:** Get the data monitoring receive frequency (Hz) of the robotic arm, which is a statistical value from the SDK for data received by the parser.

**Function Definition:**

```python
get_fps(self) -> float
```

**Return Value:** `float` (unit: Hz)

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

time.sleep(0.5)
print("robotic arm fps =", robot.get_fps(), "Hz")
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
get_motor_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6]) -> MessageAbstract[ArmMsgFeedbackHighSpd] | None
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
get_driver_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6]) -> MessageAbstract[ArmMsgFeedbackLowSpd] | None
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
get_joint_enable_status(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255]) -> bool
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
    print("Joint-1 motor is enabled")
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
get_firmware(self, timeout: float = 1.0, min_interval: float = 1.0) -> dict | None
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
set_installation_pos(self, pos: Literal["horizontal", "left", "right"] = "horizontal") -> None
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
set_motion_mode(self, motion_mode: Literal["p", "j", "l", "c", "mit", "js"] = "p") -> None
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
set_payload(self, load: Literal['empty', 'half', 'full'] = 'empty', timeout: float = 1.0) -> bool
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

## TCP Related

### Set TCP Offset — `set_tcp_offset()`

**Description:** Set the TCP (Tool Center Point) offset pose relative to the flange (in the **flange coordinate frame**). Default is no offset: `[0, 0, 0, 0, 0, 0]`.

> **Tip:** This offset value is only saved in the SDK/Driver instance and is not sent to the controller.

**Function Definition:**

```python
set_tcp_offset(self, pose: list[float]) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `pose` | `list[float]` | TCP pose offset in the flange coordinate frame `[x, y, z, roll, pitch, yaw]`: `x, y, z` are position (m); `roll, pitch, yaw` are Euler angles (rad). Range: `roll/yaw` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]` |

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])
```

---

### Get TCP Pose — `get_tcp_pose()`

**Description:** Get the TCP pose. This interface first reads the flange pose, then applies a rigid body transformation using the offset saved by `set_tcp_offset()` to compute the TCP pose. If no offset is set, the TCP pose is the same as the flange pose.

**Function Definition:**

```python
get_tcp_pose(self) -> MessageAbstract[list[float]] | None
```

**Return Value:** `MessageAbstract[list[float]] | None`

`.msg` is a `list[float]` of length 6: `[x, y, z, roll, pitch, yaw]` (m / rad).

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
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

---

### Flange Pose to TCP Pose — `get_flange2tcp_pose()`

**Description:** Given a flange pose (in the base / world coordinate frame), compute the corresponding TCP pose using the offset saved by `set_tcp_offset()`.

**Function Definition:**

```python
get_flange2tcp_pose(self, flange_pose: list[float]) -> list[float]
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `flange_pose` | `list[float]` | Flange pose `[x, y, z, roll, pitch, yaw]` (m / rad). Range: `roll/yaw` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]` |

**Return Value:** `list[float]` — TCP pose `[x, y, z, roll, pitch, yaw]` (m / rad).

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])

# Specify a flange pose directly
tcp_pose = robot.get_flange2tcp_pose([0.30, 0.0, 0.30, 0.0, 1.5707, 0.0])
print("tcp_pose =", tcp_pose)

# Use current flange pose; result should match get_tcp_pose()
flange_pose = robot.get_flange_pose()
if flange_pose is not None:
    tcp_pose = robot.get_flange2tcp_pose(flange_pose)
    print("tcp_pose =", tcp_pose)
```

---

### TCP Pose to Flange Pose — `get_tcp2flange_pose()`

**Description:** Given a target TCP pose (in the base / world coordinate frame), compute the corresponding target flange pose using the offset saved by `set_tcp_offset()`. Pass the returned flange pose to `move_p()` to **move the TCP to the target TCP pose**.

**Function Definition:**

```python
get_tcp2flange_pose(self, tcp_pose: list[float]) -> list[float]
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `tcp_pose` | `list[float]` | Target TCP pose `[x, y, z, roll, pitch, yaw]` (m / rad). Range: `roll/yaw` ∈ `[-π, π]`, `pitch` ∈ `[-π/2, π/2]` |

**Return Value:** `list[float]` — Target flange pose `[x, y, z, roll, pitch, yaw]` (m / rad), can be directly used with `move_p()`.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])

target_tcp_pose = [0.30, 0.0, 0.30, 0.0, 1.5707, 0.0]
target_flange_pose = robot.get_tcp2flange_pose(target_tcp_pose)
print("target_flange_pose =", target_flange_pose)

# robot.move_p(target_flange_pose)  # Note: this will trigger motion
```

---

## Kinematics Related

### Get IK Joint Angles — `get_ik_joint_angles()`

> **Firmware requirement:** `PiperFW.V188+` (firmware **≥ S-V1.8-8**). See [Piper Firmware Reference](firmware_reference.md#apis-with-firmware-requirements).

**Description:** Get the **inverse kinematics (IK) joint angles** feedback computed by the controller for the current Cartesian motion command.

This feedback is published over CAN after the arm executes a Cartesian motion. It reflects the joint solution the controller selected for the target pose, which may differ from the measured joint angles from [get_joint_angles()](#get-joint-angles--get_joint_angles).

> **Note:** Only available after calling [move_p()](#point-to-point-motion--move_p).

**Function Definition:**

```python
get_ik_joint_angles(self) -> MessageAbstract[list[float]] | None
```

**Return Value:** `MessageAbstract[list[float]] | None`

- Returns `None` if IK joint-angle feedback has not been received yet or the data fails validation.
- `.msg` is a `list[float]` of length 6: `[j1, j2, j3, j4, j5, j6]`, unit: **rad**.

**Usage Example:**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.V188, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.move_p([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])

while True:
    ika = robot.get_ik_joint_angles()
    if ika is not None:
        print(ika.msg)
        print(ika.hz, ika.timestamp)
    time.sleep(0.005)
```

---

### Forward Kinematics — `fk()`

**Description:** Compute the end **flange pose** from a given set of joint angles using the robot's built-in modified DH model.

This is an **offline** computation (no CAN I/O). The output pose format matches `.msg` from [get_flange_pose()](#get-flange-pose--get_flange_pose):  
`[x, y, z, roll, pitch, yaw]` in the **base frame**, where `x/y/z` are meters and `roll/pitch/yaw` are radians (ZYX RPY convention used by the SDK).

**Function Definition:**

```python
fk(self, joint_angles: list[float]) -> list[float]
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_angles` | `list[float]` | Joint angles in **rad**, length 6: `[j1, j2, j3, j4, j5, j6]` |

**Return Value:** `list[float]`

`[x, y, z, roll, pitch, yaw]` — flange pose in base frame.

**Usage Examples:**

1) Combine with [get_joint_angles()](#get-joint-angles--get_joint_angles) (current arm state → FK):

```python
ja = robot.get_joint_angles()
if ja is not None:
    flange_pose = robot.fk(ja.msg)
    print("fk flange:", flange_pose)
```

2) Combine with [get_leader_joint_angles()](#get-leader-joint-angles--get_leader_joint_angles) (leader angles → FK):

```python
mja = robot.get_leader_joint_angles()
if mja is not None:
    leader_flange_pose = robot.fk(mja.msg)
    print("leader fk flange:", leader_flange_pose)
```

3) Combine with [get_flange2tcp_pose()](#flange-pose-to-tcp-pose--get_flange2tcp_pose) (FK flange → derived TCP):

```python
ja = robot.get_joint_angles()
if ja is not None:
    flange_pose = robot.fk(ja.msg)
    tcp_pose = robot.get_flange2tcp_pose(flange_pose)
    print("fk tcp:", tcp_pose)
```

4) Compare measured flange pose vs FK result (for quick consistency checks):

```python
ja = robot.get_joint_angles()
fp = robot.get_flange_pose()
if ja is not None and fp is not None:
    fk_fp = robot.fk(ja.msg)
    print("measured flange:", fp.msg)
    print("fk flange:", fk_fp)
```

---

## SDK Config Related

### Set Auto Motion Mode Switching — `set_auto_set_motion_mode_enabled()`

**Description:** Enable or disable automatic `set_motion_mode()` switching when calling `move_*` APIs at runtime.

- `True`: keep auto-switching behavior (default).
- `False`: do not auto switch; you need to call `set_motion_mode()` manually when needed.

**Function Definition:**

```python
set_auto_set_motion_mode_enabled(self, enabled: bool) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `enabled` | `bool` | Whether to enable automatic motion-mode switching |

**Usage Example:**

```python
robot.set_auto_set_motion_mode_enabled(False)
robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.J)
robot.move_j([0.0] * robot.joint_nums)
```

---

### Get Auto Motion Mode Switching State — `get_auto_set_motion_mode_enabled()`

**Description:** Get the current runtime state of automatic `set_motion_mode()` switching before `move_*` calls.

**Default:** `True`

**Function Definition:**

```python
get_auto_set_motion_mode_enabled(self) -> bool
```

**Return Value:** `bool`

- `True`: auto-switching is enabled.
- `False`: auto-switching is disabled.

**Usage Example:**

```python
enabled = robot.get_auto_set_motion_mode_enabled()
print("auto motion mode switching:", enabled)
```

---

### Set Joint Limits Enabled — `set_joint_limits_enabled()`

**Description:** Enable or disable software joint limits at runtime.

- `True`: joint commands are clamped by configured `joint_limits` / model limits.
- `False`: skip model `joint_limits` clamp and only keep basic numeric range protection.

**Function Definition:**

```python
set_joint_limits_enabled(self, enabled: bool) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `enabled` | `bool` | Whether to enable software joint limits |

**Usage Example:**

```python
robot.set_joint_limits_enabled(False)
robot.move_j([0.0] * robot.joint_nums)
robot.set_joint_limits_enabled(True)
```

---

### Get Joint Limits Enabled State — `get_joint_limits_enabled()`

**Description:** Get whether runtime software joint-limit clamping is currently enabled.

**Default:** `False`

**Function Definition:**

```python
get_joint_limits_enabled(self) -> bool
```

**Return Value:** `bool`

- `True`: software joint limits are enabled.
- `False`: software joint limits are disabled.

**Usage Example:**

```python
enabled = robot.get_joint_limits_enabled()
print("joint limits enabled:", enabled)
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
enable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255) -> bool
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
disable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255) -> bool
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

# Wait until motion completes (5s timeout)
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("Target position reached")
        break
    if time.monotonic() - start_t > 5.0:
        print("Timed out waiting for motion completion (5s)")
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

# Wait until motion completes (5s timeout)
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("Target position reached")
        break
    if time.monotonic() - start_t > 5.0:
        print("Timed out waiting for motion completion (5s)")
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

# Wait until motion completes (5s timeout)
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("Target position reached")
        break
    if time.monotonic() - start_t > 5.0:
        print("Timed out waiting for motion completion (5s)")
        break
    time.sleep(0.1)
```

---

### Arc Motion — `move_c()`

**Description:** Perform arc trajectory planning and execution using three target flange poses: "start / midpoint / end".

**Function Definition:**

```python
move_c(self, start_pose: list[float], mid_pose: list[float], end_pose: list[float]) -> None
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

# Wait until motion completes (5s timeout)
time.sleep(0.5)
start_t = time.monotonic()
while True:
    status = robot.get_arm_status()
    if status is not None and status.msg.motion_status == 0:
        print("Target position reached")
        break
    if time.monotonic() - start_t > 5.0:
        print("Timed out waiting for motion completion (5s)")
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

> **Note:** Consecutive execution of this command will overwrite the previous target value.
>
> **`t_ff`** ranges and encoding differ by firmware — see [MIT parameters by version](firmware_reference.md#mit-move_mit-parameters-by-version).

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

> **Note:** `set_cpv_*` APIs save parameters to the motor controller Flash. Flash has a finite write lifetime, so avoid setting these parameters frequently or repeatedly during runtime; excessive writes may cause abnormal motor behavior.

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
get_flange_vel_acc_limits(self, timeout: float = 1.0, min_interval: float = 1.0) -> MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam] | None
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
input("Please move the joint to the zero position manually, then press Enter to continue...")

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

# Set both angle and speed limits
success = robot.set_joint_angle_vel_limits(
    joint_index=1,
    min_angle_limit=-2.618,
    max_angle_limit=2.618,
    max_joint_spd=3.0,
)
print("set_joint_angle_vel_limits success =", success)

# Set only max speed limit (keep angle limits unchanged)
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
set_flange_vel_acc_limits_to_default(self, timeout: float = 1.0) -> bool
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
set_joint_angle_vel_acc_limits_to_default(self, timeout: float = 1.0) -> bool
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
set_links_vel_acc_period_feedback(self, enable: bool = False, timeout: float = 1.0) -> bool
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

> 本文档描述 `pyAgxArm` SDK 为 Piper 系列机械臂提供的 Python API。涵盖实例创建、状态读取、运动控制、参数配置等全部接口。

## 目录

- [切换到 English](#piper-api-documentation)
- [导入模块](#导入模块)
- [固件版本选择](#固件版本选择)
- [创建实例并连接](#创建实例并连接)
  - [创建配置参数 — create_agx_arm_config()](#创建配置参数--create_agx_arm_config)
  - [创建机械臂 Driver 实例 — AgxArmFactory.create_arm()](#创建机械臂-driver-实例--agxarmfactorycreate_arm)
  - [创建连接 — connect()](#创建连接--connect)
  - [断开连接 — disconnect()](#断开连接--disconnect)
  - [检查通信错误状态 — has_comm_error()](#检查通信错误状态--has_comm_error)
  - [获取通信错误信息 — get_comm_error()](#获取通信错误信息--get_comm_error)
  - [初始化末端执行器 — init_effector()](#初始化末端执行器--init_effector)
- [通用状态](#通用状态)
  - [获取关节数量 — joint_nums](#获取关节数量--joint_nums)
  - [通信是否正常 — is_ok()](#通信是否正常--is_ok)
  - [获取数据接收频率 — get_fps()](#获取数据接收频率--get_fps)
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
- [TCP 相关](#tcp-相关)
  - [设置 TCP 偏移 — set_tcp_offset()](#设置-tcp-偏移--set_tcp_offset)
  - [获取 TCP 位姿 — get_tcp_pose()](#获取-tcp-位姿--get_tcp_pose)
  - [法兰位姿转 TCP 位姿 — get_flange2tcp_pose()](#法兰位姿转-tcp-位姿--get_flange2tcp_pose)
  - [TCP 位姿转法兰位姿 — get_tcp2flange_pose()](#tcp-位姿转法兰位姿--get_tcp2flange_pose)
- [运动学相关](#运动学相关)
  - [读取 IK 关节角 — get_ik_joint_angles()](#读取-ik-关节角--get_ik_joint_angles)
  - [正运动学 — fk()](#正运动学--fk)
- [SDK 配置相关](#sdk-配置相关)
  - [设置自动切换运动模式开关 — set_auto_set_motion_mode_enabled()](#设置自动切换运动模式开关--set_auto_set_motion_mode_enabled)
  - [获取自动切换运动模式开关状态 — get_auto_set_motion_mode_enabled()](#获取自动切换运动模式开关状态--get_auto_set_motion_mode_enabled)
  - [设置关节软件限位开关 — set_joint_limits_enabled()](#设置关节软件限位开关--set_joint_limits_enabled)
  - [获取关节软件限位开关状态 — get_joint_limits_enabled()](#获取关节软件限位开关状态--get_joint_limits_enabled)
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

## 导入模块

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW
```

---

## 固件版本选择

Piper 与 Nero 固件体系相互独立。SDK 通过 `create_agx_arm_config()` 的 `firmeware_version` 选择匹配驱动（软件版本号如 `"S-V1.8-8"`）。

通过 [get_firmware()](#读取固件信息--get_firmware) 读取主控固件（格式 **S-VX.X-X**），按下表选择：

| 固件版本 | `firmeware_version` | 常量 |
| --- | --- | --- |
| S-V1.8-9 及更新 | `"v189"` | `PiperFW.V189` |
| S-V1.8-8 | `"v188"` | `PiperFW.V188` |
| S-V1.8-3 ~ S-V1.8-7 | `"v183"` | `PiperFW.V183` |
| S-V1.8-2 及更早 | `"default"`（或不填） | `PiperFW.DEFAULT` |

> 版本演进、API 可用性与行为差异详见 [Piper 固件参考](firmware_reference.md#piper-固件参考)。

**使用示例（推荐 — 使用常量类获得 IDE 自动补全）：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

# 固件为 S-V1.8-2，选择 PiperFW.DEFAULT
cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
```

也兼容原始字符串写法：

```python
cfg = create_agx_arm_config(robot="piper", firmeware_version="default", channel="can0")
```

---

## 创建实例并连接

### 创建配置参数 — `create_agx_arm_config()`

**功能说明：** 生成机械臂所需的配置字典，用于后续创建 Driver 实例。

**函数定义：**

```python
create_agx_arm_config(
    robot: Literal["nero", "piper", "piper_h", "piper_l", "piper_x"],
    comm: Literal["can"] = "can",
    firmeware_version: str = "default",
    **kwargs,
) -> dict
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `robot` | `str` | 机械臂型号。推荐使用 `ArmModel` 常量：`ArmModel.PIPER` / `ArmModel.PIPER_H` / `ArmModel.PIPER_L` / `ArmModel.PIPER_X` / `ArmModel.NERO`（也兼容原始字符串 `"piper"` 等） |
| `comm` | `str` | 通讯类型，可选值：`"can"`（默认）。注意：`comm` 不是 CAN 通道名，CAN 通道由 `channel` 指定 |
| `firmeware_version` | `str` | 主控固件版本。推荐使用按机型分类的常量：Piper 系列 → `PiperFW.DEFAULT` / `PiperFW.V183` / `PiperFW.V188` / `PiperFW.V189`。选择方法见[固件版本选择](#固件版本选择)。默认 `"default"` |

**可选关键字参数（`**kwargs`）：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_limits` | `dict` | 自定义关节限位（单位：rad）。默认自动赋值，暂不会将手动输入的限位生效到实际控制中。示例见下文 |
| `channel` | `str` | CAN 通道标识，默认 `"can0"`。当前文档已验证的写法为：`"agx_cando"` 使用 `"0"`、`"1"`、`"2"` 这类设备索引字符串；`"socketcan"` 使用 Linux 下的 CAN 网卡名，例如 `"can0"` 或重命名后的接口名；`"slcan"` 在 macOS（`Darwin`）下使用串口设备路径，例如 `"/dev/ttyACM0"`。 |
| `interface` | `str` | CAN 接口类型，默认 `"socketcan"`。当前文档已验证并提供说明的取值为 Linux 下的 `"socketcan"`、Windows 下 Agilex CANDO 后端使用的 `"agx_cando"`、以及 macOS（`Darwin`）下的 `"slcan"`。 |
| `bitrate` | `int` | CAN 波特率，默认 `1000000`（1 Mbps） |
| `enable_check_can` | `bool` | 是否在创建 Comm 实例时检查 CAN 模块，默认 `True`。当前该预检查主要只对 Linux `socketcan` 生效；其他后端（如 Windows `agx_cando`、macOS `slcan`）通常会在实际打开 CAN bus 时完成可用性检查。 |
| `auto_connect` | `bool` | 是否自动创建 CAN Bus 实例，默认 `True` |
| `timeout` | `float` | CAN Bus 读写超时时间（秒），默认 `1.0` |
| `receive_own_messages` | `bool` | 是否让本地 CAN 后端接收由同一进程/设备发送出去的报文。默认 `False`。适合调试、回环测试或单节点联调，正常机械臂控制一般不建议开启。具体是否生效取决于所选 `interface`。macOS 下的 `slcan` 后端通常**不支持**该项；使用 `interface="slcan"` 时**不要**传入。 |
| `local_loopback` | `bool` | 是否开启 CAN **本地回环**。默认 `False`（关闭回环），本地终端/进程将**无法**接收到自己发送的 CAN 报文。调试时可选择开启，但**不建议**在正常使用 SDK 时开启，因为可能会占用读取 bus 的资源并影响读取性能。macOS 下的 `slcan` 后端通常**不支持**该项；使用 `interface="slcan"` 时**不要**传入。 |

**返回值：** `dict`

返回结构示例：

```json
{
    "robot": "piper",
    "firmeware_version": "default",
    "log": {
        "level": "INFO",
        "path": ""
    },
    "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    "joint_limits": {
        "joint1": [-2.617994, 2.617994],
        "joint2": [0.0, 3.141593],
        "joint3": [-2.967060, 0.0],
        "joint4": [-1.745330, 1.745330],
        "joint5": [-1.221730, 1.221730],
        "joint6": [-2.094396, 2.094396]
    },
    "comm": {
        "type": "can",
        "can": {
            "channel": "can0",
            "interface": "socketcan",
            "bitrate": 1000000,
            "enable_check_can": true,
            "auto_connect": true,
            "timeout": 1.0,
            "receive_own_messages": false,
            "local_loopback": false
        }
    }
}
```

已验证的接口与通道填写示例：

- Linux `socketcan`：`create_agx_arm_config(..., interface="socketcan", channel="can0")`
- Windows `agx_cando`：`create_agx_arm_config(..., interface="agx_cando", channel="0")`
- macOS `slcan`：`create_agx_arm_config(..., interface="slcan", channel="/dev/ttyACM0")`

在 Windows 上使用 `interface="agx_cando"` 前，需要先单独安装 `python-can-agx-cando` 插件。可先从 `https://github.com/agilexrobotics/python-can-agx-cando.git` 克隆仓库，再进入仓库目录执行 `pip3 install .` 完成安装。
在 macOS（`Darwin`）下使用 `interface="slcan"` 且默认通道时，需要先给予串口权限。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
print(cfg)
```

---

### 创建机械臂 Driver 实例 — `AgxArmFactory.create_arm()`

**功能说明：** 根据配置字典，通过工厂方法创建对应的机械臂 Driver 实例。

**函数定义：**

```python
create_arm(cls, config: dict, **kwargs) -> T
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `config` | `dict` | 由 `create_agx_arm_config()` 生成的配置字典 |

**返回值：** `Driver` — 不同臂型号、通讯方式、固件版本对应不同的实例。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
```

---

### 创建连接 — `connect()`

**功能说明：** 创建连接并启动数据读取线程。

**函数定义：**

```python
connect(self, start_read_thread: bool = True) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `start_read_thread` | `bool` | 是否启动读取数据线程，默认 `True` |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
```

---

### 断开连接 — `disconnect()`

**功能说明：** 断开机械臂连接，并释放后台线程与 CAN 资源。

该方法是 **幂等（idempotent）** 的：重复调用不会报错。通常用于“当前 `robot` 实例不再需要”的场景，例如读完固件版本后准备创建新的实例。

> **注意：** 调用 `disconnect()` 后，底层通信句柄可能会被释放；此时调用 `robot.is_connected()` 会返回 `False`。

**函数定义：**

```python
disconnect(self, join_timeout: float = 1.0) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `join_timeout` | `float` | 关闭时等待后台线程退出的超时时间（秒），默认 `1.0` |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
print(robot.is_connected())

robot.disconnect()
print(robot.is_connected())
```

---

### 检查通信错误状态 — `has_comm_error()`

**功能说明：** 判断当前通信层是否处于错误状态。

**函数定义：**

```python
has_comm_error(self) -> bool
```

**返回值：** `bool` —— `True` 表示通信层已记录错误；`False` 表示当前未检测到通信错误。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

if robot.has_comm_error():
    print("检测到通信错误。")
```

---

### 获取通信错误信息 — `get_comm_error()`

**功能说明：** 获取最近一次通信错误信息。

**函数定义：**

```python
get_comm_error(self)
```

**返回值：** `Any` —— 通信上下文记录的错误对象；若当前无错误，通常返回 `None`。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

err = robot.get_comm_error()
if err is not None:
    print("最近一次通信错误:", err)
```

---

### 初始化末端执行器 — `init_effector()`

**功能说明：** 初始化末端执行器 Driver，并返回对应的执行器实例（例如夹爪 / 灵巧手等）。

> **注意：** 同一个 `robot` 实例 **只能初始化一次** 执行器。如需切换到其它执行器类型，请创建新的机械臂实例。

**函数定义：**

```python
init_effector(self, effector: str) -> EffectorDriver
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `effector` | `str` | 执行器类型（建议使用 `robot.OPTIONS.EFFECTOR.xxx` 常量） |

**返回值：** `EffectorDriver`

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
```

---

## 通用状态

### 获取关节数量 — `joint_nums`

**功能说明：** 获取机械臂关节数量（例如 Piper 为 6）。

**属性定义：**

```python
joint_nums: int
```

**返回值：** `int`

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

print("robotic arm joint_nums =", robot.joint_nums)

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

---

### 通信是否正常 — `is_ok()`

**功能说明：** 判断机械臂数据接收是否正常。该值由 SDK 内部的数据监控逻辑根据"最近一段时间是否持续收不到数据"计算得出。

**函数定义：**

```python
is_ok(self) -> bool
```

**返回值：** `bool`

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

time.sleep(0.5)
print("robotic arm is_ok =", robot.is_ok())
```

---

### 获取数据接收频率 — `get_fps()`

**功能说明：** 获取机械臂数据监控的接收频率（Hz），是 SDK 对解析器收到数据的统计值。

**函数定义：**

```python
get_fps(self) -> float
```

**返回值：** `float`（单位：Hz）

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

time.sleep(0.5)
print("robotic arm fps =", robot.get_fps(), "Hz")
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
get_motor_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6]) -> MessageAbstract[ArmMsgFeedbackHighSpd] | None
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
get_driver_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6]) -> MessageAbstract[ArmMsgFeedbackLowSpd] | None
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
get_joint_enable_status(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255]) -> bool
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
get_firmware(self, timeout: float = 1.0, min_interval: float = 1.0) -> dict | None
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
set_installation_pos(self, pos: Literal["horizontal", "left", "right"] = "horizontal") -> None
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
set_motion_mode(self, motion_mode: Literal["p", "j", "l", "c", "mit", "js"] = "p") -> None
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
set_payload(self, load: Literal['empty', 'half', 'full'] = 'empty', timeout: float = 1.0) -> bool
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

## TCP 相关

### 设置 TCP 偏移 — `set_tcp_offset()`

**功能说明：** 设置 TCP（工具中心点）相对于法兰（`flange`）的偏移位姿（在 **法兰坐标系** 下）。默认无偏移：`[0, 0, 0, 0, 0, 0]`。

> **提示：** 该偏移值仅保存在 SDK/Driver 实例内，不会下发到控制器。

**函数定义：**

```python
set_tcp_offset(self, pose: list[float]) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `pose` | `list[float]` | TCP 在法兰坐标系下的位姿偏移 `[x, y, z, roll, pitch, yaw]`：`x, y, z` 为位置（m）；`roll, pitch, yaw` 为欧拉角（rad）。范围：`roll/yaw` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]` |

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])
```

---

### 获取 TCP 位姿 — `get_tcp_pose()`

**功能说明：** 获取 TCP 位姿。该接口会先读取法兰位姿，然后根据 `set_tcp_offset()` 保存的偏移值做刚体变换得到 TCP 位姿。若未设置偏移，则 TCP 位姿与法兰位姿相同。

**函数定义：**

```python
get_tcp_pose(self) -> MessageAbstract[list[float]] | None
```

**返回值：** `MessageAbstract[list[float]] | None`

`.msg` 为长度 6 的 `list[float]`：`[x, y, z, roll, pitch, yaw]`（m / rad）。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
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

---

### 法兰位姿转 TCP 位姿 — `get_flange2tcp_pose()`

**功能说明：** 输入法兰位姿（基座/世界坐标系下），根据 `set_tcp_offset()` 保存的偏移值算出对应的 TCP 位姿。

**函数定义：**

```python
get_flange2tcp_pose(self, flange_pose: list[float]) -> list[float]
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `flange_pose` | `list[float]` | 法兰位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。范围：`roll/yaw` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]` |

**返回值：** `list[float]` — TCP 位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])

# 直接指定法兰位姿
tcp_pose = robot.get_flange2tcp_pose([0.30, 0.0, 0.30, 0.0, 1.5707, 0.0])
print("tcp_pose =", tcp_pose)

# 从当前位姿获取，结果与 get_tcp_pose() 得到的 pose 相同
flange_pose = robot.get_flange_pose()
if flange_pose is not None:
    tcp_pose = robot.get_flange2tcp_pose(flange_pose)
    print("tcp_pose =", tcp_pose)
```

---

### TCP 位姿转法兰位姿 — `get_tcp2flange_pose()`

**功能说明：** 输入目标 TCP 位姿（基座/世界坐标系下），根据 `set_tcp_offset()` 保存的偏移值算出对应的目标法兰位姿。将返回的法兰位姿传给 `move_p()`，即可实现 **TCP 运动到目标 TCP 位姿**。

**函数定义：**

```python
get_tcp2flange_pose(self, tcp_pose: list[float]) -> list[float]
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `tcp_pose` | `list[float]` | 目标 TCP 位姿 `[x, y, z, roll, pitch, yaw]`（m / rad）。范围：`roll/yaw` ∈ `[-π, π]`，`pitch` ∈ `[-π/2, π/2]` |

**返回值：** `list[float]` — 目标法兰位姿 `[x, y, z, roll, pitch, yaw]`（m / rad），可直接用于 `move_p()`。

**使用示例：**

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.set_tcp_offset([0.0, 0.0, 0.10, 0.0, 0.0, 0.0])

target_tcp_pose = [0.30, 0.0, 0.30, 0.0, 1.5707, 0.0]
target_flange_pose = robot.get_tcp2flange_pose(target_tcp_pose)
print("target_flange_pose =", target_flange_pose)

# robot.move_p(target_flange_pose)  # 注意：会触发运动
```

---

## 运动学相关

### 读取 IK 关节角 — `get_ik_joint_angles()`

> **固件要求：** `PiperFW.V188+`（固件 **≥ S-V1.8-8**）。详见 [Piper 固件参考](firmware_reference.md#有固件要求的-api)。

**功能说明：** 获取控制器为当前笛卡尔运动指令计算的**逆解（IK）关节角**反馈。

该反馈通过 CAN 上报，在机械臂执行笛卡尔运动后可用。它表示控制器为目标位姿选定的关节解，可能与 [get_joint_angles()](#读取关节角度--get_joint_angles) 读到的实测关节角不同。

> **注意：** 仅在调用 [move_p()](#点到点运动--move_p) 之后可用。

**函数定义：**

```python
get_ik_joint_angles(self) -> MessageAbstract[list[float]] | None
```

**返回值：** `MessageAbstract[list[float]] | None`

- 若尚未收到 IK 关节角反馈或数据校验失败，返回 `None`。
- `.msg` 为长度 6 的 `list[float]`：`[j1, j2, j3, j4, j5, j6]`，单位：**rad**。

**使用示例：**

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW

cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.V188, channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

robot.move_p([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])

while True:
    ika = robot.get_ik_joint_angles()
    if ika is not None:
        print(ika.msg)
        print(ika.hz, ika.timestamp)
    time.sleep(0.005)
```

---

### 正运动学 — `fk()`

**功能说明：** 根据给定关节角度，使用机械臂内置的改进 DH（MDH）模型计算末端**法兰位姿**。

该接口为**离线计算**（不依赖 CAN 通信）。输出位姿格式与 [get_flange_pose()](#读取法兰位姿--get_flange_pose) 返回的 `.msg` 一致：  
`[x, y, z, roll, pitch, yaw]`（基坐标系），其中 `x/y/z` 单位为米，`roll/pitch/yaw` 单位为弧度（SDK 采用 ZYX 的 RPY 约定）。

**函数定义：**

```python
fk(self, joint_angles: list[float]) -> list[float]
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_angles` | `list[float]` | 关节角度（单位：rad），长度 6：`[j1, j2, j3, j4, j5, j6]` |

**返回值：** `list[float]`

`[x, y, z, roll, pitch, yaw]` — 法兰位姿（基坐标系）。

**使用示例：**

1）与 [get_joint_angles()](#读取关节角度--get_joint_angles) 组合（读取当前关节角 → FK）：

```python
ja = robot.get_joint_angles()
if ja is not None:
    flange_pose = robot.fk(ja.msg)
    print("fk 法兰:", flange_pose)
```

2）与 [get_leader_joint_angles()](#读取主导臂leader关节角度--get_leader_joint_angles) 组合（读取主导臂角度 → FK）：

```python
mja = robot.get_leader_joint_angles()
if mja is not None:
    leader_flange_pose = robot.fk(mja.msg)
    print("leader fk 法兰:", leader_flange_pose)
```

3）与 [get_flange2tcp_pose()](#法兰位姿转-tcp-位姿--get_flange2tcp_pose) 组合（FK 法兰 → 推导 TCP）：

```python
ja = robot.get_joint_angles()
if ja is not None:
    flange_pose = robot.fk(ja.msg)
    tcp_pose = robot.get_flange2tcp_pose(flange_pose)
    print("fk TCP:", tcp_pose)
```

4）对比“测得法兰位姿”与“FK 计算位姿”（快速一致性检查）：

```python
ja = robot.get_joint_angles()
fp = robot.get_flange_pose()
if ja is not None and fp is not None:
    fk_fp = robot.fk(ja.msg)
    print("测得法兰:", fp.msg)
    print("fk 法兰:", fk_fp)
```

---

## SDK 配置相关

### 设置自动切换运动模式开关 — `set_auto_set_motion_mode_enabled()`

**功能说明：** 运行时设置在调用 `move_*` 接口时，是否自动执行 `set_motion_mode()` 切换。

- `True`：保持自动切换（默认）。
- `False`：不自动切换，需要你按需手动调用 `set_motion_mode()`。

**函数定义：**

```python
set_auto_set_motion_mode_enabled(self, enabled: bool) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `enabled` | `bool` | 是否启用自动切换运动模式 |

**使用示例：**

```python
robot.set_auto_set_motion_mode_enabled(False)
robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.J)
robot.move_j([0.0] * robot.joint_nums)
```

---

### 获取自动切换运动模式开关状态 — `get_auto_set_motion_mode_enabled()`

**功能说明：** 获取运行时在调用 `move_*` 前自动执行 `set_motion_mode()` 切换的当前开关状态。

**默认值：** `True`

**函数定义：**

```python
get_auto_set_motion_mode_enabled(self) -> bool
```

**返回值：** `bool`

- `True`：自动切换已启用。
- `False`：自动切换已关闭。

**使用示例：**

```python
enabled = robot.get_auto_set_motion_mode_enabled()
print("自动切换运动模式:", enabled)
```

---

### 设置关节软件限位开关 — `set_joint_limits_enabled()`

**功能说明：** 运行时设置是否启用关节软件限位。

- `True`：按配置的 `joint_limits` / 机型限位进行夹紧保护。
- `False`：跳过机型 `joint_limits` 夹紧，仅保留基础数值范围保护。

**函数定义：**

```python
set_joint_limits_enabled(self, enabled: bool) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `enabled` | `bool` | 是否启用关节软件限位 |

**使用示例：**

```python
robot.set_joint_limits_enabled(False)
robot.move_j([0.0] * robot.joint_nums)
robot.set_joint_limits_enabled(True)
```

---

### 获取关节软件限位开关状态 — `get_joint_limits_enabled()`

**功能说明：** 获取运行时关节软件限位当前是否启用。

**默认值：** `False`

**函数定义：**

```python
get_joint_limits_enabled(self) -> bool
```

**返回值：** `bool`

- `True`：关节软件限位已启用。
- `False`：关节软件限位已关闭。

**使用示例：**

```python
enabled = robot.get_joint_limits_enabled()
print("关节软件限位:", enabled)
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
enable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255) -> bool
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
disable(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 255] = 255) -> bool
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
move_c(self, start_pose: list[float], mid_pose: list[float], end_pose: list[float]) -> None
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

> **注意：** 连续执行该指令会覆盖上一次的目标值。
>
> **`t_ff`** 量程与编码因固件而异 — 见[固件参考 · MIT 分版本参数](firmware_reference.md#mitmove_mit分版本参数)。

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

> **提示：** `set_cpv_*` 接口会将参数保存到电机主控的 Flash 中。Flash 存在写入寿命限制，请避免在运行过程中频繁或重复设置这些参数；过度写入可能导致电机出现异常。

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
get_flange_vel_acc_limits(self, timeout: float = 1.0, min_interval: float = 1.0) -> MessageAbstract[ArmMsgFeedbackCurrentEndVelAccParam] | None
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
set_flange_vel_acc_limits_to_default(self, timeout: float = 1.0) -> bool
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
set_joint_angle_vel_acc_limits_to_default(self, timeout: float = 1.0) -> bool
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
set_links_vel_acc_period_feedback(self, enable: bool = False, timeout: float = 1.0) -> bool
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
