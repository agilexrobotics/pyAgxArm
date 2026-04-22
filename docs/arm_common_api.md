# Arm Common API Documentation

> This document describes model-agnostic common APIs in `pyAgxArm`, including module import, factory/config creation, connection lifecycle, common status checks, and logging APIs.

## Table of Contents

- [Switch to 中文](#arm-通用-api-使用文档)
- [Import Module](#import-module)
- [Factory and Instance Management](#factory-and-instance-management)
  - [Set Reuse Policy — set_reuse_policy()](#set-reuse-policy--set_reuse_policy)
  - [Get Reuse Policy — get_reuse_policy()](#get-reuse-policy--get_reuse_policy)
  - [Reuse Policy Behavior Notes](#reuse-policy-behavior-notes)
  - [Detect CAN Configurations — detect_can_configs()](#detect-can-configurations--detect_can_configs)
  - [Create Configuration — create_agx_arm_config()](#create-configuration--create_agx_arm_config)
  - [Create Arm Driver Instance — create_arm()](#create-arm-driver-instance--create_arm)
  - [Lifecycle and GC Cleanup Notes](#lifecycle-and-gc-cleanup-notes)
- [Connection and Communication Status](#connection-and-communication-status)
  - [Connect — connect()](#connect--connect)
  - [Disconnect — disconnect()](#disconnect--disconnect)
  - [Get Connection Status — is_connected()](#get-connection-status--is_connected)
  - [Reconnect Explicitly — reconnect()](#reconnect-explicitly--reconnect)
  - [Check Communication — is_ok()](#check-communication--is_ok)
  - [Get Data Receive Frequency — get_fps()](#get-data-receive-frequency--get_fps)
  - [Check Communication Error Flag — has_comm_error()](#check-communication-error-flag--has_comm_error)
  - [Get Communication Error Detail — get_comm_error()](#get-communication-error-detail--get_comm_error)
  - [Communication Error Handling Notes](#communication-error-handling-notes)
- [Effector Management](#effector-management)
  - [Initialize End Effector — init_effector()](#initialize-end-effector--init_effector)
- [General Status](#general-status)
  - [Get Joint Count — joint_nums](#get-joint-count--joint_nums)
- [TCP Related](#tcp-related)
  - [Set TCP Offset — set_tcp_offset()](#set-tcp-offset--set_tcp_offset)
  - [Get TCP Pose — get_tcp_pose()](#get-tcp-pose--get_tcp_pose)
  - [Flange Pose to TCP Pose — get_flange2tcp_pose()](#flange-pose-to-tcp-pose--get_flange2tcp_pose)
  - [TCP Pose to Flange Pose — get_tcp2flange_pose()](#tcp-pose-to-flange-pose--get_tcp2flange_pose)
- [Kinematics Related](#kinematics-related)
  - [Forward Kinematics — fk()](#forward-kinematics--fk)
- [SDK Config Related](#sdk-config-related)
  - [Set Auto Motion Mode Switching — set_auto_set_motion_mode_enabled()](#set-auto-motion-mode-switching--set_auto_set_motion_mode_enabled)
  - [Set Joint Limits Enabled — set_joint_limits_enabled()](#set-joint-limits-enabled--set_joint_limits_enabled)
- [Logging APIs](#logging-apis)
  - [Logging Behavior and Setup Notes](#logging-behavior-and-setup-notes)
  - [Write Debug Log — robot.log.debug()](#write-debug-log--robotlogdebug)
  - [Write Info Log — robot.log.info()](#write-info-log--robotloginfo)
  - [Write Warning Log — robot.log.warning()](#write-warning-log--robotlogwarning)
  - [Write Error Log — robot.log.error()](#write-error-log--robotlogerror)
  - [Write Critical Log — robot.log.critical()](#write-critical-log--robotlogcritical)
  - [Write Exception Log — robot.log.exception()](#write-exception-log--robotlogexception)
  - [Log Level Constants — robot.log.Level.*](#log-level-constants--robotloglevel)
  - [Configure Logger — robot.log.configure()](#configure-logger--robotlogconfigure)
  - [Enable Console Logging — robot.log.console_enable()](#enable-console-logging--robotlogconsole_enable)
  - [Disable Console Logging — robot.log.console_disable()](#disable-console-logging--robotlogconsole_disable)
  - [Enable Bridge Logging — robot.log.bridge_enable()](#enable-bridge-logging--robotlogbridge_enable)
  - [Disable Bridge Logging — robot.log.bridge_disable()](#disable-bridge-logging--robotlogbridge_disable)
  - [Create Child Logger — robot.log.get_child()](#create-child-logger--robotlogget_child)
  - [Shutdown Logger — robot.log.shutdown()](#shutdown-logger--robotlogshutdown)

---

## Import Module

```python
from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
    NeroFW,
    Logger,
    __version__,
)
```

---

## Factory and Instance Management

### Set Reuse Policy — `set_reuse_policy()`

**Description:** Set the factory instance reuse policy.

**Function Definition:**

```python
AgxArmFactory.set_reuse_policy(reuse_policy: Literal["new", "reuse", "replace"]) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `reuse_policy` | `str` | Reuse strategy: `"new"` always create new instance; `"reuse"` return cached live instance; `"replace"` disconnect cached instance then create new one |

**Usage Example:**

```python
from pyAgxArm import AgxArmFactory

AgxArmFactory.set_reuse_policy("reuse")
```

---

### Get Reuse Policy — `get_reuse_policy()`

**Description:** Read the current factory reuse policy.

**Function Definition:**

```python
AgxArmFactory.get_reuse_policy() -> Literal["new", "reuse", "replace"]
```

**Return Value:** `Literal["new", "reuse", "replace"]`

**Usage Example:**

```python
from pyAgxArm import AgxArmFactory

print("reuse_policy =", AgxArmFactory.get_reuse_policy())
```

---

### Reuse Policy Behavior Notes

`AgxArmFactory` caches instances by **configuration fingerprint** (full config content hash). Reuse only works when fingerprints are equal.

Policy behavior differences:

| Policy | `create_arm(cfg)` behavior | `id(new_arm) == id(old_arm)` |
| --- | --- | --- |
| `"new"` | Always create a new instance and refresh cache entry | Always `False` |
| `"reuse"` | Return cached live instance when same fingerprint hits; otherwise create new | Hit: `True`; miss: `False` |
| `"replace"` | If same fingerprint hits and cached instance is live, call old `disconnect()` first, then create new | Always `False` |

Fingerprint impact:

- Fingerprint includes the whole config content (for example robot model, firmware version, channel/interface/bitrate, and other config keys).
- Any config difference means a different fingerprint, so cache entries are isolated and not reused across different configs.

GC/finalizer effect:

- Driver instances register a `weakref.finalize` cleanup hook.
- When an old instance is no longer strongly referenced (for example overwritten by the same variable), GC can eventually trigger finalizer cleanup (threads/context/log managed handlers).
- GC timing is non-deterministic; `"replace"` performs explicit `disconnect()` before new construction, so old session cleanup happens immediately.

Minimal check for address behavior:

```python
from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config

cfg = create_agx_arm_config(
    robot=ArmModel.PIPER,
    firmeware_version=PiperFW.DEFAULT,
    channel="can0",
)

AgxArmFactory.set_reuse_policy("reuse")
arm1 = AgxArmFactory.create_arm(cfg)
arm2 = AgxArmFactory.create_arm(cfg)
print("reuse same id:", id(arm1) == id(arm2))

AgxArmFactory.set_reuse_policy("new")
arm3 = AgxArmFactory.create_arm(cfg)
print("new same id:", id(arm2) == id(arm3))
```

---

### Detect CAN Configurations — `detect_can_configs()`

**Description:** Enumerate available CAN backend configurations through `python-can`.

**Function Definition:**

```python
AgxArmFactory.detect_can_configs(
    interfaces: Any = None,
    *,
    timeout: float = 5.0,
) -> list[dict[str, Any]]
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `interfaces` | `Any` | Optional backend filter, `None` means use `python-can` defaults |
| `timeout` | `float` | Detection timeout in seconds, default `5.0` |

**Return Value:** `list[dict[str, Any]]`

If current `python-can` does not support detection API, or no device is found, an empty list is returned.

**Usage Example:**

```python
from pyAgxArm import AgxArmFactory

configs = AgxArmFactory.detect_can_configs(timeout=2.0)
print("can configs =", configs)
```

---

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
| `firmeware_version` | `str` | Main controller firmware version. For model-specific selection guidance, see [Piper Firmware Version](./piper/piper_api.md#firmware-version) and [Nero Firmware Version](./nero/nero_api.md#firmware-version). Default `"default"` |

**Optional Keyword Arguments (`**kwargs`):**

| Name | Type | Description |
| --- | --- | --- |
| `joint_limits` | `dict` | Custom joint limits (unit: rad). Automatically assigned by default; manually entered limits are not currently applied to actual control. See example below |
| `auto_set_motion_mode` | `bool` | Whether the SDK should automatically switch the arm into the required motion mode before motion APIs are sent. Default `True`. Set to `False` if you want to manage motion mode switching explicitly in your own application logic. |
| `enable_joint_limits` | `bool` | Whether to enable software joint-limit clamping in runtime motion APIs. Default `True`. Set to `False` to skip model `joint_limits` clamp (basic numeric range checks still apply). |
| `channel` | `str` | CAN channel identifier. Default `"can0"`. The documented and verified combinations are: with `"agx_cando"` use device index strings such as `"0"`, `"1"`, `"2"`; with `"socketcan"` use Linux CAN netdev names such as `"can0"` or your renamed interface; with `"slcan"` use serial device paths such as `"/dev/ttyACM0"` on macOS (`Darwin`). |
| `interface` | `str` | CAN interface type, default `"socketcan"`. The documented and verified values are `"socketcan"` on Linux, `"agx_cando"` on Windows with the Agilex CANDO backend, and `"slcan"` on macOS (`Darwin`). |
| `bitrate` | `int` | CAN baud rate, default `1000000` (1 Mbps) |
| `enable_check_can` | `bool` | Whether to check the CAN module when creating a Comm instance, default `True`. This pre-check is currently only effective for Linux `socketcan`; for other backends (for example, Windows `agx_cando` and macOS `slcan`) the actual availability check happens when the CAN bus is opened. |
| `auto_connect` | `bool` | Whether to automatically create a CAN Bus instance, default `True` |
| `timeout` | `float` | CAN Bus read/write timeout (seconds), default `0.001` |
| `receive_own_messages` | `bool` | Whether the local CAN backend should receive frames sent by the same process/device. Default `False`. This is useful for debugging, loopback tests, or virtual/single-node verification, but is usually not recommended for normal arm control. Backend support depends on the selected `interface`. The `slcan` backend on macOS generally does not support this; **do not pass** it when using `interface="slcan"`. |
| `local_loopback` | `bool` | Whether to enable CAN **local loopback**. Default is `False` (loopback disabled), so your local terminal/process will **not** receive the CAN frames it sends itself. You may enable it for debugging, but it is **not recommended** for normal SDK usage because it may consume bus receive resources and impact reading performance. The `slcan` backend on macOS generally does not support this; **do not pass** it when using `interface="slcan"`. |

**Return Value:** `dict`

Example return structure:

```json
{
    "robot": "piper",
    "firmeware_version": "default",
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
            "timeout": 0.001,
            "receive_own_messages": false,
            "local_loopback": false
        }
    }
}
```

> **Note:** `auto_set_motion_mode` is added to the top-level config only when you pass it explicitly.

Verified interface/channel examples:

- Linux `socketcan`: `create_agx_arm_config(..., interface="socketcan", channel="can0")`
- Windows `agx_cando`: `create_agx_arm_config(..., interface="agx_cando", channel="0")`
- macOS `slcan`: `create_agx_arm_config(..., interface="slcan", channel="/dev/ttyACM0")`

On Windows, `interface="agx_cando"` requires the separately installed `python-can-agx-cando` plugin. Install it from `https://github.com/agilexrobotics/python-can-agx-cando.git`, then run `pip3 install .` in that repository before using `pyAgxArm`.
On macOS (`Darwin`), when using `interface="slcan"` with the default channel, grant serial permission first.

**Usage Example:**

```python
from pyAgxArm import create_agx_arm_config, ArmModel, PiperFW

cfg = create_agx_arm_config(
    robot=ArmModel.PIPER,
    firmeware_version=PiperFW.DEFAULT,
    channel="can0"
)
print(cfg)
```

---

### Create Arm Driver Instance — `create_arm()`

**Description:** Create the corresponding robotic arm Driver instance via factory method based on the configuration dictionary.

**Function Definition:**

```python
AgxArmFactory.create_arm(cls, config: dict, **kwargs) -> T
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

### Lifecycle and GC Cleanup Notes

- The driver registers a GC finalizer (`weakref.finalize`) for best-effort cleanup when the instance is garbage collected.
- GC timing is non-deterministic. For predictable resource release (threads, CAN handles, managed log handlers), call `robot.disconnect()` explicitly.
- The finalizer is a fallback safety net. It should not replace explicit lifecycle management in applications/services.

---

## Connection and Communication Status

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

> **Note:** If `start_read_thread=False`, only transport connection is established. Runtime monitor values such as `is_ok()` / `get_fps()` may be unavailable or not representative until the read thread is started.

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

### Get Connection Status — `is_connected()`

**Description:** Check whether the arm instance is currently connected.

**Function Definition:**

```python
robot.is_connected() -> bool
```

**Return Value:** `bool`

**State Lifecycle Notes:**

- Set when reader thread hits an exception (`comm.recv()` failure path).
- Cleared when communication session is rebuilt/reset (for example `create_comm()`, `init_comm()`, `start_th()`, `reconnect()`/`connect()` after teardown).
- It is not a permanent sticky flag; after successful session rebuild it can return to `False`.

**Usage Example:**

```python
print("connected =", robot.is_connected())
```

---

### Reconnect Explicitly — `reconnect()`

**Description:** Rebuild session after communication exception by reconnecting explicitly.

**Function Definition:**

```python
robot.reconnect(join_timeout: float = 1.0, start_read_thread: bool = True) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `join_timeout` | `float` | Timeout in seconds when waiting read thread to stop |
| `start_read_thread` | `bool` | Whether to restart read thread after reconnect |

Internally, `reconnect()` performs `disconnect()` then `connect()`.

**Usage Example:**

```python
robot.reconnect()
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

**Return Value:** `float` (Hz)

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

### Check Communication Error Flag — `has_comm_error()`

**Description:** Check whether communication error flag is currently set.

**Function Definition:**

```python
robot.has_comm_error() -> bool
```

**Return Value:** `bool`

**Usage Example:**

```python
print("has_comm_error =", robot.has_comm_error())
```

---

### Get Communication Error Detail — `get_comm_error()`

**Description:** Read latest communication error detail from driver context.

**Function Definition:**

```python
robot.get_comm_error()
```

**Return Value:** Usually error object or `None` when no error exists.

**State Lifecycle Notes:**

- Returns the latest exception object while comm error flag is set.
- After successful reset/rebuild (`create_comm()` / `init_comm()` / `start_th()` / `reconnect()`), this value is typically cleared back to `None`.

**Usage Example:**

```python
print("comm_error =", robot.get_comm_error())
```

---

### Communication Error Handling Notes

`CanComm` classifies common transport failures and handles them differently:

- `hard_disconnect` (for example USB-CAN unplugged / device disappears):
  - `send()` / `recv()` closes bus handles and raises a `RuntimeError`.
- `link_down` (network/interface down):
  - `send()` logs warning and returns; frame is not sent.
  - `recv()` logs warning and returns current loop iteration.
- `no_buffer` (TX buffer full):
  - `send()` logs warning and returns; this frame is dropped.

Driver-layer behavior:

- Reader thread exceptions are recorded in driver context.
- Check with `robot.has_comm_error()` and inspect with `robot.get_comm_error()`.
- `robot.reconnect()` is the recommended recovery path after communication faults.
- `connect()` already includes error-aware behavior (it will rebuild session when previous comm error exists).

Recommended monitor-recover loop (minimal pattern):

```python
import time

while True:
    if (not robot.is_connected()) or robot.has_comm_error():
        try:
            if robot.has_comm_error():
                print("comm_error:", robot.get_comm_error())
            robot.reconnect()
        except Exception as exc:
            print("reconnect failed:", exc)
            time.sleep(1.0)
            continue
    # normal business logic
    time.sleep(0.01)
```

---

## Effector Management

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
| `effector` | `str` | End-effector type. See option table below |

**Effector Options:**

| Option Constant | Raw Value | Description |
| --- | --- | --- |
| `robot.OPTIONS.EFFECTOR.AGX_GRIPPER` | `"agx_gripper"` | AGX gripper |
| `robot.OPTIONS.EFFECTOR.REVO2` | `"revo2"` | REVO2 dexterous hand |

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

**Description:** Get the number of joints of the robotic arm.

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

# Specify flange pose directly
tcp_pose = robot.get_flange2tcp_pose([0.30, 0.0, 0.30, 0.0, 1.5707, 0.0])
print("tcp_pose =", tcp_pose)

# Compute from current flange pose; result matches get_tcp_pose() pose
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

### Forward Kinematics — `fk()`

**Description:** Compute the end **flange pose** from a given set of joint angles using the robot's built-in modified DH model.

This is an **offline** computation (no CAN I/O). The output pose format matches `.msg` from `get_flange_pose()`:  
`[x, y, z, roll, pitch, yaw]` in the **base frame**, where `x/y/z` are meters and `roll/pitch/yaw` are radians (ZYX RPY convention used by the SDK).

**Function Definition:**

```python
fk(self, joint_angles: list[float]) -> list[float]
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `joint_angles` | `list[float]` | Joint angles in **rad**. Length must match the current robot's joint count (`robot.joint_nums`) |

**Return Value:** `list[float]`

`[x, y, z, roll, pitch, yaw]` — flange pose in base frame.

**Usage Examples:**

1) Combine with `get_joint_angles()` (current arm state → FK):

```python
ja = robot.get_joint_angles()
if ja is not None:
    flange_pose = robot.fk(ja.msg)
    print("fk flange:", flange_pose)
```

2) Combine with `get_leader_joint_angles()` (leader angles → FK):

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

## Logging APIs

### Logging Behavior and Setup Notes

By default, `robot.log` has no visible output:

- `Logger` starts with a `NullHandler` as library-safe default.
- You must explicitly enable at least one output handler.
- A newly created driver instance does not enable visible handlers by default; if you only call `robot.log.info(...)` without enabling handlers, you may see no output.

Recommended usage patterns:

1. Console output (local debug):

```python
robot.log.console_enable(level=robot.log.Level.INFO)
```

2. Callback bridge output (ROS/external systems):

```python
robot.log.bridge_enable(info=print, warning=print, error=print)
```

3. Direct stdlib logger customization (advanced):

```python
import logging

h = logging.StreamHandler()
h.setFormatter(robot.log.Formats.FULL)
robot.log.logger.addHandler(h)
robot.log.logger.setLevel(robot.log.Level.INFO)
```

> **Tip:** Keep handler ownership clear. `replace_handlers=True` clears existing handlers on this logger before installing managed handlers; `replace_handlers=False` keeps existing handlers and only replaces handler(s) of the same managed kind. Also note that `shutdown()` cleans managed handlers created by this `Logger` facade; handlers you add directly to `robot.log.logger` are considered custom and should be managed/removed by your own code.

### Write Debug Log — `robot.log.debug()`

**Description:** Write debug-level logs for diagnostic details.

**Function Definition:**

```python
robot.log.debug(msg: str, *args, **kwargs) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `msg` | `str` | Log message template |
| `*args` | `Any` | Positional formatting arguments for `msg` |
| `**kwargs` | `Any` | Standard Python logging keyword arguments (such as `exc_info`) |

**Usage Example:**

```python
robot.log.debug("debug message")
```

---

### Write Info Log — `robot.log.info()`

**Description:** Write info-level runtime logs.

**Function Definition:**

```python
robot.log.info(msg: str, *args, **kwargs) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `msg` | `str` | Log message template |
| `*args` | `Any` | Positional formatting arguments for `msg` |
| `**kwargs` | `Any` | Standard Python logging keyword arguments |

**Usage Example:**

```python
robot.log.info("common api smoke check done")
```

---

### Write Warning Log — `robot.log.warning()`

**Description:** Write warning-level logs for recoverable issues.

**Function Definition:**

```python
robot.log.warning(msg: str, *args, **kwargs) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `msg` | `str` | Log message template |
| `*args` | `Any` | Positional formatting arguments for `msg` |
| `**kwargs` | `Any` | Standard Python logging keyword arguments |

**Usage Example:**

```python
robot.log.warning("can frame delayed")
```

---

### Write Error Log — `robot.log.error()`

**Description:** Write error-level logs for failures and exceptions.

**Function Definition:**

```python
robot.log.error(msg: str, *args, **kwargs) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `msg` | `str` | Log message template |
| `*args` | `Any` | Positional formatting arguments for `msg` |
| `**kwargs` | `Any` | Standard Python logging keyword arguments |

**Usage Example:**

```python
robot.log.error("connect failed")
```

---

### Write Critical Log — `robot.log.critical()`

**Description:** Write critical-level logs for severe unrecoverable failures.

**Function Definition:**

```python
robot.log.critical(msg: str, *args, **kwargs) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `msg` | `str` | Log message template |
| `*args` | `Any` | Positional formatting arguments for `msg` |
| `**kwargs` | `Any` | Standard Python logging keyword arguments |

**Usage Example:**

```python
robot.log.critical("controller heartbeat lost")
```

---

### Write Exception Log — `robot.log.exception()`

**Description:** Log an error with current exception traceback (`exc_info=True`).

**Function Definition:**

```python
robot.log.exception(msg: str, *args, **kwargs) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `msg` | `str` | Log message template |
| `*args` | `Any` | Positional formatting arguments for `msg` |
| `**kwargs` | `Any` | Standard Python logging keyword arguments |

**Usage Example:**

```python
try:
    robot.connect()
except Exception:
    robot.log.exception("connect raised an exception")
```

---

### Log Level Constants — `robot.log.Level.*`

**Description:** Built-in log level constants for logger-related APIs, without importing `logging`.

**Available Constants:**

| Name | Value | Description |
| --- | --- | --- |
| `robot.log.Level.NOTSET` | `0` | No explicit level |
| `robot.log.Level.DEBUG` | `10` | Debug level |
| `robot.log.Level.INFO` | `20` | Info level |
| `robot.log.Level.WARNING` | `30` | Warning level |
| `robot.log.Level.WARN` | `30` | Alias of `WARNING` |
| `robot.log.Level.ERROR` | `40` | Error level |
| `robot.log.Level.CRITICAL` | `50` | Critical level |

**Usage Example:**

```python
robot.log.configure(level=robot.log.Level.DEBUG)
robot.log.console_enable(level=robot.log.Level.INFO)
```

---

### Configure Logger — `robot.log.configure()`

**Description:** Update logger level, propagation, formatter, and optionally replace handlers.

**Function Definition:**

```python
robot.log.configure(
    *,
    level: int | None = None,
    propagate: bool | None = None,
    formatter: logging.Formatter | None = None,
    replace_handlers: bool = False,
) -> None
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `level` | `int \| None` | Logger level (e.g. `robot.log.Level.INFO`), keep unchanged when `None` |
| `propagate` | `bool \| None` | Whether to propagate to ancestor loggers |
| `formatter` | `logging.Formatter \| None` | Formatter to apply to managed handlers |
| `replace_handlers` | `bool` | Whether to clear all handlers before applying options |

**Usage Example:**

```python
robot.log.configure(level=robot.log.Level.DEBUG, propagate=False)
```

---

### Enable Console Logging — `robot.log.console_enable()`

**Description:** Enable managed console output (stream handler) with optional throttling.

**Function Definition:**

```python
robot.log.console_enable(
    *,
    level: int = Logger.Level.INFO,
    emit_min_interval: float = 1.0,
    replace_handlers: bool = False,
    propagate: bool | None = None,
    formatter: logging.Formatter | None = None,
    handler: logging.Handler | None = None,
    stream=None,
) -> logging.Handler
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `level` | `int` | Minimum level for console output |
| `emit_min_interval` | `float` | Minimum interval for repeated same-key logs, `<=0` disables throttling |
| `replace_handlers` | `bool` | Whether to clear all handlers first |
| `propagate` | `bool \| None` | Whether to update logger propagation |
| `formatter` | `logging.Formatter \| None` | Formatter for console output |
| `handler` | `logging.Handler \| None` | Custom handler to wrap; default creates `StreamHandler` |
| `stream` | `Any` | Stream passed when creating default `StreamHandler` |

**Return Value:** `logging.Handler`

**Usage Example:**

```python
robot.log.console_enable(level=robot.log.Level.INFO, emit_min_interval=0.5)
```

---

### Disable Console Logging — `robot.log.console_disable()`

**Description:** Disable and close managed console handler.

**Function Definition:**

```python
robot.log.console_disable() -> None
```

**Usage Example:**

```python
robot.log.console_disable()
```

---

### Enable Bridge Logging — `robot.log.bridge_enable()`

**Description:** Enable callback bridge logging for external systems (for example ROS logger adapters).

**Function Definition:**

```python
robot.log.bridge_enable(
    *,
    debug: Callable[[str], None] | None = None,
    info: Callable[[str], None] | None = None,
    warning: Callable[[str], None] | None = None,
    error: Callable[[str], None] | None = None,
    critical: Callable[[str], None] | None = None,
    level: int = Logger.Level.INFO,
    emit_min_interval: float = 1.0,
    replace_handlers: bool = False,
    propagate: bool | None = None,
    formatter: logging.Formatter | None = None,
) -> logging.Handler
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `debug` | `Callable[[str], None] \| None` | Callback for debug messages |
| `info` | `Callable[[str], None] \| None` | Callback for info messages |
| `warning` | `Callable[[str], None] \| None` | Callback for warning messages |
| `error` | `Callable[[str], None] \| None` | Callback for error messages |
| `critical` | `Callable[[str], None] \| None` | Callback for critical messages |
| `level` | `int` | Minimum level for bridge output |
| `emit_min_interval` | `float` | Minimum interval for repeated same-key logs, `<=0` disables throttling |
| `replace_handlers` | `bool` | Whether to clear all handlers first |
| `propagate` | `bool \| None` | Whether to update logger propagation |
| `formatter` | `logging.Formatter \| None` | Formatter for emitted bridge text |

**Return Value:** `logging.Handler`

**Usage Example:**

```python
def ros_info(line: str):
    print("[ROS][INFO]", line)

robot.log.bridge_enable(info=ros_info)
```

---

### Disable Bridge Logging — `robot.log.bridge_disable()`

**Description:** Disable and close managed callback bridge handler.

**Function Definition:**

```python
robot.log.bridge_disable() -> None
```

**Usage Example:**

```python
robot.log.bridge_disable()
```

---

### Create Child Logger — `robot.log.get_child()`

**Description:** Create a child logger under current logger namespace.

**Function Definition:**

```python
robot.log.get_child(suffix: str) -> logging.Logger
```

**Parameters:**

| Name | Type | Description |
| --- | --- | --- |
| `suffix` | `str` | Child logger suffix name (must be non-empty) |

**Return Value:** child logger object.

**Usage Example:**

```python
sub_log = robot.log.get_child("can.rx")
sub_log.info("rx started")
```

---

### Shutdown Logger — `robot.log.shutdown()`

**Description:** Flush and shutdown logging subsystem when exiting process.

**Function Definition:**

```python
robot.log.shutdown() -> None
```

**Usage Example:**

```python
robot.log.shutdown()
```

---

# Arm 通用 API 使用文档

> 本文档汇总 `pyAgxArm` 的机型无关通用 API，包含导入方式、工厂/配置创建、连接生命周期、通用状态检查与日志接口。

## 目录

- [切换到 English](#arm-common-api-documentation)
- [导入模块](#导入模块)
- [工厂与实例管理](#工厂与实例管理)
  - [设置复用策略 — set_reuse_policy()](#设置复用策略--set_reuse_policy)
  - [读取复用策略 — get_reuse_policy()](#读取复用策略--get_reuse_policy)
  - [复用策略行为说明](#复用策略行为说明)
  - [检测 CAN 配置 — detect_can_configs()](#检测-can-配置--detect_can_configs)
  - [创建配置参数 — create_agx_arm_config()](#创建配置参数--create_agx_arm_config)
  - [创建机械臂 Driver 实例 — create_arm()](#创建机械臂-driver-实例--create_arm)
  - [生命周期与 GC 清理说明](#生命周期与-gc-清理说明)
- [连接与通信状态](#连接与通信状态)
  - [创建连接 — connect()](#创建连接--connect)
  - [断开连接 — disconnect()](#断开连接--disconnect)
  - [读取连接状态 — is_connected()](#读取连接状态--is_connected)
  - [显式重连 — reconnect()](#显式重连--reconnect)
  - [通信是否正常 — is_ok()](#通信是否正常--is_ok)
  - [获取数据接收频率 — get_fps()](#获取数据接收频率--get_fps)
  - [检查通信错误标志 — has_comm_error()](#检查通信错误标志--has_comm_error)
  - [读取通信错误详情 — get_comm_error()](#读取通信错误详情--get_comm_error)
  - [通信异常处理说明](#通信异常处理说明)
- [末端执行器管理](#末端执行器管理)
  - [初始化末端执行器 — init_effector()](#初始化末端执行器--init_effector)
- [通用状态](#通用状态)
  - [获取关节数量 — joint_nums](#获取关节数量--joint_nums)
- [TCP 相关](#tcp-相关)
  - [设置 TCP 偏移 — set_tcp_offset()](#设置-tcp-偏移--set_tcp_offset)
  - [获取 TCP 位姿 — get_tcp_pose()](#获取-tcp-位姿--get_tcp_pose)
  - [法兰位姿转 TCP 位姿 — get_flange2tcp_pose()](#法兰位姿转-tcp-位姿--get_flange2tcp_pose)
  - [TCP 位姿转法兰位姿 — get_tcp2flange_pose()](#tcp-位姿转法兰位姿--get_tcp2flange_pose)
- [运动学相关](#运动学相关)
  - [正运动学 — fk()](#正运动学--fk)
- [SDK 配置相关](#sdk-配置相关)
  - [设置自动切换运动模式开关 — set_auto_set_motion_mode_enabled()](#设置自动切换运动模式开关--set_auto_set_motion_mode_enabled)
  - [设置关节软件限位开关 — set_joint_limits_enabled()](#设置关节软件限位开关--set_joint_limits_enabled)
- [日志接口](#日志接口)
  - [日志行为与启用说明](#日志行为与启用说明)
  - [写入调试日志 — robot.log.debug()](#写入调试日志--robotlogdebug)
  - [写入信息日志 — robot.log.info()](#写入信息日志--robotloginfo)
  - [写入告警日志 — robot.log.warning()](#写入告警日志--robotlogwarning)
  - [写入错误日志 — robot.log.error()](#写入错误日志--robotlogerror)
  - [写入严重错误日志 — robot.log.critical()](#写入严重错误日志--robotlogcritical)
  - [写入异常日志 — robot.log.exception()](#写入异常日志--robotlogexception)
  - [日志级别常量 — robot.log.Level.*](#日志级别常量--robotloglevel)
  - [配置日志器 — robot.log.configure()](#配置日志器--robotlogconfigure)
  - [启用控制台日志 — robot.log.console_enable()](#启用控制台日志--robotlogconsole_enable)
  - [禁用控制台日志 — robot.log.console_disable()](#禁用控制台日志--robotlogconsole_disable)
  - [启用桥接日志 — robot.log.bridge_enable()](#启用桥接日志--robotlogbridge_enable)
  - [禁用桥接日志 — robot.log.bridge_disable()](#禁用桥接日志--robotlogbridge_disable)
  - [创建子日志器 — robot.log.get_child()](#创建子日志器--robotlogget_child)
  - [关闭日志系统 — robot.log.shutdown()](#关闭日志系统--robotlogshutdown)

---

## 导入模块

```python
from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    PiperFW,
    NeroFW,
    Logger,
    __version__,
)
```

---

## 工厂与实例管理

### 设置复用策略 — `set_reuse_policy()`

**功能说明：** 设置工厂实例复用策略。

**函数定义：**

```python
AgxArmFactory.set_reuse_policy(reuse_policy: Literal["new", "reuse", "replace"]) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `reuse_policy` | `str` | 复用策略：`"new"` 每次新建；`"reuse"` 复用缓存存活实例；`"replace"` 先断开缓存实例再新建 |

**使用示例：**

```python
from pyAgxArm import AgxArmFactory

AgxArmFactory.set_reuse_policy("reuse")
```

---

### 读取复用策略 — `get_reuse_policy()`

**功能说明：** 读取当前工厂复用策略。

**函数定义：**

```python
AgxArmFactory.get_reuse_policy() -> Literal["new", "reuse", "replace"]
```

**返回值：** `Literal["new", "reuse", "replace"]`

**使用示例：**

```python
from pyAgxArm import AgxArmFactory

print("reuse_policy =", AgxArmFactory.get_reuse_policy())
```

---

### 复用策略行为说明

`AgxArmFactory` 基于**配置指纹**（完整配置内容哈希）做实例缓存。只有指纹相同才会命中复用。

三种策略行为差异：

| 策略 | `create_arm(cfg)` 行为 | `id(new_arm) == id(old_arm)` |
| --- | --- | --- |
| `"new"` | 每次都新建实例并刷新缓存项 | 恒为 `False` |
| `"reuse"` | 同指纹命中且缓存实例存活时直接返回缓存实例；未命中则新建 | 命中为 `True`；未命中为 `False` |
| `"replace"` | 同指纹命中且缓存实例存活时，先对旧实例调用 `disconnect()`，再新建实例 | 恒为 `False` |

配置指纹影响：

- 指纹覆盖完整配置内容（如机型、固件版本、channel/interface/bitrate 及其他配置键）。
- 任一配置项变化都会形成新指纹，因此不同配置之间不会互相复用缓存实例。

GC/终结器在其中的作用：

- Driver 实例内部注册了 `weakref.finalize` 清理钩子。
- 当旧实例不再被强引用（例如同一变量被新实例覆盖赋值）时，GC 最终可触发终结器，做线程/context/日志托管 handler 的清理。
- GC 触发时机不可预测；`"replace"` 会在新建前显式 `disconnect()`，因此旧会话清理是立即、确定性的。

地址行为最小示例：

```python
from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config

cfg = create_agx_arm_config(
    robot=ArmModel.PIPER,
    firmeware_version=PiperFW.DEFAULT,
    channel="can0",
)

AgxArmFactory.set_reuse_policy("reuse")
arm1 = AgxArmFactory.create_arm(cfg)
arm2 = AgxArmFactory.create_arm(cfg)
print("reuse same id:", id(arm1) == id(arm2))

AgxArmFactory.set_reuse_policy("new")
arm3 = AgxArmFactory.create_arm(cfg)
print("new same id:", id(arm2) == id(arm3))
```

---

### 检测 CAN 配置 — `detect_can_configs()`

**功能说明：** 通过 `python-can` 枚举当前可用的 CAN 后端配置。

**函数定义：**

```python
AgxArmFactory.detect_can_configs(
    interfaces: Any = None,
    *,
    timeout: float = 5.0,
) -> list[dict[str, Any]]
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `interfaces` | `Any` | 可选后端过滤，`None` 表示使用 `python-can` 默认行为 |
| `timeout` | `float` | 检测超时（秒），默认 `5.0` |

**返回值：** `list[dict[str, Any]]`

若当前 `python-can` 不支持检测 API，或未检测到设备，返回空列表。

**使用示例：**

```python
from pyAgxArm import AgxArmFactory

configs = AgxArmFactory.detect_can_configs(timeout=2.0)
print("can configs =", configs)
```

---

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
| `firmeware_version` | `str` | 主控固件版本。按机型查看选择方法：见 [Piper 固件版本选择](./piper/piper_api.md#固件版本选择) 与 [Nero 固件版本选择](./nero/nero_api.md#固件版本选择)。默认 `"default"` |

**可选关键字参数（`**kwargs`）：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_limits` | `dict` | 自定义关节限位（单位：rad）。默认自动赋值，暂不会将手动输入的限位生效到实际控制中。示例见下文 |
| `auto_set_motion_mode` | `bool` | 是否在发送运动类 API 前由 SDK 自动切换到所需运动模式。默认 `True`。如果你希望在自己的上层逻辑中显式控制模式切换，可设为 `False`。 |
| `enable_joint_limits` | `bool` | 是否在运行时运动接口中启用关节软件限位夹紧。默认 `True`。设为 `False` 时跳过机型 `joint_limits` 夹紧（仍保留基础数值范围检查）。 |
| `channel` | `str` | CAN 通道标识，默认 `"can0"`。当前文档已验证的写法为：`"agx_cando"` 使用 `"0"`、`"1"`、`"2"` 这类设备索引字符串；`"socketcan"` 使用 Linux 下的 CAN 网卡名，例如 `"can0"` 或重命名后的接口名；`"slcan"` 在 macOS（`Darwin`）下使用串口设备路径，例如 `"/dev/ttyACM0"`。 |
| `interface` | `str` | CAN 接口类型，默认 `"socketcan"`。当前文档已验证并提供说明的取值为 Linux 下的 `"socketcan"`、Windows 下 Agilex CANDO 后端使用的 `"agx_cando"`、以及 macOS（`Darwin`）下的 `"slcan"`。 |
| `bitrate` | `int` | CAN 波特率，默认 `1000000`（1 Mbps） |
| `enable_check_can` | `bool` | 是否在创建 Comm 实例时检查 CAN 模块，默认 `True`。当前该预检查主要只对 Linux `socketcan` 生效；其他后端（如 Windows `agx_cando`、macOS `slcan`）通常会在实际打开 CAN bus 时完成可用性检查。 |
| `auto_connect` | `bool` | 是否自动创建 CAN Bus 实例，默认 `True` |
| `timeout` | `float` | CAN Bus 读写超时时间（秒），默认 `0.001` |
| `receive_own_messages` | `bool` | 是否让本地 CAN 后端接收由同一进程/设备发送出去的报文。默认 `False`。适合调试、回环测试或单节点联调，正常机械臂控制一般不建议开启。具体是否生效取决于所选 `interface`。macOS 下的 `slcan` 后端通常**不支持**该项；使用 `interface="slcan"` 时**不要**传入。 |
| `local_loopback` | `bool` | 是否开启 CAN **本地回环**。默认 `False`（关闭回环），本地终端/进程将**无法**接收到自己发送的 CAN 报文。调试时可选择开启，但**不建议**在正常使用 SDK 时开启，因为可能会占用读取 bus 的资源并影响读取性能。macOS 下的 `slcan` 后端通常**不支持**该项；使用 `interface="slcan"` 时**不要**传入。 |

**返回值：** `dict`

返回结构示例：

```json
{
    "robot": "piper",
    "firmeware_version": "default",
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
            "timeout": 0.001,
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

cfg = create_agx_arm_config(
    robot=ArmModel.PIPER,
    firmeware_version=PiperFW.DEFAULT,
    channel="can0"
)
print(cfg)
```

---

### 创建机械臂 Driver 实例 — `create_arm()`

**功能说明：** 根据配置字典，通过工厂方法创建对应的机械臂 Driver 实例。

**函数定义：**

```python
AgxArmFactory.create_arm(cls, config: dict, **kwargs) -> T
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

### 生命周期与 GC 清理说明

- Driver 内部注册了 GC 终结器（`weakref.finalize`），在对象被垃圾回收时做“尽力而为”的资源清理。
- GC 触发时机不可预测。若要确定性释放资源（线程、CAN 句柄、托管日志处理器），请显式调用 `robot.disconnect()`。
- 终结器是兜底安全机制，不应替代应用/服务中的显式生命周期管理。

---

## 连接与通信状态

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

> **注意：** 当 `start_read_thread=False` 时，仅建立传输连接，不会启动读取线程；`is_ok()` / `get_fps()` 等运行时监控值可能不可用，或不具备代表性。

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

### 读取连接状态 — `is_connected()`

**功能说明：** 查询当前机械臂实例是否处于连接状态。

**函数定义：**

```python
robot.is_connected() -> bool
```

**返回值：** `bool`

**状态生命周期说明：**

- 该标志会在读线程出现异常时被置位（`comm.recv()` 失败路径）。
- 在通信会话重建/重置后会清空（例如 `create_comm()`、`init_comm()`、`start_th()`，以及拆会话后的 `reconnect()`/`connect()`）。
- 这不是永久粘滞状态；会话成功恢复后可回到 `False`。

**使用示例：**

```python
print("connected =", robot.is_connected())
```

---

### 显式重连 — `reconnect()`

**功能说明：** 在通信异常后显式重建会话连接。

**函数定义：**

```python
robot.reconnect(join_timeout: float = 1.0, start_read_thread: bool = True) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `join_timeout` | `float` | 等待读取线程退出的超时（秒） |
| `start_read_thread` | `bool` | 重连后是否重启读取线程 |

`reconnect()` 内部会先调用 `disconnect()`，再调用 `connect()`。

**使用示例：**

```python
robot.reconnect()
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

### 检查通信错误标志 — `has_comm_error()`

**功能说明：** 检查当前是否存在通信错误标志。

**函数定义：**

```python
robot.has_comm_error() -> bool
```

**返回值：** `bool`

**使用示例：**

```python
print("has_comm_error =", robot.has_comm_error())
```

---

### 读取通信错误详情 — `get_comm_error()`

**功能说明：** 获取驱动上下文中最近一次通信错误详情。

**函数定义：**

```python
robot.get_comm_error()
```

**返回值：** 通常为错误对象；无错误时通常为 `None`。

**状态生命周期说明：**

- 当通信错误标志被置位时，这里通常返回最近一次异常对象。
- 在成功重置/重建后（`create_comm()` / `init_comm()` / `start_th()` / `reconnect()`），该值通常会清回 `None`。

**使用示例：**

```python
print("comm_error =", robot.get_comm_error())
```

---

### 通信异常处理说明

`CanComm` 会对常见传输故障做分类处理：

- `hard_disconnect`（例如 USB-CAN 被拔出、设备消失）：
  - `send()` / `recv()` 会关闭总线句柄并抛出 `RuntimeError`。
- `link_down`（网卡/接口 down）：
  - `send()` 记录 warning 后返回；当前帧不会发出。
  - `recv()` 记录 warning 后返回当前轮询。
- `no_buffer`（发送缓冲区满）：
  - `send()` 记录 warning 后返回；当前帧会被丢弃。

Driver 层行为：

- 读线程异常会写入 driver context。
- 通过 `robot.has_comm_error()` 判断，通过 `robot.get_comm_error()` 读取细节。
- 发生通信故障后，推荐调用 `robot.reconnect()` 恢复。
- `connect()` 已包含错误感知逻辑（存在历史通信错误时会重建会话）。

推荐检测-恢复循环（最小模式）：

```python
import time

while True:
    if (not robot.is_connected()) or robot.has_comm_error():
        try:
            if robot.has_comm_error():
                print("comm_error:", robot.get_comm_error())
            robot.reconnect()
        except Exception as exc:
            print("reconnect failed:", exc)
            time.sleep(1.0)
            continue
    # 正常业务逻辑
    time.sleep(0.01)
```

---

## 末端执行器管理

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
| `effector` | `str` | 执行器类型，见下方可选项表格 |

**执行器可选项：**

| 常量 | 原始值 | 说明 |
| --- | --- | --- |
| `robot.OPTIONS.EFFECTOR.AGX_GRIPPER` | `"agx_gripper"` | AGX 夹爪 |
| `robot.OPTIONS.EFFECTOR.REVO2` | `"revo2"` | REVO2 灵巧手 |

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

**功能说明：** 获取机械臂关节数量。

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

### 正运动学 — `fk()`

**功能说明：** 根据给定关节角度，使用机械臂内置的改进 DH（MDH）模型计算末端**法兰位姿**。

该接口为**离线计算**（不依赖 CAN 通信）。输出位姿格式与 `get_flange_pose()` 返回的 `.msg` 一致：  
`[x, y, z, roll, pitch, yaw]`（基坐标系），其中 `x/y/z` 单位为米，`roll/pitch/yaw` 单位为弧度（SDK 采用 ZYX 的 RPY 约定）。

**函数定义：**

```python
fk(self, joint_angles: list[float]) -> list[float]
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `joint_angles` | `list[float]` | 关节角度（单位：rad）。长度需与当前机型关节数（`robot.joint_nums`）一致 |

**返回值：** `list[float]`

`[x, y, z, roll, pitch, yaw]` — 法兰位姿（基坐标系）。

**使用示例：**

1）与 `get_joint_angles()` 组合（读取当前关节角 → FK）：

```python
ja = robot.get_joint_angles()
if ja is not None:
    flange_pose = robot.fk(ja.msg)
    print("fk 法兰:", flange_pose)
```

2）与 `get_leader_joint_angles()` 组合（读取主导臂角度 → FK）：

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

## 日志接口

### 日志行为与启用说明

`robot.log` 默认不会有可见输出：

- `Logger` 初始化时默认挂载 `NullHandler`（库安全默认行为）。
- 需要显式启用至少一种输出 handler。
- 新创建的 driver 实例默认未启用可见 handler；如果只调用 `robot.log.info(...)` 而不先启用 handler，通常看不到输出。

推荐用法：

1. 控制台输出（本地调试）：

```python
robot.log.console_enable(level=robot.log.Level.INFO)
```

2. 回调桥接输出（ROS/外部系统）：

```python
robot.log.bridge_enable(info=print, warning=print, error=print)
```

3. 直接使用标准库 logger（高级）：

```python
import logging

h = logging.StreamHandler()
h.setFormatter(robot.log.Formats.FULL)
robot.log.logger.addHandler(h)
robot.log.logger.setLevel(robot.log.Level.INFO)
```

> **提示：** 注意 handler 边界。`replace_handlers=True` 会先清空该 logger 上已有处理器再安装托管处理器；`replace_handlers=False` 会保留已有处理器，仅替换同类托管处理器。另需注意，`shutdown()` 负责清理的是该 `Logger` 封装创建的托管处理器；如果你直接往 `robot.log.logger` 挂了自定义处理器，需要由你自行管理和移除。

### 写入调试日志 — `robot.log.debug()`

**功能说明：** 输出调试级别日志，用于细粒度排查。

**函数定义：**

```python
robot.log.debug(msg: str, *args, **kwargs) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `msg` | `str` | 日志消息模板 |
| `*args` | `Any` | 用于 `msg` 的位置格式化参数 |
| `**kwargs` | `Any` | Python logging 标准关键字参数（如 `exc_info`） |

**使用示例：**

```python
robot.log.debug("debug message")
```

---

### 写入信息日志 — `robot.log.info()`

**功能说明：** 输出信息级别日志，用于常规运行记录。

**函数定义：**

```python
robot.log.info(msg: str, *args, **kwargs) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `msg` | `str` | 日志消息模板 |
| `*args` | `Any` | 用于 `msg` 的位置格式化参数 |
| `**kwargs` | `Any` | Python logging 标准关键字参数 |

**使用示例：**

```python
robot.log.info("common api smoke check done")
```

---

### 写入告警日志 — `robot.log.warning()`

**功能说明：** 输出告警级别日志，用于可恢复异常提示。

**函数定义：**

```python
robot.log.warning(msg: str, *args, **kwargs) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `msg` | `str` | 日志消息模板 |
| `*args` | `Any` | 用于 `msg` 的位置格式化参数 |
| `**kwargs` | `Any` | Python logging 标准关键字参数 |

**使用示例：**

```python
robot.log.warning("can frame delayed")
```

---

### 写入错误日志 — `robot.log.error()`

**功能说明：** 输出错误级别日志，用于失败或异常场景。

**函数定义：**

```python
robot.log.error(msg: str, *args, **kwargs) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `msg` | `str` | 日志消息模板 |
| `*args` | `Any` | 用于 `msg` 的位置格式化参数 |
| `**kwargs` | `Any` | Python logging 标准关键字参数 |

**使用示例：**

```python
robot.log.error("connect failed")
```

---

### 写入严重错误日志 — `robot.log.critical()`

**功能说明：** 输出严重错误级别日志，用于不可恢复的故障场景。

**函数定义：**

```python
robot.log.critical(msg: str, *args, **kwargs) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `msg` | `str` | 日志消息模板 |
| `*args` | `Any` | 用于 `msg` 的位置格式化参数 |
| `**kwargs` | `Any` | Python logging 标准关键字参数 |

**使用示例：**

```python
robot.log.critical("controller heartbeat lost")
```

---

### 写入异常日志 — `robot.log.exception()`

**功能说明：** 记录带异常堆栈的错误日志（等价于 `exc_info=True`）。

**函数定义：**

```python
robot.log.exception(msg: str, *args, **kwargs) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `msg` | `str` | 日志消息模板 |
| `*args` | `Any` | 用于 `msg` 的位置格式化参数 |
| `**kwargs` | `Any` | Python logging 标准关键字参数 |

**使用示例：**

```python
try:
    robot.connect()
except Exception:
    robot.log.exception("connect raised an exception")
```

---

### 日志级别常量 — `robot.log.Level.*`

**功能说明：** 日志相关 API 可直接使用内置级别常量，无需额外 `import logging`。

**可用常量：**

| 名称 | 值 | 说明 |
| --- | --- | --- |
| `robot.log.Level.NOTSET` | `0` | 无显式级别 |
| `robot.log.Level.DEBUG` | `10` | 调试级别 |
| `robot.log.Level.INFO` | `20` | 信息级别 |
| `robot.log.Level.WARNING` | `30` | 告警级别 |
| `robot.log.Level.WARN` | `30` | `WARNING` 的别名 |
| `robot.log.Level.ERROR` | `40` | 错误级别 |
| `robot.log.Level.CRITICAL` | `50` | 严重错误级别 |

**使用示例：**

```python
robot.log.configure(level=robot.log.Level.DEBUG)
robot.log.console_enable(level=robot.log.Level.INFO)
```

---

### 配置日志器 — `robot.log.configure()`

**功能说明：** 统一设置日志级别、传播行为、格式器，并可选择重置处理器。

**函数定义：**

```python
robot.log.configure(
    *,
    level: int | None = None,
    propagate: bool | None = None,
    formatter: logging.Formatter | None = None,
    replace_handlers: bool = False,
) -> None
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `level` | `int \| None` | 日志级别（如 `robot.log.Level.INFO`），为 `None` 时保持不变 |
| `propagate` | `bool \| None` | 是否向父级日志器传播 |
| `formatter` | `logging.Formatter \| None` | 应用于托管处理器的格式器 |
| `replace_handlers` | `bool` | 是否先清空全部处理器再应用配置 |

**使用示例：**

```python
robot.log.configure(level=robot.log.Level.DEBUG, propagate=False)
```

---

### 启用控制台日志 — `robot.log.console_enable()`

**功能说明：** 启用托管控制台输出（stream handler），支持重复日志节流。

**函数定义：**

```python
robot.log.console_enable(
    *,
    level: int = Logger.Level.INFO,
    emit_min_interval: float = 1.0,
    replace_handlers: bool = False,
    propagate: bool | None = None,
    formatter: logging.Formatter | None = None,
    handler: logging.Handler | None = None,
    stream=None,
) -> logging.Handler
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `level` | `int` | 控制台输出最低级别 |
| `emit_min_interval` | `float` | 相同键日志最小输出间隔，`<=0` 表示关闭节流 |
| `replace_handlers` | `bool` | 是否先清空全部处理器 |
| `propagate` | `bool \| None` | 是否更新日志传播行为 |
| `formatter` | `logging.Formatter \| None` | 控制台日志格式器 |
| `handler` | `logging.Handler \| None` | 自定义处理器；为空时默认创建 `StreamHandler` |
| `stream` | `Any` | 创建默认 `StreamHandler` 时传入的流对象 |

**返回值：** `logging.Handler`

**使用示例：**

```python
robot.log.console_enable(level=robot.log.Level.INFO, emit_min_interval=0.5)
```

---

### 禁用控制台日志 — `robot.log.console_disable()`

**功能说明：** 禁用并关闭托管控制台处理器。

**函数定义：**

```python
robot.log.console_disable() -> None
```

**使用示例：**

```python
robot.log.console_disable()
```

---

### 启用桥接日志 — `robot.log.bridge_enable()`

**功能说明：** 启用回调桥接日志输出，用于对接 ROS 或其他外部日志系统。

**函数定义：**

```python
robot.log.bridge_enable(
    *,
    debug: Callable[[str], None] | None = None,
    info: Callable[[str], None] | None = None,
    warning: Callable[[str], None] | None = None,
    error: Callable[[str], None] | None = None,
    critical: Callable[[str], None] | None = None,
    level: int = Logger.Level.INFO,
    emit_min_interval: float = 1.0,
    replace_handlers: bool = False,
    propagate: bool | None = None,
    formatter: logging.Formatter | None = None,
) -> logging.Handler
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `debug` | `Callable[[str], None] \| None` | 调试日志回调 |
| `info` | `Callable[[str], None] \| None` | 信息日志回调 |
| `warning` | `Callable[[str], None] \| None` | 告警日志回调 |
| `error` | `Callable[[str], None] \| None` | 错误日志回调 |
| `critical` | `Callable[[str], None] \| None` | 严重错误日志回调 |
| `level` | `int` | 桥接输出最低级别 |
| `emit_min_interval` | `float` | 相同键日志最小输出间隔，`<=0` 表示关闭节流 |
| `replace_handlers` | `bool` | 是否先清空全部处理器 |
| `propagate` | `bool \| None` | 是否更新日志传播行为 |
| `formatter` | `logging.Formatter \| None` | 桥接输出文本格式器 |

**返回值：** `logging.Handler`

**使用示例：**

```python
def ros_info(line: str):
    print("[ROS][INFO]", line)

robot.log.bridge_enable(info=ros_info)
```

---

### 禁用桥接日志 — `robot.log.bridge_disable()`

**功能说明：** 禁用并关闭托管桥接处理器。

**函数定义：**

```python
robot.log.bridge_disable() -> None
```

**使用示例：**

```python
robot.log.bridge_disable()
```

---

### 创建子日志器 — `robot.log.get_child()`

**功能说明：** 在当前日志命名空间下创建子日志器。

**函数定义：**

```python
robot.log.get_child(suffix: str) -> logging.Logger
```

**参数说明：**

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `suffix` | `str` | 子日志器后缀名称（必须为非空字符串） |

**返回值：** 子日志器对象。

**使用示例：**

```python
sub_log = robot.log.get_child("can.rx")
sub_log.info("rx started")
```

---

### 关闭日志系统 — `robot.log.shutdown()`

**功能说明：** 在进程退出前刷新并关闭日志系统。

**函数定义：**

```python
robot.log.shutdown() -> None
```

**使用示例：**

```python
robot.log.shutdown()
```
