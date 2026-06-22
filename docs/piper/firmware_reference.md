# Piper Firmware Reference

> Supplement to [piper_api.md](piper_api.md#piper-api-documentation). Full version matrices and behavioral details.

[Switch to 中文](#piper-固件参考)

---

## Version Evolution (newest first)

Read top to bottom: each row is **what changed vs the previous SDK driver**.

| SDK driver | Constant | Arm firmware | Changes vs previous driver |
| --- | --- | --- | --- |
| `v189` | `PiperFW.V189` | ≥ S-V1.8-9 | Inherits `v188`. **`piper_x` only:** firmware fixes **`move_mit`** joint 4/5 sign — SDK no longer negates `p_des` / `v_des` / `t_ff`; **`move_cpv_pos`** still negates `pos` on joints **4 and 5** (not fixed in S-V1.8-9). |
| `v188` | `PiperFW.V188` | S-V1.8-8 | **New:** `get_ik_joint_angles` (IK feedback CAN `0x2AA` / `0x2AB` / `0x2AC`; after `move_p` only). **Changed:** `move_mit` 12-bit `t_ff` (±16 N·m all joints), **no CRC** on MIT frame. **Changed:** `get_arm_status` / `set_motion_mode` use V188 message types (`0x2A1` decode, `0x151` TX with `ArmMsgModeCtrlV188`); MIT motion mode code **0x06** (was **0x04** on older firmware). Inherits `v183` driver chain (CPV, limits, leader-follower, etc.). |
| `v183` | `PiperFW.V183` | S-V1.8-3 ~ S-V1.8-7 | **Changed:** `move_mit` — all joints `t_ff` input **±8 N·m** (8-bit + CRC, same frame layout as `default`). No per-joint ±32 / ×0.25 scaling. Inherits `default` for all other APIs. |
| `default` | `PiperFW.DEFAULT` | ≤ S-V1.8-2 | **Baseline:** `move_mit` 8-bit + CRC; joints **1–3** `t_ff` input ±32 N·m then **×0.25** before encode; joints **4–6** ±8 N·m. Full CPV stack (joints **1–6**, CAN **`0x181`–`0x186`**). `calibrate_joint`, leader-follower, Piper-only config APIs. |

---

## How to Choose (quick lookup)

Read firmware with [get_firmware()](piper_api.md#get-firmware-info--get_firmware) (format **S-VX.X-X**).

| Your firmware | `firmeware_version` | Constant |
| --- | --- | --- |
| S-V1.8-9 or later | `"v189"` | `PiperFW.V189` |
| S-V1.8-8 | `"v188"` | `PiperFW.V188` |
| S-V1.8-3 ~ S-V1.8-7 | `"v183"` | `PiperFW.V183` |
| S-V1.8-2 or earlier | `"default"` (or omit) | `PiperFW.DEFAULT` |

---

## Robot Model Variants (`piper` / `piper_h` / `piper_l` / `piper_x`)

| Model | SDK `robot` | Driver routing |
| --- | --- | --- |
| `piper` | `ArmModel.PIPER` | `piper/default`, `piper/versions/v183`, `piper/versions/v188`, `piper/versions/v189` |
| `piper_h` | `ArmModel.PIPER_H` | Same logic as `piper` per `PiperFW` (thin subclass) |
| `piper_l` | `ArmModel.PIPER_L` | Same as `piper_h` |
| `piper_x` | `ArmModel.PIPER_X` | Same `PiperFW` routing; **extra overrides on `V188` / `V189`** |

**`piper_x` @ `PiperFW.V188`:** before calling the parent implementation, **`move_mit`** negates `p_des`, `v_des`, and `t_ff` on joints **4 and 5**; **`move_cpv_pos`** negates `pos` on joints **4 and 5**.

**`piper_x` @ `PiperFW.V189`:** inherits `v188` except **`move_mit`** no longer applies joint 4/5 sign workaround (fixed in firmware); **`move_cpv_pos`** still negates `pos` on joints **4 and 5**.

`piper_h` / `piper_l` have **no** additional overrides beyond their `piper` counterpart.

---

## APIs with Firmware Requirements

All other public APIs in [piper_api.md](piper_api.md#piper-api-documentation) are available on **every** supported `PiperFW` driver unless noted in their section.

| API / group | Minimum SDK driver | Notes |
| --- | --- | --- |
| `get_ik_joint_angles` | `PiperFW.V188` | Firmware **≥ S-V1.8-8**; feedback after `move_p` only |
| `move_mit` | All (behavior differs) | See [MIT parameters by version](#mit-move_mit-parameters-by-version) |
| CPV (`move_cpv_*`, `get/set_cpv_*`) | All | Joints **1–6** only; inherited on `V183` / `V188` |
| `set_motion_mode('cpv')` | All | On `V188`, `set_motion_mode` signature lists `p/j/l/c/mit/js` only; CPV mode still available via `OPTIONS` / string `'cpv'` from parent |
| `calibrate_joint` | All | Implemented on `default`; inherited on `V183` / `V188` |
| Piper-only APIs | All | e.g. `set_installation_pos`, `set_payload`, `set_*_to_default` — on `default` driver |

---

## MIT (`move_mit`) Parameters by Version

| Driver | Joints | `t_ff` input range (N·m) | Pre-encode scaling | `t_ff` on wire | Frame |
| --- | --- | --- | --- | --- | --- |
| `DEFAULT` | 1–3 | ±32 | **×0.25** (effective ±8 before 8-bit encode) | 8-bit | 8 bytes; low 4 bits of byte 7 = `t_ff`, high 4 bits = **CRC** |
| `DEFAULT` | 4–6 | ±8 | — | 8-bit | same |
| `V183` | 1–6 | ±8 | — | 8-bit | same as `DEFAULT` (inherits default parser codec) |
| `V188` | 1–6 | ±16 | — | **12-bit** | 8 bytes; **no CRC** (bytes 6–7 carry `t_ff`) |

Precision (N·m): `DEFAULT` 1–3 ≈ 2.51e-1, 4–6 ≈ 6.28e-2; `V183` all ≈ 6.28e-2; `V188` all ≈ 7.81e-3.

---

## API Availability Matrix (full)

Legend: **✅** supported · **⚠️** supported with version-specific behavior.

| API / capability | `DEFAULT` (≤ S-V1.8-2) | `V183` (S-V1.8-3~7) | `V188` (≥ S-V1.8-8) |
| --- | :---: | :---: | :---: |
| Connect / disconnect, `enable` / `disable`, `reset` | ✅ | ✅ | ✅ |
| `move_j` / `move_js` / `move_p` / `move_l` / `move_c` | ✅ | ✅ | ✅ |
| `move_mit` | ✅ | ✅ | ✅ |
| CPV (`move_cpv_*`, `get/set_cpv_*`) | ✅ | ✅ | ✅ |
| `calibrate_joint` | ✅ | ✅ | ✅ |
| Leader-follower (`set_leader_mode`, `set_follower_mode`, `move_leader_*`, …) | ✅ | ✅ | ✅ |
| `get_leader_joint_angles` | ✅ | ✅ | ✅ |
| `get_ik_joint_angles` | — | — | ✅ |
| Piper-only (`set_installation_pos`, `set_payload`, assistance rating, …) | ✅ | ✅ | ✅ |
| `get_arm_status` / `set_motion_mode` | ✅ | ✅ | ⚠️ V188 message types & MIT mode code |

---

## Version-Specific Behavior (full)

| Topic | `DEFAULT` | `V183` | `V188` | `V189` |
| --- | --- | --- | --- | --- |
| **`move_mit` `t_ff`** | 1–3: ±32→×0.25; 4–6: ±8; 8-bit+CRC | All joints ±8; 8-bit+CRC | All joints ±16; 12-bit, no CRC | Inherits `V188` |
| **`get_arm_status` `mode_feedback` (MIT)** | `MOVE_MIT` = **0x04** | Same as `DEFAULT` | `MOVE_MIT` = **0x06** | Inherits `V188` |
| **`set_motion_mode` / mode TX** | Default `ArmMsgModeCtrl` @ `0x151` | Inherits `DEFAULT` | `ArmMsgModeCtrlV188` @ `0x151` | Inherits `V188` |
| **`get_arm_status` RX** | Default status @ `0x2A1` | Inherits `DEFAULT` | `ArmMsgFeedbackStatusV188` @ `0x2A1` | Inherits `V188` |
| **CPV CAN IDs** | `0x181`–`0x186` (joints 1–6) | Inherits | Inherits | Inherits |
| **`piper_x` joint sign** | — | — | Joints **4, 5**: negate in `move_mit` and `move_cpv_pos` | `move_mit` fixed (no flip); `move_cpv_pos` still negates joints **4, 5** |

---

## Per-Version Quick Reference

### Firmware ≥ S-V1.8-9 → use `PiperFW.V189`

- Inherits all `V188` APIs.
- For **`piper_x`**, `move_mit` no longer negates joints 4–5 (firmware fixed); `move_cpv_pos` still negates joints 4–5.

### Firmware S-V1.8-8 → use `PiperFW.V188`

- 12-bit MIT, ±16 N·m, no CRC.
- `get_ik_joint_angles` after `move_p` (CAN `0x2AA`–`0x2AC`).
- Match `get_arm_status` / `set_motion_mode` to V188 protocol (MIT mode **0x06**).
- For **`piper_x`**, account for joints 4–5 sign convention in SDK.

### Firmware S-V1.8-3 ~ S-V1.8-7 → use `PiperFW.V183`

- 8-bit MIT + CRC; all joints `t_ff` ±8 N·m.
- Same CPV and other APIs as `DEFAULT`.

### Firmware ≤ S-V1.8-2 → use `PiperFW.DEFAULT`

- 8-bit MIT + CRC; joints 1–3 use ±32 input with ×0.25 scaling.
- Full CPV and Piper feature set.

---

# Piper 固件参考

> [piper_api.md](piper_api.md#piper-机械臂-api-使用文档) 的补充文档：完整版本矩阵与行为差异。主手册仅保留简短固件说明。

[Switch to English](#piper-firmware-reference)

---

## 版本演进（从新到旧）

自上而下阅读：每行表示**相对上一档 SDK 驱动的变化**。

| SDK 驱动 | 常量 | 机械臂固件 | 相对上一版的变化 |
| --- | --- | --- | --- |
| `v189` | `PiperFW.V189` | ≥ S-V1.8-9 | 继承 `v188`。**仅 `piper_x`：** 固件修复 **`move_mit`** 4/5 轴符号，SDK 不再对 `p_des` / `v_des` / `t_ff` 取反；**`move_cpv_pos`** 仍对 **4、5 轴** `pos` 取反（S-V1.8-9 未修复）。 |
| `v188` | `PiperFW.V188` | S-V1.8-8 | **新增：** `get_ik_joint_angles`（IK 反馈 CAN `0x2AA` / `0x2AB` / `0x2AC`；仅 `move_p` 后可用）。**变更：** `move_mit` 12-bit `t_ff`（全关节 ±16 N·m），MIT 帧**无 CRC**。**变更：** `get_arm_status` / `set_motion_mode` 使用 V188 报文（`0x2A1` 解码、`0x151` 下发 `ArmMsgModeCtrlV188`）；MIT 运动模式码 **0x06**（旧固件为 **0x04**）。继承 `v183` 驱动链（CPV、限位、主从等）。 |
| `v183` | `PiperFW.V183` | S-V1.8-3 ~ S-V1.8-7 | **变更：** `move_mit` 全关节 `t_ff` 输入 **±8 N·m**（8-bit + CRC，帧布局同 `default`）；取消 1–3 轴 ±32 / ×0.25 分轴逻辑。其余 API 继承 `default`。 |
| `default` | `PiperFW.DEFAULT` | ≤ S-V1.8-2 | **基线：** `move_mit` 8-bit + CRC；**1–3 轴** `t_ff` 输入 ±32 N·m 再 **×0.25** 后编码；**4–6 轴** ±8 N·m。完整 CPV（**1–6 轴**，CAN **`0x181`–`0x186`**）。含 `calibrate_joint`、主从、Piper 专有配置 API。 |

---

## 如何选择（速查）

通过 [get_firmware()](piper_api.md#读取固件信息--get_firmware) 读取固件（格式 **S-VX.X-X**）。

| 固件版本 | `firmeware_version` | 常量 |
| --- | --- | --- |
| S-V1.8-9 及更新 | `"v189"` | `PiperFW.V189` |
| S-V1.8-8 | `"v188"` | `PiperFW.V188` |
| S-V1.8-3 ~ S-V1.8-7 | `"v183"` | `PiperFW.V183` |
| S-V1.8-2 及更早 | `"default"`（或不填） | `PiperFW.DEFAULT` |

---

## 机型变型（`piper` / `piper_h` / `piper_l` / `piper_x`）

| 机型 | SDK `robot` | 驱动路由 |
| --- | --- | --- |
| `piper` | `ArmModel.PIPER` | `piper/default`、`piper/versions/v183`、`piper/versions/v188`、`piper/versions/v189` |
| `piper_h` | `ArmModel.PIPER_H` | 与 `piper` 相同 `PiperFW` 路由（薄子类） |
| `piper_l` | `ArmModel.PIPER_L` | 同 `piper_h` |
| `piper_x` | `ArmModel.PIPER_X` | 同 `PiperFW` 路由；**`V188` / `V189` 有额外 override** |

**`piper_x` @ `PiperFW.V188`：** 调用父类前，**`move_mit`** 对 **4、5 轴** 的 `p_des`、`v_des`、`t_ff` 取反；**`move_cpv_pos`** 对 **4、5 轴** 的 `pos` 取反。

**`piper_x` @ `PiperFW.V189`：** 继承 `v188`，但 **`move_mit`** 不再做 4/5 轴符号 workaround（固件已修复）；**`move_cpv_pos`** 仍对 **4、5 轴** `pos` 取反。

`piper_h` / `piper_l` 除对应 `piper` 驱动外**无**额外 override。

---

## 有固件要求的 API

[piper_api.md](piper_api.md#piper-机械臂-api-使用文档) 中其余公开 API 在**各 `PiperFW` 驱动上均可用**（除非该 API 小节另有说明）。

| API / 分组 | 最低 SDK 驱动 | 说明 |
| --- | --- | --- |
| `get_ik_joint_angles` | `PiperFW.V188` | 固件 **≥ S-V1.8-8**；仅 `move_p` 后有反馈 |
| `move_mit` | 均有（行为不同） | 见 [MIT 分版本参数](#mitmove_mit分版本参数) |
| CPV（`move_cpv_*`、`get/set_cpv_*`） | 均有 | 仅 **1–6 轴**；`V183` / `V188` 继承 |
| `set_motion_mode('cpv')` | 均有 | `V188` 的 `set_motion_mode` 类型标注为 `p/j/l/c/mit/js`；仍可通过父类 `OPTIONS` / 字符串 `'cpv'` 使用 CPV |
| `calibrate_joint` | 均有 | 在 `default` 实现；`V183` / `V188` 继承 |
| Piper 专有 API | 均有 | 如 `set_installation_pos`、`set_payload`、`set_*_to_default` 等 |

---

## MIT（`move_mit`）分版本参数

| 驱动 | 关节 | `t_ff` 输入范围（N·m） | 编码前处理 | 线上 `t_ff` | 帧格式 |
| --- | --- | --- | --- | --- | --- |
| `DEFAULT` | 1–3 | ±32 | **×0.25**（8-bit 编码前等效 ±8） | 8-bit | 8 字节；第 7 字节低 4 bit = `t_ff`，高 4 bit = **CRC** |
| `DEFAULT` | 4–6 | ±8 | — | 8-bit | 同上 |
| `V183` | 1–6 | ±8 | — | 8-bit | 同 `DEFAULT`（继承 default 编解码） |
| `V188` | 1–6 | ±16 | — | **12-bit** | 8 字节；**无 CRC**（第 6–7 字节为 `t_ff`） |

精度（N·m）：`DEFAULT` 1–3 ≈ 2.51e-1，4–6 ≈ 6.28e-2；`V183` 全关节 ≈ 6.28e-2；`V188` 全关节 ≈ 7.81e-3。

---

## API 支持矩阵（完整）

图例：**✅** 支持 · **⚠️** 支持但行为因版本而异。

| API / 能力 | `DEFAULT`（≤ S-V1.8-2） | `V183`（S-V1.8-3~7） | `V188`（≥ S-V1.8-8） |
| --- | :---: | :---: | :---: |
| 连接 / 断开、`enable` / `disable`、`reset` | ✅ | ✅ | ✅ |
| `move_j` / `move_js` / `move_p` / `move_l` / `move_c` | ✅ | ✅ | ✅ |
| `move_mit` | ✅ | ✅ | ✅ |
| CPV（`move_cpv_*`、`get/set_cpv_*`） | ✅ | ✅ | ✅ |
| `calibrate_joint` | ✅ | ✅ | ✅ |
| 主从（`set_leader_mode`、`set_follower_mode`、`move_leader_*` 等） | ✅ | ✅ | ✅ |
| `get_leader_joint_angles` | ✅ | ✅ | ✅ |
| `get_ik_joint_angles` | — | — | ✅ |
| Piper 专有（`set_installation_pos`、`set_payload`、助力系数等） | ✅ | ✅ | ✅ |
| `get_arm_status` / `set_motion_mode` | ✅ | ✅ | ⚠️ V188 报文类型与 MIT 模式码 |

---

## 版本差异说明（完整）

| 主题 | `DEFAULT` | `V183` | `V188` | `V189` |
| --- | --- | --- | --- | --- |
| **`move_mit` `t_ff`** | 1–3：±32→×0.25；4–6：±8；8-bit+CRC | 全关节 ±8；8-bit+CRC | 全关节 ±16；12-bit，无 CRC | 继承 `V188` |
| **`get_arm_status` 中 MIT 模式反馈** | `MOVE_MIT` = **0x04** | 同 `DEFAULT` | `MOVE_MIT` = **0x06** | 继承 `V188` |
| **`set_motion_mode` / 模式下发** | 默认 `ArmMsgModeCtrl` @ `0x151` | 继承 `DEFAULT` | `ArmMsgModeCtrlV188` @ `0x151` | 继承 `V188` |
| **`get_arm_status` 接收** | 默认状态 @ `0x2A1` | 继承 `DEFAULT` | `ArmMsgFeedbackStatusV188` @ `0x2A1` | 继承 `V188` |
| **CPV CAN ID** | `0x181`–`0x186`（1–6 轴） | 继承 | 继承 | 继承 |
| **`piper_x` 关节符号** | — | — | **4、5 轴**：`move_mit` 与 `move_cpv_pos` 取反 | `move_mit` 已修复（不取反）；`move_cpv_pos` 仍对 **4、5 轴** 取反 |

---

## 分版本用户速查

### 固件 ≥ S-V1.8-9 → `PiperFW.V189`

- 继承 `V188` 全部 API。
- **`piper_x`**：`move_mit` 不再对 4、5 轴取反（固件已修复）；`move_cpv_pos` 仍对 4、5 轴取反。

### 固件 S-V1.8-8 → `PiperFW.V188`

- 12-bit MIT，全关节 ±16 N·m，无 CRC。
- `get_ik_joint_angles`：`move_p` 后可用（CAN `0x2AA`–`0x2AC`）。
- `get_arm_status` / `set_motion_mode` 需匹配 V188 协议（MIT 模式 **0x06**）。
- **`piper_x`** 需注意 SDK 对 4、5 轴的符号约定。

### 固件 S-V1.8-3 ~ S-V1.8-7 → `PiperFW.V183`

- 8-bit MIT + CRC；全关节 `t_ff` ±8 N·m。
- CPV 及其它 API 与 `DEFAULT` 相同。

### 固件 ≤ S-V1.8-2 → `PiperFW.DEFAULT`

- 8-bit MIT + CRC；1–3 轴 ±32 输入并 ×0.25 缩放。
- 完整 CPV 与 Piper 功能集。
