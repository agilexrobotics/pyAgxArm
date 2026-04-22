# Changelog

## Table of Contents

- [Switch to 中文](#更新日志)
- [20621b94](#20621b94-en)
- [9e354493](#9e354493-en)

<a id="20621b94-en"></a>
## 20621b94

This update focuses on protocol-side capability expansion (CPV / joint error / joint assistance), instance lifecycle and communication recovery hardening, logging architecture unification, and synchronized tests/docs updates.

### Scope

- Commit range: [`20621b94...cfa5391b`](https://github.com/agilexrobotics/pyAgxArm/compare/20621b94ff15ae1b32cf81cce127cb49f1bfcaf4...cfa5391b66ed22323bcf4567053e3db10bf08af6) (**12 commits**).

### Features

- **CPV APIs**: Added CPV-related query/config/control paths and message wiring in Piper/Nero stacks, including position/velocity and parameter query/update paths (`acc`, `dcc`, `cv`, `pp`, `kp`, `ki`) exposed through driver-level APIs.
- **Piper capability expansion**:
  - Added API for clearing joint errors.
  - Added joint assistance rating query/config API set and corresponding message definitions.
- **Factory instance reuse system**:
  - Added config fingerprint cache (weakref-based) for `AgxArmFactory`.
  - Added global reuse policies: `"new"`, `"reuse"`, `"replace"`.
  - Added `detect_can_configs()` helper for CAN backend detection.
- **Communication robustness (CanComm/DriverContext)**:
  - Added communication error classification and handling (`hard_disconnect`, `link_down`, `no_buffer`).
  - Added communication error state APIs: `has_comm_error()` / `get_comm_error()`.
  - Added explicit `reconnect()` path and improved teardown/rebuild behavior.
  - Standardized comm layer behavior and cleaned interface contracts (including removal of stale `is_stopped` semantics).
- **Lifecycle cleanup and resilience**:
  - Driver instances now register GC finalizer-based best-effort cleanup.
  - Read-thread failures are captured and propagated through context error state for upper-layer recovery loops.
- **Logging architecture unification**:
  - Replaced legacy logger module with unified `Logger` facade.
  - Added managed `console` / `bridge` handlers with per-kind replacement logic, formatting presets, level constants, and throttling.
  - Added improved failure observability logs in key driver communication paths.

### Behavior Changes / Compatibility Notes

- **Config shape update**: removed `log` field from generated config structures and synced examples/docs accordingly.
- **Default log visibility**: runtime logs are not visible unless handlers are explicitly enabled (for example via `console_enable()` / `bridge_enable()`).
- **Utility module rename**: `vaildator` renamed to `validator` (import paths should use `pyAgxArm.utiles.validator`).

### Tests

- Added logger-focused tests (`tests/test_logger.py`) covering:
  - bridge/console behavior
  - throttling boundaries
  - instance-level isolation
  - managed handler cleanup in factory replace flow
- Extended virtual CAN test coverage:
  - reconnect and comm-error related paths
  - Piper/Nero/AgxGripper scenarios
  - additional slave payload coverage and helper updates

### Documentation & Maintenance

- Renamed repository assets path from `asserts/` to `assets/`.
- Updated demo scripts (`demos/*/test1.py`) to align with current config and API behavior.
- Synced API docs and first-time CAN guide updates with behavior changes in this range.

---

<a id="9e354493-en"></a>
## 9e354493

This update focuses on unified CAN communication, fuller driver and firmware branch support, improved public APIs and effector drivers, stronger packaging and test infrastructure, and added CI and kinematics (MDH) capabilities.

### Scope

- Commit range: [`9e354493...20621b94`](https://github.com/agilexrobotics/pyAgxArm/compare/9e354493c131d51246cb301eb58ad1082677f993...3d2dbdee590a754d6ef3e9953e19fb079536f9f3) (**21 commits**).

### Features

- **CAN communication**: Unified `CanComm` implementation across Linux / macOS / Windows via `python-can`; default bus receive `timeout` is **0.001 s** (`create_can_comm_config`, applied when you build the dict with `create_agx_arm_config` or by hand). Pass `timeout` into `create_agx_arm_config(...)` or set `comm.can.timeout` on your config dict for longer blocking reads. JSON files under `pyAgxArm/configs/` are reference profiles only — the library does not load them at runtime.
- **Drivers & firmware variants**: Piper (`DEFAULT`, `v183`, `v188`), Nero (`DEFAULT`, `v111`); Piper H / L / X re-export versioned subpackages; default drivers support `auto_set_motion_mode` consistent with `constants` and factory config.
- **Public API**: `ArmModel`, `PiperFW`, `NeroFW` in `arm_options`; extended `AgxArmFactory` registration; root package exports `__version__` and option enums.
- **Effectors**: AgxGripper and Revo2 drivers and message parsers aligned with current protocol usage.
- **Packaging (PEP 561)**: `py.typed`, package stubs (`*.pyi`), `license` metadata in PEP 621 table form for reliable builds on modern setuptools.
- **Tests**: Virtual CAN slaves (`tests/slaves/`), pytest suite covering factory routing, Piper/Nero motion and reads, firmware query, Piper-specific limits/crash APIs, AgxGripper and Revo2; `tests/API_COVERAGE.md` documents covered APIs and local pytest commands.
- **CI**: GitHub Actions matrix for Python 3.7–3.14 on Ubuntu 22.04 / latest; Python 3.6 runs in a container job for continued compatibility checks.
- **Kinematics (MDH)**: Modified Denavit-Hartenberg parameters for Piper / Piper H / L / X and Nero are in `pyAgxArm/configs/mdh_modified.json`. `get_mdh(robot)` returns per-link tuples `(d, a, alpha, theta_offset)`; `fk_from_mdh(mdh, joint_radians)` computes flange pose `[x, y, z, roll, pitch, yaw]` (meters, radians) with the same orientation convention as `get_flange_pose` (`R = R_z * R_y * R_x`). `robot.fk(joint_angles)` loads the model-specific MDH table during driver initialization and directly returns that 6D pose list.
- **Runtime SDK config toggles**: Added runtime switches `set_auto_set_motion_mode_enabled(enabled)` and `set_joint_limits_enabled(enabled)`. The corresponding config key `enable_joint_limits` is supported by all arm models (default `True`) and documented under dedicated "SDK Config Related / SDK 配置相关" sections in Piper/Nero API docs.

### Bug Fixes

- **Metadata**: Fixed `project.license` in `pyproject.toml` to a single PEP 621–valid form (`license = { text = "..." }`) to avoid setuptools / twine validation failures on CI.

### Miscellaneous

- Removed unused legacy `can_send` native module tree from the repository.
- **Source distribution**: `MANIFEST.in` includes `tests/` so sdist consumers receive the full test tree and docs under `tests/`.
- Series detection scripts aligned with `arm_options`; `pyAgxArm/configs/*.json` shipped as reference profiles (not loaded by the library); documentation refresh including WSL2 USB-CAN guidance where applicable.

---

# 更新日志

## 目录

- [切换到 English](#changelog)
- [20621b94](#20621b94-zh)
- [9e354493](#9e354493-zh)

<a id="20621b94-zh"></a>
## 20621b94

本次更新聚焦于协议能力扩展（CPV / 关节错误清除 / 关节助力）、实例生命周期与通信恢复机制加固、日志架构统一，以及测试与文档同步完善。

### 范围

- 提交范围：[`20621b94...cfa5391b`](https://github.com/agilexrobotics/pyAgxArm/compare/20621b94ff15ae1b32cf81cce127cb49f1bfcaf4...cfa5391b66ed22323bcf4567053e3db10bf08af6)（**12 个提交**）。

### 特性

- **CPV 接口补齐**：在 Piper/Nero 协议栈中补充 CPV 查询/配置/控制相关通路与消息定义，覆盖位置/速度与参数项（`acc`、`dcc`、`cv`、`pp`、`kp`、`ki`）的驱动层读写接口。
- **Piper 能力扩展**：
  - 新增关节错误清除接口。
  - 新增关节助力等级查询/配置接口及配套消息定义。
- **工厂实例复用体系**：
  - `AgxArmFactory` 引入基于配置指纹的弱引用缓存。
  - 新增全局复用策略：`"new"`、`"reuse"`、`"replace"`。
  - 新增 CAN 后端检测辅助接口 `detect_can_configs()`。
- **通信加固（CanComm/DriverContext）**：
  - 增加通信异常分类与处理（`hard_disconnect`、`link_down`、`no_buffer`）。
  - 新增通信错误状态接口：`has_comm_error()` / `get_comm_error()`。
  - 增加显式 `reconnect()` 恢复路径，并完善会话拆除/重建行为。
  - 统一通信层行为并清理接口契约（含移除历史 `is_stopped` 语义）。
- **生命周期清理与可恢复性**：
  - Driver 实例新增基于 GC 终结器的兜底清理逻辑。
  - 读线程异常可写入并透传至上下文错误状态，便于上层实现检测-恢复循环。
- **日志架构统一**：
  - 以统一 `Logger` 封装替换旧日志模块。
  - 提供托管 `console` / `bridge` handler、按类型替换机制、格式预设、级别常量与节流能力。
  - 在关键驱动通信失败路径补充可观测日志。

### 行为变更 / 兼容性说明

- **配置结构调整**：移除配置中的 `log` 字段，并同步示例与文档结构。
- **日志默认可见性**：运行时默认无可见日志输出；需显式启用 handler（如 `console_enable()` / `bridge_enable()`）。
- **工具模块重命名**：`vaildator` 重命名为 `validator`（导入路径应使用 `pyAgxArm.utiles.validator`）。

### 测试

- 新增日志专项测试（`tests/test_logger.py`），覆盖：
  - bridge/console 行为
  - 节流边界
  - 实例隔离
  - 工厂 replace 流程中的托管 handler 清理
- 扩展虚拟 CAN 测试覆盖：
  - reconnect 与通信错误相关路径
  - Piper/Nero/AgxGripper 场景
  - 从机载荷与测试辅助逻辑补充

### 文档与维护

- 仓库资源目录由 `asserts/` 重命名为 `assets/`。
- 更新示例脚本（`demos/*/test1.py`）以对齐当前配置与 API 行为。
- 同步更新 API 文档与首次 CAN 使用指南，覆盖本区间的行为变更。

---

<a id="9e354493-zh"></a>
## 9e354493

本次更新聚焦于 CAN 通信统一、驱动与固件分支能力补齐、公开 API 与末端驱动完善、打包与测试体系增强，并补充了 CI 与运动学（MDH）相关能力。

### 范围

- 提交范围：[`9e354493...20621b94`](https://github.com/agilexrobotics/pyAgxArm/compare/9e354493c131d51246cb301eb58ad1082677f993...3d2dbdee590a754d6ef3e9953e19fb079536f9f3)（**21 个提交**）。

### 特性

- **CAN 通信**：在 Linux / macOS / Windows 上通过 `python-can` 统一 `CanComm` 实现；默认总线接收 `timeout` 为 **0.001 s**（`create_can_comm_config` 的默认值，由 `create_agx_arm_config` 或自建配置字典使用）。需要更长阻塞读时，向 `create_agx_arm_config` 传入 `timeout=...`，或在自建配置中设置 `comm.can.timeout`。`pyAgxArm/configs/*.json` 仅为随包参考，运行时不会自动加载。
- **驱动与固件分支**：Piper（`DEFAULT`、`v183`、`v188`）、Nero（`DEFAULT`、`v111`）；Piper H / L / X 通过 `versions` 子包重导出；默认驱动与 `constants`、工厂配置一致的 `auto_set_motion_mode` 行为。
- **对外 API**：`arm_options` 中的 `ArmModel`、`PiperFW`、`NeroFW`；扩展 `AgxArmFactory` 注册表；根包导出 `__version__` 与选项枚举。
- **末端**：AgxGripper、Revo2 驱动与解析与当前协议用法对齐。
- **打包（PEP 561）**：`py.typed`、stub（`*.pyi`）、`pyproject.toml` 中采用 PEP 621 表格式 `license` 元数据，保证新版 setuptools / twine 校验通过。
- **测试**：虚拟 CAN 从机（`tests/slaves/`）、pytest 覆盖工厂路由、Piper/Nero 运动与读取、固件查询、Piper 专有极限/防撞等接口、夹爪与灵巧手；`tests/API_COVERAGE.md` 记录 API 覆盖与本地测试命令。
- **CI**：GitHub Actions 对 Python 3.7–3.14 在 Ubuntu 22.04 / latest 上矩阵构建；Python 3.6 使用容器任务以保留兼容性验证。
- **运动学（MDH）**：Piper / Piper H / L / X 与 Nero 的修正 DH 参数见 `pyAgxArm/configs/mdh_modified.json`。`get_mdh(robot)` 返回各连杆 `(d, a, alpha, theta_offset)`；`fk_from_mdh(mdh, joint_radians)` 计算法兰位姿 `[x, y, z, roll, pitch, yaw]`（米、弧度），姿态约定与 `get_flange_pose` 一致（`R = R_z·R_y·R_x`）。`robot.fk(joint_angles)` 在驱动初始化时载入对应 MDH 表，直接返回上述 6 维列表。
- **运行时 SDK 配置开关**：新增 `set_auto_set_motion_mode_enabled(enabled)` 与 `set_joint_limits_enabled(enabled)`。全部机型支持配置项 `enable_joint_limits`（默认 `True`），并在 Piper/Nero 文档中新增独立“SDK 配置相关”章节说明。

### Bug 修复

- **元数据**：将 `pyproject.toml` 中的 `project.license` 修正为符合 PEP 621 的单一合法写法（`license = { text = "..." }`），避免 CI 上 setuptools / twine 校验失败。

### 其它

- 移除仓库中未再引用的遗留 `can_send` 原生模块目录。
- **源码包**：`MANIFEST.in` 通过 `graft tests/` 将完整 `tests/` 纳入 sdist，便于下游获取测试与说明文档。
- 系列检测脚本与 `arm_options` 对齐；`pyAgxArm/configs/*.json` 作为参考配置随包分发（程序不自动读取）；文档更新（含 WSL2 USB-CAN 等相关说明）。
