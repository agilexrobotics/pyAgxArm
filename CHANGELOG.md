# Changelog

> Project version history and release notes.

## Table of Contents

- [Switch to 中文](#更新日志)
- [Version 1.0.0](#version-100)

## Version 1.0.0

First stable release of the Python SDK for Agilex robotic arms. This version unifies CAN communication, expands firmware-aware drivers, and ships with installable type hints, virtual CAN tests, and CI.

### Features

- **CAN communication**: Unified `CanComm` implementation across Linux / macOS / Windows via `python-can`.
- **Drivers & firmware variants**: Piper (`DEFAULT`, `v183`, `v188`, `v189`), Nero (`DEFAULT`, `v111`, `v112`, `v120`, `v121`); Piper H / L / X re-export versioned subpackages.
- **Public API**: `ArmModel`, `PiperFW`, `NeroFW` in `arm_options`; extended `AgxArmFactory` registration; root package exports `__version__` and option enums.
- **Firmware profile resolution**: Added the public `resolve_firmware_profile(robot, firmware_version)` API so applications can map the version returned by `get_firmware()` to the SDK-supported Piper/Nero driver profile without duplicating version rules.
- **Effectors**: AgxGripper and Revo2 drivers and message parsers aligned with current protocol usage.
- **Packaging (PEP 561)**: `py.typed`, package stubs (`*.pyi`), `license` metadata in PEP 621 table form for reliable builds on modern setuptools.
- **Tests**: Virtual CAN slaves (`tests/slaves/`), pytest suite covering factory routing, Piper/Nero motion and reads, firmware query, Piper-specific limits/crash APIs, AgxGripper and Revo2; `tests/API_COVERAGE.md` documents covered APIs and local pytest commands.
- **CI**: GitHub Actions matrix for Python 3.7–3.14 on Ubuntu 22.04 / latest; Python 3.6 runs in a container job for continued compatibility checks.
- **Kinematics (MDH)**: Modified Denavit–Hartenberg parameters for Piper / Piper H / L / X and Nero are built into `pyAgxArm.api.constants` (`ROBOT_MDH_PRESET`). `get_mdh(robot)` returns each link `(d, a, alpha, theta_offset)`; `fk_from_mdh(mdh, joint_radians)` computes flange pose `[x, y, z, roll, pitch, yaw]` (meters, radians), with the same orientation convention as `get_flange_pose` (`R = R_z * R_y * R_x`). `robot.fk(joint_angles)` loads the corresponding MDH table during driver initialization.
- **Runtime SDK config toggles**: Added `set/get_auto_set_motion_mode_enabled()` and `set/get_joint_limits_enabled()`.

### Bug Fixes

- **Metadata**: Fixed `project.license` in `pyproject.toml` to a single PEP 621–valid form (`license = { text = "..." }`) to avoid setuptools / twine validation failures on CI.

### Miscellaneous

- Removed unused legacy `can_send` native module tree from the repository.
- **Source distribution**: `MANIFEST.in` includes `tests/` so sdist consumers receive the full test tree and docs under `tests/`.
- Demos updated for series detection and `arm_options`; documentation refresh including WSL2 USB-CAN guidance where applicable.

---

# 更新日志

> 项目版本历史与发布说明。

## 目录

- [切换到 English](#changelog)
- [版本 1.0.0](#版本-100)

## 版本 1.0.0

面向 Agilex 机械臂的首个稳定版 Python SDK。本版本统一 CAN 通信层、按固件分支扩展驱动，并提供可安装的 PEP 561 类型标注、虚拟 CAN 测试与持续集成。

### 特性

- **CAN 通信**：在 Linux / macOS / Windows 上通过 `python-can` 统一 `CanComm` 实现。
- **驱动与固件分支**：Piper（`DEFAULT`、`v183`、`v188`、`v189`）、Nero（`DEFAULT`、`v111`、`v112`、`v120`、`v121`）；Piper H / L / X 通过 `versions` 子包重导出。
- **对外 API**：`arm_options` 中的 `ArmModel`、`PiperFW`、`NeroFW`；扩展 `AgxArmFactory` 注册表；根包导出 `__version__` 与选项枚举。
- **固件档位解析**：新增公开接口 `resolve_firmware_profile(robot, firmware_version)`，应用可将 `get_firmware()` 返回的版本映射为 SDK 支持的 Piper/Nero 驱动档位，无需重复维护版本规则。
- **末端**：AgxGripper、Revo2 驱动与解析与当前协议用法对齐。
- **打包（PEP 561）**：`py.typed`、stub（`*.pyi`）、`pyproject.toml` 中采用 PEP 621 表格式 `license` 元数据，保证新版 setuptools / twine 校验通过。
- **测试**：虚拟 CAN 从机（`tests/slaves/`）、pytest 覆盖工厂路由、Piper/Nero 运动与读取、固件查询、Piper 专有极限/防撞等接口、夹爪与灵巧手；`tests/API_COVERAGE.md` 记录 API 覆盖与本地测试命令。
- **CI**：GitHub Actions 对 Python 3.7–3.14 在 Ubuntu 22.04 / latest 上矩阵构建；Python 3.6 使用容器任务以保留兼容性验证。
- **运动学（MDH）**：Piper / Piper H / L / X 与 Nero 的修正 DH 参数内置于 `pyAgxArm.api.constants`（`ROBOT_MDH_PRESET`）。`get_mdh(robot)` 返回各连杆 `(d, a, alpha, theta_offset)`；`fk_from_mdh(mdh, joint_radians)` 计算法兰位姿 `[x, y, z, roll, pitch, yaw]`（米、弧度），姿态约定与 `get_flange_pose` 一致（`R = R_z·R_y·R_x`）。`robot.fk(joint_angles)` 在驱动初始化时会载入对应 MDH 表。
- **运行时 SDK 配置开关**：新增 `set/get_auto_set_motion_mode_enabled()` 与 `set/get_joint_limits_enabled()`。

### Bug 修复

- **元数据**：将 `pyproject.toml` 中的 `project.license` 修正为符合 PEP 621 的单一合法写法（`license = { text = "..." }`），避免 CI 上 setuptools / twine 校验失败。

### 其它

- 移除仓库中未再引用的遗留 `can_send` 原生模块目录。
- **源码包**：`MANIFEST.in` 通过 `graft tests/` 将完整 `tests/` 纳入 sdist，便于下游获取测试与说明文档。
- 系列检测脚本与 `arm_options` 对齐；文档更新（含 WSL2 USB-CAN 等相关说明）。
