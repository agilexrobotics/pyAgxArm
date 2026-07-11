# DaBai DCW Nero 手眼标定与使用

本文面向 Ubuntu 22.04、已激活 Conda 环境 `pyagxarm` 的工程师：Orbbec DaBai DCW 固定在 Nero 法兰，机器人通过 SocketCAN `can_piper` 通信。目标是取得 RGB/深度内参、色深外参和眼在手上变换，并将**原始深度图**像素换算到 Nero 基座坐标系。

## 范围与软件

- 相机：Orbbec DaBai DCW；SDK：Orbbec SDK v1。
- 官方源码：[OrbbecSDK](https://github.com/orbbec/OrbbecSDK)、[pyorbbecsdk](https://github.com/orbbec/pyorbbecsdk/tree/main)。使用 `pyorbbecsdk` 的 `main` 分支，Python 模块名为 `pyorbbecsdk`。
- 项目校准脚本：`pyAgxArm/demos/nero/orbbec_handeye_calib.py`。
- `pytest` 只用于测试，不是运行校准程序的运行时依赖。

## 安装 SDK v1

先激活目标环境，之后始终使用其 `python`，不要用系统 Python：

```bash
conda activate pyagxarm
which python
sudo apt update
sudo apt install -y cmake build-essential python3-dev libusb-1.0-0-dev
git clone --depth 1 --branch main https://github.com/orbbec/pyorbbecsdk.git
cd pyorbbecsdk
python -m pip install -r requirements.txt
cmake -S . -B build \
  -DPython3_ROOT_DIR="$CONDA_PREFIX" \
  -Dpybind11_DIR="$(pybind11-config --cmakedir)"
cmake --build build -j"$(nproc)"
cmake --install build
python -m pip install .
sudo bash scripts/install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

安装或刷新 udev 规则后，拔下再插入 DaBai DCW，再验证：

```bash
python examples/hello_orbbec.py
```

当前环境中的 `opencv-contrib-python 5.0.0.93` 虽暴露了相关常量，却没有 `cv2.calibrateHandEye`。校准前安装稳定版本（或明确限制为 `<5`），并确认 API 存在：

```bash
python -m pip install --upgrade 'opencv-contrib-python==4.13.0.92'
python -c 'import cv2; print(cv2.__version__, hasattr(cv2, "calibrateHandEye"))'
```

最后一条必须输出 `True`。如项目中同时安装了其他 OpenCV wheel，先清理冲突版本后再只保留一个 `opencv-contrib-python`。

## CAN 与硬件预检

确认 `can_piper` 已启动且比特率为 1 Mbps：

```bash
ip -details link show can_piper
candump can_piper
```

第一条输出应包含 `bitrate 1000000`；第二条在机器人已上电、总线接线正确且有报文时应持续显示流量。先断电再调整 CAN/电源接线，核对终端电阻、极性和共地；不要在接线或安装相机时给机械臂上电。本流程不发送任何自主运动命令：只在机器人处于安全、人工确认的静止姿态时读取法兰位姿。

## 采集与求解

将棋盘固定在工作空间内，使用 **10 x 7 个内角点**、方格边长 **0.02 m**。从仓库根目录运行：

```bash
conda activate pyagxarm
python pyAgxArm/demos/nero/orbbec_handeye_calib.py \
  --checkerboard-cols 10 \
  --checkerboard-rows 7 \
  --square-size 0.02 \
  --can-interface socketcan \
  --can-channel can_piper
```

画面中棋盘被检测到后，在每个由操作者安全摆放并静止的姿态按键：

- `s`：保存当前有效棋盘观测与当前法兰位姿。
- `c`：求解并写入标定 JSON。
- `q` 或 `ESC`：退出并关闭相机、机器人连接及窗口。

采集 15--30 个样本。每次既要改变平移，也要围绕至少两个不共线的旋转轴改变姿态；只沿单一轴转动会被运动多样性检查拒绝，因为手眼方程退化，无法可靠估计完整的刚体变换。棋盘必须固定不动，采样时避免振动、遮挡、反光和运动模糊。

默认样本文件为 `orbbec_handeye_samples.npz`，结果为 `orbbec_handeye_result.json`。每次 `s` 都会更新 NPZ，其中保存法兰位姿、棋盘 `rvec/tvec`、时间戳、相机序列号/流配置、RGB 与深度内参、畸变、深度比例、色深外参以及棋盘几何。`--samples captures/run1` 会自动补为 `captures/run1.npz`；指定非 `.npz` 扩展名会报错。恢复旧样本时，当前相机指纹、流配置、色深外参或棋盘内角点/方格尺寸不一致会被拒绝，应重新采集而不是混用。

仅对已有样本重新求解：

```bash
python pyAgxArm/demos/nero/orbbec_handeye_calib.py \
  --calibrate-only \
  --samples captures/run1.npz \
  --output captures/run1_result.json
```

结果 JSON 的 `T_color_depth`、`T_flange_color`、`T_flange_depth` 及其逆变换均会写出。所有平移和点坐标单位为米；每个变换的 `matrix_row_major_4x4` 是点转换的权威表示，XYZ/RPY 和四元数只供查看。检查 `sample_count`、`quality_warnings` 与 `consistency`，较大的棋盘基座位姿离散度表示应检查样本质量后重采。

## 原始深度像素到基座

约定 `T_A_B` 表示将 B 坐标系中的点映射到 A 坐标系：

```text
p_A = T_A_B * p_B
```

对于原始、未对齐的深度帧，`u` 为列、`v` 为行，取 `depth_raw = depth[v, u]`，并使用**深度内参**和 SDK 提供的 `depth_scale_m`：

```text
z = depth_raw * depth_scale_m
p_depth = [(u-cx)z/fx, (v-cy)z/fy, z]
p_base = T_base_flange * T_flange_depth * p_depth
```

从仓库根目录执行下例。它假设 `robot` 已按项目的 Nero 配置连接，`depth_raw_frame` 是当前 Orbbec 原始深度 `numpy` 数组；脚本目录显式加入导入路径，因此无需安装 demo 模块。

```python
import json
import sys
from pathlib import Path

import numpy as np

repo = Path.cwd()  # 必须是 pyAgxArm 仓库根目录
sys.path.insert(0, str(repo / "pyAgxArm" / "demos" / "nero"))
from orbbec_handeye_math import depth_pixel_to_base, pose6_to_matrix

result = json.loads((repo / "orbbec_handeye_result.json").read_text())
u, v = 320, 240
depth_raw = depth_raw_frame[v, u]
T_flange_depth = np.asarray(
    result["T_flange_depth"]["matrix_row_major_4x4"], dtype=float
).reshape(4, 4)
T_base_flange = pose6_to_matrix(robot.get_flange_pose().msg)

point_base_m = depth_pixel_to_base(
    u=u,
    v=v,
    depth_raw=depth_raw,
    depth_intrinsics=result["camera"]["depth_intrinsics"],
    depth_scale_m=result["camera"]["depth_scale_m"],
    T_flange_depth=T_flange_depth,
    T_base_flange=T_base_flange,
)
print(point_base_m)
```

`depth_raw <= 0`、非有限值或无效深度必须拒绝，不能当作原点。原始未对齐深度像素绝不能使用 RGB 内参；若先做 SDK 对齐，必须同时使用与对齐后图像相对应的坐标定义和内参，不能混用。

## 故障排查与验证

| 现象 | 处理 |
| --- | --- |
| `No module named pyorbbecsdk` | 激活 `pyagxarm`，执行 `which python`，按上面的 v1 `main` 构建步骤重装，再运行 `python examples/hello_orbbec.py`。 |
| `No device connected` 或权限错误 | 检查 USB 线和供电；重装/刷新 udev 规则后重新插拔相机。 |
| 没有 `calibrateHandEye` | 安装 `opencv-contrib-python==4.13.0.92`，并以 `hasattr(cv2, "calibrateHandEye")` 确认 `True`。 |
| `can_piper` 不存在或 `candump` 无报文 | 用 `ip -details link show can_piper` 检查接口和 1 Mbps，随后检查 CAN 适配器、终端电阻、接线、供电和机器人状态。 |
| 棋盘无法检测 | 核对 10 x 7 是内角点而非方格数，保持 0.02 m 方格设置，改善照明、焦距、清晰度和完整可见性。 |
| 样本多样性不足/同轴被拒绝 | 重采同时具有平移及至少两条不共线旋转轴的 15--30 个姿态。 |
| 归档几何不匹配 | 不要修改旧 NPZ 的棋盘参数或混用相机配置；用当前棋盘和相机重新采集。 |

运行自动化 Orbbec 测试（不连接真实硬件）：

```bash
python -m pytest -q \
  tests/test_orbbec_handeye_math.py \
  tests/test_orbbec_v1_camera.py \
  tests/test_orbbec_handeye_cli.py
```

单元测试不能证明真实 DaBai 的 USB/udev/SDK 流、真实 Nero 的 CAN 通信、安装刚性、棋盘检测质量或实际手眼精度。真实验收必须在 DaBai 和 Nero 接通后进行：选一个已知棋盘点，取得其对应的有效原始深度像素，按上式变换到基座系，并与该点的物理基座测量值比较误差。只有这项硬件测量能验证深度像素到基座点的最终精度。
