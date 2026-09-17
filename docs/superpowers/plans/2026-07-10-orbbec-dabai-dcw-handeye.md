# Orbbec DaBai DCW Hand-Eye Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an Orbbec SDK v1 eye-in-hand calibration demo that converts raw DaBai DCW depth pixels into metric Nero base-frame points.

**Architecture:** Keep hardware-independent transform and hand-eye math in one module, isolate all `pyorbbecsdk` calls in a small camera adapter, and keep robot/camera/UI orchestration in the CLI demo. The adapter normalizes Orbbec's depth-to-color translation from millimetres to metres, and every transform follows `T_A_B * p_B = p_A`.

**Tech Stack:** Python 3.10, NumPy, OpenCV contrib, Orbbec SDK v1 `pyorbbecsdk`, pyAgxArm SocketCAN, pytest.

---

## File Map

- Create `pyAgxArm/demos/nero/orbbec_handeye_math.py`: transforms, depth deprojection, sample persistence, hand-eye solver, consistency metrics, result serialization.
- Create `pyAgxArm/demos/nero/orbbec_v1_camera.py`: lazy SDK import, device/profile selection, frame conversion, normalized camera metadata.
- Create `pyAgxArm/demos/nero/orbbec_handeye_calib.py`: Nero connection, interactive collection, calibration-only mode, CLI and cleanup.
- Create `tests/test_orbbec_handeye_math.py`: hardware-independent geometry, persistence, solver validation, and result schema tests.
- Create `tests/test_orbbec_v1_camera.py`: SDK-boundary tests using lightweight fake SDK objects.
- Create `tests/test_orbbec_handeye_cli.py`: parser defaults and collection helper tests without hardware.
- Create `docs/nero/orbbec_dabai_handeye.md`: SDK v1 build, udev setup, calibration and depth conversion instructions.
- Delete `pyAgxArm/demos/nero/d435i_handeye_calib.py`: obsolete temporary implementation for the incorrectly identified camera.
- Delete `tests/test_d435i_handeye_calib.py`: replaced by Orbbec-specific tests.

### Task 1: Transform And Depth Geometry Core

**Files:**
- Create: `tests/test_orbbec_handeye_math.py`
- Create: `pyAgxArm/demos/nero/orbbec_handeye_math.py`

- [ ] **Step 1: Write failing checkerboard and transform tests**

Load the demo module by adding `pyAgxArm/demos/nero` to `sys.path`, then add these tests:

```python
def test_checkerboard_points_use_inner_corners_and_metres():
    points = math3d.create_checkerboard_object_points((10, 7), 0.02)
    assert points.shape == (70, 3)
    np.testing.assert_allclose(points[0], [0.0, 0.0, 0.0])
    np.testing.assert_allclose(points[-1], [0.18, 0.12, 0.0])


def test_pose6_round_trip_uses_zyx_rpy():
    pose = [0.1, -0.2, 0.3, 0.2, -0.1, 0.4]
    result = math3d.matrix_to_pose6(math3d.pose6_to_matrix(pose))
    np.testing.assert_allclose(result, pose, atol=1e-9)


def test_named_transform_maps_child_point_to_parent():
    transform = np.eye(4)
    transform[:3, 3] = [1.0, 2.0, 3.0]
    point = math3d.transform_point(transform, [0.1, 0.2, 0.3])
    np.testing.assert_allclose(point, [1.1, 2.2, 3.3])
```

- [ ] **Step 2: Run the new tests and verify RED**

Run: `python -m pytest -q tests/test_orbbec_handeye_math.py`

Expected: collection fails because `orbbec_handeye_math` does not exist.

- [ ] **Step 3: Implement the minimal transform core**

Implement these public functions with NumPy and no hardware imports:

```python
def create_checkerboard_object_points(checkerboard, square_size_m):
    cols, rows = checkerboard
    if cols <= 0 or rows <= 0 or square_size_m <= 0:
        raise ValueError("Checkerboard dimensions and square size must be positive.")
    points = np.zeros((cols * rows, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points[:, :2] *= float(square_size_m)
    return points


def transform_point(T_parent_child, point_child):
    T = np.asarray(T_parent_child, dtype=np.float64).reshape(4, 4)
    point = np.asarray(point_child, dtype=np.float64).reshape(3)
    return (T @ np.r_[point, 1.0])[:3]


def invert_transform(T_parent_child):
    T = np.asarray(T_parent_child, dtype=np.float64).reshape(4, 4)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = T[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ T[:3, 3]
    return result
```

Implement `pose6_to_matrix()` and `matrix_to_pose6()` with the repository's ZYX roll/pitch/yaw convention:

```python
def pose6_to_matrix(pose):
    x, y, z, roll, pitch, yaw = [float(value) for value in pose]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    result[:3, 3] = [x, y, z]
    return result


def matrix_to_pose6(transform):
    T = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    pitch = math.asin(float(np.clip(-T[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) < 1e-9:
        roll = 0.0
        yaw = math.atan2(-T[0, 1], T[1, 1])
    else:
        roll = math.atan2(T[2, 1], T[2, 2])
        yaw = math.atan2(T[1, 0], T[0, 0])
    return np.array([T[0, 3], T[1, 3], T[2, 3], roll, pitch, yaw])
```

- [ ] **Step 4: Run transform tests and verify GREEN**

Run: `python -m pytest -q tests/test_orbbec_handeye_math.py`

Expected: `3 passed`.

- [ ] **Step 5: Write failing depth deprojection and transform-chain tests**

```python
def test_depth_pixel_to_camera_point_converts_sdk_mm_scale_to_metres():
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}
    point = math3d.deproject_depth_pixel(420, 290, 1000, intrinsics, 0.001)
    np.testing.assert_allclose(point, [0.2, 0.1, 1.0])


def test_depth_pixel_to_base_uses_base_flange_then_flange_depth():
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}
    T_flange_depth = np.eye(4)
    T_flange_depth[:3, 3] = [0.0, 0.0, 0.1]
    T_base_flange = np.eye(4)
    T_base_flange[:3, 3] = [0.5, 0.0, 0.0]
    point = math3d.depth_pixel_to_base(
        320, 240, 1000, intrinsics, 0.001, T_flange_depth, T_base_flange
    )
    np.testing.assert_allclose(point, [0.5, 0.0, 1.1])


@pytest.mark.parametrize("depth", [0, -1, float("nan"), float("inf")])
def test_deprojection_rejects_invalid_depth(depth):
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}
    with pytest.raises(ValueError, match="depth"):
        math3d.deproject_depth_pixel(320, 240, depth, intrinsics, 0.001)
```

- [ ] **Step 6: Run depth tests and verify RED**

Run: `python -m pytest -q tests/test_orbbec_handeye_math.py`

Expected: failures report missing `deproject_depth_pixel` and `depth_pixel_to_base`.

- [ ] **Step 7: Implement depth deprojection and composition**

```python
def deproject_depth_pixel(
    u, v, depth_raw, intrinsics, depth_scale_m,
    min_depth_m=0.0, max_depth_m=None,
):
    depth = float(depth_raw) * float(depth_scale_m)
    if not np.isfinite(depth) or depth <= 0.0:
        raise ValueError("depth must be finite and greater than zero")
    if depth < float(min_depth_m) or (
        max_depth_m is not None and depth > float(max_depth_m)
    ):
        raise ValueError("depth is outside the configured range")
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    x = (float(u) - float(intrinsics["cx"])) * depth / fx
    y = (float(v) - float(intrinsics["cy"])) * depth / fy
    return np.array([x, y, depth], dtype=np.float64)


def depth_pixel_to_base(
    u, v, depth_raw, depth_intrinsics, depth_scale_m,
    T_flange_depth, T_base_flange,
):
    point_depth = deproject_depth_pixel(
        u, v, depth_raw, depth_intrinsics, depth_scale_m
    )
    return transform_point(
        np.asarray(T_base_flange) @ np.asarray(T_flange_depth), point_depth
    )
```

- [ ] **Step 8: Run the geometry tests and commit**

Run: `python -m pytest -q tests/test_orbbec_handeye_math.py`

Expected: all tests pass.

```bash
git add pyAgxArm/demos/nero/orbbec_handeye_math.py tests/test_orbbec_handeye_math.py
git commit -m "feat(nero): add hand-eye transform geometry"
```

### Task 2: Orbbec SDK v1 Metadata Normalization

**Files:**
- Create: `tests/test_orbbec_v1_camera.py`
- Create: `pyAgxArm/demos/nero/orbbec_v1_camera.py`

- [ ] **Step 1: Write failing SDK metadata tests**

Use `SimpleNamespace` objects so tests do not import `pyorbbecsdk`:

```python
def test_intrinsic_and_distortion_are_normalized_for_opencv():
    intrinsic = SimpleNamespace(width=640, height=480, fx=500.0, fy=501.0, cx=319.5, cy=239.5)
    distortion = SimpleNamespace(k1=1.0, k2=2.0, k3=3.0, k4=4.0, k5=5.0, k6=6.0, p1=0.1, p2=0.2)
    assert camera.normalize_intrinsic(intrinsic) == {
        "width": 640, "height": 480, "fx": 500.0, "fy": 501.0,
        "cx": 319.5, "cy": 239.5,
    }
    assert camera.normalize_distortion(distortion) == [1.0, 2.0, 0.1, 0.2, 3.0, 4.0, 5.0, 6.0]


def test_d2c_transform_is_depth_to_color_and_converts_mm_to_metres():
    sdk_transform = SimpleNamespace(
        rot=np.eye(3, dtype=np.float32).reshape(-1),
        transform=np.array([10.0, -20.0, 30.0], dtype=np.float32),
    )
    T_color_depth = camera.normalize_d2c_transform(sdk_transform)
    np.testing.assert_allclose(T_color_depth[:3, :3], np.eye(3))
    np.testing.assert_allclose(T_color_depth[:3, 3], [0.01, -0.02, 0.03])


def test_depth_scale_is_converted_from_sdk_mm_to_metres():
    assert camera.depth_scale_mm_to_m(1.0) == pytest.approx(0.001)
```

- [ ] **Step 2: Run SDK metadata tests and verify RED**

Run: `python -m pytest -q tests/test_orbbec_v1_camera.py`

Expected: collection fails because `orbbec_v1_camera` does not exist.

- [ ] **Step 3: Implement pure normalization helpers and lazy SDK import**

```python
def require_orbbec_sdk():
    try:
        import pyorbbecsdk
    except ImportError as exc:
        raise RuntimeError(
            "Orbbec SDK v1 is required. Follow docs/nero/orbbec_dabai_handeye.md."
        ) from exc
    return pyorbbecsdk


def normalize_intrinsic(intrinsic):
    return {
        "width": int(intrinsic.width), "height": int(intrinsic.height),
        "fx": float(intrinsic.fx), "fy": float(intrinsic.fy),
        "cx": float(intrinsic.cx), "cy": float(intrinsic.cy),
    }


def normalize_distortion(distortion):
    return [float(getattr(distortion, name)) for name in (
        "k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"
    )]


def normalize_d2c_transform(extrinsic):
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(extrinsic.rot, dtype=np.float64).reshape(3, 3)
    result[:3, 3] = np.asarray(extrinsic.transform, dtype=np.float64).reshape(3) / 1000.0
    return result


def depth_scale_mm_to_m(scale):
    value = float(scale) / 1000.0
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError("Orbbec depth scale must be finite and positive.")
    return value
```

- [ ] **Step 4: Run SDK metadata tests and verify GREEN**

Run: `python -m pytest -q tests/test_orbbec_v1_camera.py`

Expected: `3 passed`.

- [ ] **Step 5: Write failing profile-selection and metadata-fingerprint tests**

Test that `select_video_profile()` tries the requested profile first, falls back to `get_default_video_stream_profile()` only after an SDK exception, and returns `used_fallback=True`:

```python
class FakeProfiles:
    def __init__(self):
        self.requests = []
        self.default = object()

    def get_video_stream_profile(self, width, height, image_format, fps):
        self.requests.append((width, height, image_format, fps))
        raise RuntimeError("unsupported")

    def get_default_video_stream_profile(self):
        return self.default


def test_profile_selection_reports_fallback():
    profiles = FakeProfiles()
    profile, used_fallback = camera.select_video_profile(
        profiles, 1280, 720, 30, ("RGB", "MJPG")
    )
    assert profile is profiles.default
    assert used_fallback is True
    assert profiles.requests == [
        (1280, 720, "RGB", 30),
        (1280, 720, "MJPG", 30),
    ]
```

Add a `build_camera_metadata()` test with fake device/profile objects:

```python
def test_build_camera_metadata_preserves_calibration_and_profiles():
    intrinsic = SimpleNamespace(width=640, height=480, fx=500.0, fy=501.0, cx=319.5, cy=239.5)
    distortion = SimpleNamespace(k1=0.0, k2=0.0, k3=0.0, k4=0.0, k5=0.0, k6=0.0, p1=0.0, p2=0.0)
    extrinsic = SimpleNamespace(
        rot=np.eye(3, dtype=np.float32).reshape(-1),
        transform=np.array([10.0, 0.0, 0.0], dtype=np.float32),
    )
    camera_param = SimpleNamespace(
        rgb_intrinsic=intrinsic,
        depth_intrinsic=intrinsic,
        rgb_distortion=distortion,
        depth_distortion=distortion,
        transform=extrinsic,
    )
    device_info = SimpleNamespace(
        get_name=lambda: "DaBai DCW",
        get_serial_number=lambda: "ABC",
        get_firmware_version=lambda: "2460",
    )
    metadata = camera.build_camera_metadata(
        camera_param=camera_param,
        device_info=device_info,
        color_profile="1280x720@30_RGB",
        depth_profile="640x480@30_Y16",
        depth_scale_mm=1.0,
    )
    assert metadata["serial"] == "ABC"
    assert metadata["firmware"] == "2460"
    assert metadata["depth_scale_m"] == pytest.approx(0.001)
    assert metadata["color_profile"] == "1280x720@30_RGB"
    assert metadata["depth_profile"] == "640x480@30_Y16"
    np.testing.assert_allclose(
        np.array(metadata["T_color_depth_matrix"]).reshape(4, 4)[:3, 3],
        [0.01, 0.0, 0.0],
    )
```

```python
def test_metadata_fingerprint_changes_with_stream_profile():
    first = camera.camera_fingerprint("ABC", "1280x720@30_RGB", "640x480@30_Y16")
    second = camera.camera_fingerprint("ABC", "640x480@30_RGB", "640x480@30_Y16")
    assert first != second
```

- [ ] **Step 6: Run profile tests and verify RED**

Run: `python -m pytest -q tests/test_orbbec_v1_camera.py`

Expected: failures report missing profile and metadata functions.

- [ ] **Step 7: Implement the OrbbecV1Camera adapter**

Implement `OrbbecV1Camera.start()` using the verified v1 API sequence:

```python
sdk = require_orbbec_sdk()
context = sdk.Context()
devices = context.query_devices()
device = select_device(devices, self.serial_number)
pipeline = sdk.Pipeline(device)
config = sdk.Config()
color_profile, color_fallback = select_video_profile(
    pipeline.get_stream_profile_list(sdk.OBSensorType.COLOR_SENSOR),
    self.color_width, self.color_height, self.fps,
    (sdk.OBFormat.RGB, sdk.OBFormat.MJPG, sdk.OBFormat.YUYV),
)
depth_profile, depth_fallback = select_default_depth_profile(
    pipeline.get_stream_profile_list(sdk.OBSensorType.DEPTH_SENSOR)
)
config.enable_stream(color_profile)
config.enable_stream(depth_profile)
config.set_align_mode(sdk.OBAlignMode.DISABLE)
pipeline.enable_frame_sync()
pipeline.start(config)
camera_param = pipeline.get_camera_param()
```

`wait_for_frames()` must return a BGR color image, raw `uint16` depth image, depth scale in metres, and timestamps. Copy the official v1 format handling for RGB, BGR, MJPG, YUYV, I420, NV12, NV21, and UYVY. `stop()` must be idempotent.

- [ ] **Step 8: Run adapter tests and commit**

Run: `python -m pytest -q tests/test_orbbec_v1_camera.py`

Expected: all tests pass without `pyorbbecsdk` installed.

```bash
git add pyAgxArm/demos/nero/orbbec_v1_camera.py tests/test_orbbec_v1_camera.py
git commit -m "feat(nero): add Orbbec SDK v1 camera adapter"
```

### Task 3: Sample Persistence And Hand-Eye Solver

**Files:**
- Modify: `pyAgxArm/demos/nero/orbbec_handeye_math.py`
- Modify: `tests/test_orbbec_handeye_math.py`

- [ ] **Step 1: Write failing sample metadata and motion-validation tests**

```python
def test_load_samples_rejects_camera_fingerprint_mismatch(tmp_path):
    path = tmp_path / "samples.npz"
    samples = [{
        "flange_pose": [0.0] * 6,
        "target_rvec": np.zeros((3, 1)),
        "target_tvec": np.ones((3, 1)),
        "timestamp": 1.0,
    }]
    metadata = {"serial": "camera-A", "color_profile": "A", "depth_profile": "B"}
    math3d.save_samples(path, samples, metadata)
    with pytest.raises(ValueError, match="different camera or stream profile"):
        math3d.load_samples(path, expected_fingerprint="camera-B")


def test_samples_round_trip_full_camera_metadata(tmp_path):
    path = tmp_path / "samples.npz"
    metadata = {
        "serial": "camera-A",
        "color_profile": "1280x720@30_RGB",
        "depth_profile": "640x480@30_Y16",
        "T_color_depth_matrix": np.eye(4).reshape(-1).tolist(),
    }
    math3d.save_samples(path, [], metadata)
    samples, loaded_metadata = math3d.load_samples(path)
    assert samples == []
    assert loaded_metadata == metadata


def test_validate_sample_motion_rejects_repeated_orientation():
    samples = [{"flange_pose": [index * 0.01, 0, 0, 0, 0, 0]} for index in range(3)]
    with pytest.raises(ValueError, match="rotation diversity"):
        math3d.validate_sample_motion(samples)
```

- [ ] **Step 2: Run persistence tests and verify RED**

Run: `python -m pytest -q tests/test_orbbec_handeye_math.py`

Expected: failures report missing persistence and validation functions.

- [ ] **Step 3: Implement persistence and motion validation**

Store `flange_poses`, `target_rvecs`, `target_tvecs`, `timestamps`, scalar `camera_fingerprint`, and scalar `camera_metadata_json` in compressed NPZ. `load_samples()` returns `(samples, camera_metadata)` and optionally compares `expected_fingerprint` before returning. This lets `--calibrate-only` solve without opening the camera. Require at least three samples and a maximum pairwise flange rotation of at least five degrees:

```python
def validate_sample_motion(samples):
    if len(samples) < 3:
        raise ValueError("At least 3 samples are required; 15-30 are recommended.")
    rotations = [pose6_to_matrix(sample["flange_pose"])[:3, :3] for sample in samples]
    max_angle = max(
        rotation_angle(rotations[i].T @ rotations[j])
        for i in range(len(rotations)) for j in range(i + 1, len(rotations))
    )
    if max_angle < np.deg2rad(5.0):
        raise ValueError("Samples do not contain enough flange rotation diversity.")
```

- [ ] **Step 4: Run persistence tests and verify GREEN**

Run: `python -m pytest -q tests/test_orbbec_handeye_math.py`

Expected: persistence and validation tests pass.

- [ ] **Step 5: Write failing solver, result-schema, and consistency tests**

Build a deterministic synthetic eye-in-hand dataset from known `T_flange_color` and fixed `T_base_target`:

```python
@pytest.fixture
def synthetic_samples():
    cv2 = pytest.importorskip("cv2")
    T_flange_color = math3d.pose6_to_matrix([0.05, 0.01, 0.08, 0.1, -0.05, 0.2])
    T_base_target = math3d.pose6_to_matrix([0.6, 0.0, 0.2, 0.0, 0.0, 0.0])
    flange_poses = [
        [0.30, -0.20, 0.40, 0.00, 0.00, 0.00],
        [0.35, -0.10, 0.45, 0.20, -0.10, 0.10],
        [0.32, 0.00, 0.42, -0.20, 0.15, -0.10],
        [0.38, 0.10, 0.46, 0.15, 0.20, 0.25],
        [0.28, 0.18, 0.38, -0.25, -0.15, 0.20],
        [0.40, -0.05, 0.35, 0.30, 0.10, -0.20],
    ]
    samples = []
    for index, flange_pose in enumerate(flange_poses):
        T_base_flange = math3d.pose6_to_matrix(flange_pose)
        T_color_target = math3d.invert_transform(
            T_base_flange @ T_flange_color
        ) @ T_base_target
        samples.append({
            "flange_pose": flange_pose,
            "target_rvec": cv2.Rodrigues(T_color_target[:3, :3])[0],
            "target_tvec": T_color_target[:3, 3].reshape(3, 1),
            "timestamp": float(index),
        })
    return samples, T_flange_color
```

Assert that `calibrate_eye_in_hand()` recovers the known transform within `1e-5`, produces `T_flange_depth = T_flange_color @ T_color_depth`, and emits finite pairwise consistency metrics.

```python
def test_result_contains_named_depth_transform(synthetic_samples):
    samples, expected_T_flange_color = synthetic_samples
    T_color_depth = np.eye(4)
    T_color_depth[:3, 3] = [0.02, 0.0, 0.0]
    result = math3d.calibrate_eye_in_hand(
        samples,
        T_color_depth,
        camera_metadata={"serial": "ABC"},
        checkerboard=(10, 7),
        square_size_m=0.02,
    )
    assert result["schema_version"] == 1
    assert result["checkerboard"] == {
        "inner_corners": [10, 7],
        "square_size_m": 0.02,
    }
    np.testing.assert_allclose(
        np.array(result["T_flange_color"]["matrix_row_major_4x4"]).reshape(4, 4),
        expected_T_flange_color,
        atol=1e-5,
    )
    assert result["T_flange_depth"]["parent_frame"] == "flange"
    assert result["T_flange_depth"]["child_frame"] == "depth"
    expected_T_flange_depth = expected_T_flange_color @ T_color_depth
    np.testing.assert_allclose(
        np.array(result["T_flange_depth"]["matrix_row_major_4x4"]).reshape(4, 4),
        expected_T_flange_depth,
        atol=1e-5,
    )
    assert np.isfinite(result["consistency"]["translation_rms_m"])
```

- [ ] **Step 6: Run solver tests and verify RED**

Run: `python -m pytest -q tests/test_orbbec_handeye_math.py`

Expected: solver/schema tests fail because the result builder is missing.

- [ ] **Step 7: Implement solver, named transforms, and metrics**

Call `cv2.calibrateHandEye()` with `R_gripper2base`, `t_gripper2base`, `R_target2cam`, and `t_target2cam`. Build authoritative row-major matrices with explicit frames:

```python
T_flange_color = make_transform(R_cam2gripper, t_cam2gripper)
T_flange_depth = T_flange_color @ np.asarray(T_color_depth, dtype=np.float64)
result = {
    "schema_version": 1,
    "created_at": datetime.now(timezone.utc).isoformat(),
    "camera": camera_metadata,
    "checkerboard": {
        "inner_corners": [int(checkerboard[0]), int(checkerboard[1])],
        "square_size_m": float(square_size_m),
    },
    "sample_count": len(samples),
    "method": method_name.upper(),
    "T_color_depth": named_transform("color", "depth", T_color_depth),
    "T_depth_color": named_transform("depth", "color", invert_transform(T_color_depth)),
    "T_flange_color": named_transform("flange", "color", T_flange_color),
    "T_color_flange": named_transform("color", "flange", invert_transform(T_flange_color)),
    "T_flange_depth": named_transform("flange", "depth", T_flange_depth),
    "T_depth_flange": named_transform("depth", "flange", invert_transform(T_flange_depth)),
    "consistency": target_consistency_metrics(samples, T_flange_color),
}
```

Calculate pairwise translation distances and relative rotation angles between each reconstructed `T_base_target`; report mean, RMS, and maximum values in metres and degrees. Reject non-finite output before JSON serialization.

- [ ] **Step 8: Run math tests and commit**

Run: `python -m pytest -q tests/test_orbbec_handeye_math.py`

Expected: all math tests pass.

```bash
git add pyAgxArm/demos/nero/orbbec_handeye_math.py tests/test_orbbec_handeye_math.py
git commit -m "feat(nero): solve and serialize Orbbec hand-eye calibration"
```

### Task 4: Interactive Orbbec Calibration CLI

**Files:**
- Create: `pyAgxArm/demos/nero/orbbec_handeye_calib.py`
- Create: `tests/test_orbbec_handeye_cli.py`
- Delete: `pyAgxArm/demos/nero/d435i_handeye_calib.py`
- Delete: `tests/test_d435i_handeye_calib.py`

- [ ] **Step 1: Write failing parser and sample-capture tests**

```python
def test_parser_defaults_match_dabai_setup():
    args = cli.build_arg_parser().parse_args([])
    assert (args.checkerboard_cols, args.checkerboard_rows) == (10, 7)
    assert args.square_size == pytest.approx(0.02)
    assert args.can_interface == "socketcan"
    assert args.can_channel == "can_piper"
    assert (args.width, args.height, args.fps) == (1280, 720, 30)


def test_capture_sample_requires_current_detection():
    with pytest.raises(ValueError, match="checkerboard"):
        cli.capture_sample(None, [0.0] * 6, time.time())
```

- [ ] **Step 2: Run CLI tests and verify RED**

Run: `python -m pytest -q tests/test_orbbec_handeye_cli.py`

Expected: collection fails because `orbbec_handeye_calib` does not exist.

- [ ] **Step 3: Implement parser, robot factory, and pure capture helper**

The parser must include `--checkerboard-cols`, `--checkerboard-rows`, `--square-size`, `--samples`, `--output`, `--method`, `--calibrate-only`, `--can-interface`, `--can-channel`, `--firmware`, `--camera-serial`, `--width`, `--height`, and `--fps`. Import `pyAgxArm` only inside `create_nero_robot()` so `--help` and unit tests work without CAN dependencies.

```python
def capture_sample(detection, flange_pose, timestamp):
    if detection is None:
        raise ValueError("checkerboard is not detected in the current frame")
    rvec, tvec = detection
    return {
        "flange_pose": [float(value) for value in flange_pose],
        "target_rvec": np.asarray(rvec, dtype=np.float64).reshape(3, 1),
        "target_tvec": np.asarray(tvec, dtype=np.float64).reshape(3, 1),
        "timestamp": float(timestamp),
    }
```

- [ ] **Step 4: Run CLI unit tests and verify GREEN**

Run: `python -m pytest -q tests/test_orbbec_handeye_cli.py`

Expected: parser and capture tests pass.

- [ ] **Step 5: Write failing checkerboard detection test**

Generate a synthetic 10 x 7 inner-corner chessboard image and assert pose detection succeeds:

```python
def test_detect_checkerboard_pose_on_synthetic_board():
    cv2 = pytest.importorskip("cv2")
    cols, rows, square_px = 10, 7, 60
    board = np.full(((rows + 1) * square_px, (cols + 1) * square_px), 255, np.uint8)
    for row in range(rows + 1):
        for col in range(cols + 1):
            if (row + col) % 2 == 0:
                board[
                    row * square_px:(row + 1) * square_px,
                    col * square_px:(col + 1) * square_px,
                ] = 0
    board = cv2.copyMakeBorder(board, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    image = cv2.cvtColor(board, cv2.COLOR_GRAY2BGR)
    height, width = image.shape[:2]
    camera_matrix = np.array([
        [800.0, 0.0, width / 2.0],
        [0.0, 800.0, height / 2.0],
        [0.0, 0.0, 1.0],
    ])
    ok, rvec, tvec, corners = cli.detect_checkerboard_pose(
        image, camera_matrix, np.zeros((8, 1)), (cols, rows), 0.02
    )
    assert ok is True
    assert corners.shape == (cols * rows, 1, 2)
    assert np.isfinite(rvec).all()
    assert tvec[2, 0] > 0.0
```

- [ ] **Step 6: Run detection test and verify RED**

Run: `python -m pytest -q tests/test_orbbec_handeye_cli.py`

Expected: failure reports missing `detect_checkerboard_pose`.

- [ ] **Step 7: Implement collection loop and calibration-only mode**

Use the Orbbec adapter's BGR image with `findChessboardCorners`, `cornerSubPix`, and `solvePnP`. In the loop:

```text
frame -> detect current checkerboard -> draw corners/axes -> waitKey
s -> read current flange pose -> append sample -> save NPZ
c -> validate motion -> solve -> write JSON
q -> exit
```

Set `last_detection = None` whenever the current frame does not contain the board, so stale detections cannot be saved. Wrap camera, robot, and OpenCV resources in `try/finally`; stop the camera, disconnect the robot when supported, and call `cv2.destroyAllWindows()`.

In `--calibrate-only`, load the samples and camera metadata stored at collection time, solve without opening hardware, and write the requested JSON output.

- [ ] **Step 8: Remove the obsolete D435i files**

Delete only the two temporary untracked D435i files listed in this task after the Orbbec tests cover their checkerboard and transform behavior.

- [ ] **Step 9: Run CLI and all Orbbec tests, then commit**

Run: `python -m pytest -q tests/test_orbbec_handeye_math.py tests/test_orbbec_v1_camera.py tests/test_orbbec_handeye_cli.py`

Expected: all tests pass.

Run: `python pyAgxArm/demos/nero/orbbec_handeye_calib.py --help`

Expected: exit 0 and help lists Orbbec, checkerboard, CAN, camera, sample and output options without importing the SDK.

```bash
git add pyAgxArm/demos/nero/orbbec_handeye_calib.py tests/test_orbbec_handeye_cli.py
git add -u pyAgxArm/demos/nero/d435i_handeye_calib.py tests/test_d435i_handeye_calib.py
git commit -m "feat(nero): add Orbbec hand-eye calibration CLI"
```

### Task 5: DaBai DCW Setup And Operating Guide

**Files:**
- Create: `docs/nero/orbbec_dabai_handeye.md`

- [ ] **Step 1: Write the SDK v1 setup guide**

Document the exact target and source branch first:

```text
Camera: Orbbec DaBai DCW
SDK: Orbbec SDK v1
Python wrapper: https://github.com/orbbec/pyorbbecsdk, branch main
Expected Python module: pyorbbecsdk
```

Include Ubuntu 22.04 setup commands that preserve the active Conda interpreter:

```bash
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

State that the camera must be replugged after udev setup and verify it with:

```bash
python examples/hello_orbbec.py
```

- [ ] **Step 2: Document calibration operation and pose diversity**

Use the actual interface discovered on this workstation:

```bash
python pyAgxArm/demos/nero/orbbec_handeye_calib.py \
  --checkerboard-cols 10 \
  --checkerboard-rows 7 \
  --square-size 0.02 \
  --can-interface socketcan \
  --can-channel can_piper
```

Explain `s`, `c`, `q`, 15-30 varied poses, fixed checkerboard, motion safety, output files, transform naming, metre units, and how to inspect consistency metrics.

- [ ] **Step 3: Document depth-pixel conversion with a concrete example**

Show how to load `T_flange_depth`, read current `T_base_flange`, obtain `depth_raw = depth[v, u]`, and call:

```python
point_base = depth_pixel_to_base(
    u=u,
    v=v,
    depth_raw=depth_raw,
    depth_intrinsics=result["camera"]["depth_intrinsics"],
    depth_scale_m=result["camera"]["depth_scale_m"],
    T_flange_depth=np.array(
        result["T_flange_depth"]["matrix_row_major_4x4"], dtype=float
    ).reshape(4, 4),
    T_base_flange=pose6_to_matrix(robot.get_flange_pose().msg),
)
```

Warn that `u` is the column, `v` is the row, zero depth is invalid, and raw unaligned depth pixels must use depth intrinsics.

- [ ] **Step 4: Check links and commands, then commit**

Run: `rg -n "TBD|TODO|D435|RealSense|can0" docs/nero/orbbec_dabai_handeye.md`

Expected: no output.

Run: `git diff --check -- docs/nero/orbbec_dabai_handeye.md`

Expected: exit 0.

```bash
git add docs/nero/orbbec_dabai_handeye.md
git commit -m "docs(nero): add DaBai DCW hand-eye guide"
```

### Task 6: Final Automated And Hardware-Gated Verification

**Files:**
- Modify only if verification reveals a defect in the files created above.

- [ ] **Step 1: Run focused tests**

Run:

```bash
python -m pytest -q \
  tests/test_orbbec_handeye_math.py \
  tests/test_orbbec_v1_camera.py \
  tests/test_orbbec_handeye_cli.py
```

Expected: all focused tests pass with zero failures.

- [ ] **Step 2: Run the full repository test suite**

Run: `python -m pytest -q`

Expected: all repository tests pass. If virtual-CAN support is unavailable, record the exact skipped or failed hardware dependency instead of claiming a full pass.

- [ ] **Step 3: Compile and inspect CLI entry points**

Run:

```bash
python -m py_compile \
  pyAgxArm/demos/nero/orbbec_handeye_math.py \
  pyAgxArm/demos/nero/orbbec_v1_camera.py \
  pyAgxArm/demos/nero/orbbec_handeye_calib.py
```

Expected: exit 0.

Run: `python pyAgxArm/demos/nero/orbbec_handeye_calib.py --help`

Expected: exit 0 without requiring connected hardware.

- [ ] **Step 4: Probe the real Orbbec SDK and camera**

After the user installs SDK v1 and udev rules, run:

```bash
python -c "from pyorbbecsdk import Context; d=Context().query_devices(); print(d.get_count())"
```

Expected on this workstation: `1` or greater.

Start the calibration demo and verify synchronized color/depth acquisition, the selected profiles, serial number, firmware, depth scale, and checkerboard overlay. Do not claim this step passed when the SDK or physical camera is unavailable.

- [ ] **Step 5: Verify the real robot and calibration output**

With Nero powered and `can_piper` active, collect 15-30 samples and press `c`. Verify that the JSON contains finite `T_flange_color`, `T_color_depth`, and `T_flange_depth` matrices and finite consistency metrics.

Measure one checkerboard point in the Nero base frame, transform the corresponding valid depth pixel, and record the Euclidean error. This physical check is the final acceptance evidence for depth-to-base conversion.

- [ ] **Step 6: Review the final diff and commit any verification fixes**

Run: `git status --short`

Expected: only pre-existing unrelated CAN-script edits remain; no generated sample NPZ or calibration JSON is staged.

Run: `git diff --check`

Expected: exit 0.

If verification required a code correction, commit only the affected Orbbec files with:

```bash
git add pyAgxArm/demos/nero/orbbec_handeye_math.py \
  pyAgxArm/demos/nero/orbbec_v1_camera.py \
  pyAgxArm/demos/nero/orbbec_handeye_calib.py \
  tests/test_orbbec_handeye_math.py \
  tests/test_orbbec_v1_camera.py \
  tests/test_orbbec_handeye_cli.py \
  docs/nero/orbbec_dabai_handeye.md
git commit -m "fix(nero): verify Orbbec hand-eye workflow"
```
