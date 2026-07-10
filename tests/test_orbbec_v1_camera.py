"""Hardware-independent tests for the Orbbec SDK v1 camera adapter."""

from pathlib import Path
import sys
from types import SimpleNamespace
import builtins

import numpy as np
import pytest
import cv2


NERO_DEMO_DIR = Path(__file__).resolve().parents[1] / "pyAgxArm" / "demos" / "nero"
sys.path.insert(0, str(NERO_DEMO_DIR))

import orbbec_v1_camera as camera  # noqa: E402


def test_intrinsic_and_distortion_are_normalized_for_opencv():
    intrinsic = SimpleNamespace(
        width=640, height=480, fx=500.0, fy=501.0, cx=319.5, cy=239.5
    )
    distortion = SimpleNamespace(
        k1=1.0, k2=2.0, k3=3.0, k4=4.0, k5=5.0, k6=6.0, p1=0.1, p2=0.2
    )

    assert camera.normalize_intrinsic(intrinsic) == {
        "width": 640,
        "height": 480,
        "fx": 500.0,
        "fy": 501.0,
        "cx": 319.5,
        "cy": 239.5,
    }
    assert camera.normalize_distortion(distortion) == [
        1.0,
        2.0,
        0.1,
        0.2,
        3.0,
        4.0,
        5.0,
        6.0,
    ]


def test_d2c_transform_is_depth_to_color_and_converts_mm_to_metres():
    sdk_transform = SimpleNamespace(
        rot=np.eye(3, dtype=np.float32).reshape(-1),
        transform=np.array([10.0, -20.0, 30.0], dtype=np.float32),
    )

    T_color_depth = camera.normalize_d2c_transform(sdk_transform)

    np.testing.assert_allclose(T_color_depth[:3, :3], np.eye(3))
    np.testing.assert_allclose(T_color_depth[:3, 3], [0.01, -0.02, 0.03])


def test_d2c_transform_accepts_a_valid_float32_rotation():
    angle = np.pi / 5.0
    rotation = np.array(
        [
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    sdk_transform = SimpleNamespace(rot=rotation.reshape(-1), transform=[0, 0, 0])

    T_color_depth = camera.normalize_d2c_transform(sdk_transform)

    np.testing.assert_allclose(T_color_depth[:3, :3], rotation, atol=1e-7)


def test_depth_scale_is_converted_from_sdk_mm_to_metres():
    assert camera.depth_scale_mm_to_m(1.0) == pytest.approx(0.001)


class FakeProfiles:
    def __init__(self):
        self.requests = []
        self.default = object()

    def get_video_stream_profile(self, width, height, image_format, fps):
        self.requests.append((width, height, image_format, fps))
        raise RuntimeError("unsupported")

    def get_default_video_stream_profile(self):
        return self.default


def test_profile_selection_tries_formats_in_order_then_reports_fallback():
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


def test_profile_selection_returns_first_requested_profile_without_fallback():
    selected = object()

    class Profiles:
        def __init__(self):
            self.requests = []

        def get_video_stream_profile(self, width, height, image_format, fps):
            self.requests.append((width, height, image_format, fps))
            return selected

    profiles = Profiles()

    profile, used_fallback = camera.select_video_profile(
        profiles, 1280, 720, 30, ("RGB", "MJPG")
    )

    assert profile is selected
    assert used_fallback is False
    assert profiles.requests == [(1280, 720, "RGB", 30)]


def test_default_depth_profile_and_profile_string_are_exact():
    profile = SimpleNamespace(
        get_width=lambda: 640,
        get_height=lambda: 480,
        get_fps=lambda: 30,
        get_format=lambda: SimpleNamespace(name="Y16"),
    )

    class Profiles:
        def get_default_video_stream_profile(self):
            return profile

    selected, used_fallback = camera.select_default_depth_profile(Profiles())

    assert selected is profile
    assert used_fallback is True
    assert camera.profile_to_string(profile) == "640x480@30_Y16"


def test_build_camera_metadata_preserves_calibration_and_profiles():
    intrinsic = SimpleNamespace(
        width=640, height=480, fx=500.0, fy=501.0, cx=319.5, cy=239.5
    )
    distortion = SimpleNamespace(
        k1=0.1, k2=0.2, k3=0.3, k4=0.4, k5=0.5, k6=0.6, p1=0.01, p2=0.02
    )
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
        depth_scale_mm=10.0,
    )

    assert metadata["name"] == "DaBai DCW"
    assert metadata["serial"] == "ABC"
    assert metadata["firmware"] == "2460"
    assert metadata["color_profile"] == "1280x720@30_RGB"
    assert metadata["depth_profile"] == "640x480@30_Y16"
    assert metadata["color_intrinsics"] == camera.normalize_intrinsic(intrinsic)
    assert metadata["depth_intrinsics"] == camera.normalize_intrinsic(intrinsic)
    assert metadata["color_distortion"] == camera.normalize_distortion(distortion)
    assert metadata["depth_distortion"] == camera.normalize_distortion(distortion)
    assert metadata["depth_scale_m"] == pytest.approx(0.01)
    np.testing.assert_allclose(
        np.array(metadata["T_color_depth_matrix"]).reshape(4, 4)[:3, 3],
        [0.01, 0.0, 0.0],
    )
    assert metadata["camera_fingerprint"] == camera.camera_fingerprint(
        "ABC", "1280x720@30_RGB", "640x480@30_Y16"
    )


def test_camera_fingerprint_is_stable_and_changes_with_each_input():
    first = camera.camera_fingerprint("ABC", "1280x720@30_RGB", "640x480@30_Y16")

    assert first == camera.camera_fingerprint(
        "ABC", "1280x720@30_RGB", "640x480@30_Y16"
    )
    assert first != camera.camera_fingerprint("DEF", "1280x720@30_RGB", "640x480@30_Y16")
    assert first != camera.camera_fingerprint("ABC", "640x480@30_RGB", "640x480@30_Y16")
    assert first != camera.camera_fingerprint("ABC", "1280x720@30_RGB", "1280x720@30_Y16")


def test_require_orbbec_sdk_explains_how_to_install_the_optional_dependency(
    monkeypatch,
):
    original_import = builtins.__import__

    def missing_orbbec_sdk(name, *args, **kwargs):
        if name == "pyorbbecsdk":
            raise ImportError("forced missing SDK")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", missing_orbbec_sdk)

    with pytest.raises(RuntimeError) as error:
        camera.require_orbbec_sdk()

    message = str(error.value)
    assert "pyorbbecsdk" in message
    assert "docs/nero/orbbec_dabai_handeye.md" in message
    assert "https://github.com/orbbec/pyorbbecsdk/tree/main" in message


@pytest.mark.parametrize(
    "rot,transform,error",
    [
        (np.eye(3).reshape(-1), [float("nan"), 0.0, 0.0], "finite"),
        (np.diag([2.0, 1.0, 1.0]).reshape(-1), [0.0, 0.0, 0.0], "orthonormal"),
        (np.diag([-1.0, 1.0, 1.0]).reshape(-1), [0.0, 0.0, 0.0], "determinant"),
    ],
)
def test_d2c_transform_rejects_invalid_rigid_calibration(rot, transform, error):
    extrinsic = SimpleNamespace(rot=rot, transform=transform)

    with pytest.raises(ValueError, match=error):
        camera.normalize_d2c_transform(extrinsic)


@pytest.mark.parametrize("scale", [0.0, -1.0, float("nan"), float("inf")])
def test_depth_scale_rejects_nonpositive_or_nonfinite_values(scale):
    with pytest.raises(ValueError, match="finite and positive"):
        camera.depth_scale_mm_to_m(scale)


class FakeFormats:
    RGB = "RGB"
    BGR = "BGR"
    MJPG = "MJPG"
    YUYV = "YUYV"
    I420 = "I420"
    NV12 = "NV12"
    NV21 = "NV21"
    UYVY = "UYVY"


FAKE_SDK = SimpleNamespace(OBFormat=FakeFormats)


class FakeFrame:
    def __init__(self, width, height, image_format, data, timestamp=123.0):
        self.width = width
        self.height = height
        self.image_format = image_format
        self.data = data
        self.timestamp = timestamp

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_format(self):
        return self.image_format

    def get_data(self):
        return self.data

    def get_timestamp(self):
        return self.timestamp


def test_frame_to_bgr_image_preserves_bgr_and_converts_rgb():
    bgr = np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
    rgb = bgr[..., ::-1].copy()

    bgr_result = camera.frame_to_bgr_image(
        FakeFrame(2, 1, FakeFormats.BGR, bgr.tobytes()), FAKE_SDK, cv2
    )
    rgb_result = camera.frame_to_bgr_image(
        FakeFrame(2, 1, FakeFormats.RGB, rgb.tobytes()), FAKE_SDK, cv2
    )

    np.testing.assert_array_equal(bgr_result, bgr)
    np.testing.assert_array_equal(rgb_result, bgr)


@pytest.mark.parametrize(
    "image_format,conversion_code",
    [
        (FakeFormats.YUYV, cv2.COLOR_YUV2BGR_YUY2),
        (FakeFormats.UYVY, cv2.COLOR_YUV2BGR_UYVY),
    ],
)
def test_frame_to_bgr_image_converts_packed_yuv(image_format, conversion_code):
    packed = np.array([[[16, 128], [235, 128]]], dtype=np.uint8)
    frame = FakeFrame(2, 1, image_format, packed.tobytes())

    image = camera.frame_to_bgr_image(frame, FAKE_SDK, cv2)

    np.testing.assert_array_equal(image, cv2.cvtColor(packed, conversion_code))


@pytest.mark.parametrize(
    "image_format,conversion_code",
    [
        (FakeFormats.I420, cv2.COLOR_YUV2BGR_I420),
        (FakeFormats.NV12, cv2.COLOR_YUV2BGR_NV12),
        (FakeFormats.NV21, cv2.COLOR_YUV2BGR_NV21),
    ],
)
def test_frame_to_bgr_image_converts_planar_yuv(image_format, conversion_code):
    yuv = np.array([[16, 235], [81, 145], [128, 128]], dtype=np.uint8)
    frame = FakeFrame(2, 2, image_format, yuv.tobytes())

    image = camera.frame_to_bgr_image(frame, FAKE_SDK, cv2)

    np.testing.assert_array_equal(image, cv2.cvtColor(yuv, conversion_code))


def test_frame_to_bgr_image_decodes_mjpeg_and_rejects_unknown_formats():
    source = np.full((2, 2, 3), [10, 20, 30], dtype=np.uint8)
    ok, encoded = cv2.imencode(".jpg", source)
    assert ok

    decoded = camera.frame_to_bgr_image(
        FakeFrame(2, 2, FakeFormats.MJPG, encoded.tobytes()), FAKE_SDK, cv2
    )

    assert decoded.shape == source.shape
    with pytest.raises(RuntimeError, match="Unsupported Orbbec color format"):
        camera.frame_to_bgr_image(
            FakeFrame(2, 2, "H264", b""), FAKE_SDK, cv2
        )


def test_depth_frame_to_array_keeps_raw_uint16_values():
    raw_depth = np.array([[0, 1000], [2000, 65535]], dtype=np.uint16)

    depth = camera.depth_frame_to_array(
        FakeFrame(2, 2, "Y16", raw_depth.tobytes())
    )

    assert depth.dtype == np.uint16
    np.testing.assert_array_equal(depth, raw_depth)


def test_depth_frame_to_array_preserves_uint16_ndarray_values():
    raw_depth = np.array([[1, 255, 256, 1000, 65535]], dtype=np.uint16)

    depth = camera.depth_frame_to_array(FakeFrame(5, 1, "Y16", raw_depth))

    assert depth.shape == (1, 5)
    assert depth.dtype == np.uint16
    np.testing.assert_array_equal(depth, raw_depth)


class FakeProfile:
    def __init__(self, width, height, fps, image_format):
        self.width = width
        self.height = height
        self.fps = fps
        self.image_format = image_format

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_fps(self):
        return self.fps

    def get_format(self):
        return SimpleNamespace(name=self.image_format)


class FakeLifecycleProfiles:
    def __init__(self, selected=None, default=None):
        self.selected = selected
        self.default = default
        self.requests = []

    def get_video_stream_profile(self, width, height, image_format, fps):
        self.requests.append((width, height, image_format, fps))
        if self.selected is not None and image_format == FakeFormats.RGB:
            return self.selected
        raise RuntimeError("unsupported")

    def get_default_video_stream_profile(self):
        return self.default


class FakeConfig:
    def __init__(self):
        self.streams = []
        self.align_mode = None

    def enable_stream(self, profile):
        self.streams.append(profile)

    def set_align_mode(self, mode):
        self.align_mode = mode


class FakePipeline:
    def __init__(self, color_profiles, depth_profiles, camera_param, frameset=None):
        self.color_profiles = color_profiles
        self.depth_profiles = depth_profiles
        self.camera_param = camera_param
        self.frameset = frameset
        self.frame_sync_enabled = False
        self.started_config = None
        self.stop_count = 0
        self.start_error = None

    def get_stream_profile_list(self, sensor_type):
        if sensor_type == "COLOR_SENSOR":
            return self.color_profiles
        assert sensor_type == "DEPTH_SENSOR"
        return self.depth_profiles

    def enable_frame_sync(self):
        self.frame_sync_enabled = True

    def start(self, config):
        self.started_config = config
        if self.start_error is not None:
            raise self.start_error

    def get_camera_param(self):
        return self.camera_param

    def wait_for_frames(self, timeout_ms):
        assert timeout_ms == 100
        return self.frameset

    def stop(self):
        self.stop_count += 1


class FakeFrameSet:
    def __init__(self, color_frame, depth_frame):
        self.color_frame = color_frame
        self.depth_frame = depth_frame

    def get_color_frame(self):
        return self.color_frame

    def get_depth_frame(self):
        return self.depth_frame


class FakeDepthFrame(FakeFrame):
    def __init__(self, *args, depth_scale, **kwargs):
        super().__init__(*args, **kwargs)
        self.depth_scale = depth_scale

    def get_depth_scale(self):
        return self.depth_scale


def make_camera_param():
    intrinsic = SimpleNamespace(
        width=2, height=2, fx=500.0, fy=500.0, cx=1.0, cy=1.0
    )
    distortion = SimpleNamespace(
        k1=0.0, k2=0.0, k3=0.0, k4=0.0, k5=0.0, k6=0.0, p1=0.0, p2=0.0
    )
    return SimpleNamespace(
        rgb_intrinsic=intrinsic,
        depth_intrinsic=intrinsic,
        rgb_distortion=distortion,
        depth_distortion=distortion,
        transform=SimpleNamespace(rot=np.eye(3).reshape(-1), transform=[0, 0, 0]),
    )


def make_fake_sdk(devices, pipeline, config):
    device_info = SimpleNamespace(
        get_name=lambda: "DaBai DCW",
        get_serial_number=lambda: "ABC",
        get_firmware_version=lambda: "2460",
    )
    for device in devices:
        device.get_device_info = lambda info=device_info: info

    class DeviceList:
        def get_count(self):
            return len(devices)

        def get_device_by_index(self, index):
            return devices[index]

        def get_device_by_serial_number(self, serial):
            for device in devices:
                if device.serial == serial:
                    return device
            return None

    return SimpleNamespace(
        Context=lambda: SimpleNamespace(query_devices=lambda: DeviceList()),
        Pipeline=lambda device: pipeline,
        Config=lambda: config,
        OBSensorType=SimpleNamespace(COLOR_SENSOR="COLOR_SENSOR", DEPTH_SENSOR="DEPTH_SENSOR"),
        OBFormat=FakeFormats,
        OBAlignMode=SimpleNamespace(DISABLE="DISABLE"),
    )


def make_lifecycle_run(color_profile, depth_profile, depth_scale):
    color = FakeFrame(
        2,
        1,
        FakeFormats.RGB,
        np.array([[[3, 2, 1], [6, 5, 4]]], dtype=np.uint8).tobytes(),
        timestamp=101,
    )
    depth = FakeDepthFrame(
        2,
        2,
        "Y16",
        np.array([[1, 2], [3, 4]], dtype=np.uint16).tobytes(),
        timestamp=102,
        depth_scale=depth_scale,
    )
    pipeline = FakePipeline(
        FakeLifecycleProfiles(selected=color_profile),
        FakeLifecycleProfiles(default=depth_profile),
        make_camera_param(),
        FakeFrameSet(color, depth),
    )
    config = FakeConfig()
    sdk = make_fake_sdk([SimpleNamespace(serial="ABC")], pipeline, config)
    return sdk, pipeline


def test_camera_start_wait_metadata_and_idempotent_stop(monkeypatch):
    color_profile = FakeProfile(1280, 720, 30, "RGB")
    depth_profile = FakeProfile(640, 480, 30, "Y16")
    color = FakeFrame(
        2,
        1,
        FakeFormats.RGB,
        np.array([[[3, 2, 1], [6, 5, 4]]], dtype=np.uint8).tobytes(),
        timestamp=101,
    )
    depth = FakeDepthFrame(
        2,
        2,
        "Y16",
        np.array([[1, 2], [3, 4]], dtype=np.uint16).tobytes(),
        timestamp=102,
        depth_scale=10.0,
    )
    pipeline = FakePipeline(
        FakeLifecycleProfiles(selected=color_profile),
        FakeLifecycleProfiles(default=depth_profile),
        make_camera_param(),
        FakeFrameSet(color, depth),
    )
    config = FakeConfig()
    device = SimpleNamespace(serial="ABC")
    sdk = make_fake_sdk([device], pipeline, config)
    monkeypatch.setattr(camera, "require_orbbec_sdk", lambda: sdk)
    adapter = camera.OrbbecV1Camera(serial_number="ABC")

    adapter.start()
    result = adapter.wait_for_frames()

    assert pipeline.frame_sync_enabled is True
    assert pipeline.started_config is config
    assert config.streams == [color_profile, depth_profile]
    assert config.align_mode == "DISABLE"
    assert adapter.color_profile == "1280x720@30_RGB"
    assert adapter.depth_profile == "640x480@30_Y16"
    assert adapter.color_profile_used_fallback is False
    assert adapter.depth_profile_used_fallback is True
    np.testing.assert_array_equal(
        result.color_bgr, np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
    )
    np.testing.assert_array_equal(result.depth_raw, np.array([[1, 2], [3, 4]], dtype=np.uint16))
    assert result.depth_scale_m == pytest.approx(0.01)
    assert result.color_timestamp_ms == 101
    assert result.depth_timestamp_ms == 102
    assert adapter.metadata["depth_scale_m"] == pytest.approx(0.01)
    metadata_copy = adapter.metadata
    metadata_copy["serial"] = "changed"
    assert adapter.metadata["serial"] == "ABC"

    adapter.stop()
    adapter.stop()

    assert pipeline.stop_count == 1


def test_camera_restart_replaces_metadata_and_profiles_with_second_run(monkeypatch):
    first_sdk, first_pipeline = make_lifecycle_run(
        FakeProfile(1280, 720, 30, "RGB"),
        FakeProfile(640, 480, 30, "Y16"),
        depth_scale=10.0,
    )
    second_sdk, second_pipeline = make_lifecycle_run(
        FakeProfile(640, 480, 30, "RGB"),
        FakeProfile(320, 240, 15, "Y16"),
        depth_scale=2.0,
    )
    sdks = iter((first_sdk, second_sdk))
    monkeypatch.setattr(camera, "require_orbbec_sdk", lambda: next(sdks))
    adapter = camera.OrbbecV1Camera()

    adapter.start()
    first = adapter.wait_for_frames()
    adapter.stop()
    adapter.start()
    second = adapter.wait_for_frames()

    assert first.depth_scale_m == pytest.approx(0.01)
    assert second.depth_scale_m == pytest.approx(0.002)
    assert adapter.metadata["depth_scale_m"] == pytest.approx(0.002)
    assert adapter.metadata["color_profile"] == "640x480@30_RGB"
    assert adapter.metadata["depth_profile"] == "320x240@15_Y16"
    assert first_pipeline.stop_count == 1
    assert second_pipeline.stop_count == 0


def test_failed_restart_clears_previous_run_metadata_and_profiles(monkeypatch):
    first_sdk, _ = make_lifecycle_run(
        FakeProfile(1280, 720, 30, "RGB"),
        FakeProfile(640, 480, 30, "Y16"),
        depth_scale=10.0,
    )
    failed_sdk, failed_pipeline = make_lifecycle_run(
        FakeProfile(640, 480, 30, "RGB"),
        FakeProfile(320, 240, 15, "Y16"),
        depth_scale=2.0,
    )
    failed_pipeline.start_error = RuntimeError("second SDK start failed")
    sdks = iter((first_sdk, failed_sdk))
    monkeypatch.setattr(camera, "require_orbbec_sdk", lambda: next(sdks))
    adapter = camera.OrbbecV1Camera()

    adapter.start()
    adapter.wait_for_frames()
    adapter.stop()
    with pytest.raises(RuntimeError, match="second SDK start failed"):
        adapter.start()

    assert adapter.metadata is None
    assert adapter.color_profile is None
    assert adapter.depth_profile is None
    assert adapter.color_profile_used_fallback is None
    assert adapter.depth_profile_used_fallback is None


def test_camera_start_requires_exactly_one_device_without_serial(monkeypatch):
    pipeline = FakePipeline(None, None, make_camera_param())
    sdk = make_fake_sdk(
        [SimpleNamespace(serial="ABC"), SimpleNamespace(serial="DEF")], pipeline, FakeConfig()
    )
    monkeypatch.setattr(camera, "require_orbbec_sdk", lambda: sdk)

    with pytest.raises(RuntimeError, match="exactly one"):
        camera.OrbbecV1Camera().start()


def test_camera_start_reports_missing_requested_serial(monkeypatch):
    pipeline = FakePipeline(None, None, make_camera_param())
    sdk = make_fake_sdk([SimpleNamespace(serial="ABC")], pipeline, FakeConfig())
    monkeypatch.setattr(camera, "require_orbbec_sdk", lambda: sdk)

    with pytest.raises(RuntimeError, match="serial 'DEF'"):
        camera.OrbbecV1Camera(serial_number="DEF").start()


def test_camera_start_cleans_up_partially_started_pipeline_with_actionable_error(monkeypatch):
    color_profile = FakeProfile(1280, 720, 30, "RGB")
    depth_profile = FakeProfile(640, 480, 30, "Y16")
    pipeline = FakePipeline(
        FakeLifecycleProfiles(selected=color_profile),
        FakeLifecycleProfiles(default=depth_profile),
        make_camera_param(),
    )
    pipeline.start_error = RuntimeError("SDK start failed")
    sdk = make_fake_sdk([SimpleNamespace(serial="ABC")], pipeline, FakeConfig())
    monkeypatch.setattr(camera, "require_orbbec_sdk", lambda: sdk)

    with pytest.raises(
        RuntimeError,
        match="Failed to start Orbbec camera.*SDK start failed.*Check connection",
    ) as error:
        camera.OrbbecV1Camera().start()

    assert pipeline.stop_count == 1
    assert isinstance(error.value.__cause__, RuntimeError)
    assert str(error.value.__cause__) == "SDK start failed"
