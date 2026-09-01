"""Lazy Orbbec SDK v1 camera adapter for DaBai hand-eye calibration."""

import copy
from dataclasses import dataclass
import hashlib
import json
import math
import time

import numpy as np


_DOCS_PATH = "docs/nero/orbbec_dabai_handeye.md"
_SDK_SOURCE_URL = "https://github.com/orbbec/pyorbbecsdk/tree/main"
_RIGID_TRANSFORM_ATOL = 1e-6


def require_orbbec_sdk():
    """Import the optional Orbbec SDK only when camera access is requested."""
    try:
        import pyorbbecsdk
    except ImportError as exc:
        raise RuntimeError(
            "Orbbec SDK v1 module 'pyorbbecsdk' is required. Follow {} or install "
            "the official v1 source from {}.".format(_DOCS_PATH, _SDK_SOURCE_URL)
        ) from exc
    return pyorbbecsdk


def normalize_intrinsic(intrinsic):
    """Convert v1 camera intrinsics to the local metadata schema."""
    return {
        "width": int(intrinsic.width),
        "height": int(intrinsic.height),
        "fx": float(intrinsic.fx),
        "fy": float(intrinsic.fy),
        "cx": float(intrinsic.cx),
        "cy": float(intrinsic.cy),
    }


def normalize_distortion(distortion):
    """Return v1 distortion coefficients in OpenCV order."""
    return [
        float(getattr(distortion, name))
        for name in ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6")
    ]


def normalize_d2c_transform(extrinsic):
    """Return the v1 depth-to-color transform with metre translation."""
    try:
        rotation = np.asarray(extrinsic.rot, dtype=np.float64).reshape(3, 3)
        translation_mm = np.asarray(extrinsic.transform, dtype=np.float64).reshape(3)
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("Orbbec depth-to-color transform must contain 9 rotation and 3 translation values") from exc

    if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation_mm)):
        raise ValueError("Orbbec depth-to-color transform must contain only finite values")
    if not np.allclose(
        rotation.T @ rotation,
        np.eye(3),
        rtol=0.0,
        atol=_RIGID_TRANSFORM_ATOL,
    ):
        raise ValueError("Orbbec depth-to-color rotation must be orthonormal")
    if not np.isclose(
        np.linalg.det(rotation), 1.0, rtol=0.0, atol=_RIGID_TRANSFORM_ATOL
    ):
        raise ValueError("Orbbec depth-to-color rotation determinant must be +1")

    # SDK v1 exposes float32 calibration; project its rounding error onto SO(3).
    u, _, vt = np.linalg.svd(rotation)
    rotation = u @ vt

    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation_mm / 1000.0
    return result


def depth_scale_mm_to_m(scale):
    """Convert the SDK depth-unit scale from millimetres to metres."""
    value = float(scale)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("Orbbec depth scale must be finite and positive")
    return value / 1000.0


def select_video_profile(profile_list, width, height, fps, formats):
    """Select the first requested color profile, with an explicit fallback."""
    last_error = None
    for image_format in formats:
        try:
            profile = profile_list.get_video_stream_profile(
                width, height, image_format, fps
            )
        except Exception as exc:
            last_error = exc
            continue
        if profile is not None:
            return profile, False

    try:
        profile = profile_list.get_default_video_stream_profile()
    except Exception as exc:
        raise RuntimeError(
            "No requested Orbbec video profile is available and the default profile could not be selected"
        ) from exc
    if profile is None:
        raise RuntimeError(
            "No requested Orbbec video profile is available and the SDK returned no default profile"
        ) from last_error
    return profile, True


def select_default_depth_profile(profile_list):
    """Select the SDK's explicit default depth profile."""
    try:
        profile = profile_list.get_default_video_stream_profile()
    except Exception as exc:
        raise RuntimeError("Unable to select the default Orbbec depth profile") from exc
    if profile is None:
        raise RuntimeError("The SDK returned no default Orbbec depth profile")
    return profile, True


def profile_to_string(profile):
    """Return the exact resolution, rate, and format used by a stream profile."""
    image_format = profile.get_format()
    format_name = getattr(image_format, "name", None)
    if callable(format_name):
        format_name = format_name()
    if format_name is None:
        format_name = str(image_format)
    return "{}x{}@{}_{}".format(
        int(profile.get_width()),
        int(profile.get_height()),
        int(profile.get_fps()),
        format_name,
    )


def camera_fingerprint(serial, color_profile, depth_profile):
    """Return a stable identifier for a device and its selected stream profiles."""
    payload = json.dumps(
        [str(serial), str(color_profile), str(depth_profile)],
        ensure_ascii=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def build_camera_metadata(
    camera_param, device_info, color_profile, depth_profile, depth_scale_mm
):
    """Normalize v1 calibration and device information for persisted samples."""
    T_color_depth = normalize_d2c_transform(camera_param.transform)
    serial = str(device_info.get_serial_number())
    return {
        "name": str(device_info.get_name()),
        "serial": serial,
        "firmware": str(device_info.get_firmware_version()),
        "color_profile": str(color_profile),
        "depth_profile": str(depth_profile),
        "color_intrinsics": normalize_intrinsic(camera_param.rgb_intrinsic),
        "depth_intrinsics": normalize_intrinsic(camera_param.depth_intrinsic),
        "color_distortion": normalize_distortion(camera_param.rgb_distortion),
        "depth_distortion": normalize_distortion(camera_param.depth_distortion),
        "depth_scale_m": depth_scale_mm_to_m(depth_scale_mm),
        "T_color_depth_matrix": T_color_depth.reshape(-1).tolist(),
        "camera_fingerprint": camera_fingerprint(
            serial, color_profile, depth_profile
        ),
    }


def _frame_bytes(frame):
    """Return the SDK frame payload as a one-dimensional uint8 view."""
    data = frame.get_data()
    if isinstance(data, np.ndarray):
        return np.asarray(data, dtype=np.uint8).reshape(-1)
    return np.frombuffer(data, dtype=np.uint8)


def frame_to_bgr_image(frame, sdk, cv2_module):
    """Convert supported SDK v1 color frames to a BGR OpenCV image."""
    width = int(frame.get_width())
    height = int(frame.get_height())
    image_format = frame.get_format()
    formats = sdk.OBFormat
    data = _frame_bytes(frame)

    try:
        if image_format == formats.RGB:
            return cv2_module.cvtColor(
                data.reshape(height, width, 3), cv2_module.COLOR_RGB2BGR
            )
        if image_format == formats.BGR:
            return data.reshape(height, width, 3)
        if image_format == formats.MJPG:
            image = cv2_module.imdecode(data, cv2_module.IMREAD_COLOR)
            if image is None:
                raise RuntimeError("Unable to decode Orbbec MJPG color frame")
            return image
        if image_format == formats.YUYV:
            return cv2_module.cvtColor(
                data.reshape(height, width, 2), cv2_module.COLOR_YUV2BGR_YUY2
            )
        if image_format == formats.UYVY:
            return cv2_module.cvtColor(
                data.reshape(height, width, 2), cv2_module.COLOR_YUV2BGR_UYVY
            )
        if image_format == formats.I420:
            return cv2_module.cvtColor(
                data.reshape(height * 3 // 2, width),
                cv2_module.COLOR_YUV2BGR_I420,
            )
        if image_format == formats.NV12:
            return cv2_module.cvtColor(
                data.reshape(height * 3 // 2, width),
                cv2_module.COLOR_YUV2BGR_NV12,
            )
        if image_format == formats.NV21:
            return cv2_module.cvtColor(
                data.reshape(height * 3 // 2, width),
                cv2_module.COLOR_YUV2BGR_NV21,
            )
    except ValueError as exc:
        raise RuntimeError(
            "Invalid Orbbec color frame buffer for {}x{} {}".format(
                width, height, image_format
            )
        ) from exc

    raise RuntimeError("Unsupported Orbbec color format: {}".format(image_format))


def depth_frame_to_array(frame):
    """Return the v1 depth frame as an unscaled uint16 image."""
    width = int(frame.get_width())
    height = int(frame.get_height())
    data = frame.get_data()
    try:
        array = np.asarray(data)
        if array.dtype == np.uint16:
            values = np.ascontiguousarray(array).reshape(-1).copy()
        elif array.dtype == np.uint8:
            byte_data = np.ascontiguousarray(array).reshape(-1)
            values = np.frombuffer(byte_data.tobytes(), dtype=np.uint16)
        else:
            values = np.frombuffer(data, dtype=np.uint16)
        if values.size != width * height:
            raise ValueError(
                "expected {} uint16 pixels, received {}".format(
                    width * height, values.size
                )
            )
        return values.reshape(height, width)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(
            "Invalid Orbbec depth frame buffer for {}x{}".format(width, height)
        ) from exc


def _device_count(devices):
    """Return the v1 device-list count across supported binding variants."""
    get_count = getattr(devices, "get_count", None)
    if get_count is not None:
        return int(get_count())
    return len(devices)


def _select_device(devices, serial_number):
    """Select an explicitly requested device or the sole connected device."""
    count = _device_count(devices)
    if serial_number is not None:
        try:
            device = devices.get_device_by_serial_number(serial_number)
        except Exception as exc:
            raise RuntimeError(
                "Unable to select Orbbec device with serial {!r}".format(
                    serial_number
                )
            ) from exc
        if device is None:
            raise RuntimeError(
                "No Orbbec device found with serial {!r}".format(serial_number)
            )
        return device

    if count == 0:
        raise RuntimeError("No Orbbec devices were detected")
    if count != 1:
        raise RuntimeError(
            "Expected exactly one Orbbec device when serial_number is omitted; found {}"
            .format(count)
        )
    return devices.get_device_by_index(0)


def enable_frame_sync_if_supported(pipeline):
    """Enable hardware frame sync when the selected camera supports it."""
    try:
        pipeline.enable_frame_sync()
    except Exception as exc:
        if "does not support frame sync" not in str(exc).lower():
            raise
        return False
    return True


@dataclass(frozen=True)
class OrbbecFrames:
    """A BGR color image and unscaled uint16 depth image from one frame set."""

    color_bgr: np.ndarray
    depth_raw: np.ndarray
    depth_scale_m: float
    color_timestamp_ms: float
    depth_timestamp_ms: float


class OrbbecV1Camera:
    """Own an Orbbec SDK v1 pipeline for raw color and depth acquisition."""

    def __init__(
        self,
        serial_number=None,
        color_width=1280,
        color_height=720,
        fps=30,
        timeout_ms=1000,
    ):
        self.serial_number = serial_number
        self.color_width = int(color_width)
        self.color_height = int(color_height)
        self.fps = int(fps)
        self.timeout_ms = int(timeout_ms)
        self._context = None
        self._pipeline = None
        self._sdk = None
        self._camera_param = None
        self._device_info = None
        self._metadata = None
        self._started = False
        self.color_profile = None
        self.depth_profile = None
        self.color_profile_used_fallback = None
        self.depth_profile_used_fallback = None

    @property
    def metadata(self):
        """Return a mutable copy of first-frame camera metadata, if available."""
        if self._metadata is None:
            return None
        return copy.deepcopy(self._metadata)

    def _clear_run_state(self):
        """Discard state that must not survive a new pipeline start."""
        self._context = None
        self._pipeline = None
        self._sdk = None
        self._camera_param = None
        self._device_info = None
        self._metadata = None
        self._started = False
        self.color_profile = None
        self.depth_profile = None
        self.color_profile_used_fallback = None
        self.depth_profile_used_fallback = None

    def start(self):
        """Configure and start the v1 pipeline with unaligned raw depth."""
        if self._started:
            return self

        self._clear_run_state()
        pipeline = None
        try:
            sdk = require_orbbec_sdk()
            context = sdk.Context()
            device = _select_device(context.query_devices(), self.serial_number)
            pipeline = sdk.Pipeline(device)
            config = sdk.Config()
            color_profiles = pipeline.get_stream_profile_list(
                sdk.OBSensorType.COLOR_SENSOR
            )
            color_stream, color_fallback = select_video_profile(
                color_profiles,
                self.color_width,
                self.color_height,
                self.fps,
                (sdk.OBFormat.RGB, sdk.OBFormat.MJPG, sdk.OBFormat.YUYV),
            )
            depth_profiles = pipeline.get_stream_profile_list(
                sdk.OBSensorType.DEPTH_SENSOR
            )
            depth_stream, depth_fallback = select_default_depth_profile(
                depth_profiles
            )
            config.enable_stream(color_stream)
            config.enable_stream(depth_stream)
            config.set_align_mode(sdk.OBAlignMode.DISABLE)
            enable_frame_sync_if_supported(pipeline)
            pipeline.start(config)
            camera_param = pipeline.get_camera_param()
            device_info = device.get_device_info()
        except Exception as exc:
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            self._clear_run_state()
            raise RuntimeError(
                "Failed to start Orbbec camera: {}. Check connection, serial, and SDK setup."
                .format(exc)
            ) from exc

        self._sdk = sdk
        self._context = context
        self._pipeline = pipeline
        self._camera_param = camera_param
        self._device_info = device_info
        self.color_profile = profile_to_string(color_stream)
        self.depth_profile = profile_to_string(depth_stream)
        self.color_profile_used_fallback = color_fallback
        self.depth_profile_used_fallback = depth_fallback
        self._started = True
        return self

    def wait_for_frames(self):
        """Wait for a complete color and depth frame set before converting it."""
        if not self._started or self._pipeline is None:
            raise RuntimeError("Orbbec camera has not been started")

        deadline = time.monotonic() + self.timeout_ms / 1000.0
        color_frame = None
        depth_frame = None
        missing = None
        while time.monotonic() < deadline:
            remaining_ms = max(1, int(math.ceil((deadline - time.monotonic()) * 1000.0)))
            frameset = self._pipeline.wait_for_frames(remaining_ms)
            if frameset is None:
                break
            color_frame = frameset.get_color_frame()
            depth_frame = frameset.get_depth_frame()
            if color_frame is not None and depth_frame is not None:
                break
            missing = []
            if color_frame is None:
                missing.append("color")
            if depth_frame is None:
                missing.append("depth")

        if color_frame is None or depth_frame is None:
            detail = ""
            if missing:
                detail = "; last frame set was missing {} frame".format(
                    " and ".join(missing)
                )
            raise RuntimeError(
                "Timed out waiting for complete Orbbec color and depth frames after {} ms{}"
                .format(self.timeout_ms, detail)
            )

        import cv2

        depth_scale_mm = depth_frame.get_depth_scale()
        result = OrbbecFrames(
            color_bgr=frame_to_bgr_image(color_frame, self._sdk, cv2),
            depth_raw=depth_frame_to_array(depth_frame),
            depth_scale_m=depth_scale_mm_to_m(depth_scale_mm),
            color_timestamp_ms=color_frame.get_timestamp(),
            depth_timestamp_ms=depth_frame.get_timestamp(),
        )
        if self._metadata is None:
            self._metadata = build_camera_metadata(
                self._camera_param,
                self._device_info,
                self.color_profile,
                self.depth_profile,
                depth_scale_mm,
            )
        return result

    def stop(self):
        """Stop the pipeline once; repeated calls are harmless."""
        pipeline = self._pipeline
        self._pipeline = None
        self._context = None
        self._sdk = None
        self._camera_param = None
        self._device_info = None
        self._started = False
        if pipeline is not None:
            pipeline.stop()

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
        return False
