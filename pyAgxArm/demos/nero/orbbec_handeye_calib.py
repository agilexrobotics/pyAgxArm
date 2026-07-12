#!/usr/bin/env python3
"""Interactive Orbbec/Nero eye-in-hand calibration collection utility."""

import argparse
import copy
import json
import math
from pathlib import Path
import tempfile
import time

import numpy as np

from orbbec_handeye_math import (
    calibrate_eye_in_hand,
    create_checkerboard_object_points,
    load_samples,
    save_samples,
)


METHODS = ("TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS")
WINDOW_NAME = "Orbbec hand-eye calibration"


def build_arg_parser():
    """Build the command-line parser without importing optional hardware modules."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkerboard-cols", type=int, default=10)
    parser.add_argument("--checkerboard-rows", type=int, default=7)
    parser.add_argument("--square-size", type=float, default=0.02)
    parser.add_argument("--samples", type=Path, default=Path("orbbec_handeye_samples.npz"))
    parser.add_argument("--output", type=Path, default=Path("orbbec_handeye_result.json"))
    parser.add_argument("--method", choices=METHODS, default="TSAI")
    parser.add_argument("--calibrate-only", action="store_true")
    parser.add_argument("--can-interface", default="socketcan")
    parser.add_argument("--can-channel", default="can_piper")
    parser.add_argument("--firmware", default="DEFAULT")
    parser.add_argument("--camera-serial")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--frame-timeout-ms", type=int, default=1000)
    return parser


def normalize_samples_path(path):
    """Return an NPZ sample path without silently changing named extensions."""
    path = Path(path)
    if not path.suffix:
        return path.with_suffix(".npz")
    if path.suffix != ".npz":
        raise ValueError("sample files must use .npz extension: {}".format(path))
    return path


def validate_checkerboard_args(args):
    """Validate CLI checkerboard dimensions using the shared geometry convention."""
    checkerboard = (args.checkerboard_cols, args.checkerboard_rows)
    square_size_m = args.square_size
    try:
        create_checkerboard_object_points(checkerboard, square_size_m)
    except OverflowError as exc:
        raise ValueError(
            "Checkerboard dimensions must contain exactly two finite positive integers."
        ) from exc
    return checkerboard, float(square_size_m)


def collection_metadata(camera_metadata, args):
    """Copy camera metadata and bind sample collection to one checkerboard geometry."""
    checkerboard, square_size_m = validate_checkerboard_args(args)
    metadata = copy.deepcopy(camera_metadata)
    metadata["checkerboard"] = {
        "inner_corners": [int(checkerboard[0]), int(checkerboard[1])],
        "square_size_m": square_size_m,
    }
    return metadata


def stored_checkerboard_geometry(metadata):
    """Return the validated checkerboard geometry persisted with a sample archive."""
    try:
        geometry = metadata["checkerboard"]
        checkerboard = tuple(geometry["inner_corners"])
        square_size_m = geometry["square_size_m"]
        create_checkerboard_object_points(checkerboard, square_size_m)
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            "stored checkerboard geometry is missing or malformed; recapture samples "
            "with this CLI"
        ) from exc
    return (int(checkerboard[0]), int(checkerboard[1])), float(square_size_m)


def _cv2():
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("OpenCV is required for interactive hand-eye calibration") from exc
    return cv2


def create_nero_robot(args):
    """Create and connect the Nero arm only when interactive collection starts."""
    try:
        from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config
    except ImportError as exc:
        raise RuntimeError(
            "pyAgxArm is required to connect the Nero robot. Install the package "
            "and configure the requested CAN interface first."
        ) from exc

    firmware = getattr(NeroFW, str(args.firmware).upper(), None)
    if firmware is None:
        raise RuntimeError(
            "Unknown Nero firmware {!r}; use DEFAULT, V111, V112, or V120.".format(
                args.firmware
            )
        )
    robot = None
    try:
        config = create_agx_arm_config(
            robot=ArmModel.NERO,
            firmeware_version=firmware,
            interface=args.can_interface,
            channel=args.can_channel,
        )
        robot = AgxArmFactory.create_arm(config)
        robot.connect()
        return robot
    except Exception as exc:
        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                pass
        raise RuntimeError(
            "Unable to connect Nero on {}:{}: {}. Check the CAN interface, "
            "channel, power, and firmware selection.".format(
                args.can_interface, args.can_channel, exc
            )
        ) from exc


def create_camera(args):
    """Create the lazy Orbbec adapter only for interactive collection."""
    from orbbec_v1_camera import OrbbecV1Camera

    return OrbbecV1Camera(
        serial_number=args.camera_serial,
        color_width=args.width,
        color_height=args.height,
        fps=args.fps,
        timeout_ms=args.frame_timeout_ms,
    )


def get_flange_pose6(robot):
    """Read one finite six-value flange pose from a Nero driver."""
    try:
        pose = np.asarray(robot.get_flange_pose().msg, dtype=np.float64).reshape(-1)
    except (AttributeError, TypeError, ValueError) as exc:
        raise RuntimeError("Nero flange pose must contain exactly six finite numbers") from exc
    if pose.shape != (6,) or not np.all(np.isfinite(pose)):
        raise RuntimeError("Nero flange pose must contain exactly six finite numbers")
    return pose.tolist()


def camera_matrix_and_distortion(metadata):
    """Return validated color calibration arrays in OpenCV's expected layout."""
    try:
        intrinsic = metadata["color_intrinsics"]
        values = [
            float(intrinsic["fx"]),
            float(intrinsic["fy"]),
            float(intrinsic["cx"]),
            float(intrinsic["cy"]),
        ]
        distortion = np.asarray(metadata["color_distortion"], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("camera metadata lacks valid color intrinsics or distortion") from exc
    fx, fy, cx, cy = values
    if (
        not np.all(np.isfinite(values))
        or fx <= 0.0
        or fy <= 0.0
        or distortion.ndim not in (1, 2)
        or (distortion.ndim == 2 and 1 not in distortion.shape)
        or distortion.size == 0
        or not np.all(np.isfinite(distortion))
    ):
        raise ValueError("camera metadata lacks valid color intrinsics or distortion")
    return (
        np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64),
        distortion.reshape(-1, 1),
    )


def detect_checkerboard_pose(
    color_bgr, camera_matrix, dist_coeffs, checkerboard, square_size_m
):
    """Find the current checkerboard and solve its pose in the color camera frame."""
    cv2 = _cv2()
    gray = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, checkerboard, flags)
    if not found or corners is None:
        return False, None, None, None

    refined = cv2.cornerSubPix(
        gray,
        corners,
        (11, 11),
        (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
    )
    object_points = create_checkerboard_object_points(checkerboard, square_size_m)
    solved, rvec, tvec = cv2.solvePnP(
        object_points, refined, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not solved or rvec is None or tvec is None:
        return False, None, None, None
    rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
    tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)
    if not np.all(np.isfinite(rvec)) or not np.all(np.isfinite(tvec)):
        return False, None, None, None
    return True, rvec, tvec, refined


def capture_sample(detection, flange_pose, timestamp):
    """Turn a current detector result and robot pose into one persisted sample."""
    if detection is None or not detection[0]:
        raise ValueError("a current valid checkerboard detection is required")
    try:
        rvec = np.asarray(detection[1], dtype=np.float64).reshape(3, 1)
        tvec = np.asarray(detection[2], dtype=np.float64).reshape(3, 1)
        pose = np.asarray(flange_pose, dtype=np.float64).reshape(6)
        timestamp = float(timestamp)
    except (TypeError, ValueError) as exc:
        raise ValueError("sample fields must be numeric") from exc
    if not all(np.all(np.isfinite(value)) for value in (rvec, tvec, pose)) or not math.isfinite(timestamp):
        raise ValueError("sample fields must be finite")
    return {
        "flange_pose": pose.tolist(),
        "target_rvec": rvec,
        "target_tvec": tvec,
        "timestamp": timestamp,
    }


def write_result_json(path, result):
    """Write finite result JSON atomically, creating its destination directory."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        payload = json.dumps(result, ensure_ascii=True, allow_nan=False, indent=2)
    except (TypeError, ValueError) as exc:
        raise ValueError("calibration result must contain only finite JSON values") from exc
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=str(destination.parent), delete=False
        ) as handle:
            temporary = Path(handle.name)
            handle.write(payload)
            handle.write("\n")
        temporary.replace(destination)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def _color_depth_transform(metadata):
    try:
        transform = np.asarray(metadata["T_color_depth_matrix"], dtype=np.float64).reshape(4, 4)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("camera metadata lacks a valid T_color_depth_matrix") from exc
    if not np.all(np.isfinite(transform)):
        raise ValueError("camera metadata lacks a valid T_color_depth_matrix")
    return transform


def calibrate_and_write(samples, metadata, args):
    """Solve the current sample set and persist its JSON result."""
    checkerboard, square_size_m = stored_checkerboard_geometry(metadata)
    result = calibrate_eye_in_hand(
        samples,
        _color_depth_transform(metadata),
        metadata,
        checkerboard=checkerboard,
        square_size_m=square_size_m,
        method_name=args.method,
    )
    write_result_json(args.output, result)
    print("Calibration result written to {}".format(args.output))
    return result


def _metadata_compatible(saved, current):
    keys = (
        "serial",
        "firmware",
        "color_profile",
        "depth_profile",
        "color_intrinsics",
        "color_distortion",
        "depth_intrinsics",
        "depth_distortion",
        "depth_scale_m",
        "T_color_depth_matrix",
    )
    if not all(saved.get(key) == current.get(key) for key in keys):
        return False
    return (
        "camera_fingerprint" not in saved
        or saved["camera_fingerprint"] == current.get("camera_fingerprint")
    )


def load_existing_samples(path, camera_metadata, checkerboard, square_size_m):
    """Load prior samples only when they belong to this same camera configuration."""
    path = Path(path)
    if not path.exists():
        return []
    samples, saved_metadata = load_samples(path, camera_metadata["camera_fingerprint"])
    if not _metadata_compatible(saved_metadata, camera_metadata):
        raise ValueError("existing sample metadata is incompatible with this camera profile")
    saved_checkerboard, saved_square_size_m = stored_checkerboard_geometry(saved_metadata)
    if saved_checkerboard != tuple(checkerboard) or saved_square_size_m != float(
        square_size_m
    ):
        raise ValueError(
            "existing sample checkerboard geometry does not match the current "
            "checkerboard settings"
        )
    return samples


def handle_collection_key(key, detection, robot, samples, metadata, args):
    """Process capture and calibration commands; return true only after a save."""
    if key == ord("s"):
        if detection is None or not detection[0]:
            print("Skipped sample: no checkerboard is detected in the current frame.")
            return False
        sample = capture_sample(detection, get_flange_pose6(robot), time.time())
        samples.append(sample)
        Path(args.samples).parent.mkdir(parents=True, exist_ok=True)
        save_samples(args.samples, samples, metadata)
        print("Saved sample {} to {}".format(len(samples), args.samples))
        return True
    if key == ord("c"):
        try:
            calibrate_and_write(samples, metadata, args)
        except (ValueError, RuntimeError) as exc:
            print("Calibration not written: {}".format(exc))
        return False
    return False


def _draw_frame(color_bgr, detection, camera_matrix, dist_coeffs, checkerboard, sample_count, camera):
    cv2 = _cv2()
    display = color_bgr.copy()
    if detection is not None:
        _, rvec, tvec, corners = detection
        cv2.drawChessboardCorners(display, checkerboard, corners, True)
        cv2.drawFrameAxes(display, camera_matrix, dist_coeffs, rvec, tvec, 0.06)
        status = "detected"
    else:
        status = "not detected"
    profile = "color {} | depth {}".format(camera.color_profile, camera.depth_profile)
    cv2.putText(display, "samples: {} | board: {}".format(sample_count, status), (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0) if detection else (0, 0, 255), 2)
    cv2.putText(display, profile, (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    cv2.imshow(WINDOW_NAME, display)


def run_frame_loop(camera, initial_frames, robot, samples, metadata, args):
    """Run the display/detect/capture loop with raw depth left untouched."""
    camera_matrix, dist_coeffs = camera_matrix_and_distortion(metadata)
    frames = initial_frames
    while True:
        # ``frames.depth_raw`` is intentionally neither aligned nor interpreted here.
        found, rvec, tvec, corners = detect_checkerboard_pose(
            frames.color_bgr,
            camera_matrix,
            dist_coeffs,
            (args.checkerboard_cols, args.checkerboard_rows),
            args.square_size,
        )
        last_detection = (found, rvec, tvec, corners) if found else None
        _draw_frame(
            frames.color_bgr,
            last_detection,
            camera_matrix,
            dist_coeffs,
            (args.checkerboard_cols, args.checkerboard_rows),
            len(samples),
            camera,
        )
        key = _cv2().waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            return
        handle_collection_key(key, last_detection, robot, samples, metadata, args)
        frames = camera.wait_for_frames()


def destroy_windows():
    """Close OpenCV windows without importing OpenCV until collection is used."""
    _cv2().destroyAllWindows()


def run_collection(args):
    """Start hardware, collect/check samples, and clean up on every exit path."""
    camera = None
    robot = None
    try:
        checkerboard, square_size_m = validate_checkerboard_args(args)
        camera = create_camera(args)
        camera.start()
        initial_frames = camera.wait_for_frames()
        if not isinstance(camera.metadata, dict):
            raise RuntimeError("Orbbec did not provide camera metadata with the first frame")
        metadata = collection_metadata(camera.metadata, args)
        camera_matrix_and_distortion(metadata)
        _color_depth_transform(metadata)
        robot = create_nero_robot(args)
        samples = load_existing_samples(
            args.samples, metadata, checkerboard, square_size_m
        )
        print("Checkerboard: {}x{} inner corners, {:.6g} m squares.".format(args.checkerboard_cols, args.checkerboard_rows, args.square_size))
        print("Capture 15-30 poses with varied, non-collinear rotations for a reliable solve.")
        run_frame_loop(camera, initial_frames, robot, samples, metadata, args)
        return 0
    finally:
        if robot is not None:
            try:
                robot.disconnect()
            except Exception as exc:
                print("Nero cleanup warning: {}".format(exc))
        if camera is not None:
            try:
                camera.stop()
            except Exception as exc:
                print("Orbbec cleanup warning: {}".format(exc))
        try:
            destroy_windows()
        except Exception as exc:
            print("OpenCV cleanup warning: {}".format(exc))


def main(argv=None):
    """Run the calibration command and return a shell-friendly status code."""
    args = build_arg_parser().parse_args(argv)
    validate_checkerboard_args(args)
    args.samples = normalize_samples_path(args.samples)
    if args.calibrate_only:
        samples, metadata = load_samples(args.samples)
        calibrate_and_write(samples, metadata, args)
        return 0
    return run_collection(args)


if __name__ == "__main__":
    raise SystemExit(main())
