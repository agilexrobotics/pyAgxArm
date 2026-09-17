"""Hardware-independent geometry helpers for Orbbec hand-eye calibration.

Transforms follow ``T_A_B @ p_B = p_A``. Translations and points are metres,
and pose angles are radians using ZYX roll/pitch/yaw composition.
"""

import json
import math
import os
from pathlib import Path
import tempfile
import zipfile
from datetime import datetime, timezone
from numbers import Integral, Real

import numpy as np


_RIGID_TRANSFORM_ATOL = 1e-9
_RIGID_TRANSFORM_INPUT_ATOL = 1e-6
_ROTATION_AXIS_MIN_ANGLE_RAD = math.radians(1.0)
_ROTATION_AXIS_CONDITIONING_MIN_RATIO = 0.05
_SAMPLE_ARCHIVE_MEMBERS = {
    "flange_poses.npy",
    "target_rvecs.npy",
    "target_tvecs.npy",
    "timestamps.npy",
    "camera_fingerprint.npy",
    "camera_metadata_json.npy",
}
_SAMPLE_ARCHIVE_MAX_MEMBER_BYTES = 2 * 1024 * 1024
_SAMPLE_ARCHIVE_MAX_TOTAL_BYTES = 4 * 1024 * 1024
_SAMPLE_ARCHIVE_MAX_COMPRESSION_RATIO = 2000.0


def _validate_rotation(rotation):
    """Return a validated float64 proper rotation matrix."""
    try:
        result = np.asarray(rotation, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError("rotation must be a numeric array with shape (3, 3)") from exc
    if result.shape != (3, 3):
        raise ValueError("rotation must have shape (3, 3)")
    if not np.all(np.isfinite(result)):
        raise ValueError("rotation must contain only finite values")
    if not np.allclose(
        result.T @ result, np.eye(3), rtol=0.0, atol=_RIGID_TRANSFORM_ATOL
    ):
        raise ValueError("rotation must be orthonormal")
    if not np.isclose(
        np.linalg.det(result), 1.0, rtol=0.0, atol=_RIGID_TRANSFORM_ATOL
    ):
        raise ValueError("rotation determinant must be +1")
    return result


def _validate_json_value(value, name):
    """Reject non-finite values in data intended for JSON serialization."""
    if isinstance(value, dict):
        for key, child in value.items():
            if not isinstance(key, str):
                raise ValueError("{} must have string object keys".format(name))
            _validate_json_value(child, name)
    elif isinstance(value, (list, tuple)):
        for child in value:
            _validate_json_value(child, name)
    elif isinstance(value, (float, np.floating)) and not np.isfinite(value):
        raise ValueError("{} must contain only finite numbers".format(name))


def _validated_camera_metadata(camera_metadata):
    if not isinstance(camera_metadata, dict):
        raise ValueError("camera_metadata must be a JSON object")
    fingerprint = camera_metadata.get("camera_fingerprint")
    if not isinstance(fingerprint, str) or not fingerprint:
        raise ValueError("camera_metadata must include a non-empty camera_fingerprint")
    _validate_json_value(camera_metadata, "camera_metadata")
    try:
        metadata_json = json.dumps(
            camera_metadata, allow_nan=False, sort_keys=True, separators=(",", ":")
        )
    except (TypeError, ValueError) as exc:
        raise ValueError("camera_metadata must be JSON-serializable") from exc
    return metadata_json


def _normalized_samples(samples):
    try:
        items = list(samples)
    except TypeError as exc:
        raise ValueError("samples must be an iterable of sample objects") from exc

    normalized = []
    required = ("flange_pose", "target_rvec", "target_tvec", "timestamp")
    for index, sample in enumerate(items):
        if not isinstance(sample, dict):
            raise ValueError("sample {} must be an object".format(index))
        missing = [field for field in required if field not in sample]
        if missing:
            raise ValueError("sample {} missing required field {}".format(index, missing[0]))
        try:
            flange_pose = np.asarray(sample["flange_pose"], dtype=np.float64).reshape(6)
            target_rvec = np.asarray(sample["target_rvec"], dtype=np.float64).reshape(3, 1)
            target_tvec = np.asarray(sample["target_tvec"], dtype=np.float64).reshape(3, 1)
            timestamp = float(sample["timestamp"])
        except (TypeError, ValueError) as exc:
            raise ValueError("sample {} has invalid numeric fields".format(index)) from exc
        if not (
            np.all(np.isfinite(flange_pose))
            and np.all(np.isfinite(target_rvec))
            and np.all(np.isfinite(target_tvec))
            and np.isfinite(timestamp)
        ):
            raise ValueError("sample {} must contain only finite values".format(index))
        # Constructing the transform also validates any pose-derived rotation.
        pose6_to_matrix(flange_pose)
        normalized.append(
            {
                "flange_pose": flange_pose.tolist(),
                "target_rvec": target_rvec,
                "target_tvec": target_tvec,
                "timestamp": timestamp,
            }
        )
    return normalized


def _validate_rigid_transform(transform):
    """Return a validated float64 homogeneous rigid transform."""
    try:
        result = np.asarray(transform, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError("transform must be a numeric array with shape (4, 4)") from exc

    if result.shape != (4, 4):
        raise ValueError("transform must have shape (4, 4)")
    if not np.all(np.isfinite(result)):
        raise ValueError("transform must contain only finite values")
    if not np.allclose(
        result[3], [0.0, 0.0, 0.0, 1.0], rtol=0.0, atol=_RIGID_TRANSFORM_ATOL
    ):
        raise ValueError("transform bottom row must be [0, 0, 0, 1]")

    rotation = result[:3, :3]
    if not np.allclose(
        rotation.T @ rotation,
        np.eye(3),
        rtol=0.0,
        atol=_RIGID_TRANSFORM_INPUT_ATOL,
    ):
        raise ValueError("transform rotation must be orthonormal")
    if not np.isclose(
        np.linalg.det(rotation), 1.0, rtol=0.0, atol=_RIGID_TRANSFORM_INPUT_ATOL
    ):
        raise ValueError("transform rotation determinant must be +1")

    result = result.copy()
    u, _, vt = np.linalg.svd(rotation)
    result[:3, :3] = u @ vt
    return result


def create_checkerboard_object_points(checkerboard, square_size_m):
    """Return planar checkerboard inner-corner coordinates in metres."""
    try:
        dimensions = tuple(checkerboard)
    except TypeError as exc:
        raise ValueError(
            "Checkerboard dimensions must contain exactly two finite positive integers."
        ) from exc
    if len(dimensions) != 2:
        raise ValueError(
            "Checkerboard dimensions must contain exactly two finite positive integers."
        )

    normalized_dimensions = []
    for dimension in dimensions:
        if isinstance(dimension, (bool, np.bool_)) or not isinstance(dimension, Real):
            raise ValueError(
                "Checkerboard dimensions must contain exactly two finite positive integers."
            )
        numeric_dimension = float(dimension)
        if (
            not np.isfinite(numeric_dimension)
            or numeric_dimension <= 0.0
            or not numeric_dimension.is_integer()
        ):
            raise ValueError(
                "Checkerboard dimensions must contain exactly two finite positive integers."
            )
        normalized_dimensions.append(int(dimension))
    cols, rows = normalized_dimensions

    square_size_m = float(square_size_m)
    if not np.isfinite(square_size_m) or square_size_m <= 0:
        raise ValueError(
            "Checkerboard dimensions and square size must be finite and positive."
        )

    points = np.zeros((cols * rows, 3), dtype=np.float64)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points[:, :2] *= square_size_m
    return points


def pose6_to_matrix(pose):
    """Convert ``[x, y, z, roll, pitch, yaw]`` to a homogeneous transform."""
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
    """Convert a homogeneous transform to a ZYX roll/pitch/yaw pose."""
    T = _validate_rigid_transform(transform)
    pitch = math.asin(float(np.clip(-T[2, 0], -1.0, 1.0)))

    if abs(math.cos(pitch)) < 1e-9:
        roll = 0.0
        yaw = math.atan2(-T[0, 1], T[1, 1])
    else:
        roll = math.atan2(T[2, 1], T[2, 2])
        yaw = math.atan2(T[1, 0], T[0, 0])

    return np.array(
        [T[0, 3], T[1, 3], T[2, 3], roll, pitch, yaw], dtype=np.float64
    )


def transform_point(T_parent_child, point_child):
    """Map a 3D point from a child frame into its parent frame."""
    transform = _validate_rigid_transform(T_parent_child)
    point = np.asarray(point_child, dtype=np.float64).reshape(3)
    return (transform @ np.r_[point, 1.0])[:3]


def invert_transform(T_parent_child):
    """Return the rigid inverse of a homogeneous transform."""
    transform = _validate_rigid_transform(T_parent_child)
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = transform[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ transform[:3, 3]
    return inverse


def deproject_depth_pixel(
    u,
    v,
    depth_raw,
    intrinsics,
    depth_scale_m,
    min_depth_m=0.0,
    max_depth_m=None,
):
    """Deproject one raw depth pixel into the metric depth-camera frame."""
    depth_raw = float(depth_raw)
    depth_scale_m = float(depth_scale_m)
    if not np.isfinite(depth_raw) or depth_raw <= 0.0:
        raise ValueError("depth_raw must be finite and greater than zero")
    if not np.isfinite(depth_scale_m) or depth_scale_m <= 0.0:
        raise ValueError("depth_scale_m must be finite and greater than zero")

    min_depth_m = float(min_depth_m)
    if not np.isfinite(min_depth_m) or min_depth_m < 0.0:
        raise ValueError("min_depth_m must be finite and non-negative")
    if max_depth_m is not None:
        max_depth_m = float(max_depth_m)
        if not np.isfinite(max_depth_m):
            raise ValueError("max_depth_m must be finite")
        if max_depth_m < min_depth_m:
            raise ValueError(
                "max_depth_m must be greater than or equal to min_depth_m"
            )

    depth_m = depth_raw * depth_scale_m
    if not np.isfinite(depth_m) or depth_m <= 0.0:
        raise ValueError("depth must be finite and greater than zero")
    if depth_m < min_depth_m or (
        max_depth_m is not None and depth_m > max_depth_m
    ):
        raise ValueError("depth is outside the configured range")

    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    if not np.isfinite(fx) or not np.isfinite(fy) or fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be finite and positive")

    u = float(u)
    v = float(v)
    cx = float(intrinsics["cx"])
    cy = float(intrinsics["cy"])
    for name, value in (("u", u), ("v", v), ("cx", cx), ("cy", cy)):
        if not np.isfinite(value):
            raise ValueError("{} must be finite".format(name))

    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    return np.array([x, y, depth_m], dtype=np.float64)


def depth_pixel_to_base(
    u,
    v,
    depth_raw,
    depth_intrinsics,
    depth_scale_m,
    T_flange_depth,
    T_base_flange,
):
    """Deproject a depth pixel and map it through flange into the base frame."""
    point_depth = deproject_depth_pixel(
        u, v, depth_raw, depth_intrinsics, depth_scale_m
    )
    T_base_depth = np.asarray(T_base_flange) @ np.asarray(T_flange_depth)
    return transform_point(T_base_depth, point_depth)


def save_samples(path, samples, camera_metadata):
    """Save hand-eye samples in a compressed, pickle-free NPZ archive."""
    metadata_json = _validated_camera_metadata(camera_metadata)
    normalized = _normalized_samples(samples)
    count = len(normalized)
    if count:
        flange_poses = np.asarray(
            [sample["flange_pose"] for sample in normalized], dtype=np.float64
        ).reshape(count, 6)
        target_rvecs = np.stack(
            [sample["target_rvec"] for sample in normalized]
        ).astype(np.float64, copy=False)
        target_tvecs = np.stack(
            [sample["target_tvec"] for sample in normalized]
        ).astype(np.float64, copy=False)
        timestamps = np.asarray(
            [sample["timestamp"] for sample in normalized], dtype=np.float64
        )
    else:
        flange_poses = np.empty((0, 6), dtype=np.float64)
        target_rvecs = np.empty((0, 3, 1), dtype=np.float64)
        target_tvecs = np.empty((0, 3, 1), dtype=np.float64)
        timestamps = np.empty((0,), dtype=np.float64)

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile(
            dir=path.parent, prefix=".{}-".format(path.name), suffix=".npz", delete=False
        ) as temp_file:
            temp_path = Path(temp_file.name)
        np.savez_compressed(
            temp_path,
            flange_poses=flange_poses,
            target_rvecs=target_rvecs,
            target_tvecs=target_tvecs,
            timestamps=timestamps,
            camera_fingerprint=np.asarray(camera_metadata["camera_fingerprint"]),
            camera_metadata_json=np.asarray(metadata_json),
        )
        os.replace(temp_path, path)
    finally:
        if temp_path is not None:
            try:
                temp_path.unlink()
            except FileNotFoundError:
                pass


def _preflight_sample_archive(path):
    """Validate ZIP members and declared sizes before NumPy materializes arrays."""
    try:
        with zipfile.ZipFile(path) as archive:
            members = archive.infolist()
    except (OSError, EOFError, IOError, zipfile.BadZipFile) as exc:
        raise ValueError("could not load sample archive: {}".format(exc)) from exc

    names = [member.filename for member in members]
    unexpected = sorted(set(names).difference(_SAMPLE_ARCHIVE_MEMBERS))
    if unexpected:
        raise ValueError("sample archive contains unexpected member {}".format(unexpected[0]))
    missing = sorted(_SAMPLE_ARCHIVE_MEMBERS.difference(names))
    if missing:
        raise ValueError("sample archive missing required field {}".format(missing[0][:-4]))
    if len(members) != len(_SAMPLE_ARCHIVE_MEMBERS) or len(set(names)) != len(names):
        raise ValueError("sample archive exceeds resource limit with duplicate members")

    total_size = 0
    for member in members:
        if member.file_size > _SAMPLE_ARCHIVE_MAX_MEMBER_BYTES:
            raise ValueError("sample archive member exceeds resource limit")
        total_size += member.file_size
        if total_size > _SAMPLE_ARCHIVE_MAX_TOTAL_BYTES:
            raise ValueError("sample archive total exceeds resource limit")
        if member.file_size and (
            not member.compress_size
            or member.file_size / member.compress_size > _SAMPLE_ARCHIVE_MAX_COMPRESSION_RATIO
        ):
            raise ValueError("sample archive compression exceeds resource limit")


def load_samples(path, expected_fingerprint=None):
    """Load a sample archive, rejecting malformed data and fingerprint mismatches."""
    required = {
        "flange_poses",
        "target_rvecs",
        "target_tvecs",
        "timestamps",
        "camera_fingerprint",
        "camera_metadata_json",
    }
    _preflight_sample_archive(path)
    try:
        with np.load(path, allow_pickle=False) as archive:
            missing = sorted(required.difference(archive.files))
            if missing:
                raise ValueError("sample archive missing required field {}".format(missing[0]))
            try:
                flange_poses = np.asarray(archive["flange_poses"], dtype=np.float64)
                target_rvecs = np.asarray(archive["target_rvecs"], dtype=np.float64)
                target_tvecs = np.asarray(archive["target_tvecs"], dtype=np.float64)
                timestamps = np.asarray(archive["timestamps"], dtype=np.float64)
                fingerprint_data = archive["camera_fingerprint"]
                metadata_data = archive["camera_metadata_json"]
            except (TypeError, ValueError) as exc:
                raise ValueError("sample archive has invalid numeric fields") from exc
    except ValueError:
        raise
    except (OSError, EOFError, IOError, zipfile.BadZipFile) as exc:
        raise ValueError("could not load sample archive: {}".format(exc)) from exc

    if fingerprint_data.shape != () or metadata_data.shape != ():
        raise ValueError("sample archive metadata fields must be scalar values")
    fingerprint_item = fingerprint_data.item()
    if not isinstance(fingerprint_item, (str, np.str_)):
        raise ValueError("sample archive camera fingerprint must be a scalar string")
    try:
        fingerprint = str(fingerprint_item)
        metadata = json.loads(str(metadata_data.item()))
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError("sample archive has invalid camera metadata JSON") from exc
    _validated_camera_metadata(metadata)
    if metadata["camera_fingerprint"] != fingerprint:
        raise ValueError("sample archive camera_fingerprint does not match metadata")
    if expected_fingerprint is not None and fingerprint != expected_fingerprint:
        raise ValueError("sample archive camera fingerprint does not match expected fingerprint")

    count = flange_poses.shape[0] if flange_poses.ndim else -1
    expected_shapes = {
        "flange_poses": (count, 6),
        "target_rvecs": (count, 3, 1),
        "target_tvecs": (count, 3, 1),
        "timestamps": (count,),
    }
    actual_arrays = {
        "flange_poses": flange_poses,
        "target_rvecs": target_rvecs,
        "target_tvecs": target_tvecs,
        "timestamps": timestamps,
    }
    for name, expected_shape in expected_shapes.items():
        values = actual_arrays[name]
        if values.shape != expected_shape:
            raise ValueError(
                "sample archive field {} must have shape {}".format(name, expected_shape)
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("sample archive field {} must contain only finite values".format(name))

    return (
        [
            {
                "flange_pose": flange_poses[index].tolist(),
                "target_rvec": target_rvecs[index].reshape(3, 1),
                "target_tvec": target_tvecs[index].reshape(3, 1),
                "timestamp": float(timestamps[index]),
            }
            for index in range(count)
        ],
        metadata,
    )


def rotation_angle(rotation):
    """Return the principal angle of a finite proper rotation matrix, in radians."""
    matrix = _validate_rotation(rotation)
    cosine = float(np.clip((np.trace(matrix) - 1.0) / 2.0, -1.0, 1.0))
    return math.acos(cosine)


def _rotation_log_vector(rotation):
    """Return the axis-angle logarithm of a finite proper rotation matrix."""
    matrix = _validate_rotation(rotation)
    angle = rotation_angle(matrix)
    if angle < _ROTATION_AXIS_MIN_ANGLE_RAD:
        return None
    if math.pi - angle < 1e-6:
        eigenvalues, eigenvectors = np.linalg.eig(matrix)
        axis = np.real(eigenvectors[:, np.argmin(np.abs(eigenvalues - 1.0))])
        axis /= np.linalg.norm(axis)
    else:
        axis = np.array(
            [
                matrix[2, 1] - matrix[1, 2],
                matrix[0, 2] - matrix[2, 0],
                matrix[1, 0] - matrix[0, 1],
            ],
            dtype=np.float64,
        ) / (2.0 * math.sin(angle))
    return angle * axis


def validate_sample_motion(samples):
    """Validate sample count and orientation diversity for hand-eye calibration."""
    normalized = _normalized_samples(samples)
    if len(normalized) < 3:
        raise ValueError("at least 3 samples are required for hand-eye calibration")

    transforms = [pose6_to_matrix(sample["flange_pose"]) for sample in normalized]
    translations = np.asarray([transform[:3, 3] for transform in transforms])
    max_rotation = 0.0
    translation_span = 0.0
    rotation_log_vectors = []
    for left in range(len(transforms)):
        for right in range(left + 1, len(transforms)):
            relative = invert_transform(transforms[left]) @ transforms[right]
            max_rotation = max(max_rotation, rotation_angle(relative[:3, :3]))
            log_vector = _rotation_log_vector(relative[:3, :3])
            if log_vector is not None:
                rotation_log_vectors.append(log_vector)
            translation_span = max(
                translation_span,
                float(np.linalg.norm(translations[left] - translations[right])),
            )
    max_rotation_deg = math.degrees(max_rotation)
    if max_rotation_deg < 5.0:
        raise ValueError(
            "insufficient rotation diversity: capture 15-30 samples with diverse "
            "orientations (maximum relative rotation must be at least 5 degrees)"
        )
    singular_values = np.linalg.svd(
        np.asarray(rotation_log_vectors, dtype=np.float64), compute_uv=False
    )
    axis_conditioning = (
        float(singular_values[1] / singular_values[0])
        if len(singular_values) >= 2 and singular_values[0] > 0.0
        else 0.0
    )
    if axis_conditioning < _ROTATION_AXIS_CONDITIONING_MIN_RATIO:
        raise ValueError(
            "insufficient non-collinear axes for rotational observability; capture "
            "motions about at least two independent rotation axes"
        )
    return {
        "sample_count": len(normalized),
        "max_rotation_deg": float(max_rotation_deg),
        "translation_span_m": float(translation_span),
        "rotation_axis_conditioning": axis_conditioning,
    }


def make_transform(rotation, translation):
    """Create a validated homogeneous transform from rotation and translation."""
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = _validate_rotation(rotation)
    try:
        vector = np.asarray(translation, dtype=np.float64).reshape(3)
    except (TypeError, ValueError) as exc:
        raise ValueError("translation must be a numeric array with shape (3,)") from exc
    if not np.all(np.isfinite(vector)):
        raise ValueError("translation must contain only finite values")
    result[:3, 3] = vector
    return _validate_rigid_transform(result)


def rotation_to_quaternion_xyzw(rotation):
    """Convert a proper rotation matrix to a normalized ``[x, y, z, w]`` quaternion."""
    matrix = _validate_rotation(rotation)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = [(matrix[2, 1] - matrix[1, 2]) / scale,
                      (matrix[0, 2] - matrix[2, 0]) / scale,
                      (matrix[1, 0] - matrix[0, 1]) / scale, 0.25 * scale]
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = [0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale,
                          (matrix[0, 2] + matrix[2, 0]) / scale,
                          (matrix[2, 1] - matrix[1, 2]) / scale]
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = [(matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                          (matrix[1, 2] + matrix[2, 1]) / scale,
                          (matrix[0, 2] - matrix[2, 0]) / scale]
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = [(matrix[0, 2] + matrix[2, 0]) / scale,
                          (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale,
                          (matrix[1, 0] - matrix[0, 1]) / scale]
    quaternion = np.asarray(quaternion, dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    if not np.all(np.isfinite(quaternion)):
        raise ValueError("rotation produced a non-finite quaternion")
    return quaternion


def named_transform(parent_frame, child_frame, transform):
    """Return a JSON-ready, explicitly framed transform description."""
    if not isinstance(parent_frame, str) or not parent_frame:
        raise ValueError("parent_frame must be a non-empty string")
    if not isinstance(child_frame, str) or not child_frame:
        raise ValueError("child_frame must be a non-empty string")
    matrix = _validate_rigid_transform(transform)
    return {
        "parent_frame": parent_frame,
        "child_frame": child_frame,
        "translation_m": [float(value) for value in matrix[:3, 3]],
        "pose6_xyz_rpy": [float(value) for value in matrix_to_pose6(matrix)],
        "quaternion_xyzw": [
            float(value) for value in rotation_to_quaternion_xyzw(matrix[:3, :3])
        ],
        "matrix_row_major_4x4": [float(value) for value in matrix.reshape(-1)],
    }


def _rvec_to_rotation(rvec):
    """Convert a Rodrigues vector to a rotation without requiring OpenCV."""
    vector = np.asarray(rvec, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(vector)):
        raise ValueError("Rodrigues vector must contain only finite values")
    angle = float(np.linalg.norm(vector))
    skew = np.array(
        [[0.0, -vector[2], vector[1]], [vector[2], 0.0, -vector[0]], [-vector[1], vector[0], 0.0]],
        dtype=np.float64,
    )
    if angle < 1e-12:
        rotation = np.eye(3) + skew + 0.5 * (skew @ skew)
    else:
        rotation = (
            np.eye(3)
            + math.sin(angle) / angle * skew
            + (1.0 - math.cos(angle)) / (angle * angle) * (skew @ skew)
        )
    return _validate_rotation(rotation)


def target_consistency_metrics(samples, T_flange_color):
    """Measure pairwise agreement of checkerboard poses reconstructed in base frame."""
    normalized = _normalized_samples(samples)
    flange_color = _validate_rigid_transform(T_flange_color)
    base_targets = []
    for sample in normalized:
        color_target = make_transform(
            _rvec_to_rotation(sample["target_rvec"]), sample["target_tvec"]
        )
        base_targets.append(
            pose6_to_matrix(sample["flange_pose"]) @ flange_color @ color_target
        )

    translation_distances = []
    rotation_angles = []
    for left in range(len(base_targets)):
        for right in range(left + 1, len(base_targets)):
            translation_distances.append(
                float(
                    np.linalg.norm(
                        base_targets[left][:3, 3] - base_targets[right][:3, 3]
                    )
                )
            )
            relative = invert_transform(base_targets[left]) @ base_targets[right]
            rotation_angles.append(math.degrees(rotation_angle(relative[:3, :3])))

    def summary(values):
        data = np.asarray(values, dtype=np.float64)
        if not len(data):
            return 0.0, 0.0, 0.0
        return (
            float(np.mean(data)),
            float(math.sqrt(np.mean(data * data))),
            float(np.max(data)),
        )

    translation_mean, translation_rms, translation_max = summary(translation_distances)
    rotation_mean, rotation_rms, rotation_max = summary(rotation_angles)
    metrics = {
        "translation_mean_m": translation_mean,
        "translation_rms_m": translation_rms,
        "translation_max_m": translation_max,
        "rotation_mean_deg": rotation_mean,
        "rotation_rms_deg": rotation_rms,
        "rotation_max_deg": rotation_max,
    }
    _validate_json_value(metrics, "consistency metrics")
    return metrics


def calibrate_eye_in_hand(
    samples,
    T_color_depth,
    camera_metadata,
    checkerboard=(10, 7),
    square_size_m=0.02,
    method_name="TSAI",
):
    """Solve and serialize an eye-in-hand calibration from checkerboard samples."""
    if not isinstance(method_name, str):
        raise ValueError("method_name must name a supported hand-eye method")
    method = method_name.upper()
    supported_methods = {"TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS"}
    if method not in supported_methods:
        raise ValueError(
            "unknown hand-eye method {!r}; supported methods are {}".format(
                method_name, ", ".join(sorted(supported_methods))
            )
        )
    _validated_camera_metadata(camera_metadata)
    create_checkerboard_object_points(checkerboard, square_size_m)
    normalized = _normalized_samples(samples)
    motion = validate_sample_motion(normalized)
    color_depth = _validate_rigid_transform(T_color_depth)

    try:
        import cv2
    except ImportError as exc:
        raise ValueError("OpenCV is required to solve hand-eye calibration") from exc
    if not hasattr(cv2, "calibrateHandEye"):
        raise ValueError("this OpenCV build does not provide calibrateHandEye")

    method_constants = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    rotations_gripper_to_base = []
    translations_gripper_to_base = []
    rotations_target_to_camera = []
    translations_target_to_camera = []
    for sample in normalized:
        base_flange = pose6_to_matrix(sample["flange_pose"])
        rotations_gripper_to_base.append(base_flange[:3, :3])
        translations_gripper_to_base.append(base_flange[:3, 3].reshape(3, 1))
        rotation_target_to_camera, _ = cv2.Rodrigues(sample["target_rvec"])
        rotations_target_to_camera.append(rotation_target_to_camera)
        translations_target_to_camera.append(sample["target_tvec"])

    try:
        rotation_camera_to_gripper, translation_camera_to_gripper = cv2.calibrateHandEye(
            rotations_gripper_to_base,
            translations_gripper_to_base,
            rotations_target_to_camera,
            translations_target_to_camera,
            method=method_constants[method],
        )
    except cv2.error as exc:
        raise ValueError("OpenCV hand-eye calibration failed: {}".format(exc)) from exc
    flange_color = make_transform(rotation_camera_to_gripper, translation_camera_to_gripper)
    flange_depth = flange_color @ color_depth
    _validate_rigid_transform(flange_depth)
    consistency = target_consistency_metrics(normalized, flange_color)

    quality_warnings = []
    if motion["sample_count"] < 15:
        quality_warnings.append(
            "Only {} samples were captured; 15-30 diverse samples are recommended.".format(
                motion["sample_count"]
            )
        )
    result = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "camera": camera_metadata,
        "checkerboard": {
            "inner_corners": [int(checkerboard[0]), int(checkerboard[1])],
            "square_size_m": float(square_size_m),
        },
        "sample_count": motion["sample_count"],
        "method": method,
        "quality_warnings": quality_warnings,
        "T_color_depth": named_transform("color", "depth", color_depth),
        "T_depth_color": named_transform("depth", "color", invert_transform(color_depth)),
        "T_flange_color": named_transform("flange", "color", flange_color),
        "T_color_flange": named_transform("color", "flange", invert_transform(flange_color)),
        "T_flange_depth": named_transform("flange", "depth", flange_depth),
        "T_depth_flange": named_transform("depth", "flange", invert_transform(flange_depth)),
        "consistency": consistency,
    }
    _validate_json_value(result, "calibration result")
    try:
        json.dumps(result, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise ValueError("calibration result is not JSON-serializable") from exc
    return result
