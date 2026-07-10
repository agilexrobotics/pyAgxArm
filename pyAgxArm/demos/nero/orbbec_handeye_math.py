"""Hardware-independent geometry helpers for Orbbec hand-eye calibration.

Transforms follow ``T_A_B @ p_B = p_A``. Translations and points are metres,
and pose angles are radians using ZYX roll/pitch/yaw composition.
"""

import math

import numpy as np


_RIGID_TRANSFORM_ATOL = 1e-9


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
        atol=_RIGID_TRANSFORM_ATOL,
    ):
        raise ValueError("transform rotation must be orthonormal")
    if not np.isclose(
        np.linalg.det(rotation), 1.0, rtol=0.0, atol=_RIGID_TRANSFORM_ATOL
    ):
        raise ValueError("transform rotation determinant must be +1")
    return result


def create_checkerboard_object_points(checkerboard, square_size_m):
    """Return planar checkerboard inner-corner coordinates in metres."""
    cols, rows = checkerboard
    square_size_m = float(square_size_m)
    if cols <= 0 or rows <= 0 or not np.isfinite(square_size_m) or square_size_m <= 0:
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
