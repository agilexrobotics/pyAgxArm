"""Hardware-independent tests for Orbbec hand-eye geometry helpers."""

import math
from pathlib import Path
import sys

import numpy as np
import pytest


NERO_DEMO_DIR = Path(__file__).resolve().parents[1] / "pyAgxArm" / "demos" / "nero"
sys.path.insert(0, str(NERO_DEMO_DIR))

import orbbec_handeye_math as math3d  # noqa: E402


def test_checkerboard_points_use_inner_corners_and_metres():
    points = math3d.create_checkerboard_object_points((10, 7), 0.02)

    assert points.shape == (70, 3)
    assert np.issubdtype(points.dtype, np.floating)
    np.testing.assert_allclose(points[0], [0.0, 0.0, 0.0])
    np.testing.assert_allclose(points[-1], [0.18, 0.12, 0.0])


@pytest.mark.parametrize(
    "checkerboard,square_size_m",
    [
        ((0, 7), 0.02),
        ((10, 0), 0.02),
        ((-10, 7), 0.02),
        ((10, -7), 0.02),
        ((10, 7), 0.0),
        ((10, 7), -0.02),
    ],
)
def test_checkerboard_rejects_non_positive_dimensions_and_size(
    checkerboard, square_size_m
):
    with pytest.raises(ValueError):
        math3d.create_checkerboard_object_points(checkerboard, square_size_m)


def test_pose6_to_matrix_uses_zyx_rpy_and_homogeneous_translation():
    pose = [0.1, -0.2, 0.3, math.pi / 2.0, 0.0, math.pi / 2.0]

    transform = math3d.pose6_to_matrix(pose)

    roll_rotation = np.array(
        [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
    )
    yaw_rotation = np.array(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    )
    expected = np.eye(4)
    expected[:3, :3] = yaw_rotation @ roll_rotation
    expected[:3, 3] = pose[:3]
    np.testing.assert_allclose(transform, expected, atol=1e-15)


def test_pose6_round_trip_uses_zyx_rpy():
    pose = [0.1, -0.2, 0.3, 0.2, -0.1, 0.4]

    result = math3d.matrix_to_pose6(math3d.pose6_to_matrix(pose))

    np.testing.assert_allclose(result, pose, atol=1e-9)


@pytest.mark.parametrize("pitch", [math.pi / 2.0, -math.pi / 2.0])
def test_matrix_to_pose6_handles_gimbal_lock(pitch):
    transform = math3d.pose6_to_matrix([0.1, -0.2, 0.3, 0.4, pitch, -0.7])

    recovered_pose = math3d.matrix_to_pose6(transform)

    assert np.all(np.isfinite(recovered_pose))
    np.testing.assert_allclose(
        math3d.pose6_to_matrix(recovered_pose), transform, atol=1e-9
    )


def test_transform_point_maps_child_point_to_parent():
    transform = math3d.pose6_to_matrix(
        [1.0, 2.0, 3.0, 0.0, 0.0, math.pi / 2.0]
    )

    point = math3d.transform_point(transform, [1.0, 0.0, 0.0])

    np.testing.assert_allclose(point, [1.0, 3.0, 3.0], atol=1e-15)


def test_invert_transform_composes_to_identity():
    transform = math3d.pose6_to_matrix([0.1, -0.2, 0.3, 0.2, -0.1, 0.4])

    inverse = math3d.invert_transform(transform)

    np.testing.assert_allclose(transform @ inverse, np.eye(4), atol=1e-15)
    np.testing.assert_allclose(inverse @ transform, np.eye(4), atol=1e-15)


def test_depth_pixel_to_camera_point_converts_raw_depth_to_metres():
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}

    point = math3d.deproject_depth_pixel(
        420, 290, 1000, intrinsics, depth_scale_m=0.001
    )

    np.testing.assert_allclose(point, [0.2, 0.1, 1.0])


@pytest.mark.parametrize("depth_raw", [0, -1, float("nan"), float("inf")])
def test_deprojection_rejects_invalid_depth(depth_raw):
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}

    with pytest.raises(ValueError, match="depth"):
        math3d.deproject_depth_pixel(
            320, 240, depth_raw, intrinsics, depth_scale_m=0.001
        )


@pytest.mark.parametrize(
    "depth_scale_m", [0, -0.001, float("nan"), float("inf")]
)
def test_deprojection_rejects_scale_that_produces_invalid_depth(depth_scale_m):
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}

    with pytest.raises(ValueError, match="depth"):
        math3d.deproject_depth_pixel(
            320, 240, 1000, intrinsics, depth_scale_m=depth_scale_m
        )


@pytest.mark.parametrize(
    "key,value",
    [("fx", 0.0), ("fx", -1.0), ("fy", 0.0), ("fy", -1.0)],
)
def test_deprojection_rejects_non_positive_focal_lengths(key, value):
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}
    intrinsics[key] = value

    with pytest.raises(ValueError, match="focal lengths"):
        math3d.deproject_depth_pixel(
            320, 240, 1000, intrinsics, depth_scale_m=0.001
        )


@pytest.mark.parametrize(
    "bounds",
    [
        {"min_depth_m": 1.01},
        {"max_depth_m": 0.99},
    ],
)
def test_deprojection_rejects_depth_outside_configured_range(bounds):
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}

    with pytest.raises(ValueError, match="configured range"):
        math3d.deproject_depth_pixel(
            320, 240, 1000, intrinsics, depth_scale_m=0.001, **bounds
        )


def test_deprojection_accepts_depth_at_configured_range_boundaries():
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}

    point = math3d.deproject_depth_pixel(
        320,
        240,
        1000,
        intrinsics,
        depth_scale_m=0.001,
        min_depth_m=1.0,
        max_depth_m=1.0,
    )

    np.testing.assert_allclose(point, [0.0, 0.0, 1.0])


def test_depth_pixel_to_base_uses_base_flange_then_flange_depth():
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}
    T_flange_depth = np.eye(4)
    T_flange_depth[:3, 3] = [0.0, 0.0, 0.1]
    T_base_flange = np.eye(4)
    T_base_flange[:3, 3] = [0.5, 0.0, 0.0]

    point = math3d.depth_pixel_to_base(
        320,
        240,
        1000,
        intrinsics,
        0.001,
        T_flange_depth,
        T_base_flange,
    )

    np.testing.assert_allclose(point, [0.5, 0.0, 1.1])
