"""Hardware-independent tests for Orbbec hand-eye geometry helpers."""

import builtins
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import zipfile

import numpy as np
import pytest


NERO_DEMO_DIR = Path(__file__).resolve().parents[1] / "pyAgxArm" / "demos" / "nero"
sys.path.insert(0, str(NERO_DEMO_DIR))

import orbbec_handeye_math as math3d  # noqa: E402


def _sample(flange_pose, target_rvec=(0.0, 0.0, 0.0), target_tvec=(0.0, 0.0, 1.0), timestamp=1.0):
    return {
        "flange_pose": list(flange_pose),
        "target_rvec": np.asarray(target_rvec, dtype=np.float64).reshape(3, 1),
        "target_tvec": np.asarray(target_tvec, dtype=np.float64).reshape(3, 1),
        "timestamp": timestamp,
    }


def _diverse_samples():
    return [
        _sample([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], timestamp=1.0),
        _sample([0.1, 0.0, 0.0, 0.2, 0.0, 0.0], timestamp=2.0),
        _sample([0.0, 0.1, 0.0, 0.0, -0.25, 0.1], timestamp=3.0),
    ]


def _same_axis_samples():
    return [
        _sample([0.00, 0.00, 0.00, 0.00, 0.00, 0.00], timestamp=1.0),
        _sample([0.10, 0.02, 0.00, 0.20, 0.00, 0.00], timestamp=2.0),
        _sample([0.00, 0.08, 0.03, -0.30, 0.00, 0.00], timestamp=3.0),
        _sample([-0.04, 0.01, 0.07, 0.45, 0.00, 0.00], timestamp=4.0),
    ]


def _write_sample_archive(
    path,
    camera_fingerprint=np.asarray("camera-1"),
    flange_poses=None,
):
    if flange_poses is None:
        flange_poses = np.zeros((0, 6), dtype=np.float64)
    np.savez_compressed(
        path,
        flange_poses=flange_poses,
        target_rvecs=np.zeros((0, 3, 1), dtype=np.float64),
        target_tvecs=np.zeros((0, 3, 1), dtype=np.float64),
        timestamps=np.zeros((0,), dtype=np.float64),
        camera_fingerprint=camera_fingerprint,
        camera_metadata_json=np.asarray(json.dumps({"camera_fingerprint": "camera-1"})),
    )


def _synthetic_handeye_samples():
    cv2 = pytest.importorskip("cv2")
    if not hasattr(cv2, "calibrateHandEye"):
        pytest.skip("OpenCV build does not provide calibrateHandEye")
    T_flange_color = math3d.pose6_to_matrix([0.05, 0.01, 0.08, 0.1, -0.05, 0.2])
    T_base_target = math3d.pose6_to_matrix([0.7, -0.2, 0.5, -0.3, 0.2, 0.4])
    flange_poses = [
        [0.00, 0.00, 0.00, 0.00, 0.00, 0.00],
        [0.10, -0.03, 0.02, 0.25, 0.10, -0.15],
        [-0.04, 0.08, 0.06, -0.30, 0.20, 0.25],
        [0.03, 0.06, -0.04, 0.15, -0.35, 0.30],
        [-0.08, -0.02, 0.10, -0.20, -0.15, -0.35],
        [0.06, 0.04, 0.03, 0.35, 0.25, 0.10],
    ]
    samples = []
    for index, flange_pose in enumerate(flange_poses):
        T_color_target = math3d.invert_transform(
            math3d.pose6_to_matrix(flange_pose) @ T_flange_color
        ) @ T_base_target
        rvec, _ = cv2.Rodrigues(T_color_target[:3, :3])
        samples.append(_sample(flange_pose, rvec, T_color_target[:3, 3], index + 1.0))
    return samples, T_flange_color


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
        ((float("nan"), 7), 0.02),
        ((float("inf"), 7), 0.02),
        ((-float("inf"), 7), 0.02),
        ((10.5, 7), 0.02),
        ((10, 6.5), 0.02),
        (("10", 7), 0.02),
        ((True, 7), 0.02),
        ((10, False), 0.02),
        ((10,), 0.02),
        ((10, 7, 1), 0.02),
        ((10, 7), 0.0),
        ((10, 7), -0.02),
        ((10, 7), float("nan")),
        ((10, 7), float("inf")),
    ],
)
def test_checkerboard_rejects_invalid_dimensions_and_size(
    checkerboard, square_size_m
):
    with pytest.raises(ValueError, match="Checkerboard dimensions"):
        math3d.create_checkerboard_object_points(checkerboard, square_size_m)


def test_checkerboard_accepts_integral_numpy_scalar_dimensions():
    points = math3d.create_checkerboard_object_points(
        (np.int64(10), np.int32(7)), 0.02
    )

    assert points.shape == (70, 3)


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


def test_rigid_transform_normalizes_valid_float32_calibration_rotation():
    transform = np.array(
        [
            [0.999997258, -0.002327133, 0.000345531, -0.012587452],
            [0.002327119, 0.999997318, 0.000041355, 0.000032290],
            [-0.000345626, -0.000040551, 0.999999940, -0.001149079],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )

    normalized = math3d._validate_rigid_transform(transform)

    np.testing.assert_allclose(normalized[:3, 3], transform[:3, 3], atol=0.0)
    np.testing.assert_allclose(
        normalized[:3, :3].T @ normalized[:3, :3],
        np.eye(3),
        rtol=0.0,
        atol=1e-12,
    )
    assert np.linalg.det(normalized[:3, :3]) == pytest.approx(1.0, abs=1e-12)


@pytest.mark.parametrize(
    "transform,error_pattern",
    [
        pytest.param(np.eye(3), "shape", id="wrong-shape"),
        pytest.param(
            np.diag([float("nan"), 1.0, 1.0, 1.0]),
            "finite",
            id="nonfinite",
        ),
        pytest.param(
            np.array(
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [1.0, 0.0, 0.0, 1.0],
                ]
            ),
            "bottom row",
            id="bad-bottom-row",
        ),
        pytest.param(
            np.diag([2.0, 1.0, 1.0, 1.0]),
            "orthonormal",
            id="scaled-rotation",
        ),
        pytest.param(
            np.diag([-1.0, 1.0, 1.0, 1.0]),
            "determinant",
            id="reflection",
        ),
    ],
)
@pytest.mark.parametrize(
    "operation", ["matrix_to_pose6", "invert_transform", "transform_point"]
)
def test_geometry_helpers_reject_malformed_rigid_transforms(
    operation, transform, error_pattern
):
    function = getattr(math3d, operation)

    with pytest.raises(ValueError, match=error_pattern):
        if operation == "transform_point":
            function(transform, [0.0, 0.0, 0.0])
        else:
            function(transform)


def test_depth_pixel_to_camera_point_converts_raw_depth_to_metres():
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}

    point = math3d.deproject_depth_pixel(
        420, 290, 1000, intrinsics, depth_scale_m=0.001
    )

    np.testing.assert_allclose(point, [0.2, 0.1, 1.0])


@pytest.mark.parametrize("depth_raw", [0, -1, float("nan"), float("inf")])
def test_deprojection_rejects_nonfinite_or_nonpositive_raw_depth(depth_raw):
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}

    with pytest.raises(ValueError, match="depth_raw"):
        math3d.deproject_depth_pixel(
            320, 240, depth_raw, intrinsics, depth_scale_m=0.001
        )


@pytest.mark.parametrize(
    "depth_scale_m", [0, -0.001, float("nan"), float("inf")]
)
def test_deprojection_rejects_nonfinite_or_nonpositive_depth_scale(depth_scale_m):
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}

    with pytest.raises(ValueError, match="depth_scale_m"):
        math3d.deproject_depth_pixel(
            320, 240, 1000, intrinsics, depth_scale_m=depth_scale_m
        )


def test_deprojection_rejects_negative_raw_depth_with_negative_scale():
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}

    with pytest.raises(ValueError, match="depth_raw"):
        math3d.deproject_depth_pixel(
            320, 240, -1000, intrinsics, depth_scale_m=-0.001
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


@pytest.mark.parametrize("key", ["fx", "fy"])
@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_deprojection_rejects_nonfinite_focal_lengths(key, value):
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}
    intrinsics[key] = value

    with pytest.raises(ValueError, match="finite and positive"):
        math3d.deproject_depth_pixel(
            320, 240, 1000, intrinsics, depth_scale_m=0.001
        )


@pytest.mark.parametrize("field", ["u", "v", "cx", "cy"])
@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_deprojection_rejects_nonfinite_pixel_coordinates(field, value):
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}
    arguments = {"u": 320.0, "v": 240.0}
    if field in intrinsics:
        intrinsics[field] = value
    else:
        arguments[field] = value

    with pytest.raises(ValueError, match=field):
        math3d.deproject_depth_pixel(
            arguments["u"],
            arguments["v"],
            1000,
            intrinsics,
            depth_scale_m=0.001,
        )


@pytest.mark.parametrize(
    "min_depth_m", [float("nan"), float("inf"), -float("inf"), -0.01]
)
def test_deprojection_rejects_invalid_minimum_depth_bound(min_depth_m):
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}

    with pytest.raises(ValueError, match="min_depth_m"):
        math3d.deproject_depth_pixel(
            320,
            240,
            1000,
            intrinsics,
            depth_scale_m=0.001,
            min_depth_m=min_depth_m,
        )


@pytest.mark.parametrize(
    "max_depth_m", [float("nan"), float("inf"), -float("inf")]
)
def test_deprojection_rejects_nonfinite_maximum_depth_bound(max_depth_m):
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}

    with pytest.raises(ValueError, match="max_depth_m"):
        math3d.deproject_depth_pixel(
            320,
            240,
            1000,
            intrinsics,
            depth_scale_m=0.001,
            max_depth_m=max_depth_m,
        )


def test_deprojection_rejects_maximum_depth_below_minimum():
    intrinsics = {"fx": 500.0, "fy": 500.0, "cx": 320.0, "cy": 240.0}

    with pytest.raises(ValueError, match="max_depth_m"):
        math3d.deproject_depth_pixel(
            320,
            240,
            1000,
            intrinsics,
            depth_scale_m=0.001,
            min_depth_m=1.0,
            max_depth_m=0.5,
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
    T_flange_depth[:3, :3] = [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ]
    T_flange_depth[:3, 3] = [0.1, 0.2, 0.3]
    T_base_flange = np.eye(4)
    T_base_flange[:3, :3] = [
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
    T_base_flange[:3, 3] = [0.5, -0.1, 0.2]

    point = math3d.depth_pixel_to_base(
        420,
        290,
        1000,
        intrinsics,
        0.001,
        T_flange_depth,
        T_base_flange,
    )

    # p_depth=[0.2, 0.1, 1.0], p_flange=[0.3, -0.8, 0.4].
    np.testing.assert_allclose(point, [1.3, 0.2, 0.6])


def test_sample_persistence_round_trip_preserves_metadata_and_float64(tmp_path):
    samples = _diverse_samples()
    metadata = {
        "camera_fingerprint": "orbbec:abc123",
        "serial_number": "ABC123",
        "intrinsics": {"fx": 600.0, "fy": 601.0},
    }
    path = tmp_path / "samples.npz"

    math3d.save_samples(path, samples, metadata)
    loaded, loaded_metadata = math3d.load_samples(path, "orbbec:abc123")

    assert loaded_metadata == metadata
    assert len(loaded) == len(samples)
    assert loaded[0]["flange_pose"] == samples[0]["flange_pose"]
    assert loaded[0]["target_rvec"].shape == (3, 1)
    assert loaded[0]["target_tvec"].shape == (3, 1)
    assert loaded[0]["target_rvec"].dtype == np.float64
    assert loaded[0]["target_tvec"].dtype == np.float64
    assert loaded[0]["timestamp"] == 1.0


def test_sample_persistence_round_trips_empty_samples_with_deterministic_shapes(tmp_path):
    path = tmp_path / "empty.npz"
    math3d.save_samples(path, [], {"camera_fingerprint": "camera-1"})

    with np.load(path, allow_pickle=False) as archive:
        assert archive["flange_poses"].shape == (0, 6)
        assert archive["target_rvecs"].shape == (0, 3, 1)
        assert archive["target_tvecs"].shape == (0, 3, 1)
        assert archive["timestamps"].shape == (0,)

    samples, metadata = math3d.load_samples(path)
    assert samples == []
    assert metadata == {"camera_fingerprint": "camera-1"}


def test_save_samples_write_failure_preserves_existing_archive_and_cleans_temp(
    tmp_path, monkeypatch
):
    path = tmp_path / "samples.npz"
    previous_samples = [_sample([1.0, 2.0, 3.0, 0.1, 0.2, 0.3], timestamp=4.0)]
    previous_metadata = {"camera_fingerprint": "previous-camera", "serial": "OLD"}
    math3d.save_samples(path, previous_samples, previous_metadata)
    previous_bytes = path.read_bytes()

    def fail_after_temp_creation(temp_path, **fields):
        Path(temp_path).write_bytes(b"partial archive")
        raise OSError("simulated write failure")

    monkeypatch.setattr(np, "savez_compressed", fail_after_temp_creation)

    with pytest.raises(OSError, match="simulated write failure"):
        math3d.save_samples(
            path,
            _diverse_samples(),
            {"camera_fingerprint": "new-camera", "serial": "NEW"},
        )

    assert path.read_bytes() == previous_bytes
    loaded, metadata = math3d.load_samples(path, "previous-camera")
    assert metadata == previous_metadata
    assert loaded[0]["flange_pose"] == previous_samples[0]["flange_pose"]
    assert list(tmp_path.iterdir()) == [path]


def test_save_samples_replaces_existing_archive_and_cleans_temp(tmp_path):
    path = tmp_path / "samples.npz"
    math3d.save_samples(
        path,
        [_sample([1.0, 2.0, 3.0, 0.1, 0.2, 0.3], timestamp=4.0)],
        {"camera_fingerprint": "previous-camera", "serial": "OLD"},
    )

    new_samples = _diverse_samples()
    new_metadata = {"camera_fingerprint": "new-camera", "serial": "NEW"}
    math3d.save_samples(path, new_samples, new_metadata)

    loaded, metadata = math3d.load_samples(path, "new-camera")
    assert metadata == new_metadata
    assert len(loaded) == len(new_samples)
    assert loaded[0]["flange_pose"] == new_samples[0]["flange_pose"]
    assert list(tmp_path.iterdir()) == [path]


def test_sample_persistence_rejects_fingerprint_mismatch_and_missing_fields(tmp_path):
    path = tmp_path / "samples.npz"
    math3d.save_samples(path, _diverse_samples(), {"camera_fingerprint": "camera-1"})

    with pytest.raises(ValueError, match="fingerprint"):
        math3d.load_samples(path, expected_fingerprint="other-camera")

    corrupt_path = tmp_path / "corrupt.npz"
    np.savez_compressed(corrupt_path, flange_poses=np.zeros((0, 6)))
    with pytest.raises(ValueError, match="missing required field"):
        math3d.load_samples(corrupt_path)


def test_sample_persistence_rejects_truncated_archive_with_value_error(tmp_path):
    path = tmp_path / "truncated.npz"
    math3d.save_samples(path, _diverse_samples(), {"camera_fingerprint": "camera-1"})
    path.write_bytes(path.read_bytes()[:20])

    with pytest.raises(ValueError, match="could not load sample archive"):
        math3d.load_samples(path)


def test_sample_persistence_rejects_non_serializable_metadata(tmp_path):
    with pytest.raises(ValueError, match="JSON-serializable"):
        math3d.save_samples(
            tmp_path / "samples.npz",
            [],
            {"camera_fingerprint": "camera-1", "unsupported": {1, 2}},
        )


def test_sample_persistence_rejects_extra_archive_members_before_loading(tmp_path):
    path = tmp_path / "extra-member.npz"
    _write_sample_archive(path)
    with zipfile.ZipFile(path, "a") as archive:
        archive.writestr("unexpected.npy", b"not an array")

    with pytest.raises(ValueError, match="unexpected member"):
        math3d.load_samples(path)


def test_sample_persistence_rejects_archive_member_over_resource_limit(tmp_path):
    path = tmp_path / "oversized-member.npz"
    _write_sample_archive(path, flange_poses=np.zeros((50000, 6), dtype=np.float64))

    with pytest.raises(ValueError, match="resource limit"):
        math3d.load_samples(path)


def test_sample_persistence_rejects_non_string_scalar_fingerprint(tmp_path):
    path = tmp_path / "integer-fingerprint.npz"
    _write_sample_archive(path, camera_fingerprint=np.asarray(123))

    with pytest.raises(ValueError, match="fingerprint.*string"):
        math3d.load_samples(path)


def test_validate_sample_motion_requires_count_and_rotation_diversity():
    with pytest.raises(ValueError, match="at least 3"):
        math3d.validate_sample_motion(_diverse_samples()[:2])

    repeated = [_sample([0.02 * index, 0.0, 0.0, 0, 0, 0]) for index in range(3)]
    with pytest.raises(ValueError, match="rotation diversity.*15-30"):
        math3d.validate_sample_motion(repeated)


def test_validate_sample_motion_reports_diverse_motion_summary():
    summary = math3d.validate_sample_motion(_diverse_samples())

    assert summary["sample_count"] == 3
    assert summary["max_rotation_deg"] > 5.0
    assert summary["translation_span_m"] > 0.0


def test_validate_sample_motion_rejects_rotations_about_only_one_axis():
    with pytest.raises(ValueError, match="non-collinear axes.*observability"):
        math3d.validate_sample_motion(_same_axis_samples())


def test_calibration_rejects_same_axis_rotations_before_opencv(monkeypatch):
    fake_cv2 = SimpleNamespace(
        CALIB_HAND_EYE_TSAI=0,
        CALIB_HAND_EYE_PARK=1,
        CALIB_HAND_EYE_HORAUD=2,
        CALIB_HAND_EYE_ANDREFF=3,
        CALIB_HAND_EYE_DANIILIDIS=4,
        calibrateHandEye=lambda *args, **kwargs: pytest.fail("OpenCV must not run"),
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    with pytest.raises(ValueError, match="non-collinear axes.*observability"):
        math3d.calibrate_eye_in_hand(
            _same_axis_samples(), np.eye(4), {"camera_fingerprint": "camera-1"}
        )


def test_validate_sample_motion_reports_conditioning_for_synthetic_samples():
    samples, _ = _synthetic_handeye_samples()

    summary = math3d.validate_sample_motion(samples)

    assert summary["rotation_axis_conditioning"] >= 0.05


def test_named_transform_serializes_inverse_and_normalized_quaternion():
    transform = math3d.make_transform(
        math3d.pose6_to_matrix([0, 0, 0, 0.2, -0.1, 0.4])[:3, :3],
        [0.1, -0.2, 0.3],
    )
    named = math3d.named_transform("flange", "color", transform)
    inverse = math3d.named_transform("color", "flange", math3d.invert_transform(transform))

    assert named["parent_frame"] == "flange"
    assert named["child_frame"] == "color"
    assert len(named["matrix_row_major_4x4"]) == 16
    assert np.isclose(np.linalg.norm(named["quaternion_xyzw"]), 1.0)
    assert np.all(np.isfinite(np.asarray(named["pose6_xyz_rpy"])))
    np.testing.assert_allclose(
        np.asarray(named["matrix_row_major_4x4"]).reshape(4, 4)
        @ np.asarray(inverse["matrix_row_major_4x4"]).reshape(4, 4),
        np.eye(4),
        atol=1e-15,
    )


@pytest.mark.parametrize(
    "method_name,atol",
    [
        ("TSAI", 1e-7),
        ("PARK", 1e-7),
        ("HORAUD", 1e-7),
        ("ANDREFF", 1e-7),
        ("DANIILIDIS", 1e-6),
    ],
)
def test_calibrate_eye_in_hand_recovers_deterministic_synthetic_transform(method_name, atol):
    samples, expected_T_flange_color = _synthetic_handeye_samples()
    T_color_depth = math3d.pose6_to_matrix([0.01, -0.02, 0.03, 0.03, 0.01, -0.02])

    result = math3d.calibrate_eye_in_hand(
        samples,
        T_color_depth,
        {"camera_fingerprint": "camera-1"},
        method_name=method_name,
    )

    recovered = np.asarray(result["T_flange_color"]["matrix_row_major_4x4"]).reshape(4, 4)
    np.testing.assert_allclose(recovered, expected_T_flange_color, atol=atol)
    recovered_depth = np.asarray(result["T_flange_depth"]["matrix_row_major_4x4"]).reshape(4, 4)
    np.testing.assert_allclose(
        recovered_depth, expected_T_flange_color @ T_color_depth, atol=atol
    )


def test_calibration_reports_actionable_error_when_opencv_import_is_blocked(monkeypatch):
    original_import = builtins.__import__

    def blocked_import(name, *args, **kwargs):
        if name == "cv2":
            raise ImportError("blocked for test")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", blocked_import)

    with pytest.raises(ValueError, match="OpenCV is required"):
        math3d.calibrate_eye_in_hand(
            _diverse_samples(), np.eye(4), {"camera_fingerprint": "camera-1"}
        )


def test_calibration_result_has_schema_warning_and_near_zero_consistency():
    samples, _ = _synthetic_handeye_samples()
    result = math3d.calibrate_eye_in_hand(
        samples,
        np.eye(4),
        {"camera_fingerprint": "camera-1"},
    )

    assert result["schema_version"] == 1
    assert result["sample_count"] == 6
    assert result["checkerboard"] == {"inner_corners": [10, 7], "square_size_m": 0.02}
    assert result["quality_warnings"]
    assert result["consistency"]["translation_max_m"] < 1e-7
    assert result["consistency"]["rotation_max_deg"] < 1e-5
    assert all(np.isfinite(value) for value in result["consistency"].values())


def test_calibration_rejects_unknown_method_and_nonfinite_sample_data():
    samples = _diverse_samples()
    with pytest.raises(ValueError, match="method"):
        math3d.calibrate_eye_in_hand(
            samples, np.eye(4), {"camera_fingerprint": "camera-1"}, method_name="BAD"
        )

    samples[0]["target_tvec"][0, 0] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        math3d.calibrate_eye_in_hand(
            samples, np.eye(4), {"camera_fingerprint": "camera-1"}
        )
