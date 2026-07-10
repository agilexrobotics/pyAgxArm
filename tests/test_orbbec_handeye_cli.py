"""Hardware-free tests for the interactive Orbbec hand-eye CLI."""

import importlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest


NERO_DEMO_DIR = Path(__file__).resolve().parents[1] / "pyAgxArm" / "demos" / "nero"
sys.path.insert(0, str(NERO_DEMO_DIR))

import orbbec_handeye_calib as cli  # noqa: E402


def _metadata(fingerprint="camera-1"):
    return {
        "camera_fingerprint": fingerprint,
        "color_profile": "1280x720@30_RGB",
        "depth_profile": "640x576@30_Y16",
        "color_intrinsics": {
            "width": 1280,
            "height": 720,
            "fx": 900.0,
            "fy": 901.0,
            "cx": 640.0,
            "cy": 360.0,
        },
        "color_distortion": [0.1, -0.1, 0.0, 0.0, 0.01],
        "depth_scale_m": 0.001,
        "T_color_depth_matrix": np.eye(4).reshape(-1).tolist(),
    }


def _detection():
    return (
        True,
        np.array([[0.1], [0.2], [0.3]], dtype=np.float64),
        np.array([[0.4], [0.5], [0.6]], dtype=np.float64),
        np.zeros((70, 1, 2), dtype=np.float32),
    )


def _sample(timestamp=1.0):
    return {
        "flange_pose": [0.0] * 6,
        "target_rvec": np.zeros((3, 1), dtype=np.float64),
        "target_tvec": np.array([[0.0], [0.0], [1.0]], dtype=np.float64),
        "timestamp": timestamp,
    }


def test_parser_defaults_and_handeye_method_choices():
    args = cli.build_arg_parser().parse_args([])

    assert (args.checkerboard_cols, args.checkerboard_rows, args.square_size) == (
        10,
        7,
        0.02,
    )
    assert args.samples == Path("orbbec_handeye_samples.npz")
    assert args.output == Path("orbbec_handeye_result.json")
    assert args.method == "TSAI"
    assert (args.width, args.height, args.fps) == (1280, 720, 30)
    with pytest.raises(SystemExit):
        cli.build_arg_parser().parse_args(["--method", "NOPE"])


def test_import_and_help_do_not_load_optional_sdk_or_robot(monkeypatch, capsys):
    real_import = __import__

    def guarded_import(name, *args, **kwargs):
        if name in {"pyorbbecsdk", "pyAgxArm"}:
            raise AssertionError("optional dependency imported eagerly: {}".format(name))
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr("builtins.__import__", guarded_import)
    sys.modules.pop("orbbec_handeye_calib", None)
    module = importlib.import_module("orbbec_handeye_calib")

    with pytest.raises(SystemExit) as exc:
        module.main(["--help"])
    assert exc.value.code == 0
    assert "--calibrate-only" in capsys.readouterr().out


def test_capture_sample_requires_current_valid_detection_and_normalizes_values():
    with pytest.raises(ValueError, match="current"):
        cli.capture_sample(None, [0.0] * 6, 1.0)

    sample = cli.capture_sample(_detection(), np.arange(6), 4.5)

    assert sample["flange_pose"] == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert sample["target_rvec"].shape == (3, 1)
    assert sample["target_tvec"].shape == (3, 1)
    assert sample["timestamp"] == 4.5


def test_get_flange_pose6_validates_exactly_six_finite_numbers():
    robot = SimpleNamespace(
        get_flange_pose=lambda: SimpleNamespace(msg=[1, 2, 3, 4, 5, 6])
    )
    assert cli.get_flange_pose6(robot) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

    for pose in ([1, 2], [1, 2, 3, 4, 5, float("nan")]):
        robot = SimpleNamespace(get_flange_pose=lambda pose=pose: SimpleNamespace(msg=pose))
        with pytest.raises(RuntimeError, match="six finite"):
            cli.get_flange_pose6(robot)


def test_camera_matrix_and_distortion_normalize_valid_metadata():
    matrix, distortion = cli.camera_matrix_and_distortion(_metadata())

    np.testing.assert_allclose(
        matrix, [[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]]
    )
    assert distortion.shape in {(5,), (5, 1)}
    with pytest.raises(ValueError):
        cli.camera_matrix_and_distortion({})
    invalid = _metadata()
    invalid["color_distortion"] = [[0.0, 0.0], [0.0, 0.0]]
    with pytest.raises(ValueError, match="distortion"):
        cli.camera_matrix_and_distortion(invalid)


def test_detect_checkerboard_pose_returns_current_pnp_solution():
    checkerboard = (10, 7)
    square_px = 48
    image = np.full((8 * square_px + 50, 11 * square_px + 50, 3), 255, dtype=np.uint8)
    for row in range(8):
        for col in range(11):
            if (row + col) % 2:
                cv2.rectangle(
                    image,
                    (25 + col * square_px, 25 + row * square_px),
                    (25 + (col + 1) * square_px, 25 + (row + 1) * square_px),
                    (0, 0, 0),
                    -1,
                )
    matrix = np.array([[1000.0, 0.0, image.shape[1] / 2], [0.0, 1000.0, image.shape[0] / 2], [0.0, 0.0, 1.0]])

    ok, rvec, tvec, corners = cli.detect_checkerboard_pose(
        image, matrix, np.zeros(5), checkerboard, 0.02
    )

    assert ok is True
    assert corners.shape[0] == 70
    assert np.all(np.isfinite(rvec))
    assert float(tvec[2]) > 0.0


def test_failed_detection_returns_no_stale_pose():
    image = np.full((320, 320, 3), 127, dtype=np.uint8)
    ok, rvec, tvec, corners = cli.detect_checkerboard_pose(
        image, np.eye(3), np.zeros(5), (10, 7), 0.02
    )

    assert (ok, rvec, tvec, corners) == (False, None, None, None)


def test_json_writer_creates_parent_and_rejects_non_finite_values(tmp_path):
    destination = tmp_path / "results" / "result.json"
    cli.write_result_json(destination, {"value": 1.0})

    assert json.loads(destination.read_text(encoding="utf-8")) == {"value": 1.0}
    with pytest.raises(ValueError, match="finite"):
        cli.write_result_json(destination, {"value": float("nan")})


def test_calibrate_only_uses_persisted_metadata_without_hardware(tmp_path, monkeypatch):
    sample_path = tmp_path / "samples.npz"
    output_path = tmp_path / "result.json"
    captured = {}

    monkeypatch.setattr(cli, "load_samples", lambda path: ([_sample()], _metadata()))
    monkeypatch.setattr(
        cli,
        "calibrate_eye_in_hand",
        lambda samples, transform, metadata, **kwargs: captured.update(
            samples=samples, transform=transform, metadata=metadata, kwargs=kwargs
        )
        or {"result": "ok"},
    )
    monkeypatch.setattr(cli, "create_nero_robot", lambda args: pytest.fail("robot used"))
    monkeypatch.setattr(cli, "create_camera", lambda args: pytest.fail("camera used"))

    assert cli.main(["--calibrate-only", "--samples", str(sample_path), "--output", str(output_path)]) == 0
    assert captured["metadata"]["camera_fingerprint"] == "camera-1"
    np.testing.assert_allclose(captured["transform"], np.eye(4))
    assert json.loads(output_path.read_text(encoding="utf-8")) == {"result": "ok"}


def test_collection_key_handler_rejects_stale_detection_and_preserves_metadata(tmp_path, monkeypatch):
    calls = []
    metadata = _metadata()
    robot = SimpleNamespace(get_flange_pose=lambda: SimpleNamespace(msg=[0] * 6))
    args = cli.build_arg_parser().parse_args(["--samples", str(tmp_path / "samples.npz")])

    monkeypatch.setattr(cli, "save_samples", lambda path, samples, saved_metadata: calls.append((path, samples, saved_metadata)))

    assert cli.handle_collection_key(ord("s"), None, robot, [], metadata, args) is False
    samples = []
    assert cli.handle_collection_key(ord("s"), _detection(), robot, samples, metadata, args) is True
    assert len(samples) == 1
    assert calls[0][2] == metadata
    assert cli.handle_collection_key(ord("q"), _detection(), robot, samples, metadata, args) is False


def test_collection_cleanup_runs_when_frame_processing_raises(monkeypatch):
    events = []

    class Camera:
        metadata = _metadata()
        color_profile = "1280x720@30_RGB"
        depth_profile = "640x576@30_Y16"

        def start(self):
            events.append("start")
            return self

        def wait_for_frames(self):
            return SimpleNamespace(color_bgr=np.zeros((20, 20, 3), dtype=np.uint8))

        def stop(self):
            events.append("stop")

    robot = SimpleNamespace(disconnect=lambda: events.append("disconnect"))
    monkeypatch.setattr(cli, "create_camera", lambda args: Camera())
    monkeypatch.setattr(cli, "create_nero_robot", lambda args: robot)
    monkeypatch.setattr(cli, "load_existing_samples", lambda *args: [])
    monkeypatch.setattr(cli, "destroy_windows", lambda: events.append("windows"))
    monkeypatch.setattr(
        cli,
        "run_frame_loop",
        lambda *args: (_ for _ in ()).throw(RuntimeError("frame failure")),
    )

    with pytest.raises(RuntimeError, match="frame failure"):
        cli.run_collection(cli.build_arg_parser().parse_args([]))
    assert events == ["start", "disconnect", "stop", "windows"]
