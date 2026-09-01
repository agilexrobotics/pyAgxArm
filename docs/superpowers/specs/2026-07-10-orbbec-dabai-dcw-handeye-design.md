# Nero + Orbbec DaBai DCW Eye-in-Hand Calibration Design

## Goal

Add an eye-in-hand calibration workflow for an Orbbec DaBai DCW mounted on the
Nero flange. The workflow must produce the transforms and camera parameters
needed to convert a raw depth pixel or depth-camera 3D point into the Nero base
coordinate system.

The calibration target is a fixed checkerboard with 10 x 7 inner corners and a
0.02 m square size.

## Scope

The implementation will:

- use the Orbbec SDK v1 Python wrapper from the official `pyorbbecsdk` `main`
  branch;
- acquire synchronized color and depth frames from the DaBai DCW;
- read color/depth intrinsics, distortion, depth scale, and depth-to-color
  extrinsics from the SDK;
- detect the checkerboard in the color image and pair each observation with the
  current Nero flange pose;
- calculate the camera-to-flange hand-eye transform with OpenCV;
- derive the depth-camera-to-flange transform using the SDK extrinsics;
- save all parameters in a versioned JSON result;
- provide testable helpers for converting depth pixels and depth-camera points
  into flange and base coordinates.

The implementation will not perform autonomous robot motion, checkerboard
printing, firmware upgrades, or object detection and grasp planning.

## Dependencies

The target environment is Ubuntu 22.04 x86_64 with Python 3.10. The Orbbec
DaBai DCW is an Orbbec SDK v1/OpenNI-protocol device. The official
`pyorbbecsdk` `main` branch will be built for the active `pyagxarm` environment.

Required runtime packages are:

- `pyorbbecsdk` v1;
- `opencv-contrib-python`;
- `numpy`;
- the existing `pyAgxArm` package and Linux SocketCAN support.

SDK installation and udev-rule setup will be documented separately from the
calibration program. Missing SDK, permissions, camera streams, or CAN devices
must produce actionable error messages.

## Coordinate Convention

`T_A_B` is a 4 x 4 homogeneous transform that maps a point expressed in frame
`B` into frame `A`:

```text
p_A = T_A_B * p_B
```

All translations and 3D points are represented in metres. Homogeneous points
are column vectors.

The relevant frames are:

- `base`: Nero base frame;
- `flange`: Nero flange frame;
- `color`: Orbbec color optical frame;
- `depth`: Orbbec depth optical frame;
- `target`: checkerboard frame.

The Nero driver supplies `T_base_flange`. OpenCV hand-eye calibration supplies
`T_flange_color`. The Orbbec SDK supplies `T_color_depth`. Therefore:

```text
T_flange_depth = T_flange_color * T_color_depth
p_base = T_base_flange * T_flange_depth * p_depth
```

The implementation will normalize and validate the SDK extrinsic direction at
its adapter boundary. Both forward and inverse transforms will be written to
the result file with explicit frame names.

## Components

### Orbbec adapter

A small adapter isolates SDK-specific API calls from calibration math. It will:

- select the requested device by serial number, or the only attached Orbbec
  device when no serial is supplied;
- configure color and depth profiles;
- return synchronized BGR color images and metric depth frames;
- expose normalized color/depth intrinsics and `T_color_depth`;
- convert SDK frame formats to NumPy arrays;
- close the pipeline reliably on normal exit and errors.

Depth is retained in its native optical frame for transform correctness. Color
alignment may be displayed for diagnostics, but it will not silently change the
coordinate frame used by conversion helpers.

### Calibration collector

The collector connects to Nero through the configured SocketCAN channel and
opens the Orbbec adapter. For each color frame it detects the 10 x 7 inner
corners, refines them to sub-pixel precision, and estimates `T_color_target`
with the SDK color intrinsics and `solvePnP`.

Keyboard controls remain:

- `s`: save the current valid checkerboard observation and flange pose;
- `c`: calculate calibration and write the result;
- `q`: close the camera, robot connection, and window.

A saved sample contains the flange pose, target rotation/translation, timestamp,
and camera metadata fingerprint. Samples from incompatible stream profiles or
camera serial numbers are rejected. At least three samples are mathematically
required; fewer than fifteen produces a quality warning. The operator is told
to collect 15-30 poses with varied translation and rotation.

### Hand-eye solver

The solver uses `cv2.calibrateHandEye`, with Tsai as the default and the existing
OpenCV method choices retained. It computes `T_flange_color`, combines it with
the SDK `T_color_depth`, and reports both transforms and their inverses.

The solver also evaluates consistency of the fixed checkerboard across samples.
It reports translation and rotation dispersion so a numerically valid but poor
calibration is visible to the operator.

### Depth conversion helpers

Pure NumPy helpers will:

1. deproject `(u, v, depth_raw)` using the depth intrinsics and depth scale;
2. transform `p_depth` to `p_flange` with `T_flange_depth`;
3. obtain the current `T_base_flange` from Nero;
4. transform the point to `p_base`.

Invalid depth values (zero, non-finite, or outside the configured range) are
rejected instead of returning a plausible zero point.

## Result Format

The JSON output contains:

- schema version and creation timestamp;
- camera name, serial number, firmware version, and stream profiles;
- checkerboard dimensions and square size in metres;
- color and depth intrinsic matrices and distortion coefficients;
- depth unit scale in metres per raw unit;
- `T_color_depth` and `T_depth_color`;
- `T_flange_color` and `T_color_flange`;
- `T_flange_depth` and `T_depth_flange`;
- hand-eye method, sample count, and consistency metrics.

Every transform entry includes frame names, a row-major 4 x 4 matrix, XYZ/RPY,
and quaternion representation where applicable. The matrix is authoritative for
point conversion.

## CLI

The Orbbec calibration demo will preserve the existing checkerboard, CAN,
sample, output, resolution, frame-rate, serial-number, and hand-eye-method
options. Defaults are:

```text
checkerboard: 10 x 7 inner corners
square size: 0.02 m
CAN: socketcan / can_piper
color: 1280 x 720 at 30 FPS, with a supported-profile fallback
method: TSAI
```

The program prints the profile actually selected by the SDK. Unsupported
profiles fail with a list of available profiles or use an explicitly reported
fallback; they are never changed silently.

## Error Handling

Startup validates dependencies and hardware in this order:

1. OpenCV and Orbbec SDK imports;
2. Orbbec device discovery and permissions;
3. color/depth profile selection and calibration data;
4. SocketCAN device existence;
5. Nero connection and flange-pose availability.

Partial startup is unwound in reverse order. Calibration is not written when
camera metadata changes, the transform contains non-finite values, or sample
motion is degenerate.

## Testing

Hardware-independent tests cover:

- conversion of Orbbec intrinsics and extrinsics into normalized matrices;
- depth pixel deprojection and metre conversion;
- the full `depth -> color -> flange -> base` transform chain;
- transform inversion and frame-direction checks;
- result JSON schema fields and round-trip loading;
- invalid depth, incompatible sample metadata, and insufficient samples.

SDK-facing tests use lightweight fake SDK objects only at the adapter boundary.
Existing hand-eye math tests remain in place and are renamed for the Orbbec
workflow.

Hardware verification requires the real DaBai DCW and Nero:

- SDK device discovery and synchronized color/depth preview;
- checkerboard detection and 15-30 saved poses;
- calibration consistency metrics;
- a measured checkerboard point transformed into the base frame and compared
  against a physical measurement.

## Acceptance Criteria

The feature is complete when:

- the DaBai DCW color and depth streams open through Orbbec SDK v1;
- the 10 x 7, 0.02 m checkerboard can be sampled with Nero flange poses;
- the JSON result contains all camera calibration data and named transforms;
- a valid raw depth pixel can be converted to a metric Nero base-frame point;
- automated tests pass without connected hardware;
- hardware-only verification steps and any unverified limitations are clearly
  reported.
