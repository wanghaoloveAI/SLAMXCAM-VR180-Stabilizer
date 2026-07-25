# Technical Spec

## Product Goal

Build a Windows desktop stabilizer for SLAM XCAM VR180 3D footage. The app should accept raw fisheye footage plus 50 Hz or 200 Hz IMU logs, use lens calibration to stabilize motion in spherical/fisheye space, and export a stitched SBS VR180 3D video with metadata.

## Supported Inputs

1. Stitched SBS fisheye
   - one video file
   - expected aspect ratio: 2:1
   - left eye and right eye are side-by-side

2. Dual fisheye
   - two videos
   - each video expected aspect ratio: 1:1
   - one left-eye file and one right-eye file

3. IMU log
   - 50 Hz or 200 Hz
   - CSV initially
   - gyro in rad/s or deg/s
   - quaternion logs accepted when available

4. Calibration
   - per-eye fisheye intrinsics
   - distortion coefficients
   - stereo extrinsics
   - IMU-to-camera rotation

## Calibration Presets

The standard model-to-calibration mapping is private to the product build:

- SLAM XCAM 2025 -> local model-specific calibration
- SLAM XCAM 2026 -> local model-specific calibration

SLAM XCAM 2025 and 2026 share the same external IMU CSV schema and should use
the same IMU parsing, timestamp interpolation, and coordinate-conversion
pipeline. The model selector is only responsible for selecting lens distortion
and calibration data unless future hardware evidence shows a different physical
IMU mounting orientation.

The mapping is stored in `config/lenses/*.json` through the `default_calibration`
field. The pipeline loads that file automatically when the user does not provide
a manual calibration override.

Public documentation must not include sensor identifiers, source calibration
paths, exact intrinsics, distortion coefficients, or RMS quality values.

## Processing Model

The intended renderer is:

1. Decode video frames.
2. Split SBS into left/right views when needed.
3. Convert output pixel to VR180 ray.
4. Apply stabilized camera orientation from IMU.
5. Project ray through the calibrated fisheye model into source image coordinates.
6. Sample source image with interpolation.
7. Compose stabilized SBS output.
8. Encode and inject VR180 metadata.

For dual-input footage, step 2 becomes synchronized decode of left and right sources.
Both eyes must use the same IMU-derived orientation trajectory and the same
per-frame stabilization correction. The renderer may apply different lens
intrinsics/distortion maps per eye, but it must not compute independent left-eye
and right-eye stabilization curves because that would change stereo geometry and
damage VR180 3D comfort.

Detailed image-processing algorithm options and implementation order are in
[IMAGE_PROCESSING_ALGORITHMS.md](IMAGE_PROCESSING_ALGORITHMS.md).

## IMU Pipeline

Detailed strategy definitions are in
[IMU_PROCESSING_STRATEGIES.md](IMU_PROCESSING_STRATEGIES.md).

The external CSV schema is:

```csv
timestamp(us),acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z
```

The same schema may arrive as CSV or Excel workbook (`.xlsx` / `.xlsm`). The
workbook export uses the first sheet and the same header row.

Processing steps:

1. Probe video parameters before IMU processing.
2. Read width, height, layout, field of view, frame rate, frame count, frame
   duration, and stream duration.
3. Parse microsecond IMU timestamps, accelerometer samples, and gyro samples.
4. Normalize IMU timestamps relative to video start.
5. Integrate gyro to orientation.
6. Apply the shared SLAM XCAM IMU-to-camera coordinate conversion.
7. Interpolate the 200 Hz IMU trajectory onto the video frame timestamps.
8. Support both 30 fps and 50 fps video by using actual frame timestamps, not
   integer sample ratios.
9. Clean or clamp abnormal timestamp deltas before smoothing and frame sampling.
10. Smooth orientation trajectory.
11. Generate per-frame correction quaternions.

Stereo rule:

- Build one IMU orientation trajectory per clip.
- Sample one correction quaternion per video frame timestamp.
- Apply that same correction to both the left-eye and right-eye fisheye
  reprojection passes.
- Keep eye-specific differences limited to lens calibration, crop, projection,
  and stereo extrinsics.

Current prototype implements parsing and basic gyro integration only.

## Observed Sample Pair

Files:

- `Slam_20260620_162535_456.mp4`
- `Slam_20260620_162535_456_imu.csv`

Video:

- 7680 x 3840
- HEVC
- SBS 2:1 layout
- about 6.6188 s
- 189 frames
- about 28.555 fps
- includes Stereo 3D side data: side by side
- includes Spherical Mapping side data currently reported by FFmpeg as tiled equirectangular
- includes a `camm` data stream with 597 frames over about 6.5929 s

External IMU CSV:

- 338 samples
- about 6.7745 s
- about 49.746 Hz
- `timestamp(us),acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z`

Sync implication:

- the external CSV duration is about 0.1557 s longer than the video stream
- first implementation needs a user-adjustable IMU offset
- automatic sync can later compare visual motion, external CSV gyro, and embedded `camm` motion metadata

## Rendering Milestones

M1: Windows prototype
- GUI and CLI
- input validation
- IMU parsing
- lens/calibration loading
- FFmpeg remux with metadata
- stabilization plan JSON

M2: CPU renderer
- frame extraction/decode
- fisheye ray model
- per-frame remap
- SBS output

M3: GPU renderer
- wgpu/OpenCL/Vulkan path
- real-time preview
- encoder presets

M4: Production package
- `.exe` or installer
- signed Windows release
- batch queue
- calibration management

## Open Reference Projects

- VR180-Silver-Bullet: Rust/egui-style VR180 stabilization reference.
- Gyroflow: mature gyro stabilization architecture and lens profile concepts.
- FFmpeg: decode/encode/remux and hardware acceleration.
