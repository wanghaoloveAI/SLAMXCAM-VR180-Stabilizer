# SLAM XCAM IMU Processing Strategies

This document defines the first three IMU processing strategies for SLAM XCAM
VR180 3D stabilization.

## Shared Inputs

All strategies use the same SLAM XCAM IMU file format:

```csv
timestamp(us),acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z
```

The same IMU schema is used by SLAM XCAM 2025 and SLAM XCAM 2026. The model
selector only changes lens calibration and distortion parameters.

The IMU processor also receives video timing:

- `frame_rate`: 30 fps or 50 fps, read from ffprobe.
- `frame_count`: total decoded video frames.
- `duration_s`: video stream duration when available.
- `imu_offset_s`: user-adjustable offset from video time to IMU time.

The output must be one shared per-frame correction curve for both eyes:

```text
frame_index
video_time_s
imu_time_s = video_time_s + imu_offset_s
raw_orientation_q
smooth_orientation_q
correction_q
correction_matrix3
```

Both left and right eyes use the same `correction_q` for the same frame index.
Eye-specific differences are limited to calibration, distortion, crop, and
stereo extrinsics.

## Video Frame Time Mapping

For constant-frame-rate input, frame timestamps are generated from fps and frame
count:

```text
video_time_s(frame_index) = frame_index / frame_rate
```

With 200 Hz IMU:

- 30 fps video has about 6.667 IMU samples per video frame.
- 50 fps video has about 4.000 IMU samples per video frame.

Because 30 fps does not map to an integer number of 200 Hz samples, the IMU
trajectory must be sampled by timestamp interpolation, not by sample index.
Quaternion sampling uses slerp between the two surrounding IMU samples.

Future variable-frame-rate support should replace the formula above with actual
packet/frame PTS timestamps from ffprobe/ffmpeg.

## Coordinate System Rule

SLAM XCAM is normally used in landscape orientation, so the IMU-to-camera
rotation is a critical calibration constant:

```text
imu_body_axes -> camera_rig_axes -> left/right lens axes -> VR180 output axes
```

The first implementation should keep one shared SLAM XCAM axis mapping for both
2025 and 2026. If later hardware testing proves the physical IMU mounting is
different, the mapping can be split by model. Until then, only lens calibration
is model-specific.

The coordinate transform must be applied before frame interpolation and
smoothing so all strategies operate in the camera/output coordinate convention.

## Strategy A: Gyro Integration + Smoothed Orientation Curve

Goal: a simple, deterministic first-pass stabilizer.

Pipeline:

```text
load IMU
-> clean timestamps
-> integrate gyro to raw orientation q_raw(t)
-> apply shared IMU-to-camera transform
-> smooth q_raw(t) with a quaternion low-pass / bidirectional smoother
-> sample q_raw and q_smooth at each video frame timestamp
-> correction_q = q_raw * inverse(q_smooth)
```

Recommended parameters:

- `smooth_ms`: 500 to 1200 ms.
- `max_correction_deg`: 10 to 25 deg.
- `dt_min_s`: 0.001 s.
- `dt_max_s`: 0.020 s for 200 Hz logs.

Behavior:

- Best first implementation target.
- Uses only gyro.
- Sensitive to gyro bias over long clips.
- Good for validating axis mapping, timestamp interpolation, and SBS shared-eye
  correction.

Implementation status:

- Current prototype already has the core pieces: gyro integration, quaternion
  interpolation, bidirectional smoothing, elastic clamp, and per-frame plan.
- Next improvement: explicit timestamp cleanup and named strategy reporting.

## Strategy B: Gyro + Accelerometer Fusion

Goal: reduce roll/pitch drift by using gravity from accelerometer data.

Pipeline:

```text
load IMU
-> clean timestamps
-> estimate gyro orientation prediction
-> estimate gravity direction from accelerometer after low-pass filtering
-> correct roll/pitch drift with complementary/Mahony-style feedback
-> apply shared IMU-to-camera transform
-> smooth fused orientation
-> sample per video frame
-> correction_q = q_fused_raw * inverse(q_fused_smooth)
```

Recommended parameters:

- `gravity_lowpass_ms`: 300 to 800 ms.
- `acc_weight`: start around 0.01 to 0.05.
- `acc_norm_gate`: only trust acceleration near gravity, for example
  8.0 to 11.8 m/s^2.
- `smooth_ms`: 500 to 1200 ms.

Behavior:

- Better horizon stability than pure gyro.
- Acceleration must be gated during fast movement, because the accelerometer
  measures both gravity and device motion.
- Useful for landscape capture where small roll errors are obvious.

Implementation status:

- Design target for the second IMU pass.
- The loader already stores `acc_x/acc_y/acc_z`; fusion math is still planned.

## Strategy C: Gyroflow-Style Sync + Smoothing Window

Goal: prioritize video/IMU sync quality and tunable smoothing behavior.

Pipeline:

```text
load IMU
-> clean timestamps
-> integrate gyro / optional fusion
-> search or use manual imu_offset_s
-> build frame timestamp series from video fps + frame_count
-> slerp raw orientation to every frame timestamp
-> apply Gyroflow-style smoothing window on the frame-aligned curve
-> compute correction_q per frame
```

Recommended parameters:

- `manual_offset_s`: exposed in UI from the beginning.
- `sync_search_range_s`: +/- 0.5 s when auto sync is implemented.
- `sync_step_s`: 0.002 to 0.005 s.
- `smooth_ms`: 300 to 1500 ms.
- `max_correction_deg`: 10 to 30 deg.

Behavior:

- Best route for strong visible stabilization once sync is calibrated.
- More robust when video frame rate is 30 fps or 50 fps because all output is
  explicitly frame-aligned.
- Auto sync should later compare visual motion, external IMU, and embedded
  `camm` data when available.

Implementation status:

- Manual offset and frame-aligned interpolation already exist.
- Auto offset search and preview diagnostics are planned.

## Required Timestamp Cleanup

The real 200 Hz XLSX sample has a median dt near 5.062 ms, but includes a few
very short gaps and a few gaps above 10 ms. All strategies should share this
cleanup policy before orientation integration:

```text
1. Sort by timestamp if needed.
2. Drop exact duplicate or non-increasing timestamps unless preserving a sample
   with dt = 0 is explicitly needed for diagnostics.
3. Clamp extremely small dt values to 0 for integration.
4. Split or clamp large dt gaps before gyro integration so one missing interval
   does not create a large orientation jump.
5. Store cleanup statistics in the plan JSON.
```

For the first implementation, keep cleanup conservative:

- `dt <= 0`: keep sample but integrate with `dt = 0`.
- `0 < dt < 0.001`: integrate with actual dt but report it.
- `dt > 0.020`: clamp integration dt to 0.020 and report it.

## Recommended Build Order

1. Finalize Strategy A as the active default.
2. Add timestamp cleanup statistics to the stabilization plan.
3. Add a frame-time table based on video fps and frame count.
4. Add Strategy C manual-offset diagnostics.
5. Add Strategy B gravity correction after axis mapping is visually verified.
