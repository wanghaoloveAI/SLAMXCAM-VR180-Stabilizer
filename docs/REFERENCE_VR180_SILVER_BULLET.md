# VR180 Silver Bullet Reference Notes

Reference checkout:

```text
reference/VR180-Silver-Bullet
```

## Architecture Lessons To Adopt

VR180 Silver Bullet 2.0 uses a Rust workspace with clear crate boundaries:

- `vr180-core`: pure math, project model, gyro/quaternion logic, metadata atom builders
- `vr180-fisheye`: lens calibration and fisheye projection
- `vr180-pipeline`: decode, IMU preparation, GPU rendering, encode, metadata injection
- `vr180-gui`: product UI only

For this project, the matching Python-stage boundaries are:

- `slam_stabilizer.core`: quaternion math, stabilization smoothing, frame pose plans
- `slam_stabilizer.fisheye`: SLAM XCAM lens models and calibration projection math
- `slam_stabilizer.pipeline`: video probing, IMU loading, render/export orchestration
- `slam_stabilizer.qt_gui`: Windows GUI

## Algorithm Lessons To Adopt

Stabilization should not simply invert raw gyro orientation.

The useful pattern is:

1. Convert IMU to a timestamped raw quaternion trajectory.
2. Smooth the trajectory with bidirectional velocity-dampened SLERP.
3. Clamp extreme correction softly so output does not demand impossible crop/FOV.
4. For each video frame, interpolate both raw and smoothed trajectory at frame time.
5. Convert the smoothed output camera ray into the raw source camera frame.
   In this renderer convention the sampling correction is
   `q_correction = inverse(q_raw) * q_smooth`.
6. Send the correction matrix to the fisheye renderer.

Silver Bullet also treats rolling shutter as a first-class path by computing
per-row quaternions. We will add that after frame-level stabilization works.

## Projection Lessons To Adopt

The reference renderer projects output pixels to rays, applies stabilization,
then projects the ray into the source fisheye lens:

```text
output pixel -> output ray -> correction rotation -> source ray -> source fisheye UV
```

For normalized fisheye SBS output, Silver Bullet uses an equidistant output
disk and a calibrated source fisheye model. For SLAM XCAM this means the
2025/2026 profiles need real measured intrinsics before production-quality
results are possible.

## Metadata Lessons To Adopt

Silver Bullet does native MP4 atom injection for YouTube VR180 and Apple APMP.
Our current FFmpeg metadata tags are a prototype. A production version should
write proper `st3d/sv3d` and optionally Apple `vexu/hfov` atoms.

