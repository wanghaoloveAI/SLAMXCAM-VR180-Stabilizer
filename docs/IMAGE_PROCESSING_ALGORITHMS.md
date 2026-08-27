# Image Processing Algorithms

This document defines the image-processing side of the SLAM XCAM Spatial Video
stabilizer. The IMU pipeline produces one correction rotation per video frame.
The renderer applies that same correction to both eyes, then uses per-eye lens
calibration to sample the source image.

## Required Inputs

The image pipeline must receive these values before processing frames:

- Video parameters: width, height, layout, frame rate, frame count, duration,
  frame duration, codec, and field of view.
- Lens calibration: selected by SLAM XCAM model.
  - 2025: local model-specific calibration.
  - 2026: local model-specific calibration.
- IMU frame plan: one correction matrix or quaternion per video frame.
- User settings: output projection, render width, distortion correction,
  optional crop/FOV policy, and optional color/LUT settings.

## Render Device Priority

For production performance, the renderer should choose compute devices in this
order:

1. Discrete GPU, especially NVIDIA RTX/GTX/Quadro or AMD Radeon.
2. Integrated GPU, such as Intel Iris/UHD/Arc integrated graphics.
3. CPU fallback.

The GUI exposes this as an automatic render-device option:

```text
Auto: discrete GPU -> integrated GPU -> CPU
```

Render-device selection is not an image algorithm. CPU, integrated GPU, and
discrete GPU are execution backends for the selected renderer.

The stabilizer has two image-processing algorithms:

- Reference Renderer: calibrated ray-based fisheye reprojection.
- STMap Renderer: lookup-table based remapping using precomputed STMap files.

The Reference Renderer supports both a CPU implementation and an OpenGL 3.3
GPU implementation. Automatic device selection prefers a discrete GPU, then an
integrated GPU, and falls back to CPU without changing the projection math.

## Stereo Safety Rule

The stabilizer must build one motion trajectory for the clip, not one trajectory
per eye.

- Left and right eyes use the same frame timestamp.
- Left and right eyes use the same IMU correction rotation.
- Left and right eyes may use different lens intrinsics, distortion maps, crop,
  and stereo extrinsics.
- The renderer must not smooth or rotate each eye independently, because that
  can change stereo geometry and create VR discomfort.

## Coordinate Flow

For each output frame and each eye:

```text
output pixel
-> normalized output disk coordinate
-> output VR180 ray
-> frame stabilization rotation
-> optional eye/lens rotation
-> calibrated source fisheye projection
-> source UV sample
-> output pixel color
```

The basic equation is:

```text
source_ray = R_lens * R_stabilization * output_ray
source_uv = project_fisheye(source_ray, K, D)
```

Where:

- `R_stabilization` comes from the IMU frame plan.
- `R_lens` is an optional lens/output alignment rotation from calibration.
- `K` is camera intrinsics: `fx, fy, cx, cy`.
- `D` is fisheye distortion: `k1, k2, k3, k4`.

## Projection Model

The current calibration files use an OpenCV fisheye style model. For a source
ray:

```text
theta = acos(z)
phi = atan2(y, x)
theta_d = theta * (1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8)
u = cx + fx * theta_d * cos(phi)
v = cy - fy * theta_d * sin(phi)
```

Pixels outside the valid fisheye circle, outside calibration bounds, or outside
the requested output FOV are filled with black unless a later edge-fill mode is
selected.

## Supported Source Layouts

### VR1803D Fisheye SBS

The source frame is already side-by-side:

```text
left_eye  = source[:, 0:height]
right_eye = source[:, height:height*2]
```

The output should remain side-by-side fisheye unless the user selects a
half-equirectangular export mode.

### Separate Left/Right Fisheye

The decoder must synchronize both input videos by frame index and timestamp.
Each eye is processed with the same stabilization correction, then packed into a
side-by-side output frame.

### VR1803D Equirectangular

For half-equirectangular input, the output ray is mapped to latitude/longitude
instead of fisheye UV. Stabilization still happens in ray space:

```text
output pixel -> spherical ray -> corrected source ray -> equirect UV
```

This mode should preserve stereo side-by-side packing and metadata.

## Algorithm Options

### Reference Renderer: Calibrated Fisheye Reprojection

Purpose: correctness-first reference renderer.

Steps:

1. Decode source frames with FFmpeg.
2. Split SBS into left/right eyes, or decode separate left/right streams.
3. For each frame, fetch the correction matrix from the IMU frame plan.
4. Build or reuse per-eye remap grids for that frame.
5. Project output rays through the correction matrix.
6. Project corrected rays into calibrated source fisheye UV.
7. Sample source with bilinear interpolation.
8. Encode SBS output and inject VR180 metadata.

Pros:

- Simple and easy to verify.
- Good for proving calibration, orientation, and synchronization.

Cons:

- Too slow for full 8K production.
- Per-frame remap computation is expensive on CPU.

Status: active.

Execution backend:

- Current: NumPy CPU and OpenGL 3.3 GPU, with CPU fallback.
- Official calibration uses the separate native Calibration Runtime boundary.

The open GPU backend:

- Select render device by priority: discrete GPU, integrated GPU, CPU fallback.
- Upload source frames as GPU textures.
- Send per-frame correction matrix, per-eye intrinsics, distortion, and FOV to
  shader constants.
- Compute output ray, rotate it, project to source UV, and sample the source
  texture in shader or compute code.
- Match the CPU Reference Renderer within small interpolation differences.

### STMap Renderer: Lookup Remap

Purpose: fastest calibrated mapping when a lens STMap is available.

Steps:

1. Load per-eye STMap files for the selected camera model.
2. Convert output ray coordinates through stabilization rotation.
3. Use STMap as the lens projection lookup, or precompose STMap with the current
   correction when possible.
4. Sample source frame by UV map.

Pros:

- Can match offline calibration tools closely.
- Reduces repeated lens polynomial math.

Cons:

- STMap resolution, origin, channel order, and y-flip must be verified.
- Dynamic stabilization still needs per-frame rotation handling.

Status: planned; STMap source files need runtime binding.

### Algorithm D: Rolling Shutter Row Reprojection

Purpose: improve high-frequency motion and fast pan/tilt footage.

Steps:

1. Estimate exposure readout time from camera profile or user setting.
2. For every output row, compute row timestamp:

```text
row_time = frame_time + row_index / frame_height * readout_time
```

3. Interpolate IMU orientation at each row timestamp.
4. Use per-row correction rotation during ray reprojection.

Pros:

- Better stabilization for fast motion.
- Matches the VR180 Silver Bullet direction.

Cons:

- More expensive.
- Needs real readout timing for 2025 and 2026 models.

Status: later quality upgrade.

## Recommended Implementation Order

1. Finish the Reference Renderer for SBS fisheye with local calibrated model
   data.
2. Add direct visual diagnostics: original/stabilized split preview, motion graph,
   and per-frame correction magnitude.
3. Add separate left/right fisheye decode and SBS packing.
4. Add equirectangular input/output mode.
5. Add GPU backend for the Reference Renderer using the same projection math.
6. Add STMap Renderer after STMap axis, y-flip, and value range are verified.
7. Add rolling-shutter row correction after frame-level stabilization is visibly
   correct.

## Validation Metrics

Each renderer change should be checked with:

- Frame count preserved.
- Frame rate preserved.
- Output duration within one frame of source duration.
- Left/right correction matrices are identical per frame.
- Metadata reports left-right stereo and VR180 projection.
- Static scene stabilization reduces gyro-derived rotation energy.
- Straight calibration targets do not drift differently between eyes.
- No black border enters the central comfort region under normal correction.

## Current Engineering Decision

Use the Reference Renderer as the first correctness target. GPU acceleration is
an execution backend for the Reference Renderer, not a separate image algorithm.
STMap Renderer is the second algorithm and should be implemented after STMap
axis, y-flip, and value range are verified.
