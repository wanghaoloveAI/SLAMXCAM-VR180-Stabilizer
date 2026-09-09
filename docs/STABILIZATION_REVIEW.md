# Stabilization review, 2026-09-08

## Confirmed findings and changes

- Post-plan correction velocity limiting previously capped CSV correction at
  25 deg/s and SLAMIMU correction at 200 deg/s. This changes the virtual target
  and leaves rapid camera motion uncancelled. The default is now disabled;
  an explicitly requested limit remains available in the job API.
- Clamping after horizon leveling reintroduced roll for large corrections.
  Following-motion clamping now precedes leveling.
- The desktop worker ignored the mode payload and always selected horizon lock.
  The mode selector now routes three-axis smoothing, horizon lock, and fixed
  orientation through the actual frame planner and existing renderers.
- Missing IMU coverage now produces a warning with the affected frame count.
  Unknown modes and invalid pose timelines fail explicitly.

## Processing contract

Video probe -> sensor timestamps (SLAMIMU) or average-FPS timeline (CSV)
-> camera-axis IMU -> 6D VQF -> target orientation -> inverse(raw) * target
-> shared stereo rotation -> per-source-row rolling-shutter refinement
-> calibrated fisheye sampling -> encoding and metadata.

Normal mode smooths all three rotational axes while following intended movement.
Horizon mode additionally levels roll using world gravity.
Fixed orientation uses the first frame's smoothed orientation for the entire clip.
The same frame correction is passed to both eyes; do not estimate independent
left/right stabilization trajectories.

## Remaining investigation and development

- Match encoded frame PTS to sensor timestamps, including trims, dropped frames,
  and variable frame rate. Frame-count equality alone cannot prove pairing.
- Account for exposure midpoint and per-frame rolling-shutter skew instead of
  relying on one clip-wide readout value. Confirm timestamp semantics with App.
- VQF currently uses a median sample period for irregular IMU input; quantify
  gaps and resample or integrate with correct timing before changing fusion.
- Near a vertical optical axis, gravity-projected up becomes singular. A
  continuity-aware heading policy needs dedicated pole-crossing tests.
- Validate distortion/projection provenance to avoid double correction on
  already processed SBS inputs.
- Add stereo-shared coverage measurement and a controlled crop/FOV policy.
  Strong pitch/yaw corrections can expose uncaptured pixels in a hemisphere.
- Translation, parallax and motion blur are not removed by rotational IMU
  correction. Image-assisted residual estimation must use stereo constraints;
  independent 2D warps can damage binocular viewing.
- Verify official Runtime CPU/GPU pixel parity and real video before release.

## Verification boundary

Synthetic tests exercise rapid roll at 30/50 fps from 200 Hz IMU, mixed-axis
jitter reduction, a fixed target, and identical stereo-eye sampling. These
validate mathematical behavior, not measured user-video quality. Real footage
must be compared using matched timestamps, residual roll, scene motion, edge
coverage, and vertical disparity before calling the improvement validated.

Reference: https://docs.gyroflow.xyz/app/getting-started/basic-usage/stabilization
