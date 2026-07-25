# SLAM XCAM IMU Format

SLAM XCAM 2025 and SLAM XCAM 2026 use the same external IMU CSV schema.
The model choice only changes the lens calibration and distortion parameters,
not the IMU parser or axis-processing pipeline.

The provided sample file `Slam_20260524_140325_610_imu.csv` uses this header:

```csv
timestamp(us),acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z
```

Observed sample properties:

- rows: 1549
- duration: about 7.82 s
- average rate: about 197.8 Hz
- median delta: about 5.06 ms
- expected source rate: 200 Hz
- timestamp unit: microseconds
- gyro values appear to be radians per second
- acceleration values appear to be m/s^2
- acceleration norm is about 10.0 m/s^2 on average, close to gravity plus
  device motion/noise.
- the data contains a small number of irregular timestamp deltas, including
  very short gaps below 1 ms and a few gaps above 10 ms.

Important parser behavior:

- `timestamp(us)` is converted to seconds.
- IMU integration uses actual timestamp deltas, not a fixed 50 Hz or 200 Hz step.
- non-increasing timestamps are tolerated for now by using `dt = 0` during gyro integration.
- acceleration is stored for future gravity correction and horizon stabilization.
- `.csv`, `.xlsx`, and `.xlsm` IMU files are supported by the loader.
- VR180 3D output uses one shared IMU trajectory for both eyes. The left and
  right reprojection passes must receive the same per-frame correction
  quaternion.

Additional observed SLAM XCAM header-only file:

- `videoandimu/Slam_20260622_043403_304_imu.csv`
- header: `timestamp(us),acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z`
- note: the checked file contains only the header, so it confirms schema but
  cannot be used to estimate IMU rate or duration.

Run:

```powershell
.\inspect_imu.bat "D:\path\sample_imu.csv"
```
