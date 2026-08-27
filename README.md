# SLAM XCAM VR180 Stabilizer

Windows-first stabilizer prototype for SLAM XCAM VR180 3D footage.

This repository contains the open-source Windows application, PySide6 desktop
UI, 6D VQF IMU processing, horizon stabilization, two-iteration rolling-shutter
correction, and CPU/OpenGL Reference Renderers.

## Implementation Stack

The current prototype is mainly written in Python:

- GUI: PySide6 / Qt for Python
- video analysis and stabilization pipeline: Python
- IMU parsing, quaternion integration, smoothing, and correction matrices: Python + NumPy
- calibrated fisheye reprojection: NumPy CPU or OpenGL 3.3 GPU
- video decoding, encoding, remuxing, and metadata writing: FFmpeg subprocesses
- Windows executable packaging: PyInstaller
- official calibration boundary: versioned native C ABI loaded with `ctypes`

The application code is Apache-2.0 open source. Official measured SLAM XCAM
2025/2026 calibration and its native renderer are distributed separately as a
proprietary Calibration Runtime DLL. Users can instead supply their own JSON
calibration and use the fully open CPU/OpenGL renderer.

This project targets two input modes:

- stitched SBS fisheye video, 2:1 aspect ratio
- separate left/right 1:1 fisheye videos

It also models two lens profiles:

- SLAM XCAM 2025
- SLAM XCAM 2026

Calibration support:

- Official models automatically use the separately installed Calibration Runtime.
- The runtime ABI is public, but official K/D parameters and runtime source are not.
- Custom measured JSON calibration remains supported by the open renderer.
- See [docs/CALIBRATION_RUNTIME.md](docs/CALIBRATION_RUNTIME.md).

Current capabilities:

- PySide6 Windows GUI inspired by Gyroflow and DJI Studio
- SBS 2:1 fisheye input analysis
- SLAM motion container plus legacy 50 Hz / 200 Hz IMU table parsing
- 6D VQF gyro/accelerometer fusion and horizon-lock smoothing
- per-frame correction matrix generation
- two-iteration per-row rolling-shutter correction
- CPU and OpenGL 3.3 SBS fisheye reprojection renderers
- prototype H.264 MP4 output with VR180/SBS metadata tags

Current limitations:

- Official Calibration Runtime GPU acceleration is still planned; its initial
  native implementation prioritizes isolation and output parity.
- FFmpeg decode/encode is not yet hardware accelerated or zero-copy.
- Dual left/right 1:1 fisheye input is planned but not fully rendered yet.

## Run On This Machine

Install dependencies:

```powershell
python -m pip install -r requirements.txt
```

Run the GUI:

```powershell
python SLAM_XCAM_Stabilizer_Qt.pyw
```

CLI example:

```powershell
python -m slam_stabilizer.cli --input-sbs "D:\path\input.mp4" --imu "D:\path\imu.csv" --lens-profile config\lenses\slam_xcam_2026.json --calibration "D:\path\calibration.json" --output "D:\path\output_vr180.mp4" --render-width 1280
```

If running from source, set `PYTHONPATH=src` or use the provided `.bat` and
PowerShell launchers. Set `PYTHON_EXE` only when the desired Python executable
is not available as `python` in `PATH`.

## Expected IMU File

The parser accepts common column aliases and supports `.csv`, `.xlsx`, and
`.xlsm` files. Excel workbooks use the first sheet.

Minimum gyro format:

```csv
timestamp(us),acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z
81014,9.3635,1.6165,3.66,-0.030517578,0.25177002,0.05340576
83530,9.3635,1.6165,3.66,-0.030517578,0.25177002,0.05340576
```

`timestamp_s,gx,gy,gz` is also accepted.

Quaternion format is also accepted:

```csv
timestamp_s,qw,qx,qy,qz
0.000,1,0,0,0
0.020,0.9999,0.001,0.002,0.003
```

See [docs/IMU_FORMAT.md](docs/IMU_FORMAT.md).
See [docs/IMU_PROCESSING_STRATEGIES.md](docs/IMU_PROCESSING_STRATEGIES.md)
for the first three IMU processing strategies.

## Reference Direction

This project was structured after reviewing
[silverqsy/VR180-Silver-Bullet](https://github.com/silverqsy/VR180-Silver-Bullet).
The reference repo is not vendored here; see
[docs/REFERENCE_VR180_SILVER_BULLET.md](docs/REFERENCE_VR180_SILVER_BULLET.md)
for the architecture notes we are borrowing and adapting.

## Prototype Commands

Inspect a real IMU file:

```powershell
.\inspect_imu.bat "D:\path\sample_imu.csv"
```

Inspect a matched video/IMU pair:

```powershell
.\inspect_pair.bat --video "D:\path\input.mp4" --imu "D:\path\input_imu.csv" --json "D:\path\pair_report.json"
```

## Calibration File

See [config/calibration.schema.json](config/calibration.schema.json).

Custom calibration JSON stores user-provided lens intrinsics, distortion
coefficients, stereo extrinsics, and optional IMU-to-camera rotation. When no
custom JSON is selected, export requires the separately installed official
Calibration Runtime. Public example files are documentation only and cannot be
used for export.

## Windows Build Environment

Current prototype requires:

- FFmpeg in PATH
- Python 3.12
- NumPy
- PySide6

Build a Windows GUI executable:

```powershell
python -m PyInstaller --noconfirm --clean --onefile --windowed --name SLAM_XCAM_Stabilizer_GUI --paths src --add-data "config;config" SLAM_XCAM_Stabilizer_Qt.pyw
```

Recommended long-term production stack:

- Rust core for IMU integration, lens model, and GPU reprojection
- wgpu for cross-platform GPU rendering
- egui or Tauri for Windows desktop UI
- FFmpeg for decode/encode/remux

## License

Apache License 2.0. See [LICENSE](LICENSE).

The separately distributed official Calibration Runtime and embedded measured
calibration are proprietary and excluded from Apache-2.0. See
[OFFICIAL_RUNTIME_LICENSE_NOTICE.md](OFFICIAL_RUNTIME_LICENSE_NOTICE.md).
