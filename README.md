# SLAM XCAM VR180 Stabilizer

Windows-first stabilizer prototype for SLAM XCAM VR180 3D footage.

This repository is an early open-source prototype. It already includes a
PySide6 desktop UI, IMU parsing, per-frame quaternion smoothing, and a CPU
fisheye reprojection renderer. The CPU renderer is intended for correctness
testing at low resolutions before the project moves to a Rust/wgpu GPU engine.

## Implementation Stack

The current prototype is mainly written in Python:

- GUI: PySide6 / Qt for Python
- video analysis and stabilization pipeline: Python
- IMU parsing, quaternion integration, smoothing, and correction matrices: Python + NumPy
- video decoding, encoding, remuxing, and metadata writing: FFmpeg subprocesses
- Windows executable packaging: PyInstaller

The current stabilization core is not yet written in Rust or C++. For production
quality and speed, the recommended next step is to migrate the fisheye
reprojection renderer and IMU/lens math core to Rust + wgpu/GPU acceleration,
while keeping the desktop UI in Qt or moving it later to Tauri/egui.

This project targets two input modes:

- stitched SBS fisheye video, 2:1 aspect ratio
- separate left/right 1:1 fisheye videos

It also models two lens profiles:

- SLAM XCAM 2025
- SLAM XCAM 2026

Calibration support:

- The app supports per-model calibration files.
- Public documentation intentionally does not publish sensor identifiers,
  calibration matrices, distortion coefficients, RMS values, or private source
  paths.
- In the GUI, selecting `2025` or `2026` loads the matching local calibration
  profile when available.

Current capabilities:

- PySide6 Windows GUI inspired by Gyroflow and DJI Studio
- SBS 2:1 fisheye input analysis
- 50 Hz / 200 Hz SLAM XCAM IMU CSV parsing
- quaternion integration and bidirectional SLERP smoothing
- per-frame correction matrix generation
- CPU SBS fisheye reprojection renderer
- prototype H.264 MP4 output with VR180/SBS metadata tags

Current limitations:

- The renderer is CPU-only and slow at high resolution.
- Lens profiles are placeholders until real 2025/2026 calibration files are available.
- IMU axis/sign mapping and video/IMU offset still need calibration tools.
- Rolling shutter correction is not implemented yet.
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

If running from source, set `PYTHONPATH=src` or use the provided `.bat` launchers
that point at the local Codex Python runtime on the original development
machine.

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

The calibration file is where we will store per-lens intrinsics, distortion coefficients, stereo extrinsics, and optional IMU-to-camera rotation. The app can run without it for validation, but real stabilization quality depends on calibration.

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
