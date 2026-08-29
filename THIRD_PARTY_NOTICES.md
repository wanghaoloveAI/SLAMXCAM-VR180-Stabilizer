# Third-Party Notices

## VQF 2.1.2

The project includes the pure Python `PyVQF` implementation from:

- Project: VQF
- Author: Daniel Laidig
- Source: https://github.com/dlaidig/vqf/
- License: MIT
- Vendored file: `src/slam_stabilizer/vendor/pyvqf.py`

The vendored source retains its original copyright and SPDX license header.

## FFmpeg portable runtime

The Windows portable release distributes unmodified `ffmpeg.exe` and
`ffprobe.exe` as separate programs from the Gyan.dev FFmpeg Windows build.

- Build: 2026-03-15 git-6ba0b59d8b full build
- Source: https://github.com/FFmpeg/FFmpeg/commit/6ba0b59d8b
- License: GNU GPL version 3
- Runtime license and build information: `licenses/ffmpeg/`

FFmpeg is invoked through subprocesses and is not linked into the SLAM XCAM
Studio application.

## Android SDK Platform-Tools

The Windows portable release includes the Android Debug Bridge executable and
its required DLLs from Android SDK Platform-Tools 37.0.0.

- Project: Android SDK Platform-Tools
- Source: https://developer.android.com/tools/releases/platform-tools
- Notices: `licenses/android-platform-tools/NOTICE.txt`
