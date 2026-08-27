# Official Calibration Runtime Boundary

The SLAM XCAM stabilizer application, IMU processing, stabilization logic,
rolling-shutter timing, public renderer, GUI, and build files are Apache-2.0
open source. Official SLAM XCAM 2025/2026 measured lens calibration is delivered
separately in a proprietary native runtime.

## Why This Boundary Exists

Hexadecimal, Base64, or encrypted data with a key embedded in open-source code
does not protect calibration. The official runtime therefore does not expose K,
D, lens rotation, source dimensions, projection maps, or decoded calibration
objects through its ABI. It accepts RGB frames and stabilization matrices and
returns rendered RGB frames.

The native runtime raises the cost of extracting official calibration, but no
offline client-side protection can guarantee that a determined reverse engineer
will never recover data from process or GPU memory.

## Public ABI

The stable C ABI is declared in
`native/calibration_runtime/include/slam_xcam_calibration_runtime.h`.

The runtime is installed outside the repository:

```text
calibration_runtime/
  slam_xcam_calibration_runtime.dll
  slam_xcam_calibration_runtime.manifest.json
```

The manifest pins the ABI version and DLL SHA-256. The host rejects a missing,
modified, or incompatible runtime. `SLAM_XCAM_CALIBRATION_RUNTIME` may point to
either the DLL itself or its containing directory.

## Open Calibration Path

Users and contributors can bypass the official runtime and pass their own
calibration JSON with `--calibration`. That path remains fully open and uses the
public CPU/OpenGL Reference Renderer.

## Licensing

The runtime DLL and official calibration are not licensed under Apache-2.0.
They require a separate distribution and license notice. Do not commit the DLL,
its manifest, private calibration JSON, symbols, map dumps, or private runtime
source to the public repository.

## Private Runtime Build Requirements

The private Windows runtime is built with Rust's
`x86_64-pc-windows-msvc` target. The build machine needs Visual Studio Build
Tools with both components below:

- MSVC x64/x86 C++ build tools
- Windows 11 SDK, including `kernel32.lib`

After creating the release DLL, generate its lowercase SHA-256 and place it in
a copy of `native/calibration_runtime/manifest.example.json`. Ship only the DLL,
manifest, and proprietary license notice. Keep source, calibration JSON, import
libraries, PDB files, and build directories private.

The ABI intentionally allows the private implementation to move from CPU to an
internal GPU renderer without changing the open application. Official GPU code
must keep calibration constants and projection maps inside the DLL and must not
pass K/D as public OpenGL uniforms.
