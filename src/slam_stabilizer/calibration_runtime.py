from __future__ import annotations

import ctypes
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any

import numpy as np


ABI_VERSION = 1
RUNTIME_FILENAME = "slam_xcam_calibration_runtime.dll"
MANIFEST_FILENAME = "slam_xcam_calibration_runtime.manifest.json"
MODEL_IDS = {"slam_xcam_2025": 2025, "slam_xcam_2026": 2026}
DISTORTION_CORRECTION_FLAG = 1


class CalibrationRuntimeUnavailable(RuntimeError):
    pass


@dataclass(frozen=True)
class CalibrationRuntimeInfo:
    dll_path: Path
    version: str
    sha256: str
    models: tuple[str, ...]


def runtime_search_directories() -> list[Path]:
    directories: list[Path] = []
    configured = os.environ.get("SLAM_XCAM_CALIBRATION_RUNTIME")
    if configured:
        configured_path = Path(configured).expanduser().resolve()
        directories.append(configured_path.parent if configured_path.is_file() else configured_path)
    if getattr(sys, "frozen", False):
        bundle_root = Path(getattr(sys, "_MEIPASS", Path(sys.executable).resolve().parent))
        directories.append(bundle_root / "calibration_runtime")
        directories.append(Path(sys.executable).resolve().parent / "calibration_runtime")
    repository_root = Path(__file__).resolve().parents[2]
    directories.extend(
        [
            repository_root / "calibration_runtime",
            Path.cwd() / "calibration_runtime",
        ]
    )
    unique: list[Path] = []
    for directory in directories:
        if directory not in unique:
            unique.append(directory)
    return unique


def discover_runtime() -> Path | None:
    configured = os.environ.get("SLAM_XCAM_CALIBRATION_RUNTIME")
    if configured:
        configured_path = Path(configured).expanduser().resolve()
        if configured_path.is_file():
            return configured_path
        if configured_path.is_dir():
            candidate = configured_path / RUNTIME_FILENAME
            return candidate if candidate.is_file() else None
        return None
    for directory in runtime_search_directories():
        candidate = directory / RUNTIME_FILENAME
        if candidate.is_file():
            return candidate.resolve()
    return None


def _load_and_verify_manifest(dll_path: Path) -> dict[str, Any]:
    manifest_path = dll_path.with_name(MANIFEST_FILENAME)
    if not manifest_path.is_file():
        raise CalibrationRuntimeUnavailable(f"Calibration Runtime manifest is missing: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("format") != "slam_xcam_calibration_runtime_manifest_v1":
        raise CalibrationRuntimeUnavailable("Unsupported Calibration Runtime manifest format.")
    if int(manifest.get("abi_version", 0)) != ABI_VERSION:
        raise CalibrationRuntimeUnavailable("Calibration Runtime manifest ABI does not match the application.")
    expected = str(manifest.get("sha256", "")).lower()
    actual = hashlib.sha256(dll_path.read_bytes()).hexdigest()
    if not expected or expected != actual:
        raise CalibrationRuntimeUnavailable("Calibration Runtime DLL SHA-256 verification failed.")
    return manifest


class CalibrationRuntime:
    def __init__(self, dll_path: Path | None = None) -> None:
        path = (dll_path or discover_runtime())
        if path is None:
            raise CalibrationRuntimeUnavailable(
                "The built-in Calibration Runtime was not found. Reinstall the complete SLAM XCAM Studio package."
            )
        self.path = Path(path).resolve()
        manifest = _load_and_verify_manifest(self.path)
        try:
            self.library = ctypes.WinDLL(str(self.path))
        except OSError as exc:
            raise CalibrationRuntimeUnavailable(f"Could not load Calibration Runtime: {exc}") from exc
        self._configure_api()
        if self.library.slam_cal_abi_version() != ABI_VERSION:
            raise CalibrationRuntimeUnavailable("Calibration Runtime DLL ABI does not match the application.")
        version = self.library.slam_cal_runtime_version().decode("utf-8", errors="replace")
        self.info = CalibrationRuntimeInfo(
            dll_path=self.path,
            version=version,
            sha256=str(manifest["sha256"]),
            models=tuple(str(model) for model in manifest.get("models", [])),
        )

    def _configure_api(self) -> None:
        lib = self.library
        lib.slam_cal_abi_version.restype = ctypes.c_uint32
        lib.slam_cal_runtime_version.restype = ctypes.c_char_p
        lib.slam_cal_model_available.argtypes = [ctypes.c_uint32]
        lib.slam_cal_model_available.restype = ctypes.c_int32
        lib.slam_cal_create.argtypes = [
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_float,
            ctypes.POINTER(ctypes.c_void_p),
        ]
        lib.slam_cal_create.restype = ctypes.c_int32
        lib.slam_cal_render_rgb24.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_uint32,
        ]
        lib.slam_cal_render_rgb24.restype = ctypes.c_int32
        lib.slam_cal_destroy.argtypes = [ctypes.c_void_p]
        if hasattr(lib, "slam_cal_backend_name"):
            lib.slam_cal_backend_name.argtypes = [ctypes.c_void_p]
            lib.slam_cal_backend_name.restype = ctypes.c_char_p
        lib.slam_cal_last_error.restype = ctypes.c_char_p

    def supports(self, camera_model: str) -> bool:
        model_id = MODEL_IDS.get(camera_model)
        return bool(model_id and self.library.slam_cal_model_available(model_id) == 1)

    def create_renderer(
        self,
        camera_model: str,
        eye_size: int,
        distortion_correction: bool,
        field_of_view_deg: float,
    ) -> "CalibrationRuntimeRenderer":
        model_id = MODEL_IDS.get(camera_model)
        if model_id is None or not self.supports(camera_model):
            raise CalibrationRuntimeUnavailable(f"Calibration Runtime does not support {camera_model}.")
        flags = DISTORTION_CORRECTION_FLAG if distortion_correction else 0
        handle = ctypes.c_void_p()
        status = self.library.slam_cal_create(
            model_id,
            int(eye_size),
            flags,
            float(field_of_view_deg),
            ctypes.byref(handle),
        )
        self._check(status)
        if not handle.value:
            raise CalibrationRuntimeUnavailable("Calibration Runtime returned an empty renderer handle.")
        return CalibrationRuntimeRenderer(self, handle, int(eye_size))

    def _check(self, status: int) -> None:
        if status == 0:
            return
        message = self.library.slam_cal_last_error()
        detail = message.decode("utf-8", errors="replace") if message else f"status {status}"
        raise RuntimeError(f"Calibration Runtime failed: {detail}")


class CalibrationRuntimeRenderer:
    def __init__(self, runtime: CalibrationRuntime, handle: ctypes.c_void_p, eye_size: int) -> None:
        self.runtime = runtime
        self.handle = handle
        self.eye_size = eye_size

    @property
    def backend_name(self) -> str:
        function = getattr(self.runtime.library, "slam_cal_backend_name", None)
        if function is None:
            return "Native CPU"
        value = function(self.handle)
        return value.decode("utf-8", errors="replace") if value else "Unknown native backend"

    def render(
        self,
        frame: np.ndarray,
        correction_matrix: np.ndarray,
        row_matrices: np.ndarray | None,
    ) -> np.ndarray:
        expected_shape = (self.eye_size, self.eye_size * 2, 3)
        if frame.shape != expected_shape:
            raise ValueError(f"Runtime input shape {frame.shape} does not match {expected_shape}.")
        input_frame = np.ascontiguousarray(frame, dtype=np.uint8)
        correction = np.ascontiguousarray(correction_matrix, dtype=np.float32).reshape(9)
        rows = (
            np.ascontiguousarray(row_matrices, dtype=np.float32).reshape(-1, 9)
            if row_matrices is not None and len(row_matrices) > 0
            else None
        )
        output = np.empty_like(input_frame)
        row_pointer = (
            rows.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
            if rows is not None
            else ctypes.POINTER(ctypes.c_float)()
        )
        status = self.runtime.library.slam_cal_render_rgb24(
            self.handle,
            input_frame.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
            input_frame.strides[0],
            correction.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            row_pointer,
            0 if rows is None else len(rows),
            output.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
            output.strides[0],
        )
        self.runtime._check(status)
        return output

    def close(self) -> None:
        if self.handle and self.handle.value:
            self.runtime.library.slam_cal_destroy(self.handle)
            self.handle = ctypes.c_void_p()

    def __enter__(self) -> "CalibrationRuntimeRenderer":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()
