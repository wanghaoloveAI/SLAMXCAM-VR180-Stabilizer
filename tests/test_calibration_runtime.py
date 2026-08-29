from __future__ import annotations

import hashlib
import json
import sys

import pytest

from slam_stabilizer.calibration_runtime import (
    CalibrationRuntime,
    CalibrationRuntimeUnavailable,
    MANIFEST_FILENAME,
    RUNTIME_FILENAME,
    _load_and_verify_manifest,
    discover_runtime,
)


def _write_manifest(directory, payload: bytes, sha256: str | None = None) -> None:
    (directory / RUNTIME_FILENAME).write_bytes(payload)
    (directory / MANIFEST_FILENAME).write_text(
        json.dumps(
            {
                "format": "slam_xcam_calibration_runtime_manifest_v1",
                "abi_version": 1,
                "runtime_version": "test",
                "models": ["slam_xcam_2025", "slam_xcam_2026"],
                "sha256": sha256 or hashlib.sha256(payload).hexdigest(),
            }
        ),
        encoding="utf-8",
    )


def test_runtime_manifest_accepts_matching_dll(tmp_path) -> None:
    payload = b"test-runtime"
    _write_manifest(tmp_path, payload)

    manifest = _load_and_verify_manifest(tmp_path / RUNTIME_FILENAME)

    assert manifest["abi_version"] == 1


def test_runtime_manifest_rejects_modified_dll(tmp_path) -> None:
    _write_manifest(tmp_path, b"modified", sha256=hashlib.sha256(b"original").hexdigest())

    with pytest.raises(CalibrationRuntimeUnavailable, match="SHA-256"):
        _load_and_verify_manifest(tmp_path / RUNTIME_FILENAME)


def test_runtime_reports_missing_installation(tmp_path, monkeypatch) -> None:
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("SLAM_XCAM_CALIBRATION_RUNTIME", str(tmp_path / "missing"))

    with pytest.raises(CalibrationRuntimeUnavailable, match="was not found"):
        CalibrationRuntime()


def test_frozen_app_discovers_bundled_runtime(tmp_path, monkeypatch) -> None:
    runtime_dir = tmp_path / "calibration_runtime"
    runtime_dir.mkdir()
    bundled_dll = runtime_dir / RUNTIME_FILENAME
    bundled_dll.write_bytes(b"bundled-runtime")
    monkeypatch.delenv("SLAM_XCAM_CALIBRATION_RUNTIME", raising=False)
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    monkeypatch.setattr(sys, "_MEIPASS", str(tmp_path), raising=False)

    assert discover_runtime() == bundled_dll.resolve()
