from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class LensProfile:
    path: Path
    name: str
    camera_model: str
    gyro_units: str
    default_imu_rate_hz: int
    projection: str
    default_calibration_path: Path | None
    raw: dict[str, Any]

    @classmethod
    def from_file(cls, path: str | Path) -> "LensProfile":
        p = Path(path)
        data = json.loads(p.read_text(encoding="utf-8"))
        default_calibration = data.get("default_calibration")
        default_calibration_path = None
        if default_calibration:
            default_calibration_path = (p.parent / str(default_calibration)).resolve()
        return cls(
            path=p,
            name=data["name"],
            camera_model=data["camera_model"],
            gyro_units=data.get("gyro_units", "rad_s"),
            default_imu_rate_hz=int(data.get("default_imu_rate_hz", 50)),
            projection=data.get("projection", "equidistant"),
            default_calibration_path=default_calibration_path,
            raw=data,
        )


@dataclass(frozen=True)
class Calibration:
    path: Path | None
    camera_model: str | None
    gyro_units: str | None
    raw: dict[str, Any]

    @classmethod
    def optional_from_file(cls, path: str | Path | None) -> "Calibration":
        if not path:
            return cls(path=None, camera_model=None, gyro_units=None, raw={})
        p = Path(path)
        data = json.loads(p.read_text(encoding="utf-8"))
        return cls(
            path=p,
            camera_model=data.get("camera_model"),
            gyro_units=data.get("gyro_units"),
            raw=data,
        )


@dataclass(frozen=True)
class VideoInput:
    mode: str
    input_sbs: Path | None
    input_left: Path | None
    input_right: Path | None

    def validate(self) -> None:
        if self.mode == "sbs":
            if not self.input_sbs or not self.input_sbs.exists():
                raise FileNotFoundError("SBS input video was not found.")
            return
        if self.mode == "dual":
            if not self.input_left or not self.input_left.exists():
                raise FileNotFoundError("Left input video was not found.")
            if not self.input_right or not self.input_right.exists():
                raise FileNotFoundError("Right input video was not found.")
            return
        raise ValueError(f"Unsupported input mode: {self.mode}")

