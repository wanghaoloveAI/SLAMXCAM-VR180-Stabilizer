from __future__ import annotations

import csv
from dataclasses import dataclass
from math import cos, radians, sin, sqrt
from pathlib import Path
from typing import Any

import numpy as np


@dataclass(frozen=True)
class ImuSample:
    timestamp_s: float
    quaternion_wxyz: tuple[float, float, float, float]
    acceleration_xyz: tuple[float, float, float] | None = None
    gyro_xyz: tuple[float, float, float] | None = None


def _col(row: dict[str, Any], *names: str) -> Any | None:
    lowered = {k.lower().strip(): v for k, v in row.items()}
    for name in names:
        if name in lowered and lowered[name] not in ("", None):
            return lowered[name]
    return None


def _load_rows(path: Path) -> list[dict[str, Any]]:
    suffix = path.suffix.lower()
    if suffix in (".xlsx", ".xlsm"):
        try:
            from openpyxl import load_workbook
        except ImportError as exc:
            raise ImportError("Reading XLSX IMU files requires openpyxl.") from exc

        workbook = load_workbook(path, read_only=True, data_only=True)
        sheet = workbook.worksheets[0]
        rows = sheet.iter_rows(values_only=True)
        header = next(rows, None)
        if not header:
            return []
        names = [str(value).strip() if value is not None else "" for value in header]
        return [
            {name: value for name, value in zip(names, row)}
            for row in rows
            if row is not None and any(value is not None for value in row)
        ]

    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        return list(reader)


def _normalize(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if norm == 0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=float,
    )


def _delta_from_gyro(gx: float, gy: float, gz: float, dt: float, units: str) -> np.ndarray:
    if units == "deg_s":
        gx, gy, gz = radians(gx), radians(gy), radians(gz)
    omega = np.array([gx, gy, gz], dtype=float)
    angle = float(np.linalg.norm(omega) * dt)
    if angle == 0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = omega / np.linalg.norm(omega)
    half = angle / 2.0
    return _normalize(np.array([cos(half), *(axis * sin(half))], dtype=float))


def load_imu_csv(path: str | Path, gyro_units: str = "rad_s") -> list[ImuSample]:
    rows = _load_rows(Path(path))

    if not rows:
        raise ValueError("IMU file is empty.")

    samples: list[ImuSample] = []
    q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    last_t: float | None = None

    for row in rows:
        t_raw = _col(row, "timestamp_s", "time_s", "t", "time", "timestamp")
        t_us_raw = _col(row, "timestamp(us)", "timestamp_us", "time_us", "timestamp_usec")
        if t_raw is None:
            if t_us_raw is None:
                raise ValueError("IMU CSV needs a timestamp_s/time_s/t or timestamp(us) column.")
            t = float(t_us_raw) / 1_000_000.0
        else:
            t = float(t_raw)

        ax = _col(row, "acc_x", "accel_x", "accelerometer_x", "ax")
        ay = _col(row, "acc_y", "accel_y", "accelerometer_y", "ay")
        az = _col(row, "acc_z", "accel_z", "accelerometer_z", "az")
        acceleration = None
        if ax is not None and ay is not None and az is not None:
            acceleration = (float(ax), float(ay), float(az))

        qw = _col(row, "qw", "quat_w", "w")
        qx = _col(row, "qx", "quat_x", "x")
        qy = _col(row, "qy", "quat_y", "y")
        qz = _col(row, "qz", "quat_z", "z")
        if all(v is not None for v in (qw, qx, qy, qz)):
            q = _normalize(np.array([float(qw), float(qx), float(qy), float(qz)], dtype=float))
        else:
            gx = _col(row, "gx", "gyro_x", "omega_x")
            gy = _col(row, "gy", "gyro_y", "omega_y")
            gz = _col(row, "gz", "gyro_z", "omega_z")
            if gx is None or gy is None or gz is None:
                raise ValueError("IMU CSV needs either qw/qx/qy/qz or gx/gy/gz columns.")
            gyro = (float(gx), float(gy), float(gz))
            dt = 0.0 if last_t is None else max(0.0, t - last_t)
            q = _normalize(_quat_mul(q, _delta_from_gyro(*gyro, dt, gyro_units)))
        if all(v is not None for v in (qw, qx, qy, qz)):
            gyro = None

        samples.append(
            ImuSample(
                timestamp_s=t,
                quaternion_wxyz=tuple(float(v) for v in q),
                acceleration_xyz=acceleration,
                gyro_xyz=gyro,
            )
        )
        last_t = t

    return samples


def summarize_rate(samples: list[ImuSample]) -> float:
    if len(samples) < 2:
        return 0.0
    deltas = [b.timestamp_s - a.timestamp_s for a, b in zip(samples, samples[1:]) if b.timestamp_s > a.timestamp_s]
    if not deltas:
        return 0.0
    return 1.0 / (sum(deltas) / len(deltas))
