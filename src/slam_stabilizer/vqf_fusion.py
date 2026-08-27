from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin

import numpy as np

from .core.quaternion import Quat
from .imu import ImuSample
from .vendor.pyvqf import PyVQF


@dataclass(frozen=True)
class VqfDiagnostics:
    sample_rate_hz: float
    bias_rad_s_xyz: tuple[float, float, float]
    bias_deg_s_xyz: tuple[float, float, float]
    bias_sigma_rad_s: float
    rest_fraction: float
    orientation_window_s: float


def _hemisphere_aligned_window_average(
    times: list[float],
    quaternions: list[Quat],
    window_s: float,
) -> list[Quat]:
    if len(quaternions) < 2 or window_s <= 0.0:
        return quaternions
    half = window_s / 2.0
    output: list[Quat] = []
    left = 0
    right = 0
    for index, timestamp in enumerate(times):
        while left < len(times) and times[left] < timestamp - half:
            left += 1
        while right < len(times) and times[right] <= timestamp + half:
            right += 1
        reference = quaternions[index]
        values = np.zeros(4, dtype=float)
        for quaternion in quaternions[left:right]:
            sign = -1.0 if reference.dot(quaternion) < 0.0 else 1.0
            values += sign * np.array(quaternion.as_tuple(), dtype=float)
        output.append(Quat.from_iter(values.tolist()))
    return output


def fuse_6d_vqf(
    samples: list[ImuSample],
    orientation_window_s: float = 0.015,
) -> tuple[list[ImuSample], VqfDiagnostics]:
    """Run the official magnetometer-free PyVQF implementation."""

    if len(samples) < 2:
        raise ValueError("6D VQF requires at least two IMU samples.")
    if any(sample.gyro_xyz is None or sample.acceleration_xyz is None for sample in samples):
        raise ValueError("6D VQF requires gyro and acceleration at every sample.")

    deltas = np.diff(np.array([sample.timestamp_s for sample in samples], dtype=float))
    positive_deltas = deltas[deltas > 0.0]
    if positive_deltas.size == 0:
        raise ValueError("6D VQF requires strictly increasing IMU timestamps.")
    sample_period_s = float(np.median(positive_deltas))
    gyro = np.array([sample.gyro_xyz for sample in samples], dtype=float)
    acceleration = np.array([sample.acceleration_xyz for sample in samples], dtype=float)

    filter_6d = PyVQF(sample_period_s, magDistRejectionEnabled=False)
    result = filter_6d.updateBatch(gyro, acceleration)
    raw_quaternions = [Quat.from_iter(values.tolist()) for values in result["quat6D"]]

    # PyVQF uses an Earth frame with +Z vertical. The renderer and horizon
    # target use +Y as world up, so rotate Earth -90 degrees around X.
    half_angle = radians(-90.0) / 2.0
    earth_z_to_world_y = Quat(cos(half_angle), sin(half_angle), 0.0, 0.0)
    world_quaternions = [earth_z_to_world_y.mul(quaternion) for quaternion in raw_quaternions]
    times = [sample.timestamp_s for sample in samples]
    world_quaternions = _hemisphere_aligned_window_average(
        times,
        world_quaternions,
        orientation_window_s,
    )

    fused = [
        ImuSample(
            timestamp_s=sample.timestamp_s,
            quaternion_wxyz=quaternion.as_tuple(),
            acceleration_xyz=sample.acceleration_xyz,
            gyro_xyz=sample.gyro_xyz,
        )
        for sample, quaternion in zip(samples, world_quaternions)
    ]
    final_bias = result["bias"][-1]
    final_sigma = float(result["biasSigma"][-1])
    radians_to_degrees = 180.0 / np.pi
    diagnostics = VqfDiagnostics(
        sample_rate_hz=1.0 / sample_period_s,
        bias_rad_s_xyz=tuple(float(value) for value in final_bias),
        bias_deg_s_xyz=tuple(float(value * radians_to_degrees) for value in final_bias),
        bias_sigma_rad_s=final_sigma,
        rest_fraction=float(np.mean(result["restDetected"])),
        orientation_window_s=orientation_window_s,
    )
    return fused, diagnostics
