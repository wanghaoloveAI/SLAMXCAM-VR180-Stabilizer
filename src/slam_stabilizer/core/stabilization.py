from __future__ import annotations

from dataclasses import dataclass
from math import log

from .quaternion import Quat


@dataclass(frozen=True)
class SmoothParams:
    """Gyroflow-style smoothing parameters inspired by VR180 Silver Bullet."""

    smooth_ms: float = 1000.0
    fast_ms: float = 50.0
    responsiveness: float = 1.0
    max_vel_deg_s: float = 200.0
    max_correction_deg: float = 15.0


@dataclass(frozen=True)
class FrameStabilization:
    frame_index: int
    time_s: float
    raw_wxyz: tuple[float, float, float, float]
    smooth_wxyz: tuple[float, float, float, float]
    correction_wxyz: tuple[float, float, float, float]
    correction_matrix3: list[list[float]]


def interpolate_quat(times: list[float], quats: list[Quat], t: float) -> Quat:
    if not times or not quats:
        return Quat.identity()
    if t <= times[0]:
        return quats[0]
    if t >= times[-1]:
        return quats[-1]
    lo = 0
    hi = len(times) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if times[mid] <= t:
            lo = mid
        else:
            hi = mid
    span = max(1e-9, times[hi] - times[lo])
    return quats[lo].slerp(quats[hi], (t - times[lo]) / span)


def bidirectional_smooth(times: list[float], raw: list[Quat], params: SmoothParams) -> list[Quat]:
    n = len(raw)
    if n <= 1 or params.smooth_ms <= 0.0:
        return list(raw)

    velocity = [0.0] * n
    for i in range(1, n):
        dt = max(1e-6, times[i] - times[i - 1])
        velocity[i] = raw[i - 1].angular_distance_deg(raw[i]) / dt

    # Silver Bullet pre-smooths angular velocity over about 200 ms so jitter
    # does not collapse the smoother into the fast-motion path.
    for i in range(1, n):
        dt = max(1e-6, times[i] - times[i - 1])
        alpha = min(1.0, dt / 0.2)
        velocity[i] = velocity[i - 1] * (1.0 - alpha) + velocity[i] * alpha
    for i in range(n - 2, -1, -1):
        dt = max(1e-6, times[i + 1] - times[i])
        alpha = min(1.0, dt / 0.2)
        velocity[i] = velocity[i + 1] * (1.0 - alpha) + velocity[i] * alpha

    def alpha_for(i: int, dt: float) -> float:
        vel_norm = max(0.0, min(1.0, velocity[i] / max(1e-6, params.max_vel_deg_s)))
        ratio = vel_norm ** params.responsiveness
        tau_ms = params.smooth_ms * (1.0 - ratio) + params.fast_ms * ratio
        tau_s = max(1e-6, tau_ms / 1000.0)
        return dt / (tau_s + dt)

    fwd = [raw[0]]
    for i in range(1, n):
        dt = max(1e-6, times[i] - times[i - 1])
        fwd.append(fwd[i - 1].slerp(raw[i], alpha_for(i, dt)))

    bwd = [Quat.identity()] * n
    bwd[-1] = raw[-1]
    for i in range(n - 2, -1, -1):
        dt = max(1e-6, times[i + 1] - times[i])
        bwd[i] = bwd[i + 1].slerp(raw[i], alpha_for(i, dt))

    return [fwd[i].slerp(bwd[i], 0.5) for i in range(n)]


def soft_elastic_clamp(raw: Quat, smoothed: Quat, max_corr_deg: float) -> Quat:
    if max_corr_deg <= 0:
        return smoothed
    angle = raw.angular_distance_deg(smoothed)
    if angle <= max_corr_deg:
        return smoothed
    soft_angle = max_corr_deg * (1.0 + log(angle / max_corr_deg))
    t = max(0.0, min(1.0, soft_angle / max(1e-6, angle)))
    return raw.slerp(smoothed, t)


def build_frame_stabilization(
    imu_times: list[float],
    imu_quats: list[Quat],
    frame_count: int,
    frame_rate: float,
    imu_offset_s: float = 0.0,
    params: SmoothParams = SmoothParams(),
) -> list[FrameStabilization]:
    if frame_count <= 0 or frame_rate <= 0:
        return []
    smoothed = bidirectional_smooth(imu_times, imu_quats, params)
    frames: list[FrameStabilization] = []
    for frame_index in range(frame_count):
        video_t = frame_index / frame_rate
        imu_t = video_t + imu_offset_s
        raw_q = interpolate_quat(imu_times, imu_quats, imu_t)
        smooth_q = interpolate_quat(imu_times, smoothed, imu_t)
        smooth_q = soft_elastic_clamp(raw_q, smooth_q, params.max_correction_deg)
        correction = raw_q.mul(smooth_q.conjugate())
        frames.append(
            FrameStabilization(
                frame_index=frame_index,
                time_s=video_t,
                raw_wxyz=raw_q.as_tuple(),
                smooth_wxyz=smooth_q.as_tuple(),
                correction_wxyz=correction.as_tuple(),
                correction_matrix3=correction.to_matrix3(),
            )
        )
    return frames

