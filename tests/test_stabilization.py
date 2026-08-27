from __future__ import annotations

from math import cos, radians, sin

import numpy as np

from slam_stabilizer.core.quaternion import Quat
from slam_stabilizer.core.stabilization import (
    FrameStabilization,
    limit_correction_velocity,
    stabilization_correction,
)


def test_stabilization_correction_maps_target_output_to_raw_source() -> None:
    raw_roll = Quat(cos(radians(45.0)), 0.0, 0.0, sin(radians(45.0)))
    correction = stabilization_correction(raw_roll, Quat.identity())

    assert correction.z < 0.0
    mapped_x = np.array(correction.to_matrix3()) @ np.array([1.0, 0.0, 0.0])
    np.testing.assert_allclose(mapped_x, [0.0, -1.0, 0.0], atol=1e-6)


def test_stabilization_correction_uses_raw_inverse_then_target_order() -> None:
    raw = Quat(cos(radians(30.0)), 0.0, sin(radians(30.0)), 0.0)
    smoothed = Quat(cos(radians(20.0)), sin(radians(20.0)), 0.0, 0.0)

    correction = stabilization_correction(raw, smoothed)
    expected = raw.conjugate().mul(smoothed)

    np.testing.assert_allclose(correction.as_tuple(), expected.as_tuple(), atol=1e-9)


def test_limit_correction_velocity_caps_frame_step_and_keeps_pose_consistent() -> None:
    raw = Quat(cos(radians(5.0)), sin(radians(5.0)), 0.0, 0.0)
    requested = Quat(cos(radians(10.0)), 0.0, 0.0, sin(radians(10.0)))
    identity = Quat.identity()
    frames = [
        FrameStabilization(0, 0.0, identity.as_tuple(), identity.as_tuple(), identity.as_tuple(), identity.to_matrix3()),
        FrameStabilization(1, 0.1, raw.as_tuple(), raw.mul(requested).as_tuple(), requested.as_tuple(), requested.to_matrix3()),
    ]

    limited = limit_correction_velocity(frames, frame_rate=10.0, max_velocity_deg_s=25.0)
    correction = Quat.from_iter(limited[1].correction_wxyz)
    target = Quat.from_iter(limited[1].smooth_wxyz)

    assert abs(identity.angular_distance_deg(correction) - 2.5) < 1e-6
    assert target.angular_distance_deg(raw.mul(correction)) < 1e-6


def test_limit_correction_velocity_uses_actual_variable_frame_interval() -> None:
    identity = Quat.identity()
    requested = Quat(cos(radians(5.0)), 0.0, 0.0, sin(radians(5.0)))
    frames = [
        FrameStabilization(0, 0.0, identity.as_tuple(), identity.as_tuple(), identity.as_tuple(), identity.to_matrix3()),
        FrameStabilization(1, 0.1, identity.as_tuple(), requested.as_tuple(), requested.as_tuple(), requested.to_matrix3()),
    ]

    limited = limit_correction_velocity(frames, frame_rate=30.0, max_velocity_deg_s=25.0)

    assert abs(identity.angular_distance_deg(Quat.from_iter(limited[1].correction_wxyz)) - 2.5) < 1e-6
