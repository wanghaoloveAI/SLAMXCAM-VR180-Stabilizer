from __future__ import annotations

from math import cos, radians, sin

from slam_stabilizer.core.quaternion import Quat
from slam_stabilizer.core.stabilization import horizon_locked_target


def test_horizon_locked_target_removes_roll_around_forward_axis() -> None:
    rolled = Quat(cos(radians(45.0)), 0.0, 0.0, sin(radians(45.0)))
    locked = horizon_locked_target(rolled)

    up = locked.rotate_vector((0.0, 1.0, 0.0))
    forward = locked.rotate_vector((0.0, 0.0, 1.0))

    assert abs(up[0]) < 1e-9
    assert abs(up[1] - 1.0) < 1e-9
    assert abs(up[2]) < 1e-9
    assert abs(forward[0]) < 1e-9
    assert abs(forward[1]) < 1e-9
    assert abs(forward[2] - 1.0) < 1e-9


def test_horizon_locked_target_preserves_local_z_optical_direction() -> None:
    yaw_pitch = Quat(cos(radians(25.0)), 0.0, sin(radians(25.0)), 0.0).mul(
        Quat(cos(radians(15.0)), sin(radians(15.0)), 0.0, 0.0)
    )
    rolled = yaw_pitch.mul(Quat(cos(radians(35.0)), 0.0, 0.0, sin(radians(35.0))))
    locked = horizon_locked_target(rolled)

    original_forward = rolled.rotate_vector((0.0, 0.0, 1.0))
    locked_forward = locked.rotate_vector((0.0, 0.0, 1.0))

    for original, corrected in zip(original_forward, locked_forward):
        assert abs(original - corrected) < 1e-9
