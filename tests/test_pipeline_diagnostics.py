from __future__ import annotations

from math import sqrt

import numpy as np

from slam_stabilizer.core.quaternion import Quat
from slam_stabilizer.core.stabilization import FrameStabilization
from slam_stabilizer.imu import ImuSample
from slam_stabilizer.pipeline import (
    _build_frame_debug_records,
    _gravity_reference,
)


def test_gravity_reference_aligns_measured_acceleration_with_world_up() -> None:
    acceleration = (1.0, 9.6, 1.4)
    samples = [
        ImuSample(
            timestamp_s=index * 0.005,
            quaternion_wxyz=Quat.identity().as_tuple(),
            acceleration_xyz=acceleration,
            gyro_xyz=(0.0, 0.0, 0.0),
        )
        for index in range(50)
    ]

    average, alignment = _gravity_reference(samples, [sample.timestamp_s for sample in samples])

    assert average is not None
    assert alignment is not None
    magnitude = sqrt(sum(value * value for value in average))
    measured_up = tuple(value / magnitude for value in average)
    np.testing.assert_allclose(alignment.rotate_vector(measured_up), (0.0, 1.0, 0.0), atol=1e-6)


def test_frame_debug_records_identify_bracketing_imu_samples() -> None:
    samples = [
        ImuSample(0.0, Quat.identity().as_tuple(), (0.0, 9.8, 0.0), (0.0, 0.0, 0.0)),
        ImuSample(0.005, Quat.identity().as_tuple(), (0.0, 9.8, 0.0), (1.0, 2.0, 3.0)),
        ImuSample(0.010, Quat.identity().as_tuple(), (0.0, 9.8, 0.0), (2.0, 4.0, 6.0)),
    ]
    frame = FrameStabilization(
        frame_index=0,
        time_s=0.0075,
        raw_wxyz=Quat.identity().as_tuple(),
        smooth_wxyz=Quat.identity().as_tuple(),
        correction_wxyz=Quat.identity().as_tuple(),
        correction_matrix3=Quat.identity().to_matrix3(),
    )

    record = _build_frame_debug_records([frame], samples, [0.0, 0.005, 0.010], 0.0)[0]

    assert record["imu_lower_index"] == 1
    assert record["imu_upper_index"] == 2
    assert abs(float(record["imu_alpha"]) - 0.5) < 1e-9
    np.testing.assert_allclose(record["gyro_xyz"], (1.5, 3.0, 4.5), atol=1e-9)
