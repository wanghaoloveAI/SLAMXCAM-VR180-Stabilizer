from __future__ import annotations

import numpy as np

from slam_stabilizer.core.quaternion import Quat
from slam_stabilizer.imu import ImuSample
from slam_stabilizer.vqf_fusion import fuse_6d_vqf


def test_vqf_6d_stationary_aligns_acceleration_with_world_up() -> None:
    samples = [
        ImuSample(
            timestamp_s=index * 0.005,
            quaternion_wxyz=Quat.identity().as_tuple(),
            acceleration_xyz=(0.0, 0.0, 9.81),
            gyro_xyz=(0.0, 0.0, 0.0),
        )
        for index in range(400)
    ]

    fused, diagnostics = fuse_6d_vqf(samples)
    orientation = Quat.from_iter(fused[-1].quaternion_wxyz)

    np.testing.assert_allclose(
        orientation.rotate_vector((0.0, 0.0, 1.0)),
        (0.0, 1.0, 0.0),
        atol=1e-3,
    )
    assert abs(diagnostics.sample_rate_hz - 200.0) < 1e-6
    assert max(abs(value) for value in diagnostics.bias_deg_s_xyz) < 0.05
