from __future__ import annotations

from math import sin

from slam_stabilizer.imu import load_imu_csv


def test_load_imu_csv_applies_axis_mapping_before_integration(tmp_path) -> None:
    imu_path = tmp_path / "imu.csv"
    imu_path.write_text(
        "timestamp_s,gx,gy,gz\n"
        "0,1,0,0\n"
        "1,1,0,0\n",
        encoding="utf-8",
    )

    samples = load_imu_csv(
        imu_path,
        axis_rotation=[
            0.0,
            1.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    )

    assert samples[-1].gyro_xyz == (0.0, 1.0, 0.0)
    assert abs(samples[-1].quaternion_wxyz[2] - sin(0.5)) < 1e-9


def test_load_imu_csv_applies_gyro_scale_before_integration(tmp_path) -> None:
    imu_path = tmp_path / "scaled_imu.csv"
    imu_path.write_text(
        "timestamp_s,gx,gy,gz\n"
        "0,0,0,1\n"
        "1,0,0,1\n",
        encoding="utf-8",
    )

    samples = load_imu_csv(imu_path, gyro_scale=0.45)

    assert samples[-1].gyro_xyz == (0.0, 0.0, 0.45)
    assert abs(samples[-1].quaternion_wxyz[3] - sin(0.225)) < 1e-9
