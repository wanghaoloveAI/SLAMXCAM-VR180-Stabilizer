from __future__ import annotations

import sqlite3

from slam_stabilizer.imu import load_slamimu


def test_load_slamimu_uses_camera_timeline_and_sensor_bias(tmp_path) -> None:
    path = tmp_path / "sample_motion.slamimu"
    connection = sqlite3.connect(path)
    connection.executescript(
        """
        CREATE TABLE session_metadata (key TEXT PRIMARY KEY, value TEXT);
        CREATE TABLE imu_samples (
            timestamp_ns INTEGER NOT NULL,
            sensor_type TEXT NOT NULL,
            x REAL, y REAL, z REAL, w REAL,
            bias_x REAL, bias_y REAL, bias_z REAL,
            accuracy INTEGER
        );
        CREATE TABLE video_frames (
            frame_number INTEGER PRIMARY KEY,
            sensor_timestamp_ns INTEGER,
            codec_pts_us INTEGER,
            rolling_shutter_skew_ns INTEGER
        );
        INSERT INTO session_metadata VALUES ('gyro_unit', 'rad/s');
        INSERT INTO video_frames VALUES (0, 1000000000, 1000000, 30000000);
        INSERT INTO video_frames VALUES (1, 1033333333, 1033333, 30000000);
        INSERT INTO video_frames VALUES (2, 1099999999, 1099999, 30000000);
        """
    )
    for timestamp_ns in (990000000, 1000000000, 1010000000, 1020000000, 1100000000):
        connection.execute(
            "INSERT INTO imu_samples VALUES (?, 'GYRO_UNCALIBRATED', 1, 0, 0, NULL, .25, 0, 0, 3)",
            (timestamp_ns,),
        )
        connection.execute(
            "INSERT INTO imu_samples VALUES (?, 'ACCEL_UNCALIBRATED', 0, 9.8, 0, NULL, 0, 0, 0, 3)",
            (timestamp_ns,),
        )
    connection.commit()
    connection.close()

    data = load_slamimu(
        path,
        axis_rotation=[
            0.0, 1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 1.0,
        ],
    )

    assert data.frame_times_s == [0.0, 0.033333333, 0.099999999]
    assert data.samples[0].timestamp_s == -0.01
    assert data.samples[0].gyro_xyz == (0.0, 0.75, 0.0)
    assert data.samples[0].acceleration_xyz == (9.8, 0.0, 0.0)
    assert data.rolling_shutter_skew_s == 0.03
