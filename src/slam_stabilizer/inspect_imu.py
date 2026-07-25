from __future__ import annotations

import argparse
from pathlib import Path
import statistics

from .imu import load_imu_csv, summarize_rate


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect SLAM XCAM IMU CSV")
    parser.add_argument("imu", type=Path)
    parser.add_argument("--gyro-units", default="rad_s", choices=["rad_s", "deg_s"])
    args = parser.parse_args()

    samples = load_imu_csv(args.imu, gyro_units=args.gyro_units)
    timestamps = [s.timestamp_s for s in samples]
    deltas = [b - a for a, b in zip(timestamps, timestamps[1:]) if b > a]
    non_increasing = sum(1 for a, b in zip(timestamps, timestamps[1:]) if b <= a)

    print(f"file: {args.imu}")
    print(f"samples: {len(samples)}")
    print(f"duration_s: {timestamps[-1] - timestamps[0]:.6f}")
    print(f"estimated_rate_hz: {summarize_rate(samples):.3f}")
    print(f"dt_ms_min/median/max: {min(deltas) * 1000:.3f} / {statistics.median(deltas) * 1000:.3f} / {max(deltas) * 1000:.3f}")
    print(f"non_increasing_timestamps: {non_increasing}")

    for label, attr in (("acc", "acceleration_xyz"), ("gyro", "gyro_xyz")):
        values = [getattr(s, attr) for s in samples if getattr(s, attr) is not None]
        if not values:
            continue
        cols = list(zip(*values))
        summary = []
        for col in cols:
            summary.append(f"{min(col):.6g}..{max(col):.6g}, mean={statistics.mean(col):.6g}")
        print(f"{label}_xyz: {summary}")

    print(f"first_quaternion_wxyz: {samples[0].quaternion_wxyz}")
    print(f"last_quaternion_wxyz: {samples[-1].quaternion_wxyz}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

