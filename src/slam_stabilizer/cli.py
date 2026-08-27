from __future__ import annotations

import argparse
from pathlib import Path

from .models import VideoInput
from .pipeline import StabilizationJob, run_job


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SLAM XCAM VR180 stabilizer prototype")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--input-sbs", type=Path, help="2:1 stitched SBS fisheye video")
    mode.add_argument("--input-left", type=Path, help="Left 1:1 fisheye video")
    parser.add_argument("--input-right", type=Path, help="Right 1:1 fisheye video, required with --input-left")
    parser.add_argument("--imu", required=True, type=Path, help="IMU CSV file")
    parser.add_argument("--lens-profile", required=True, type=Path, help="SLAM XCAM lens profile JSON")
    parser.add_argument("--calibration", type=Path, help="Optional calibrated lens/stereo JSON")
    parser.add_argument("--output", required=True, type=Path, help="Output VR180 video path")
    parser.add_argument("--ffmpeg", default="ffmpeg", help="FFmpeg executable")
    parser.add_argument("--imu-offset-s", type=float, default=-0.167, help="IMU time offset relative to video")
    parser.add_argument("--gyro-scale", type=float, default=0.45, help="Scale applied to mapped gyro values")
    parser.add_argument(
        "--max-correction-velocity-deg-s",
        type=float,
        default=25.0,
        help="Maximum frame-to-frame correction angular velocity",
    )
    parser.add_argument("--smooth-ms", type=float, default=1000.0, help="Stabilization smoothing time")
    parser.add_argument("--max-correction-deg", type=float, default=15.0, help="Soft correction limit")
    parser.add_argument(
        "--stabilization-mode",
        default="normal",
        choices=["normal", "horizon-lock"],
        help="Stabilization mode. normal is active; horizon-lock is planned.",
    )
    parser.add_argument(
        "--imu-algorithm",
        default="gyro-integration-smoothing",
        choices=["gyro-integration-smoothing", "gyro-acc-fusion", "gyroflow-style-sync"],
        help="IMU processing strategy",
    )
    parser.add_argument("--render-mode", default="cpu_stabilized", choices=["cpu_stabilized", "remux"])
    parser.add_argument(
        "--image-algorithm",
        default="reference-renderer",
        choices=["reference-renderer", "stmap-renderer", "cpu-fisheye-reprojection", "gpu-fisheye-reprojection", "stmap-remap"],
        help="Image processing algorithm. Reference Renderer is active; STMap Renderer is planned.",
    )
    parser.add_argument("--render-width", type=int, default=1920, help="CPU renderer output width")
    parser.add_argument(
        "--render-backend",
        default="auto",
        choices=["auto", "discrete_gpu", "integrated_gpu", "cpu"],
        help="Renderer device preference. Auto priority is discrete GPU, integrated GPU, then CPU.",
    )
    parser.add_argument(
        "--distortion-correction",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Apply calibrated fisheye distortion coefficients",
    )
    parser.add_argument("--field-of-view-deg", type=float, default=180.0, choices=[180.0, 190.0, 200.0])
    return parser


def main() -> int:
    args = build_parser().parse_args()
    video = VideoInput(
        mode="sbs" if args.input_sbs else "dual",
        input_sbs=args.input_sbs,
        input_left=args.input_left,
        input_right=args.input_right,
    )
    plan = run_job(
        StabilizationJob(
            video=video,
            imu_path=args.imu,
            lens_profile_path=args.lens_profile,
            calibration_path=args.calibration,
            output_path=args.output,
            ffmpeg=args.ffmpeg,
            imu_offset_s=args.imu_offset_s,
            gyro_scale=args.gyro_scale,
            max_correction_velocity_deg_s=args.max_correction_velocity_deg_s,
            smooth_ms=args.smooth_ms,
            max_correction_deg=args.max_correction_deg,
            stabilization_mode=args.stabilization_mode,
            imu_algorithm=args.imu_algorithm,
            image_algorithm=args.image_algorithm,
            render_mode=args.render_mode,
            render_width=args.render_width,
            render_backend_preference=args.render_backend,
            distortion_correction=args.distortion_correction,
            field_of_view_deg=args.field_of_view_deg,
        )
    )
    print(f"Done. Stabilization plan: {plan}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
