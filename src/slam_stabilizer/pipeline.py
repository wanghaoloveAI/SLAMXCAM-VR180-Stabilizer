from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Callable

from .core.quaternion import Quat
from .core.stabilization import SmoothParams, build_frame_stabilization
from .cpu_renderer import CpuRenderOptions, render_stabilized_sbs_cpu
from .imu import load_imu_csv, summarize_rate
from .metadata import remux_with_metadata
from .models import Calibration, LensProfile, VideoInput
from .render_backend import select_render_backend
from .video_probe import probe_video_parameters

ProgressCallback = Callable[[int, str], None]
REFERENCE_RENDERER_ALIASES = {"reference-renderer", "cpu-fisheye-reprojection", "gpu-fisheye-reprojection"}
STMAP_RENDERER_ALIASES = {"stmap-renderer", "stmap-remap"}


@dataclass(frozen=True)
class StabilizationJob:
    video: VideoInput
    imu_path: Path
    lens_profile_path: Path
    calibration_path: Path | None
    output_path: Path
    ffmpeg: str = "ffmpeg"
    imu_offset_s: float = 0.0
    smooth_ms: float = 1000.0
    max_correction_deg: float = 15.0
    imu_algorithm: str = "gyro-integration-smoothing"
    distortion_correction: bool = True
    field_of_view_deg: float = 180.0
    image_algorithm: str = "reference-renderer"
    output_projection: str = "VR180 fisheye SBS"
    metadata_target: str = "YouTube VR180"
    render_mode: str = "cpu_stabilized"
    render_width: int = 1920
    render_backend_preference: str = "auto"


def run_job(job: StabilizationJob, progress: ProgressCallback | None = None) -> Path:
    def report(percent: int, message: str) -> None:
        if progress:
            progress(percent, message)

    report(2, "Validating input files")
    job.video.validate()
    if not job.imu_path.exists():
        raise FileNotFoundError("IMU file was not found.")

    report(8, "Loading lens profile and calibration")
    profile = LensProfile.from_file(job.lens_profile_path)
    calibration_path = job.calibration_path or profile.default_calibration_path
    calibration = Calibration.optional_from_file(calibration_path)
    gyro_units = calibration.gyro_units or profile.gyro_units

    input_video = job.video.input_sbs or job.video.input_left
    assert input_video is not None

    report(15, "Probing video parameters before IMU processing")
    video_params, probe = probe_video_parameters(input_video, field_of_view_deg=job.field_of_view_deg)
    if video_params.frame_rate <= 0:
        raise ValueError("Video frame rate could not be detected.")
    if video_params.frame_count <= 0:
        raise ValueError("Video frame count could not be detected or estimated.")
    backend = select_render_backend(job.render_backend_preference)
    image_algorithm = normalize_image_algorithm(job.image_algorithm)
    if image_algorithm != "reference-renderer":
        raise NotImplementedError(
            "Only Reference Renderer is active. "
            f"Selected image algorithm is not implemented yet: {image_algorithm}"
        )

    report(25, "Parsing IMU data using video timing")
    imu_samples = load_imu_csv(job.imu_path, gyro_units=gyro_units)
    imu_rate = summarize_rate(imu_samples)
    imu_times = [s.timestamp_s for s in imu_samples]
    imu_quats = [Quat.from_iter(s.quaternion_wxyz) for s in imu_samples]

    report(45, "Building per-frame IMU stabilization plan")
    frame_plan = build_frame_stabilization(
        imu_times=imu_times,
        imu_quats=imu_quats,
        frame_count=video_params.frame_count,
        frame_rate=video_params.frame_rate,
        imu_offset_s=job.imu_offset_s,
        params=SmoothParams(
            smooth_ms=job.smooth_ms,
            max_correction_deg=job.max_correction_deg,
        ),
    )

    plan = {
        "status": "prototype_plan_generated",
        "note": "Reference Renderer is active for SBS fisheye input. The render device is selected separately.",
        "input_mode": job.video.mode,
        "lens_profile": profile.raw,
        "calibration_loaded": bool(calibration.raw),
        "calibration_path": str(calibration.path) if calibration.path else None,
        "stabilization": {
            "imu_algorithm": job.imu_algorithm,
            "imu_offset_s": job.imu_offset_s,
            "smooth_ms": job.smooth_ms,
            "max_correction_deg": job.max_correction_deg,
            "distortion_correction": job.distortion_correction,
            "field_of_view_deg": job.field_of_view_deg,
            "frame_rate": video_params.frame_rate,
            "frame_count": video_params.frame_count,
            "duration_s": video_params.duration_s,
        },
        "video_parameters": video_params.to_dict(),
        "export": {
            "output_projection": job.output_projection,
            "metadata_target": job.metadata_target,
            "image_algorithm": image_algorithm,
            "image_algorithm_requested": job.image_algorithm,
            "render_mode": job.render_mode,
            "render_width": job.render_width,
            "render_backend": backend.to_dict(),
        },
        "imu": {
            "path": str(job.imu_path),
            "samples": len(imu_samples),
            "estimated_rate_hz": imu_rate,
            "samples_per_video_frame": (imu_rate / video_params.frame_rate) if video_params.frame_rate else 0.0,
            "video_frame_timing_source": "frame_index / frame_rate",
            "first_orientation_wxyz": imu_samples[0].quaternion_wxyz,
            "last_orientation_wxyz": imu_samples[-1].quaternion_wxyz,
        },
        "video_probe": probe,
        "frame_stabilization": [asdict(frame) for frame in frame_plan],
        "render_target": {
            "output_path": str(job.output_path),
            "projection": job.output_projection,
            "metadata": "left_right stereo, spherical/vr180 tags",
        },
    }

    plan_path = job.output_path.with_suffix(job.output_path.suffix + ".plan.json")
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    report(70, "Writing stabilization plan JSON")
    plan_path.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")

    if job.video.mode == "sbs":
        if job.render_mode == "remux":
            report(82, "Writing prototype video and VR180 metadata")
            remux_with_metadata(input_video, job.output_path, ffmpeg=job.ffmpeg)
        else:
            report(82, "Running Reference Renderer")
            if backend.selected != "cpu":
                report(83, f"GPU detected ({backend.selected_name}); current Reference Renderer implementation is still CPU")
            render_stabilized_sbs_cpu(
                input_video=input_video,
                output_video=job.output_path,
                frame_plan=frame_plan,
                frame_rate=video_params.frame_rate,
                options=CpuRenderOptions(
                    output_width=job.render_width,
                    distortion_correction=job.distortion_correction,
                    field_of_view_deg=job.field_of_view_deg,
                ),
                calibration=calibration.raw,
                progress=report,
            )
    else:
        raise NotImplementedError("Dual fisheye stitching is planned but not implemented in the prototype renderer.")

    report(100, "Complete")
    return plan_path


def normalize_image_algorithm(value: str) -> str:
    # Keep older UI/CLI values readable while the public model moves to two renderer algorithms.
    normalized = (value or "reference-renderer").strip().lower()
    if normalized in REFERENCE_RENDERER_ALIASES:
        return "reference-renderer"
    if normalized in STMAP_RENDERER_ALIASES:
        return "stmap-renderer"
    return normalized
