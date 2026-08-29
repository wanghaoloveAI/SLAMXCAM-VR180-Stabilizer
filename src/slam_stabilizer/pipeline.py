from __future__ import annotations

from bisect import bisect_right
import csv
from dataclasses import asdict, dataclass
import json
from math import sqrt
from pathlib import Path
from typing import Callable

from .calibration_runtime import CalibrationRuntime, CalibrationRuntimeUnavailable
from .core.quaternion import Quat
from .core.rolling_shutter import build_rolling_shutter_matrices
from .core.stabilization import SmoothParams, build_frame_stabilization, limit_correction_velocity
from .cpu_renderer import (
    ROLLING_SHUTTER_ROW_ITERATIONS,
    CpuRenderOptions,
    build_cfr_frame_slots,
    render_stabilized_sbs_cpu,
)
from .gpu_renderer import GpuRendererUnavailable, render_stabilized_sbs_gpu
from .imu import ImuSample, load_imu_csv, load_slamimu, summarize_rate
from .metadata import remux_with_metadata
from .models import Calibration, LensProfile, VideoInput
from .render_backend import select_render_backend
from .runtime_renderer import render_stabilized_sbs_runtime
from .video_probe import probe_video_parameters
from .vqf_fusion import fuse_6d_vqf

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
    imu_offset_s: float = -0.167
    gyro_scale: float = 0.45
    max_correction_velocity_deg_s: float = 25.0
    smooth_ms: float = 1000.0
    max_correction_deg: float = 15.0
    imu_algorithm: str = "gyro-integration-smoothing"
    stabilization_mode: str = "normal"
    distortion_correction: bool = True
    field_of_view_deg: float = 180.0
    image_algorithm: str = "reference-renderer"
    output_projection: str = "VR180 fisheye SBS"
    metadata_target: str = "YouTube VR180"
    render_mode: str = "cpu_stabilized"
    render_width: int = 1920
    render_backend_preference: str = "auto"
    rolling_shutter_correction: bool = True


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
    calibration_runtime: CalibrationRuntime | None = None
    if job.calibration_path:
        calibration = Calibration.optional_from_file(job.calibration_path)
        if "example_public" in str(calibration.raw.get("source_schema", "")):
            raise ValueError("Public example calibration cannot be used for export.")
        calibration_source = "custom_json"
    else:
        try:
            candidate_runtime = CalibrationRuntime()
            if not candidate_runtime.supports(profile.camera_model):
                raise CalibrationRuntimeUnavailable(
                    f"Official Calibration Runtime does not support {profile.camera_model}."
                )
            calibration_runtime = candidate_runtime
            calibration = Calibration(path=None, camera_model=profile.camera_model, gyro_units=None, raw={})
            calibration_source = "official_runtime"
            report(10, f"Official Calibration Runtime loaded: {candidate_runtime.info.version}")
        except CalibrationRuntimeUnavailable as exc:
            raise CalibrationRuntimeUnavailable(str(exc)) from exc
    gyro_units = calibration.gyro_units or profile.gyro_units
    imu_to_camera_rotation = calibration.raw.get("imu_to_camera_rotation") or profile.raw.get(
        "imu_to_camera_rotation"
    )

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
    stabilization_mode = normalize_stabilization_mode(job.stabilization_mode)
    if image_algorithm != "reference-renderer":
        raise NotImplementedError(
            "Only Reference Renderer is active. "
            f"Selected image algorithm is not implemented yet: {image_algorithm}"
        )

    report(25, "Parsing IMU data using video timing")
    slamimu_data = None
    if job.imu_path.suffix.lower() == ".slamimu":
        slamimu_data = load_slamimu(
            job.imu_path,
            axis_rotation=imu_to_camera_rotation,
            gyro_filter_window_s=0.0 if job.imu_algorithm == "gyro-acc-fusion" else 0.015,
        )
        imu_samples = slamimu_data.samples
        imu_times = [sample.timestamp_s for sample in imu_samples]
        frame_times_s = slamimu_data.frame_times_s
        frame_pose_times_s = [
            time_s + (slamimu_data.rolling_shutter_skew_s or 0.0) * 0.5
            for time_s in frame_times_s
        ]
        if len(frame_times_s) != video_params.frame_count:
            raise ValueError(
                "SLAM IMU/video frame mismatch: "
                f"database has {len(frame_times_s)} frames, video has {video_params.frame_count}."
            )
        effective_imu_offset_s = 0.0
        effective_gyro_scale = 1.0
        effective_max_correction_velocity_deg_s = 200.0
        imu_time_origin_s = slamimu_data.timestamp_origin_ns / 1_000_000_000.0
        frame_timing_source = "slamimu video_frames.sensor_timestamp_ns"
        dropped_intervals = sum(
            1
            for before, after in zip(frame_times_s, frame_times_s[1:])
            if after - before > (1.5 / max(1e-9, video_params.frame_rate))
        )
        report(
            35,
            (
                f"SLAM IMU synchronized: {len(imu_samples)} gyro samples, "
                f"{len(frame_times_s)} video frames, {dropped_intervals} long frame intervals"
            ),
        )
    else:
        imu_samples = load_imu_csv(
            job.imu_path,
            gyro_units=gyro_units,
            axis_rotation=imu_to_camera_rotation,
            gyro_scale=job.gyro_scale,
        )
        imu_time_origin_s = imu_samples[0].timestamp_s
        imu_times = [sample.timestamp_s - imu_time_origin_s for sample in imu_samples]
        frame_times_s = None
        frame_pose_times_s = None
        effective_imu_offset_s = job.imu_offset_s
        effective_gyro_scale = job.gyro_scale
        effective_max_correction_velocity_deg_s = job.max_correction_velocity_deg_s
        frame_timing_source = "frame_index / average_frame_rate"
        dropped_intervals = 0
    imu_rate = summarize_rate(imu_samples)
    vqf_diagnostics = None
    if job.imu_algorithm == "gyro-acc-fusion":
        report(40, "Running 6D VQF gyro + acceleration fusion")
        imu_samples, vqf_diagnostics = fuse_6d_vqf(imu_samples)
    imu_quats = [Quat.from_iter(s.quaternion_wxyz) for s in imu_samples]
    if vqf_diagnostics is not None:
        gravity_vector = None
        gravity_alignment = None
    else:
        gravity_vector, gravity_alignment = _gravity_reference(
            imu_samples,
            imu_times,
            window_start_s=-0.25 if slamimu_data else 0.0,
            window_end_s=0.0 if slamimu_data else 0.25,
        )
        if gravity_alignment is not None:
            imu_quats = [gravity_alignment.mul(q) for q in imu_quats]
    effective_max_correction_deg = 90.0 if stabilization_mode == "horizon-lock" else job.max_correction_deg

    report(45, "Building per-frame IMU stabilization plan")
    frame_plan = build_frame_stabilization(
        imu_times=imu_times,
        imu_quats=imu_quats,
        frame_count=video_params.frame_count,
        frame_rate=video_params.frame_rate,
        imu_offset_s=effective_imu_offset_s,
        params=SmoothParams(
            smooth_ms=job.smooth_ms,
            max_correction_deg=effective_max_correction_deg,
        ),
        stabilization_mode=stabilization_mode,
        frame_times_s=frame_times_s,
        imu_query_times_s=frame_pose_times_s,
    )
    frame_plan = limit_correction_velocity(
        frame_plan,
        frame_rate=video_params.frame_rate,
        max_velocity_deg_s=effective_max_correction_velocity_deg_s,
    )
    output_frame_rate, output_frame_slots = build_cfr_frame_slots(
        [frame.time_s for frame in frame_plan],
        video_params.frame_rate,
    )
    rolling_shutter_plan = None
    rolling_shutter_max_row_correction_deg = 0.0
    if (
        job.rolling_shutter_correction
        and slamimu_data is not None
        and slamimu_data.rolling_shutter_skew_s
    ):
        report(52, "Building per-row rolling-shutter correction matrices")
        rolling_shutter_plan, rolling_shutter_max_row_correction_deg = build_rolling_shutter_matrices(
            imu_times=imu_times,
            imu_quaternions=imu_quats,
            frame_start_times_s=frame_times_s or [],
            readout_s=slamimu_data.rolling_shutter_skew_s,
            row_count=max(1, job.render_width // 2),
        )
    correction_angles = [
        Quat.identity().angular_distance_deg(Quat.from_iter(frame.correction_wxyz))
        for frame in frame_plan
    ]
    max_correction_angle = max(correction_angles) if correction_angles else 0.0
    avg_correction_angle = (sum(correction_angles) / len(correction_angles)) if correction_angles else 0.0
    if max_correction_angle <= 0.01:
        report(
            60,
            "Warning: stabilization correction is effectively zero; check IMU values, units, timing, and axis mapping",
        )
    else:
        report(
            60,
            (
                "Stabilization corrections generated: "
                f"average {avg_correction_angle:.2f} deg, maximum {max_correction_angle:.2f} deg"
            ),
        )
    frame_debug = _build_frame_debug_records(
        frame_plan=frame_plan,
        imu_samples=imu_samples,
        imu_times=imu_times,
        imu_offset_s=effective_imu_offset_s,
        imu_query_times_s=frame_pose_times_s,
    )
    for index, record in enumerate(frame_debug):
        percent = 61 + int((index + 1) * 7 / max(1, len(frame_debug)))
        report(percent, _frame_debug_message(record))

    plan_path = job.output_path.with_suffix(job.output_path.suffix + ".plan.json")
    frame_log_path = job.output_path.with_suffix(job.output_path.suffix + ".frames.csv")
    plan = {
        "status": "prototype_plan_generated",
        "note": "Reference Renderer is active for SBS fisheye input. The render device is selected separately.",
        "input_mode": job.video.mode,
        "lens_profile": profile.raw,
        "calibration_loaded": bool(calibration.raw) or calibration_runtime is not None,
        "calibration_path": str(calibration.path) if calibration.path else None,
        "calibration_source": calibration_source,
        "calibration_runtime": (
            {
                "version": calibration_runtime.info.version,
                "abi": 1,
                "verified": True,
            }
            if calibration_runtime
            else None
        ),
        "stabilization": {
            "imu_algorithm": job.imu_algorithm,
            "stabilization_mode": stabilization_mode,
            "imu_offset_s": effective_imu_offset_s,
            "requested_imu_offset_s": job.imu_offset_s,
            "gyro_scale": effective_gyro_scale,
            "requested_gyro_scale": job.gyro_scale,
            "max_correction_velocity_deg_s": effective_max_correction_velocity_deg_s,
            "requested_max_correction_velocity_deg_s": job.max_correction_velocity_deg_s,
            "smooth_ms": job.smooth_ms,
            "requested_max_correction_deg": job.max_correction_deg,
            "effective_max_correction_deg": effective_max_correction_deg,
            "max_generated_correction_deg": max_correction_angle,
            "avg_generated_correction_deg": avg_correction_angle,
            "correction_convention": "inverse(raw) * target",
            "distortion_correction": job.distortion_correction,
            "field_of_view_deg": job.field_of_view_deg,
            "frame_rate": video_params.frame_rate,
            "frame_count": video_params.frame_count,
            "duration_s": video_params.duration_s,
            "output_frame_rate": output_frame_rate,
            "output_frame_count": (
                output_frame_slots[-1] + 1 if output_frame_slots else len(frame_plan)
            ),
            "duplicated_timing_slots": (
                output_frame_slots[-1] + 1 - len(frame_plan) if output_frame_slots else 0
            ),
            "rolling_shutter_correction": bool(rolling_shutter_plan),
            "rolling_shutter_row_iterations": (
                ROLLING_SHUTTER_ROW_ITERATIONS if rolling_shutter_plan else 0
            ),
            "rolling_shutter_max_row_correction_deg": rolling_shutter_max_row_correction_deg,
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
            "video_frame_timing_source": frame_timing_source,
            "long_frame_intervals": dropped_intervals,
            "timestamp_origin_s": imu_time_origin_s,
            "normalized_timeline": True,
            "axis_mapping": "imu_to_camera_rotation",
            "imu_to_camera_rotation": imu_to_camera_rotation,
            "gravity_reference_camera_xyz": gravity_vector,
            "gravity_alignment_wxyz": gravity_alignment.as_tuple() if gravity_alignment else None,
            "rolling_shutter_skew_s": (
                slamimu_data.rolling_shutter_skew_s if slamimu_data else None
            ),
            "gyro_filter_window_s": (
                slamimu_data.gyro_filter_window_s if slamimu_data else None
            ),
            "session_metadata": slamimu_data.metadata if slamimu_data else None,
            "vqf_6d": asdict(vqf_diagnostics) if vqf_diagnostics else None,
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
        "diagnostics": {
            "frame_log_csv": str(frame_log_path),
            "frame_log_schema": "per video frame: bracketing IMU samples, interpolated sensors, poses, correction axis/angle/matrix",
        },
        "frame_debug": frame_debug,
    }

    plan_path.parent.mkdir(parents=True, exist_ok=True)
    report(69, f"Writing per-frame diagnostics: {frame_log_path}")
    _write_frame_debug_csv(frame_log_path, frame_debug)
    report(70, "Writing stabilization plan JSON")
    plan_path.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")

    if job.video.mode == "sbs":
        if job.render_mode == "remux":
            report(82, "Writing prototype video and VR180 metadata")
            remux_with_metadata(input_video, job.output_path, ffmpeg=job.ffmpeg)
        else:
            report(82, "Running Reference Renderer")
            render_options = CpuRenderOptions(
                output_width=job.render_width,
                distortion_correction=job.distortion_correction,
                field_of_view_deg=job.field_of_view_deg,
            )
            renderer_execution = {
                "requested": backend.selected,
                "actual": "cpu",
                "name": "CPU Reference Renderer",
                "fallback_reason": None,
            }
            if calibration_runtime is not None:
                runtime_result = render_stabilized_sbs_runtime(
                    input_video=input_video,
                    output_video=job.output_path,
                    frame_plan=frame_plan,
                    frame_rate=video_params.frame_rate,
                    options=render_options,
                    runtime=calibration_runtime,
                    camera_model=profile.camera_model,
                    rolling_shutter_plan=rolling_shutter_plan,
                    progress=report,
                )
                renderer_execution.update(
                    {
                        "requested": "official_calibration_runtime",
                        "actual": "native_calibration_runtime",
                        "name": runtime_result.renderer_name,
                        "api": runtime_result.api,
                    }
                )
            elif backend.selected != "cpu":
                try:
                    gpu_result = render_stabilized_sbs_gpu(
                        input_video=input_video,
                        output_video=job.output_path,
                        frame_plan=frame_plan,
                        frame_rate=video_params.frame_rate,
                        options=render_options,
                        calibration=calibration.raw,
                        rolling_shutter_plan=rolling_shutter_plan,
                        progress=report,
                    )
                    renderer_execution.update(
                        {
                            "actual": "gpu_opengl",
                            "name": gpu_result.renderer_name,
                            "api": gpu_result.api,
                        }
                    )
                except GpuRendererUnavailable as exc:
                    report(83, f"GPU Renderer unavailable; falling back to CPU: {exc}")
                    renderer_execution["fallback_reason"] = str(exc)
                    render_stabilized_sbs_cpu(
                        input_video=input_video,
                        output_video=job.output_path,
                        frame_plan=frame_plan,
                        frame_rate=video_params.frame_rate,
                        options=render_options,
                        calibration=calibration.raw,
                        rolling_shutter_plan=rolling_shutter_plan,
                        progress=report,
                    )
            else:
                render_stabilized_sbs_cpu(
                    input_video=input_video,
                    output_video=job.output_path,
                    frame_plan=frame_plan,
                    frame_rate=video_params.frame_rate,
                    options=render_options,
                    calibration=calibration.raw,
                    rolling_shutter_plan=rolling_shutter_plan,
                    progress=report,
                )
            plan["export"]["renderer_execution"] = renderer_execution
            plan["export"]["render_mode"] = (
                "gpu_stabilized"
                if renderer_execution["actual"] == "gpu_opengl"
                else (
                    "official_runtime_stabilized"
                    if renderer_execution["actual"] == "native_calibration_runtime"
                    else "cpu_stabilized"
                )
            )
            plan_path.write_text(json.dumps(plan, ensure_ascii=False, indent=2), encoding="utf-8")
    else:
        raise NotImplementedError("Dual fisheye stitching is planned but not implemented in the prototype renderer.")

    report(100, "Complete")
    return plan_path


def _gravity_reference(
    samples: list[ImuSample],
    normalized_times: list[float],
    window_start_s: float = 0.0,
    window_end_s: float = 0.25,
) -> tuple[tuple[float, float, float] | None, Quat | None]:
    vectors = [
        sample.acceleration_xyz
        for sample, time_s in zip(samples, normalized_times)
        if window_start_s <= time_s <= window_end_s and sample.acceleration_xyz is not None
    ]
    if not vectors:
        return None, None
    average = tuple(sum(vector[axis] for vector in vectors) / len(vectors) for axis in range(3))
    magnitude = sqrt(sum(value * value for value in average))
    if magnitude < 2.0 or magnitude > 20.0:
        return average, None
    source = tuple(value / magnitude for value in average)
    target = (0.0, 1.0, 0.0)
    dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(source, target))))
    cross = (
        source[1] * target[2] - source[2] * target[1],
        source[2] * target[0] - source[0] * target[2],
        source[0] * target[1] - source[1] * target[0],
    )
    if dot < -0.999999:
        return average, Quat(0.0, 1.0, 0.0, 0.0)
    return average, Quat(1.0 + dot, cross[0], cross[1], cross[2]).normalized()


def _interpolate_vector(
    before: tuple[float, float, float] | None,
    after: tuple[float, float, float] | None,
    alpha: float,
) -> tuple[float, float, float] | None:
    if before is None:
        return after
    if after is None:
        return before
    return tuple(before[i] + (after[i] - before[i]) * alpha for i in range(3))


def _build_frame_debug_records(
    frame_plan,
    imu_samples: list[ImuSample],
    imu_times: list[float],
    imu_offset_s: float,
    imu_query_times_s: list[float] | None = None,
) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    previous_correction: Quat | None = None
    previous_time_s: float | None = None
    for frame in frame_plan:
        pose_time_s = (
            imu_query_times_s[frame.frame_index]
            if imu_query_times_s is not None
            else frame.time_s
        )
        imu_time_s = pose_time_s + imu_offset_s
        upper = min(len(imu_times) - 1, bisect_right(imu_times, imu_time_s))
        lower = max(0, upper - 1)
        span = max(1e-9, imu_times[upper] - imu_times[lower])
        alpha = 0.0 if lower == upper else max(0.0, min(1.0, (imu_time_s - imu_times[lower]) / span))
        gyro = _interpolate_vector(imu_samples[lower].gyro_xyz, imu_samples[upper].gyro_xyz, alpha)
        acceleration = _interpolate_vector(
            imu_samples[lower].acceleration_xyz,
            imu_samples[upper].acceleration_xyz,
            alpha,
        )
        correction = Quat.from_iter(frame.correction_wxyz)
        correction_angle_deg = Quat.identity().angular_distance_deg(correction)
        correction_delta_deg = (
            previous_correction.angular_distance_deg(correction)
            if previous_correction is not None
            else 0.0
        )
        frame_dt_s = (
            max(1e-9, frame.time_s - previous_time_s)
            if previous_time_s is not None
            else 0.0
        )
        correction_velocity_deg_s = correction_delta_deg / frame_dt_s if frame_dt_s > 0.0 else 0.0
        half_angle_sin = sqrt(max(0.0, 1.0 - correction.w * correction.w))
        if half_angle_sin > 1e-9:
            correction_axis = (
                correction.x / half_angle_sin,
                correction.y / half_angle_sin,
                correction.z / half_angle_sin,
            )
        else:
            correction_axis = (0.0, 0.0, 0.0)
        records.append(
            {
                "frame_index": frame.frame_index,
                "video_time_s": frame.time_s,
                "imu_query_time_s": imu_time_s,
                "imu_lower_index": lower,
                "imu_lower_time_s": imu_times[lower],
                "imu_upper_index": upper,
                "imu_upper_time_s": imu_times[upper],
                "imu_alpha": alpha,
                "gyro_xyz": gyro,
                "acceleration_xyz": acceleration,
                "raw_wxyz": frame.raw_wxyz,
                "target_wxyz": frame.smooth_wxyz,
                "correction_wxyz": frame.correction_wxyz,
                "correction_angle_deg": correction_angle_deg,
                "correction_delta_deg": correction_delta_deg,
                "correction_velocity_deg_s": correction_velocity_deg_s,
                "correction_axis_xyz": correction_axis,
                "correction_matrix3": frame.correction_matrix3,
            }
        )
        previous_correction = correction
        previous_time_s = frame.time_s
    return records


def _frame_debug_message(record: dict[str, object]) -> str:
    gyro = record["gyro_xyz"] or (0.0, 0.0, 0.0)
    axis = record["correction_axis_xyz"]
    return (
        f"Frame {int(record['frame_index']):06d} | "
        f"video={float(record['video_time_s']):.6f}s | "
        f"imu={float(record['imu_query_time_s']):.6f}s | "
        f"samples={record['imu_lower_index']}->{record['imu_upper_index']} "
        f"a={float(record['imu_alpha']):.3f} | "
        f"gyro=({gyro[0]:+.5f},{gyro[1]:+.5f},{gyro[2]:+.5f}) | "
        f"corr={float(record['correction_angle_deg']):.3f}deg "
        f"delta={float(record['correction_delta_deg']):.3f}deg "
        f"vel={float(record['correction_velocity_deg_s']):.2f}deg/s "
        f"axis=({axis[0]:+.3f},{axis[1]:+.3f},{axis[2]:+.3f})"
    )


def _write_frame_debug_csv(path: Path, records: list[dict[str, object]]) -> None:
    fields = [
        "frame_index", "video_time_s", "imu_query_time_s",
        "imu_lower_index", "imu_lower_time_s", "imu_upper_index", "imu_upper_time_s", "imu_alpha",
        "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z",
        "raw_w", "raw_x", "raw_y", "raw_z",
        "target_w", "target_x", "target_y", "target_z",
        "correction_w", "correction_x", "correction_y", "correction_z",
        "correction_angle_deg", "correction_delta_deg", "correction_velocity_deg_s",
        "axis_x", "axis_y", "axis_z",
        "m00", "m01", "m02", "m10", "m11", "m12", "m20", "m21", "m22",
    ]
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for record in records:
            gyro = record["gyro_xyz"] or ("", "", "")
            acceleration = record["acceleration_xyz"] or ("", "", "")
            raw = record["raw_wxyz"]
            target = record["target_wxyz"]
            correction = record["correction_wxyz"]
            axis = record["correction_axis_xyz"]
            matrix = record["correction_matrix3"]
            writer.writerow(
                {
                    "frame_index": record["frame_index"],
                    "video_time_s": record["video_time_s"],
                    "imu_query_time_s": record["imu_query_time_s"],
                    "imu_lower_index": record["imu_lower_index"],
                    "imu_lower_time_s": record["imu_lower_time_s"],
                    "imu_upper_index": record["imu_upper_index"],
                    "imu_upper_time_s": record["imu_upper_time_s"],
                    "imu_alpha": record["imu_alpha"],
                    "gyro_x": gyro[0], "gyro_y": gyro[1], "gyro_z": gyro[2],
                    "acc_x": acceleration[0], "acc_y": acceleration[1], "acc_z": acceleration[2],
                    "raw_w": raw[0], "raw_x": raw[1], "raw_y": raw[2], "raw_z": raw[3],
                    "target_w": target[0], "target_x": target[1], "target_y": target[2], "target_z": target[3],
                    "correction_w": correction[0], "correction_x": correction[1],
                    "correction_y": correction[2], "correction_z": correction[3],
                    "correction_angle_deg": record["correction_angle_deg"],
                    "correction_delta_deg": record["correction_delta_deg"],
                    "correction_velocity_deg_s": record["correction_velocity_deg_s"],
                    "axis_x": axis[0], "axis_y": axis[1], "axis_z": axis[2],
                    "m00": matrix[0][0], "m01": matrix[0][1], "m02": matrix[0][2],
                    "m10": matrix[1][0], "m11": matrix[1][1], "m12": matrix[1][2],
                    "m20": matrix[2][0], "m21": matrix[2][1], "m22": matrix[2][2],
                }
            )


def normalize_image_algorithm(value: str) -> str:
    # Keep older UI/CLI values readable while the public model moves to two renderer algorithms.
    normalized = (value or "reference-renderer").strip().lower()
    if normalized in REFERENCE_RENDERER_ALIASES:
        return "reference-renderer"
    if normalized in STMAP_RENDERER_ALIASES:
        return "stmap-renderer"
    return normalized


def normalize_stabilization_mode(value: str) -> str:
    normalized = (value or "normal").strip().lower()
    if normalized in {"normal", "ordinary", "standard"}:
        return "normal"
    if normalized in {"horizon-lock", "horizon", "horizon-stabilization"}:
        return "horizon-lock"
    return normalized
