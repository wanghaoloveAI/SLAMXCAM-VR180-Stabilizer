from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from statistics import median
import subprocess
from typing import Any

import numpy as np

from .core.quaternion import Quat
from .core.stabilization import FrameStabilization
from .process import hidden_subprocess_kwargs


ROLLING_SHUTTER_ROW_ITERATIONS = 2


@dataclass(frozen=True)
class CpuRenderOptions:
    output_width: int = 1920
    crf: int = 20
    preset: str = "veryfast"
    distortion_correction: bool = True
    field_of_view_deg: float = 180.0


def build_cfr_frame_slots(
    frame_times_s: list[float],
    fallback_frame_rate: float,
) -> tuple[float, list[int]]:
    """Map variable capture timestamps onto nominal CFR slots without retiming frames."""

    if not frame_times_s:
        return max(1e-9, fallback_frame_rate), []
    intervals = [
        current - previous
        for previous, current in zip(frame_times_s, frame_times_s[1:])
        if current > previous
    ]
    nominal_frame_rate = (
        1.0 / median(intervals)
        if intervals
        else max(1e-9, fallback_frame_rate)
    )
    origin = frame_times_s[0]
    slots: list[int] = []
    previous_slot = -1
    for timestamp in frame_times_s:
        desired_slot = int(round((timestamp - origin) * nominal_frame_rate))
        slot = max(previous_slot + 1, desired_slot)
        slots.append(slot)
        previous_slot = slot
    return nominal_frame_rate, slots


def _build_eye_maps(
    eye_size: int,
    matrix: np.ndarray,
    lens: dict[str, Any] | None = None,
    source_size: tuple[int, int] | None = None,
    distortion_correction: bool = True,
    field_of_view_deg: float = 180.0,
    rolling_shutter_matrices: np.ndarray | None = None,
    rolling_shutter_iterations: int = ROLLING_SHUTTER_ROW_ITERATIONS,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    center = (eye_size - 1) / 2.0
    radius = eye_size / 2.0
    yy, xx = np.indices((eye_size, eye_size), dtype=np.float32)
    dx = (xx - center) / radius
    dy = (center - yy) / radius
    r_norm = np.sqrt(dx * dx + dy * dy)
    valid = r_norm <= 1.0

    phi = np.arctan2(dy, dx)
    half_fov_rad = np.deg2rad(float(field_of_view_deg)) / 2.0
    theta = r_norm * half_fov_rad
    sin_t = np.sin(theta)
    dirs = np.stack(
        [
            sin_t * np.cos(phi),
            sin_t * np.sin(phi),
            np.cos(theta),
        ],
        axis=0,
    ).reshape(3, -1)

    base_src_dirs = matrix @ dirs
    src_dirs = base_src_dirs
    lens_rotation = None
    if lens and lens.get("output_to_lens_rotation"):
        lens_rotation = np.array(lens["output_to_lens_rotation"], dtype=np.float32).reshape(3, 3)
    if rolling_shutter_matrices is not None and len(rolling_shutter_matrices) > 0:
        # The row-specific rotation changes the source row itself. Resolve that
        # dependency twice, always applying the selected matrix to the base ray.
        for _ in range(max(1, int(rolling_shutter_iterations))):
            lookup_dirs = lens_rotation @ src_dirs if lens_rotation is not None else src_dirs
            lookup_z = np.clip(lookup_dirs[2].reshape(eye_size, eye_size), -1.0, 1.0)
            lookup_theta = np.arccos(lookup_z)
            lookup_phi = np.arctan2(
                lookup_dirs[1].reshape(eye_size, eye_size),
                lookup_dirs[0].reshape(eye_size, eye_size),
            )
            if lens:
                _, lookup_y = _project_calibrated_fisheye(
                    theta=lookup_theta,
                    phi=lookup_phi,
                    eye_size=eye_size,
                    lens=lens,
                    source_size=source_size,
                    distortion_correction=distortion_correction,
                )
            else:
                lookup_radius = (lookup_theta / half_fov_rad) * radius
                lookup_y = center - lookup_radius * np.sin(lookup_phi)
            source_rows = np.clip(
                np.rint(lookup_y),
                0,
                len(rolling_shutter_matrices) - 1,
            ).astype(np.int32)
            selected = rolling_shutter_matrices[source_rows.reshape(-1)]
            src_dirs = np.einsum("nij,nj->ni", selected, base_src_dirs.T).T
    if lens_rotation is not None:
        src_dirs = lens_rotation @ src_dirs
    x = src_dirs[0].reshape(eye_size, eye_size)
    y = src_dirs[1].reshape(eye_size, eye_size)
    z = np.clip(src_dirs[2].reshape(eye_size, eye_size), -1.0, 1.0)

    src_theta = np.arccos(z)
    src_phi = np.arctan2(y, x)

    if lens:
        src_x, src_y = _project_calibrated_fisheye(
            theta=src_theta,
            phi=src_phi,
            eye_size=eye_size,
            lens=lens,
            source_size=source_size,
            distortion_correction=distortion_correction,
        )
    else:
        src_r = (src_theta / half_fov_rad) * radius
        src_x = center + src_r * np.cos(src_phi)
        src_y = center - src_r * np.sin(src_phi)

    valid &= src_theta <= half_fov_rad
    valid &= src_x >= 0
    valid &= src_x < eye_size - 1
    valid &= src_y >= 0
    valid &= src_y < eye_size - 1
    return src_x.astype(np.float32), src_y.astype(np.float32), valid


def _project_calibrated_fisheye(
    theta: np.ndarray,
    phi: np.ndarray,
    eye_size: int,
    lens: dict[str, Any],
    source_size: tuple[int, int] | None,
    distortion_correction: bool = True,
) -> tuple[np.ndarray, np.ndarray]:
    source_width = float((source_size or (eye_size, eye_size))[0] or eye_size)
    source_height = float((source_size or (eye_size, eye_size))[1] or eye_size)
    scale_x = eye_size / source_width
    scale_y = eye_size / source_height

    fx = float(lens.get("fx", eye_size / 2.0)) * scale_x
    fy = float(lens.get("fy", eye_size / 2.0)) * scale_y
    cx = float(lens.get("cx", (source_width - 1.0) / 2.0)) * scale_x
    cy = float(lens.get("cy", (source_height - 1.0) / 2.0)) * scale_y
    coeffs = [float(v) for v in lens.get("distortion", [])[:4]] if distortion_correction else []
    coeffs += [0.0] * (4 - len(coeffs))
    k1, k2, k3, k4 = coeffs

    theta2 = theta * theta
    theta4 = theta2 * theta2
    theta6 = theta4 * theta2
    theta8 = theta4 * theta4
    theta_d = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8)

    src_x = cx + fx * theta_d * np.cos(phi)
    src_y = cy - fy * theta_d * np.sin(phi)
    return src_x, src_y


def _sample_bilinear(image: np.ndarray, map_x: np.ndarray, map_y: np.ndarray, valid: np.ndarray) -> np.ndarray:
    h, w, channels = image.shape
    x0 = np.floor(map_x).astype(np.int32)
    y0 = np.floor(map_y).astype(np.int32)
    x1 = np.clip(x0 + 1, 0, w - 1)
    y1 = np.clip(y0 + 1, 0, h - 1)
    x0 = np.clip(x0, 0, w - 1)
    y0 = np.clip(y0, 0, h - 1)

    wx = (map_x - x0)[..., None]
    wy = (map_y - y0)[..., None]
    top = image[y0, x0] * (1.0 - wx) + image[y0, x1] * wx
    bottom = image[y1, x0] * (1.0 - wx) + image[y1, x1] * wx
    sampled = top * (1.0 - wy) + bottom * wy

    out = np.zeros((h, w, channels), dtype=np.uint8)
    out[valid] = np.clip(sampled[valid], 0, 255).astype(np.uint8)
    return out


def _render_sbs_frame(
    frame: np.ndarray,
    matrix: np.ndarray,
    left_lens: dict[str, Any] | None = None,
    right_lens: dict[str, Any] | None = None,
    source_size: tuple[int, int] | None = None,
    distortion_correction: bool = True,
    field_of_view_deg: float = 180.0,
    rolling_shutter_matrices: np.ndarray | None = None,
) -> np.ndarray:
    height, width, _ = frame.shape
    eye_size = height
    left = frame[:, :eye_size, :]
    right = frame[:, eye_size : eye_size * 2, :]
    out = np.zeros_like(frame)

    left_map_x, left_map_y, left_valid = _build_eye_maps(
        eye_size,
        matrix,
        left_lens,
        source_size,
        distortion_correction,
        field_of_view_deg,
        rolling_shutter_matrices,
    )
    right_map_x, right_map_y, right_valid = _build_eye_maps(
        eye_size,
        matrix,
        right_lens,
        source_size,
        distortion_correction,
        field_of_view_deg,
        rolling_shutter_matrices,
    )
    out[:, :eye_size, :] = _sample_bilinear(left, left_map_x, left_map_y, left_valid)
    out[:, eye_size : eye_size * 2, :] = _sample_bilinear(right, right_map_x, right_map_y, right_valid)
    return out


def _source_size_from_calibration(calibration: dict[str, Any] | None) -> tuple[int, int] | None:
    if not calibration:
        return None
    image_size = calibration.get("image_size") or {}
    width = int(image_size.get("width", 0) or 0)
    height = int(image_size.get("height", 0) or 0)
    if width > 0 and height > 0:
        return width, height
    return None


def render_stabilized_sbs_cpu(
    input_video: Path,
    output_video: Path,
    frame_plan: list[FrameStabilization],
    frame_rate: float,
    options: CpuRenderOptions,
    calibration: dict[str, Any] | None = None,
    rolling_shutter_plan: list[np.ndarray] | None = None,
    progress=None,
) -> None:
    output_width = max(640, int(options.output_width))
    if output_width % 2:
        output_width += 1
    output_height = output_width // 2
    eye_size = output_height
    frame_size = output_width * output_height * 3
    output_frame_rate, frame_slots = build_cfr_frame_slots(
        [frame.time_s for frame in frame_plan],
        frame_rate,
    )
    output_frame_count = frame_slots[-1] + 1 if frame_slots else len(frame_plan)
    output_video.parent.mkdir(parents=True, exist_ok=True)
    left_lens = calibration.get("left") if calibration else None
    right_lens = calibration.get("right") if calibration else None
    source_size = _source_size_from_calibration(calibration)

    decode_args = [
        "ffmpeg",
        "-v",
        "error",
        "-i",
        str(input_video),
        "-an",
        "-vf",
        f"scale={output_width}:{output_height}",
        # Preserve one decoded image per captured frame. FFmpeg's default
        # timestamp sync may otherwise insert frames before IMU correction,
        # shifting every subsequent frame away from its Camera2 timestamp.
        "-vsync",
        "0",
        "-pix_fmt",
        "rgb24",
        "-f",
        "rawvideo",
        "-",
    ]
    encode_args = [
        "ffmpeg",
        "-y",
        "-v",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{output_width}x{output_height}",
        "-r",
        f"{output_frame_rate:.6f}",
        "-i",
        "-",
        "-i",
        str(input_video),
        "-map",
        "0:v:0",
        "-map",
        "1:a?",
        "-c:v",
        "libx264",
        "-preset",
        options.preset,
        "-crf",
        str(options.crf),
        "-pix_fmt",
        "yuv420p",
        "-c:a",
        "copy",
        "-movflags",
        "+faststart",
        "-t",
        f"{output_frame_count / max(1e-9, output_frame_rate):.6f}",
        "-metadata:s:v:0",
        "stereo_mode=left_right",
        "-metadata:s:v:0",
        "projection=fisheye",
        "-metadata:s:v:0",
        "spherical=true",
        "-metadata:s:v:0",
        "vr180=true",
        str(output_video),
    ]

    decode = subprocess.Popen(
        decode_args,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        **hidden_subprocess_kwargs(),
    )
    encode = subprocess.Popen(
        encode_args,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        **hidden_subprocess_kwargs(),
    )
    assert decode.stdout is not None
    assert encode.stdin is not None

    try:
        total = len(frame_plan)
        previous_rendered: np.ndarray | None = None
        next_output_slot = 0
        for index, frame_stab in enumerate(frame_plan):
            raw = decode.stdout.read(frame_size)
            if len(raw) < frame_size:
                break
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((output_height, output_width, 3))
            matrix = np.array(frame_stab.correction_matrix3, dtype=np.float32)
            rendered = _render_sbs_frame(
                frame,
                matrix,
                left_lens,
                right_lens,
                source_size,
                options.distortion_correction,
                options.field_of_view_deg,
                (
                    rolling_shutter_plan[index]
                    if rolling_shutter_plan is not None and index < len(rolling_shutter_plan)
                    else None
                ),
            )
            target_slot = frame_slots[index] if index < len(frame_slots) else next_output_slot
            while next_output_slot < target_slot:
                filler = previous_rendered if previous_rendered is not None else rendered
                encode.stdin.write(filler.tobytes())
                next_output_slot += 1
            encode.stdin.write(rendered.tobytes())
            next_output_slot += 1
            previous_rendered = rendered
            if progress:
                percent = 82 + int((index + 1) * 16 / max(1, total))
                correction_deg = Quat.identity().angular_distance_deg(
                    Quat.from_iter(frame_stab.correction_wxyz)
                )
                progress(
                    min(98, percent),
                    (
                        f"Rendering frame {index + 1}/{total} | "
                        f"video={frame_stab.time_s:.6f}s | correction={correction_deg:.3f}deg"
                        f"{' | rolling-shutter rows active' if rolling_shutter_plan else ''}"
                    ),
                )
    finally:
        try:
            encode.stdin.close()
        except Exception:
            pass
        decode_stderr = decode.stderr.read().decode("utf-8", errors="replace") if decode.stderr else ""
        encode_stdout = encode.stdout.read() if encode.stdout else b""
        encode_stderr = encode.stderr.read().decode("utf-8", errors="replace") if encode.stderr else ""
        decode_code = decode.wait()
        encode_code = encode.wait()

    if decode_code != 0:
        raise RuntimeError(f"FFmpeg decode failed: {decode_stderr}")
    if encode_code != 0:
        raise RuntimeError(f"FFmpeg encode failed: {encode_stderr}")
