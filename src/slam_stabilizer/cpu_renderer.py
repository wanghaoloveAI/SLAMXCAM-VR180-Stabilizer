from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import subprocess
from typing import Any

import numpy as np

from .core.stabilization import FrameStabilization
from .process import hidden_subprocess_kwargs


@dataclass(frozen=True)
class CpuRenderOptions:
    output_width: int = 1920
    crf: int = 20
    preset: str = "veryfast"
    distortion_correction: bool = True
    field_of_view_deg: float = 180.0


def _build_eye_maps(
    eye_size: int,
    matrix: np.ndarray,
    lens: dict[str, Any] | None = None,
    source_size: tuple[int, int] | None = None,
    distortion_correction: bool = True,
    field_of_view_deg: float = 180.0,
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

    src_dirs = matrix @ dirs
    if lens and lens.get("output_to_lens_rotation"):
        lens_rotation = np.array(lens["output_to_lens_rotation"], dtype=np.float32).reshape(3, 3)
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
    )
    right_map_x, right_map_y, right_valid = _build_eye_maps(
        eye_size,
        matrix,
        right_lens,
        source_size,
        distortion_correction,
        field_of_view_deg,
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
    progress=None,
) -> None:
    output_width = max(640, int(options.output_width))
    if output_width % 2:
        output_width += 1
    output_height = output_width // 2
    eye_size = output_height
    frame_size = output_width * output_height * 3
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
        f"{frame_rate:.6f}",
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
        "-shortest",
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
            )
            encode.stdin.write(rendered.tobytes())
            if progress and (index % 5 == 0 or index + 1 == total):
                percent = 82 + int((index + 1) * 16 / max(1, total))
                progress(min(98, percent), f"Rendering stabilized frame {index + 1}/{total}")
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
