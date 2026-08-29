from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import subprocess

import numpy as np

from .calibration_runtime import CalibrationRuntime
from .core.quaternion import Quat
from .core.stabilization import FrameStabilization
from .cpu_renderer import CpuRenderOptions, build_cfr_frame_slots
from .process import hidden_subprocess_kwargs


@dataclass(frozen=True)
class RuntimeRenderResult:
    renderer_name: str
    api: str


def render_stabilized_sbs_runtime(
    input_video: Path,
    output_video: Path,
    frame_plan: list[FrameStabilization],
    frame_rate: float,
    options: CpuRenderOptions,
    runtime: CalibrationRuntime,
    camera_model: str,
    rolling_shutter_plan: list[np.ndarray] | None = None,
    progress=None,
) -> RuntimeRenderResult:
    output_width = max(640, int(options.output_width))
    if output_width % 2:
        output_width += 1
    output_height = output_width // 2
    frame_size = output_width * output_height * 3
    output_frame_rate, frame_slots = build_cfr_frame_slots(
        [frame.time_s for frame in frame_plan], frame_rate
    )
    output_frame_count = frame_slots[-1] + 1 if frame_slots else len(frame_plan)
    output_video.parent.mkdir(parents=True, exist_ok=True)

    renderer = runtime.create_renderer(
        camera_model=camera_model,
        eye_size=output_height,
        distortion_correction=options.distortion_correction,
        field_of_view_deg=options.field_of_view_deg,
    )
    actual_backend = renderer.backend_name
    if progress:
        progress(82, f"Official Calibration Runtime active: {runtime.info.version} | {actual_backend}")

    decode_args = [
        "ffmpeg", "-v", "error", "-i", str(input_video), "-an",
        "-vf", f"scale={output_width}:{output_height}", "-vsync", "0",
        "-pix_fmt", "rgb24", "-f", "rawvideo", "-",
    ]
    encode_args = [
        "ffmpeg", "-y", "-v", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{output_width}x{output_height}", "-r", f"{output_frame_rate:.6f}",
        "-i", "-", "-i", str(input_video), "-map", "0:v:0", "-map", "1:a?",
        "-c:v", "libx264", "-preset", options.preset, "-crf", str(options.crf),
        "-pix_fmt", "yuv420p", "-c:a", "copy", "-movflags", "+faststart",
        "-t", f"{output_frame_count / max(1e-9, output_frame_rate):.6f}",
        "-metadata:s:v:0", "stereo_mode=left_right",
        "-metadata:s:v:0", "projection=fisheye",
        "-metadata:s:v:0", "spherical=true",
        "-metadata:s:v:0", "vr180=true",
        str(output_video),
    ]
    decode = subprocess.Popen(
        decode_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, **hidden_subprocess_kwargs()
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

    rendered_count = 0
    try:
        total = len(frame_plan)
        previous_rendered: np.ndarray | None = None
        next_output_slot = 0
        for index, frame_stab in enumerate(frame_plan):
            raw = decode.stdout.read(frame_size)
            if len(raw) < frame_size:
                break
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((output_height, output_width, 3))
            row_matrices = (
                rolling_shutter_plan[index]
                if rolling_shutter_plan is not None and index < len(rolling_shutter_plan)
                else None
            )
            rendered = renderer.render(
                frame,
                np.asarray(frame_stab.correction_matrix3, dtype=np.float32),
                row_matrices,
            )
            target_slot = frame_slots[index] if index < len(frame_slots) else next_output_slot
            while next_output_slot < target_slot:
                filler = previous_rendered if previous_rendered is not None else rendered
                encode.stdin.write(filler.tobytes())
                next_output_slot += 1
            encode.stdin.write(rendered.tobytes())
            next_output_slot += 1
            previous_rendered = rendered
            rendered_count += 1
            if progress:
                percent = 82 + int((index + 1) * 16 / max(1, total))
                correction_deg = Quat.identity().angular_distance_deg(
                    Quat.from_iter(frame_stab.correction_wxyz)
                )
                progress(
                    min(98, percent),
                    f"Runtime rendering frame {index + 1}/{total} | "
                    f"video={frame_stab.time_s:.6f}s | correction={correction_deg:.3f}deg"
                    f"{' | rolling-shutter rows active' if rolling_shutter_plan else ''}",
                )
    finally:
        renderer.close()
        try:
            encode.stdin.close()
        except Exception:
            pass
        decode_stderr = decode.stderr.read().decode("utf-8", errors="replace") if decode.stderr else ""
        encode_stderr = encode.stderr.read().decode("utf-8", errors="replace") if encode.stderr else ""
        decode_code = decode.wait()
        encode_code = encode.wait()

    if rendered_count != len(frame_plan):
        raise RuntimeError(
            f"Calibration Runtime received {rendered_count} frames but expected {len(frame_plan)}."
        )
    if decode_code != 0:
        raise RuntimeError(f"FFmpeg decode failed: {decode_stderr}")
    if encode_code != 0:
        raise RuntimeError(f"FFmpeg encode failed: {encode_stderr}")
    return RuntimeRenderResult(
        renderer_name=f"Official Calibration Runtime {runtime.info.version}",
        api=actual_backend,
    )


ABI_LABEL = "v1"
