from __future__ import annotations

from dataclasses import asdict, dataclass
from fractions import Fraction
import json
from pathlib import Path
import subprocess
from typing import Any

from .process import hidden_subprocess_kwargs


def ffprobe_json(path: str | Path) -> dict[str, Any]:
    args = [
        "ffprobe",
        "-v",
        "error",
        "-print_format",
        "json",
        "-show_streams",
        "-show_format",
        str(path),
    ]
    completed = subprocess.run(args, check=True, capture_output=True, text=True, **hidden_subprocess_kwargs())
    return json.loads(completed.stdout)


def parse_rate(rate: str | None) -> float:
    if not rate or rate == "0/0":
        return 0.0
    return float(Fraction(rate))


def primary_video_stream(probe: dict[str, Any]) -> dict[str, Any]:
    for stream in probe.get("streams", []):
        if stream.get("codec_type") == "video":
            return stream
    raise ValueError("No video stream found.")


def stream_duration_s(stream: dict[str, Any]) -> float:
    value = stream.get("duration")
    return float(value) if value is not None else 0.0


def format_duration_s(probe: dict[str, Any]) -> float:
    value = probe.get("format", {}).get("duration")
    return float(value) if value is not None else 0.0


def classify_layout(width: int, height: int) -> str:
    ratio = width / height if height else 0.0
    if abs(ratio - 2.0) < 0.03:
        return "sbs_2_to_1"
    if abs(ratio - 1.0) < 0.03:
        return "single_fisheye_1_to_1"
    return "unknown"


@dataclass(frozen=True)
class VideoParameters:
    path: str
    width: int
    height: int
    codec: str
    layout: str
    frame_rate: float
    frame_count: int
    duration_s: float
    field_of_view_deg: float
    aspect_ratio: float
    frame_duration_s: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def probe_video_parameters(path: str | Path, field_of_view_deg: float = 180.0) -> tuple[VideoParameters, dict[str, Any]]:
    probe = ffprobe_json(path)
    stream = primary_video_stream(probe)
    width = int(stream.get("width", 0) or 0)
    height = int(stream.get("height", 0) or 0)
    frame_rate = parse_rate(stream.get("avg_frame_rate")) or parse_rate(stream.get("r_frame_rate"))
    duration = stream_duration_s(stream) or format_duration_s(probe)
    frame_count = int(stream.get("nb_frames", 0) or 0)
    if frame_count <= 0 and frame_rate > 0 and duration > 0:
        frame_count = int(round(duration * frame_rate))

    params = VideoParameters(
        path=str(path),
        width=width,
        height=height,
        codec=str(stream.get("codec_name", "") or ""),
        layout=classify_layout(width, height),
        frame_rate=frame_rate,
        frame_count=frame_count,
        duration_s=duration,
        field_of_view_deg=float(field_of_view_deg),
        aspect_ratio=(width / height) if height else 0.0,
        frame_duration_s=(1.0 / frame_rate) if frame_rate > 0 else 0.0,
    )
    return params, probe
