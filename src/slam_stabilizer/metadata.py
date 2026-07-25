from __future__ import annotations

from pathlib import Path
import subprocess

from .process import hidden_subprocess_kwargs


VR180_FFMPEG_METADATA = {
    "stereo_mode": "left_right",
    "projection": "fisheye",
    "spherical": "true",
    "vr180": "true",
}


def remux_with_metadata(input_video: Path, output_video: Path, ffmpeg: str = "ffmpeg") -> None:
    output_video.parent.mkdir(parents=True, exist_ok=True)
    args = [
        ffmpeg,
        "-y",
        "-i",
        str(input_video),
        "-map",
        "0:v:0",
        "-map",
        "0:a?",
        "-c",
        "copy",
    ]
    for key, value in VR180_FFMPEG_METADATA.items():
        args.extend([f"-metadata:s:v:0", f"{key}={value}"])
    args.append(str(output_video))
    subprocess.run(args, check=True, capture_output=True, text=True, **hidden_subprocess_kwargs())
