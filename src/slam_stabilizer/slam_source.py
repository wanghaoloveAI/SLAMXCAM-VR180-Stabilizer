from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import subprocess
from typing import Any, Callable

from .process import hidden_subprocess_kwargs
from .video_probe import ffprobe_json, parse_rate


SESSION_METADATA_MIME = "application/vnd.slam.xcam.session+json"
CONTAINER_TYPE = "SLAM_XCAM_8K50_SOURCE"


@dataclass(frozen=True)
class SlamSourceInfo:
    path: Path
    manifest: dict[str, Any]
    left_video_ordinal: int
    right_video_ordinal: int
    audio_ordinal: int | None
    eye_width: int
    eye_height: int
    requested_fps: float
    frame_count: int

    @property
    def output_width(self) -> int:
        return self.eye_width * 2

    @property
    def output_height(self) -> int:
        return self.eye_height


@dataclass(frozen=True)
class LegacySlamSourceInfo:
    sync_path: Path
    left_path: Path
    right_path: Path
    audio_path: Path | None
    sync: dict[str, Any]
    eye_width: int
    eye_height: int
    requested_fps: float
    frame_count: int

    @property
    def output_width(self) -> int:
        return self.eye_width * 2

    @property
    def output_height(self) -> int:
        return self.eye_height


def _extract_manifest(path: Path, ffmpeg: str = "ffmpeg") -> dict[str, Any] | None:
    completed = subprocess.run(
        [
            ffmpeg,
            "-v",
            "error",
            "-i",
            str(path),
            "-map",
            "0:d:0",
            "-c",
            "copy",
            "-f",
            "data",
            "-",
        ],
        check=False,
        capture_output=True,
        **hidden_subprocess_kwargs(),
    )
    if completed.returncode != 0 or not completed.stdout:
        return None
    # Ordinary MP4 files may contain binary telemetry in their first data
    # stream. Only SLAM XCAM's session metadata stream is UTF-8 JSON.
    try:
        payload = completed.stdout.decode("utf-8", errors="strict").strip("\x00\r\n ")
    except UnicodeDecodeError:
        return None
    try:
        value = json.loads(payload)
    except json.JSONDecodeError:
        return None
    return value if isinstance(value, dict) else None


def parse_slam_source(
    path: str | Path,
    probe: dict[str, Any],
    manifest: dict[str, Any] | None,
) -> SlamSourceInfo | None:
    if not manifest or manifest.get("containerType") != CONTAINER_TYPE:
        return None
    videos = [stream for stream in probe.get("streams", []) if stream.get("codec_type") == "video"]
    audios = [stream for stream in probe.get("streams", []) if stream.get("codec_type") == "audio"]
    if len(videos) < 2:
        raise ValueError("SLAM source container does not contain both eye video tracks.")

    role_to_absolute_index: dict[str, int] = {}
    for track in manifest.get("tracks", []):
        role = str(track.get("role", "")).upper()
        try:
            role_to_absolute_index[role] = int(track.get("index"))
        except (TypeError, ValueError):
            continue

    def video_ordinal(role: str, fallback: int) -> int:
        absolute = role_to_absolute_index.get(role)
        if absolute is None:
            return fallback
        for ordinal, stream in enumerate(videos):
            if int(stream.get("index", -1)) == absolute:
                return ordinal
        raise ValueError(f"Manifest {role} track index {absolute} was not found in MP4.")

    output = manifest.get("output", {})
    eye_width = int(output.get("eyeWidth") or videos[0].get("width") or 0)
    eye_height = int(output.get("eyeHeight") or videos[0].get("height") or 0)
    requested_fps = float(output.get("requestedFps") or 0)
    if requested_fps <= 0:
        requested_fps = parse_rate(videos[0].get("avg_frame_rate")) or parse_rate(
            videos[0].get("r_frame_rate")
        )
    if eye_width <= 0 or eye_height <= 0 or requested_fps <= 0:
        raise ValueError("SLAM source manifest has invalid output dimensions or frame rate.")

    left_video_ordinal = video_ordinal("LEFT", 0)
    right_video_ordinal = video_ordinal("RIGHT", 1)

    role_sample_counts: dict[str, int] = {}
    for track in manifest.get("tracks", []):
        role = str(track.get("role", "")).upper()
        try:
            role_sample_counts[role] = int(track.get("sampleCount") or 0)
        except (TypeError, ValueError):
            continue
    left_frames = role_sample_counts.get("LEFT") or int(videos[left_video_ordinal].get("nb_frames") or 0)
    right_frames = role_sample_counts.get("RIGHT") or int(videos[right_video_ordinal].get("nb_frames") or 0)
    frame_count = min(value for value in (left_frames, right_frames) if value > 0) if any(
        value > 0 for value in (left_frames, right_frames)
    ) else 0

    audio_ordinal = 0 if audios else None
    return SlamSourceInfo(
        path=Path(path),
        manifest=manifest,
        left_video_ordinal=left_video_ordinal,
        right_video_ordinal=right_video_ordinal,
        audio_ordinal=audio_ordinal,
        eye_width=eye_width,
        eye_height=eye_height,
        requested_fps=requested_fps,
        frame_count=frame_count,
    )


def inspect_slam_source(
    path: str | Path,
    ffmpeg: str = "ffmpeg",
) -> SlamSourceInfo | None:
    source = Path(path)
    probe = ffprobe_json(source)
    manifest = _extract_manifest(source, ffmpeg=ffmpeg)
    return parse_slam_source(source, probe, manifest)


def validate_processed_source(info: SlamSourceInfo) -> None:
    processing = info.manifest.get("sourceProcessing", {})
    distortion = processing.get("distortionCorrection", {})
    alignment = processing.get("stereoAlignment", {})
    timing = info.manifest.get("timing", {})
    if processing.get("imageStage") != "processed_fisheye":
        raise ValueError("This source is not marked as processed fisheye footage.")
    if distortion.get("applied") is not True:
        raise ValueError("Distortion correction is not marked as applied; raw-source export is not implemented yet.")
    if alignment.get("applied") is not True:
        raise ValueError("Stereo X/Y alignment is not marked as applied; raw-source export is not implemented yet.")
    if int(timing.get("queueOverflowCount") or 0) > 0:
        raise ValueError("The source container reports encoded-sample queue overflow and cannot be exported reliably.")
    if timing.get("writerFailure") not in (None, "", "null"):
        raise ValueError(f"The source container reports a muxer failure: {timing['writerFailure']}")
    if int(timing.get("unmatchedLeftFrames") or 0) > 0 or int(
        timing.get("unmatchedRightFrames") or 0
    ) > 0:
        raise ValueError("The source container contains unmatched left/right frames.")


def build_export_command(
    info: SlamSourceInfo,
    output: str | Path,
    encoder: str,
    ffmpeg: str = "ffmpeg",
) -> list[str]:
    left = info.left_video_ordinal
    right = info.right_video_ordinal
    command = [
        ffmpeg,
        "-y",
        "-v",
        "error",
        "-i",
        str(info.path),
        "-filter_complex",
        f"[0:v:{left}][0:v:{right}]hstack=inputs=2:shortest=1[v]",
        "-map",
        "[v]",
    ]
    if info.audio_ordinal is not None:
        command.extend(["-map", f"0:a:{info.audio_ordinal}?"])
    command.extend(
        [
            "-c:v",
            encoder,
            "-b:v",
            "160M",
            "-maxrate",
            "200M",
            "-bufsize",
            "400M",
            "-pix_fmt",
            "yuv420p",
            "-fps_mode",
            "passthrough",
            "-c:a",
            "copy",
            "-metadata:s:v:0",
            "stereo_mode=left_right",
            "-metadata:s:v:0",
            "projection=vr180_fisheye",
            "-movflags",
            "+faststart",
            "-progress",
            "pipe:1",
            "-nostats",
            str(output),
        ]
    )
    return command


def default_sbs_export_path(source: str | Path) -> Path:
    path = Path(source)
    stem = path.stem.removesuffix("_SOURCE")
    return path.with_name(f"{stem}_VR180.mp4")


def inspect_legacy_slam_source(
    sync_path: str | Path,
    left_path: str | Path,
    right_path: str | Path,
    audio_path: str | Path | None = None,
) -> LegacySlamSourceInfo:
    sync_file = Path(sync_path)
    try:
        sync = json.loads(sync_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"Invalid legacy sync JSON: {sync_file.name}") from exc
    if not isinstance(sync, dict):
        raise ValueError(f"Invalid legacy sync JSON: {sync_file.name}")

    left_file = Path(left_path)
    right_file = Path(right_path)
    left_probe = ffprobe_json(left_file)
    right_probe = ffprobe_json(right_file)
    left_videos = [s for s in left_probe.get("streams", []) if s.get("codec_type") == "video"]
    right_videos = [s for s in right_probe.get("streams", []) if s.get("codec_type") == "video"]
    if not left_videos or not right_videos:
        raise ValueError("Legacy stereo session does not contain both eye video streams.")
    left = left_videos[0]
    right = right_videos[0]
    left_size = (int(left.get("width") or 0), int(left.get("height") or 0))
    right_size = (int(right.get("width") or 0), int(right.get("height") or 0))
    if left_size != right_size or min(left_size) <= 0:
        raise ValueError(f"Legacy left/right dimensions do not match: {left_size} vs {right_size}.")
    left_fps = parse_rate(left.get("avg_frame_rate")) or parse_rate(left.get("r_frame_rate"))
    right_fps = parse_rate(right.get("avg_frame_rate")) or parse_rate(right.get("r_frame_rate"))
    if left_fps <= 0 or right_fps <= 0 or abs(left_fps - right_fps) > 0.05:
        raise ValueError(f"Legacy left/right frame rates do not match: {left_fps} vs {right_fps}.")
    left_frames = int(left.get("nb_frames") or 0)
    right_frames = int(right.get("nb_frames") or 0)
    frame_count = min(v for v in (left_frames, right_frames) if v > 0) if any(
        v > 0 for v in (left_frames, right_frames)
    ) else 0
    audio_file = Path(audio_path) if audio_path else None
    if audio_file is not None and not audio_file.exists():
        audio_file = None
    return LegacySlamSourceInfo(
        sync_path=sync_file,
        left_path=left_file,
        right_path=right_file,
        audio_path=audio_file,
        sync=sync,
        eye_width=left_size[0],
        eye_height=left_size[1],
        requested_fps=left_fps,
        frame_count=frame_count,
    )


def default_legacy_sbs_export_path(sync_path: str | Path) -> Path:
    path = Path(sync_path)
    stem = path.stem.removesuffix("_sync")
    return path.with_name(f"{stem}_VR180.mp4")


def build_legacy_export_command(
    info: LegacySlamSourceInfo,
    output: str | Path,
    encoder: str,
    ffmpeg: str = "ffmpeg",
) -> list[str]:
    command = [
        ffmpeg,
        "-y",
        "-v",
        "error",
        "-i",
        str(info.left_path),
        "-i",
        str(info.right_path),
    ]
    has_audio = info.audio_path is not None
    if has_audio:
        command.extend(["-i", str(info.audio_path)])

    filters = [
        "[0:v:0]setpts=PTS-STARTPTS[left]",
        "[1:v:0]setpts=PTS-STARTPTS[right]",
        "[left][right]hstack=inputs=2:shortest=1[v]",
    ]
    if has_audio:
        offset_us = int(info.sync.get("audioVideoOffsetUs") or 0)
        video_duration_us = int(info.sync.get("videoDurationUs") or 0)
        if offset_us >= 0:
            audio_filter = f"[2:a:0]asetpts=PTS-STARTPTS,adelay={offset_us / 1000:.3f}:all=1"
        else:
            audio_filter = f"[2:a:0]atrim=start={-offset_us / 1_000_000:.6f},asetpts=PTS-STARTPTS"
        if video_duration_us > 0:
            audio_filter += f",atrim=end={video_duration_us / 1_000_000:.6f}"
        filters.append(audio_filter + "[a]")

    command.extend(["-filter_complex", ";".join(filters), "-map", "[v]"])
    if has_audio:
        command.extend(["-map", "[a]"])
    command.extend(
        [
            "-c:v",
            encoder,
            "-b:v",
            "160M",
            "-maxrate",
            "200M",
            "-bufsize",
            "400M",
            "-pix_fmt",
            "yuv420p",
            "-fps_mode",
            "passthrough",
        ]
    )
    if has_audio:
        command.extend(["-c:a", "aac", "-b:a", "192k"])
    command.extend(
        [
            "-metadata:s:v:0",
            "stereo_mode=left_right",
            "-metadata:s:v:0",
            "projection=vr180_fisheye",
            "-movflags",
            "+faststart",
            "-shortest",
            "-progress",
            "pipe:1",
            "-nostats",
            str(output),
        ]
    )
    return command


def available_hevc_encoders(ffmpeg: str = "ffmpeg") -> list[str]:
    completed = subprocess.run(
        [ffmpeg, "-hide_banner", "-encoders"],
        check=True,
        capture_output=True,
        text=True,
        **hidden_subprocess_kwargs(),
    )
    text = completed.stdout + completed.stderr
    preferred = ["hevc_nvenc", "hevc_amf", "hevc_qsv", "libx265"]
    return [encoder for encoder in preferred if encoder in text]


def export_slam_source(
    info: SlamSourceInfo,
    output: str | Path,
    ffmpeg: str = "ffmpeg",
    progress_callback: Callable[[int, int], None] | None = None,
) -> str:
    validate_processed_source(info)
    target = Path(output)
    target.parent.mkdir(parents=True, exist_ok=True)
    encoders = available_hevc_encoders(ffmpeg)
    if not encoders:
        raise RuntimeError("No HEVC encoder is available in FFmpeg.")

    failures: list[str] = []
    for encoder in encoders:
        command = build_export_command(info, target, encoder, ffmpeg=ffmpeg)
        if progress_callback is None:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                **hidden_subprocess_kwargs(),
            )
            returncode = completed.returncode
            stderr = completed.stderr
        else:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
                **hidden_subprocess_kwargs(),
            )
            if process.stdout is not None:
                for line in process.stdout:
                    key, separator, value = line.strip().partition("=")
                    if separator and key == "frame":
                        try:
                            progress_callback(int(value), info.frame_count)
                        except ValueError:
                            pass
            stderr = process.stderr.read() if process.stderr is not None else ""
            returncode = process.wait()
        if returncode == 0 and target.exists() and target.stat().st_size > 0:
            if progress_callback is not None:
                progress_callback(info.frame_count, info.frame_count)
            return encoder
        failures.append(f"{encoder}: {stderr.strip()[-1000:]}")
        if target.exists():
            target.unlink()
    raise RuntimeError("All HEVC encoders failed:\n" + "\n".join(failures))


def export_legacy_slam_source(
    info: LegacySlamSourceInfo,
    output: str | Path,
    ffmpeg: str = "ffmpeg",
    progress_callback: Callable[[int, int], None] | None = None,
) -> str:
    target = Path(output)
    target.parent.mkdir(parents=True, exist_ok=True)
    encoders = available_hevc_encoders(ffmpeg)
    if not encoders:
        raise RuntimeError("No HEVC encoder is available in FFmpeg.")

    failures: list[str] = []
    for encoder in encoders:
        command = build_legacy_export_command(info, target, encoder, ffmpeg=ffmpeg)
        if progress_callback is None:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                **hidden_subprocess_kwargs(),
            )
            returncode = completed.returncode
            stderr = completed.stderr
        else:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
                **hidden_subprocess_kwargs(),
            )
            if process.stdout is not None:
                for line in process.stdout:
                    key, separator, value = line.strip().partition("=")
                    if separator and key == "frame":
                        try:
                            progress_callback(int(value), info.frame_count)
                        except ValueError:
                            pass
            stderr = process.stderr.read() if process.stderr is not None else ""
            returncode = process.wait()
        if returncode == 0 and target.exists() and target.stat().st_size > 0:
            if progress_callback is not None:
                progress_callback(info.frame_count, info.frame_count)
            return encoder
        failures.append(f"{encoder}: {stderr.strip()[-1000:]}")
        if target.exists():
            target.unlink()
    raise RuntimeError("All HEVC encoders failed:\n" + "\n".join(failures))
