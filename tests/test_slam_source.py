from __future__ import annotations

from pathlib import Path
import subprocess

import pytest

from slam_stabilizer.slam_source import (
    _extract_manifest,
    build_legacy_export_command,
    build_export_command,
    default_legacy_sbs_export_path,
    default_sbs_export_path,
    inspect_legacy_slam_source,
    parse_slam_source,
    validate_processed_source,
)


def _manifest() -> dict:
    return {
        "formatVersion": 1,
        "containerType": "SLAM_XCAM_8K50_SOURCE",
        "tracks": [
            {"index": 0, "role": "LEFT", "type": "video"},
            {"index": 1, "role": "RIGHT", "type": "video"},
            {"index": 2, "role": "AUDIO", "type": "audio"},
            {"index": 3, "role": "SESSION_METADATA", "type": "metadata"},
        ],
        "timing": {
            "queueOverflowCount": 0,
            "writerFailure": None,
            "unmatchedLeftFrames": 0,
            "unmatchedRightFrames": 0,
        },
        "sourceProcessing": {
            "imageStage": "processed_fisheye",
            "distortionCorrection": {"applied": True, "mode": "180"},
            "stereoAlignment": {"applied": True},
        },
        "output": {
            "eyeWidth": 3840,
            "eyeHeight": 3840,
            "requestedFps": 50,
        },
    }


def _probe() -> dict:
    return {
        "streams": [
            {"index": 0, "codec_type": "video", "width": 3840, "height": 3840},
            {"index": 1, "codec_type": "video", "width": 3840, "height": 3840},
            {"index": 2, "codec_type": "audio"},
            {"index": 3, "codec_type": "data"},
        ]
    }


def test_parse_and_build_8k50_source_export() -> None:
    info = parse_slam_source(Path("capture.mp4"), _probe(), _manifest())
    assert info is not None
    assert (info.output_width, info.output_height, info.requested_fps) == (7680, 3840, 50)
    validate_processed_source(info)

    command = build_export_command(info, "export.mp4", "hevc_nvenc")
    assert "[0:v:0][0:v:1]hstack=inputs=2:shortest=1[v]" in command
    assert "-r" not in command
    assert command[command.index("-fps_mode") + 1] == "passthrough"
    assert command[command.index("-c:a") + 1] == "copy"
    assert command[-1] == "export.mp4"


def test_default_sbs_export_path() -> None:
    assert default_sbs_export_path("Slam_20260828_044009_016_8K50_SOURCE.mp4") == Path(
        "Slam_20260828_044009_016_8K50_VR180.mp4"
    )


@pytest.mark.parametrize(
    ("key", "value", "message"),
    [
        ("queueOverflowCount", 1, "queue overflow"),
        ("writerFailure", "MUXER_WRITE_FAILED", "muxer failure"),
        ("unmatchedRightFrames", 2, "unmatched left/right"),
    ],
)
def test_rejects_incomplete_source(key: str, value: object, message: str) -> None:
    manifest = _manifest()
    manifest["timing"][key] = value
    info = parse_slam_source("capture.mp4", _probe(), manifest)
    assert info is not None
    with pytest.raises(ValueError, match=message):
        validate_processed_source(info)


def test_non_slam_mp4_is_not_detected() -> None:
    manifest = _manifest()
    manifest["containerType"] = "ordinary_mp4"
    assert parse_slam_source("capture.mp4", _probe(), manifest) is None


def test_binary_data_track_is_not_treated_as_utf8_manifest(monkeypatch) -> None:
    completed = subprocess.CompletedProcess(
        args=["ffmpeg"],
        returncode=0,
        stdout=b"telemetry:\x9c\x00\xff\x80",
        stderr=b"",
    )
    monkeypatch.setattr("slam_stabilizer.slam_source.subprocess.run", lambda *_args, **_kwargs: completed)

    assert _extract_manifest(Path("ordinary.mp4")) is None


def test_legacy_four_file_session_uses_sync_audio_timing(tmp_path, monkeypatch) -> None:
    sync = tmp_path / "Slam_20260731_131131_864_sync.json"
    left = tmp_path / "Slam_20260731_131131_864_L.mp4"
    right = tmp_path / "Slam_20260731_131131_864_R.mp4"
    audio = tmp_path / "Slam_20260731_131131_864_A.m4a"
    sync.write_text(
        '{"audioVideoOffsetUs":28331,"videoDurationUs":60547342,"audioTailTrimUs":381576}',
        encoding="utf-8",
    )
    for path in (left, right, audio):
        path.write_bytes(b"test")

    probe = {
        "streams": [
            {
                "codec_type": "video",
                "width": 3840,
                "height": 3840,
                "avg_frame_rate": "50/1",
                "nb_frames": "3000",
            }
        ]
    }
    monkeypatch.setattr("slam_stabilizer.slam_source.ffprobe_json", lambda _path: probe)
    info = inspect_legacy_slam_source(sync, left, right, audio)
    command = build_legacy_export_command(info, "out.mp4", "hevc_qsv")

    assert info.frame_count == 3000
    assert info.output_width == 7680
    filter_graph = command[command.index("-filter_complex") + 1]
    assert "[left][right]hstack=inputs=2:shortest=1[v]" in filter_graph
    assert "adelay=28.331:all=1" in filter_graph
    assert "atrim=end=60.547342" in filter_graph
    assert "-r" not in command
    assert default_legacy_sbs_export_path(sync).name == "Slam_20260731_131131_864_VR180.mp4"
