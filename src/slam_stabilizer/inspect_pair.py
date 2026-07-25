from __future__ import annotations

import argparse
import json
from pathlib import Path

from .imu import load_imu_csv, summarize_rate
from .video_probe import classify_layout, ffprobe_json, format_duration_s, parse_rate, primary_video_stream, stream_duration_s


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect a SLAM XCAM video and IMU pair")
    parser.add_argument("--video", required=True, type=Path)
    parser.add_argument("--imu", required=True, type=Path)
    parser.add_argument("--json", type=Path, help="Optional report JSON path")
    args = parser.parse_args()

    probe = ffprobe_json(args.video)
    video = primary_video_stream(probe)
    samples = load_imu_csv(args.imu)
    imu_duration = samples[-1].timestamp_s - samples[0].timestamp_s
    video_duration = stream_duration_s(video) or format_duration_s(probe)
    width = int(video.get("width", 0))
    height = int(video.get("height", 0))
    frame_rate = parse_rate(video.get("avg_frame_rate")) or parse_rate(video.get("r_frame_rate"))
    nb_frames = int(video.get("nb_frames", 0) or 0)
    data_streams = [s for s in probe.get("streams", []) if s.get("codec_type") == "data"]

    report = {
        "video": {
            "path": str(args.video),
            "codec": video.get("codec_name"),
            "width": width,
            "height": height,
            "layout": classify_layout(width, height),
            "duration_s": video_duration,
            "frame_rate": frame_rate,
            "frames": nb_frames,
            "side_data": video.get("side_data_list", []),
            "data_streams": [
                {
                    "codec_tag": s.get("codec_tag_string"),
                    "handler_name": s.get("tags", {}).get("handler_name"),
                    "duration_s": float(s.get("duration", 0) or 0),
                    "frames": int(s.get("nb_frames", 0) or 0),
                }
                for s in data_streams
            ],
        },
        "imu": {
            "path": str(args.imu),
            "samples": len(samples),
            "duration_s": imu_duration,
            "rate_hz": summarize_rate(samples),
            "first_timestamp_s": samples[0].timestamp_s,
            "last_timestamp_s": samples[-1].timestamp_s,
            "first_orientation_wxyz": samples[0].quaternion_wxyz,
            "last_orientation_wxyz": samples[-1].quaternion_wxyz,
        },
        "sync": {
            "duration_delta_s": imu_duration - video_duration,
            "imu_samples_per_video_frame": len(samples) / nb_frames if nb_frames else 0,
        },
    }

    print(f"video: {args.video}")
    print(f"  {width}x{height}, layout={report['video']['layout']}, codec={video.get('codec_name')}")
    print(f"  duration={video_duration:.6f}s, fps={frame_rate:.3f}, frames={nb_frames}")
    if data_streams:
        print(f"  data_streams={report['video']['data_streams']}")
    print(f"imu: {args.imu}")
    print(f"  samples={len(samples)}, duration={imu_duration:.6f}s, rate={report['imu']['rate_hz']:.3f}Hz")
    print(f"sync:")
    print(f"  imu_minus_video_duration={report['sync']['duration_delta_s']:.6f}s")
    print(f"  imu_samples_per_video_frame={report['sync']['imu_samples_per_video_frame']:.3f}")

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"report_json: {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

