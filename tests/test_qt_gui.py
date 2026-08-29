from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

from slam_stabilizer.qt_gui import (
    DeviceFileWorker,
    _format_file_size,
    _format_gib,
    _parse_adb_devices,
    _parse_device_media,
    _parse_storage_stat,
    probe_slam_xcam_device,
)


def test_device_export_converts_slam_source_to_sbs(tmp_path, monkeypatch) -> None:
    source_info = object()
    worker = DeviceFileWorker(
        "export",
        "test-device",
        [{"remote_path": "/sdcard/DCIM/VR180/clip_8K50_SOURCE.mp4", "name": "clip_8K50_SOURCE.mp4"}],
        str(tmp_path),
    )

    def fake_pull(_remote_path: str, local_path: Path, _expected_size: int, progress) -> None:
        local_path.write_bytes(b"source")
        progress(100)

    def fake_export(info: object, output: Path, progress_callback=None) -> str:
        assert info is source_info
        if progress_callback is not None:
            progress_callback(45, 90)
            progress_callback(90, 90)
        output.write_bytes(b"sbs")
        return "hevc_amf"

    monkeypatch.setattr(worker, "_pull", fake_pull)
    monkeypatch.setattr("slam_stabilizer.qt_gui.inspect_slam_source", lambda _path: source_info)
    monkeypatch.setattr("slam_stabilizer.qt_gui.validate_processed_source", lambda _info: None)
    monkeypatch.setattr("slam_stabilizer.qt_gui.export_slam_source", fake_export)
    statuses = []
    workflow = []
    worker.file_status.connect(lambda *args: statuses.append(args))
    worker.workflow_progress.connect(lambda *args: workflow.append(args))

    result = worker._export()

    output = tmp_path / "clip_8K50_VR180.mp4"
    assert result["paths"] == [str(output)]
    assert result["converted_sources"] == [
        {
            "source": "clip_8K50_SOURCE.mp4",
            "output": "clip_8K50_VR180.mp4",
            "encoder": "hevc_amf",
        }
    ]
    assert output.read_bytes() == b"sbs"
    assert not (tmp_path / "clip_8K50_SOURCE.mp4").exists()
    assert [status[1] for status in statuses] == ["copying", "stitching", "completed"]
    assert (45, 45, "stage_copy", "clip_8K50_SOURCE.mp4") in workflow
    assert (75, 75, "stage_stitch", "clip_8K50_SOURCE.mp4") in workflow
    assert workflow[-1] == (100, 100, "stage_complete", "clip_8K50_SOURCE.mp4")


def test_device_export_copies_regular_stitched_video_without_conversion(tmp_path, monkeypatch) -> None:
    worker = DeviceFileWorker(
        "export",
        "test-device",
        [{"remote_path": "/sdcard/DCIM/VR180/stitched.mp4", "name": "stitched.mp4"}],
        str(tmp_path),
    )
    def fake_pull(_remote: str, local: Path, _expected_size: int, progress) -> None:
        local.write_bytes(b"stitched")
        progress(100)

    monkeypatch.setattr(worker, "_pull", fake_pull)
    monkeypatch.setattr("slam_stabilizer.qt_gui.inspect_slam_source", lambda _path: None)
    statuses = []
    worker.file_status.connect(lambda *args: statuses.append(args))

    result = worker._export()

    output = tmp_path / "stitched.mp4"
    assert result["paths"] == [str(output)]
    assert output.read_bytes() == b"stitched"
    assert [status[1] for status in statuses] == ["copying", "copied"]


def test_parse_connected_adb_device() -> None:
    serial, state = _parse_adb_devices(
        "List of devices attached\nM97081AAYF080900185\tdevice product:kalama model:Kalama\n"
    )
    assert serial == "M97081AAYF080900185"
    assert state == "device"


def test_parse_device_storage_stat() -> None:
    total, used, available = _parse_storage_stat("4096 23255503 12962474\n")
    assert total == 95_254_540_288
    assert available == 53_094_293_504
    assert used == total - available
    assert _format_gib(available) == "49.4 GB"


def test_device_probe_preserves_remote_stat_format(monkeypatch) -> None:
    calls: list[list[str]] = []

    def fake_run(arguments: list[str], timeout_s: float = 8.0) -> str:
        calls.append(arguments)
        if arguments == ["devices"]:
            return "List of devices attached\nSERIAL123\tdevice\n"
        if arguments[-1] == "stat -f -c '%S %b %a' /sdcard":
            return "4096 100 25\n"
        if arguments[-2:] == ["getprop", "ro.soc.model"]:
            return "QCS8550\n"
        raise AssertionError(arguments)

    monkeypatch.setattr("slam_stabilizer.qt_gui._run_adb", fake_run)
    result = probe_slam_xcam_device()
    assert result["serial"] == "SERIAL123"
    assert result["total_bytes"] == 409600
    assert calls[1][-1] == "stat -f -c '%S %b %a' /sdcard"


def test_parse_supported_device_media_only() -> None:
    output = "\n".join(
        [
            "/sdcard/DCIM/VR180/new.heif|4096|200",
            "/sdcard/DCIM/VR180/clip.MP4|1048576|300",
            "/sdcard/DCIM/VR180/photo.dng|2048|100",
            "/sdcard/DCIM/VR180/motion.slamimu|512|400",
            "/sdcard/DCIM/VR180/broken.jpg|not-a-size|500",
        ]
    )
    media = _parse_device_media(output)
    assert [item["name"] for item in media] == ["clip.MP4", "new.heif", "photo.dng"]
    assert _format_file_size(int(media[0]["size_bytes"])) == "1.0 MB"


def test_parse_groups_legacy_four_file_session() -> None:
    base = "/sdcard/DCIM/VR180/Slam_20260731_131131_864"
    output = "\n".join(
        [
            f"{base}_L.mp4|600|100",
            f"{base}_R.mp4|590|101",
            f"{base}_A.m4a|10|102",
            f"{base}_sync.json|2|103",
            "/sdcard/DCIM/VR180/ordinary.mp4|1000|200",
        ]
    )
    media = _parse_device_media(output)
    assert [item["name"] for item in media] == [
        "ordinary.mp4",
        "Slam_20260731_131131_864_VR180.mp4",
    ]
    legacy = media[1]
    assert legacy["kind"] == "legacy_stereo_session"
    assert legacy["size_bytes"] == 1202
    assert set(legacy["components"]) == {"left", "right", "audio", "sync"}


def test_device_export_converts_legacy_session_to_sbs(tmp_path, monkeypatch) -> None:
    base = "/sdcard/DCIM/VR180/Slam_test"
    components = {
        role: {
            "remote_path": f"{base}{suffix}",
            "name": f"Slam_test{suffix}",
            "size_bytes": 10,
        }
        for role, suffix in {
            "left": "_L.mp4",
            "right": "_R.mp4",
            "audio": "_A.m4a",
            "sync": "_sync.json",
        }.items()
    }
    worker = DeviceFileWorker(
        "export",
        "test-device",
        [
            {
                "remote_path": components["sync"]["remote_path"],
                "name": "Slam_test_VR180.mp4",
                "kind": "legacy_stereo_session",
                "components": components,
            }
        ],
        str(tmp_path),
    )

    def fake_pull(_remote: str, local: Path, _size: int, progress) -> None:
        local.write_bytes(b"part")
        progress(100)

    monkeypatch.setattr(worker, "_pull", fake_pull)
    monkeypatch.setattr("slam_stabilizer.qt_gui.inspect_legacy_slam_source", lambda *_args: object())

    def fake_export(_info, output: Path, progress_callback=None) -> str:
        output.write_bytes(b"legacy-sbs")
        if progress_callback:
            progress_callback(100, 100)
        return "hevc_qsv"

    monkeypatch.setattr("slam_stabilizer.qt_gui.export_legacy_slam_source", fake_export)
    result = worker._export()
    output = tmp_path / "Slam_test_VR180.mp4"
    assert result["paths"] == [str(output)]
    assert output.read_bytes() == b"legacy-sbs"
    assert not list(tmp_path.glob("slam_xcam_legacy_*"))


def test_studio_window_structure_in_isolated_qt_process(tmp_path) -> None:
    root = Path(__file__).resolve().parents[1]
    env = os.environ.copy()
    env["QT_QPA_PLATFORM"] = "offscreen"
    env["PYTHONPATH"] = str(root / "src")
    env["SLAM_XCAM_SETTINGS_PATH"] = str(tmp_path / "settings.ini")
    probe = r'''
import json
from PySide6.QtWidgets import QApplication, QLabel
from slam_stabilizer.qt_gui import MainWindow

app = QApplication([])
window = MainWindow()
window._apply_theme("minimal", persist=False)
labels = [label.text() for label in window.findChildren(QLabel)]
minimal_style = window.styleSheet()
window._apply_theme("pixel", persist=False)
default_tabs = [button.text() for button in window.page_tab_buttons]
default_editor = "Editing (In Development)" in labels and "In Development" in labels
window._apply_language("zh_CN")
result = {
    "title": window.windowTitle(),
    "pages": window.page_stack.count(),
    "tabs": default_tabs,
    "chinese_tabs": [button.text() for button in window.page_tab_buttons],
    "language_menu": window.language_menu.title(),
    "language_saved": window.settings.value("ui/language"),
    "device_views": window.device_content_stack.count(),
    "device_files": window.device_file_list.count(),
    "export_enabled": window.export_device_files_button.isEnabled(),
    "device_fields": [window.device_name_value.text(), window.device_storage_value.text(), window.device_processor_value.text()],
    "calibration_controls_hidden": not any(text in labels for text in ("Calibration source", "Custom calibration JSON")),
    "editor_development": default_editor,
    "minimal_style": "#f2f2f2" in minimal_style,
    "pixel_style": "#151515" in window.styleSheet(),
}
window.close()
restored = MainWindow()
result["restored_language"] = restored.language
result["restored_tabs"] = [button.text() for button in restored.page_tab_buttons]
restored.close()
print(json.dumps(result, ensure_ascii=True))
'''
    output_path = tmp_path / "qt-probe-output.txt"
    with output_path.open("wb") as output:
        subprocess.run(
            [sys.executable, "-c", probe],
            cwd=root,
            env=env,
            stdout=output,
            stderr=subprocess.STDOUT,
            check=True,
        )
    result = json.loads(output_path.read_text(encoding="utf-8").strip().splitlines()[-1])

    assert result == {
        "title": "SLAM XCAM Studio",
        "pages": 3,
        "tabs": ["Stabilization", "Camera Management", "Editing"],
        "chinese_tabs": ["防抖处理", "相机管理", "剪辑"],
        "language_menu": "语言设置",
        "language_saved": "zh_CN",
        "restored_language": "zh_CN",
        "restored_tabs": ["防抖处理", "相机管理", "剪辑"],
        "device_views": 2,
        "device_files": 0,
        "export_enabled": False,
        "device_fields": ["--", "--", "--"],
        "calibration_controls_hidden": True,
        "editor_development": True,
        "minimal_style": True,
        "pixel_style": True,
    }
