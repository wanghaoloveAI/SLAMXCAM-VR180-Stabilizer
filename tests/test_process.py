from __future__ import annotations

from pathlib import Path

from slam_stabilizer.process import bundled_tool_directories, configure_bundled_tools, find_tool


def test_portable_tool_directory_precedes_system_path(tmp_path: Path, monkeypatch) -> None:
    ffmpeg_dir = tmp_path / "tools" / "ffmpeg" / "bin"
    platform_tools_dir = tmp_path / "tools" / "platform-tools"
    ffmpeg_dir.mkdir(parents=True)
    platform_tools_dir.mkdir(parents=True)
    (ffmpeg_dir / "ffmpeg.exe").write_bytes(b"")
    (platform_tools_dir / "adb.exe").write_bytes(b"")

    monkeypatch.setenv("SLAM_XCAM_TOOLS_DIR", str(tmp_path))
    monkeypatch.setenv("PATH", "")

    directories = bundled_tool_directories()
    assert directories[:2] == [ffmpeg_dir.resolve(), platform_tools_dir.resolve()]
    configure_bundled_tools()
    assert Path(find_tool("ffmpeg") or "").resolve() == (ffmpeg_dir / "ffmpeg.exe").resolve()
    assert Path(find_tool("adb") or "").resolve() == (platform_tools_dir / "adb.exe").resolve()
