from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys


def bundled_tool_directories() -> list[Path]:
    """Return portable tool directories in preferred lookup order."""

    roots: list[Path] = []
    configured_root = os.environ.get("SLAM_XCAM_TOOLS_DIR")
    if configured_root:
        roots.append(Path(configured_root))
    bundle_root = getattr(sys, "_MEIPASS", None)
    if bundle_root:
        roots.append(Path(bundle_root))
    if getattr(sys, "frozen", False):
        roots.append(Path(sys.executable).resolve().parent)
    else:
        roots.append(Path(__file__).resolve().parents[2])

    candidates: list[Path] = []
    for root in roots:
        candidates.extend(
            [
                root / "tools" / "ffmpeg" / "bin",
                root / "tools" / "platform-tools",
                root / "platform-tools",
            ]
        )
    unique: list[Path] = []
    for path in candidates:
        resolved = path.resolve()
        if resolved.is_dir() and resolved not in unique:
            unique.append(resolved)
    return unique


def configure_bundled_tools() -> list[Path]:
    """Prepend portable tools to PATH so existing subprocess calls use them."""

    directories = bundled_tool_directories()
    if directories:
        current = os.environ.get("PATH", "")
        existing = [entry for entry in current.split(os.pathsep) if entry]
        os.environ["PATH"] = os.pathsep.join([*(str(path) for path in directories), *existing])
    return directories


def find_tool(name: str) -> str | None:
    configure_bundled_tools()
    return shutil.which(name)


def hidden_subprocess_kwargs() -> dict:
    """Return subprocess kwargs that prevent console popups on Windows."""

    if hasattr(subprocess, "CREATE_NO_WINDOW"):
        return {"creationflags": subprocess.CREATE_NO_WINDOW}
    return {}


configure_bundled_tools()

