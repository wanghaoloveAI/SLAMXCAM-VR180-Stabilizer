from __future__ import annotations

import subprocess


def hidden_subprocess_kwargs() -> dict:
    """Return subprocess kwargs that prevent console popups on Windows."""

    if hasattr(subprocess, "CREATE_NO_WINDOW"):
        return {"creationflags": subprocess.CREATE_NO_WINDOW}
    return {}

