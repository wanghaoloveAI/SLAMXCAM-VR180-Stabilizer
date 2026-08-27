$ErrorActionPreference = "Stop"
$python = if ($env:PYTHON_EXE) { $env:PYTHON_EXE } else { "python" }
$env:PYTHONPATH = Join-Path $PSScriptRoot "src"
& $python -m slam_stabilizer.qt_gui

