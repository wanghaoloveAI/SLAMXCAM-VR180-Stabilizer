$ErrorActionPreference = "Stop"
$python = "C:\Users\hao wang\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
$env:PYTHONPATH = Join-Path $PSScriptRoot "src"
& $python -m slam_stabilizer.inspect_imu @args

