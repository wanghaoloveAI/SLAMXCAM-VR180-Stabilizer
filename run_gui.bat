@echo off
pushd "%~dp0"
set "PYTHON_EXE=C:\Users\hao wang\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
set "PYTHONPATH=%~dp0src"
"%PYTHON_EXE%" -m slam_stabilizer.gui
popd
