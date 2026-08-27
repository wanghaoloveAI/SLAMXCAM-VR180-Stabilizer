@echo off
pushd "%~dp0"
if not defined PYTHON_EXE set "PYTHON_EXE=python"
set "PYTHONPATH=%~dp0src"
"%PYTHON_EXE%" -m slam_stabilizer.qt_gui
popd
