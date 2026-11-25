@echo off
chcp 65001 >nul
cd /d "%~dp0"
set "ROTATION_PYTHON=python"
if exist ".venv\Scripts\python.exe" set "ROTATION_PYTHON=.venv\Scripts\python.exe"
if exist ".runtime\python\python.exe" set "ROTATION_PYTHON=.runtime\python\python.exe"
"%ROTATION_PYTHON%" -X utf8 -B current_motion_entry_20261125.py --mode horizontal --execute
pause
