@echo off
set PYTHONIOENCODING=utf-8
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_sac_path_empty.py" --id12-stiffness-check --execute
pause
