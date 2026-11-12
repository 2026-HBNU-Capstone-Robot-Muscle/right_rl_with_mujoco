@echo off
set PYTHONIOENCODING=utf-8
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_sac_start_empty.py" --execute
pause
