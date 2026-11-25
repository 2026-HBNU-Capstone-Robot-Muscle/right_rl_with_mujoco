@echo off
setlocal
set PYTHONIOENCODING=utf-8
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_roots_empty.py" --execute
set trial_exit=%errorlevel%
pause
exit /b %trial_exit%
