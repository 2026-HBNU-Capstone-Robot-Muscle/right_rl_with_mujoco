@echo off
set PYTHONIOENCODING=utf-8
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_hand_prepare_current_empty.py" --execute
set "leap_hand_exit=%errorlevel%"
pause
exit /b %leap_hand_exit%
