@echo off
set PYTHONIOENCODING=utf-8
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_book_dynamic_yaw_trial.py" --execute
set "book_small_yaw_exit=%errorlevel%"
pause
exit /b %book_small_yaw_exit%
