@echo off
set PYTHONIOENCODING=utf-8
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_book_cube_grasp_trial.py" --execute
set "book_grasp_exit=%errorlevel%"
pause
exit /b %book_grasp_exit%
