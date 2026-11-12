@echo off
set PYTHONIOENCODING=utf-8
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_pose_pair_readonly.py" --capture
pause
