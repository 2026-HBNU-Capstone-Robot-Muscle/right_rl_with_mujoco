@echo off
set PYTHONIOENCODING=utf-8
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_pose_pair_readonly.py" --capture --reference middle_ring_existing_side
set "middle_ring_readonly_exit=%errorlevel%"
pause
exit /b %middle_ring_readonly_exit%
