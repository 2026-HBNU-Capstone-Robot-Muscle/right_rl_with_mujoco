@echo off
set PYTHONIOENCODING=utf-8
echo DRIVE SETTINGS READ ONLY. Connect USB and motor power; torque stays OFF.
echo Cube removed. Close other motor programs. No motion or setting writes.
"%~dp0.runtime\python\python.exe" -B "%~dp0hand_readonly_check.py" --drive-settings
pause
