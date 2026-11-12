@echo off
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -X utf8 -B "%~dp0cube_red_green_capture_20261026.py" --capture
pause
