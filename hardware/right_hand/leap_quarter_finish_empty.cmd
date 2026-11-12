@echo off
cd /d "%~dp0"
echo EMPTY HAND FOR ENTIRE RUN. Cube outside hand and camera. No PLACE prompt.
echo One SPACE in terminal. Initial camera wait is automatic; same reviewed motor path.
".runtime\python\python.exe" -X utf8 -B "leap_quarter_finish_empty_preview_fixed_20261025.py" --execute
pause
