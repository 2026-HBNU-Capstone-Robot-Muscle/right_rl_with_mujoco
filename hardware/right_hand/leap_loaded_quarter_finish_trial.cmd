@echo off
cd /d "%~dp0"
echo FIRST SPACE with EMPTY hand. Cube outside hand and camera.
echo PLACE: green UP, orange toward WRIST/ID41. SECOND SPACE starts reference plus new finish.
echo REMOVE CUBE before THIRD SPACE and automatic OFF. Q/Esc/Ctrl+C stops.
".runtime\python\python.exe" -X utf8 -B "leap_loaded_quarter_finish_trial_20261025.py" --execute
pause
