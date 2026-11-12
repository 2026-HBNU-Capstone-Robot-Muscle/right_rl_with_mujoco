@echo off
cd /d "%~dp0"
echo SLOW1.5x duration. Voltage cutoff unchanged. Power wiring unchanged.
echo EMPTY at FIRST SPACE. Cube only at PLACE. REMOVE before THIRD SPACE/OFF.
".runtime\python\python.exe" -X utf8 -B "leap_slow_forward_placement_20261026.py" --execute
pause
