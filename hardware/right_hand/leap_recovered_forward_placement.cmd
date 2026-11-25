@echo off
cd /d "%~dp0"
echo EMPTY at FIRST SPACE. Current posture is measured; do not reshape fingers.
echo Cube only at PLACE: about5mm toward tips. GREEN UP, ORANGE toward wrist.
echo REMOVE before THIRD SPACE/OFF. Q/Esc/Ctrl+C stops.
".runtime\python\python.exe" -X utf8 -B "leap_recovered_forward_placement_20261026.py" --execute
pause
