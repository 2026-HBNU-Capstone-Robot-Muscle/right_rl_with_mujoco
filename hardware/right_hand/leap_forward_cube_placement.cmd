@echo off
cd /d "%~dp0"
echo EMPTY hand for FIRST SPACE. Cube only at PLACE CUBE.
echo PLACE: about5mm toward fingertips from previous placement. GREEN UP, ORANGE toward WRIST/ID41.
echo SAME turn targets; REMOVE CUBE before THIRD SPACE/OFF. Q/Esc/Ctrl+C stops.
start "" "right_hand_rotation_20261024\loaded_finish_review_0057_20261026\forward_placement_guide.html"
".runtime\python\python.exe" -X utf8 -B "leap_forward_cube_placement_20261026.py" --execute
pause
