@echo off
cd /d "%~dp0"
echo STAGE 1: recorded forward rotation, then fixed 10-second hold.
echo This command does not run the new two-turn cycle.
echo Start EMPTY. Place the cube only when PLACE is printed.
".runtime\python\python.exe" -X utf8 -B "leap_forward_hold_observe_20261104.py" --execute
pause
