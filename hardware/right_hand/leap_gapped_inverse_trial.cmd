@echo off
setlocal
cd /d "%~dp0"
".runtime\python\python.exe" -X utf8 -B "leap_gapped_inverse_trial_20261029.py" --execute
pause
endlocal
