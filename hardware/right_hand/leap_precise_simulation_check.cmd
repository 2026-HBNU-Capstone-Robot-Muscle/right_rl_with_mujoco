@echo off
cd /d "%~dp0"
"%~dp0.runtime\python\python.exe" -X utf8 -B "%~dp0run_precise_simulation_review_20261028.py" --run-simulation
pause
