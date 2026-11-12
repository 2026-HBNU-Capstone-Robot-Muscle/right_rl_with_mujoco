@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
set "LEAP_SUMMARY_READY="
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_trial_summary.py" --begin
if not errorlevel 1 set "LEAP_SUMMARY_READY=1"
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_cube_grasp_trial.py" --execute
set "LEAP_TRIAL_EXIT_CODE=%errorlevel%"
if defined LEAP_SUMMARY_READY "%~dp0.runtime\python\python.exe" -B "%~dp0leap_trial_summary.py" --finish
pause
exit /b %LEAP_TRIAL_EXIT_CODE%
