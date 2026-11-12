@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
echo EMPTY HAND at FIRST SPACE. Cube only after PLACE CUBE.
echo EMPTY opening: release/CONFIRM, ring clearance, then thumb and other fingers.
echo Revised contact candidate: small contact refinement, then experimental 3deg yaw goal.
echo Simulated range 0.93-3.24deg. Physical rotation unverified; no live SAC.
echo Camera exposure -5 from saved comparison; original settings restored on exit.
echo Read recovery: two fresh healthy snapshots before the next target; no goal resend.
echo Keep hands clear until REMOVE CUBE. Q/Esc/Ctrl+C stops.
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_grasped_yaw_trial.py" --execute
set "LEAP_GRASPED_YAW_EXIT_CODE=%errorlevel%"
pause
exit /b %LEAP_GRASPED_YAW_EXIT_CODE%
