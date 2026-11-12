@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
echo SLOW HAND DEMO. Remove cube before starting. Cube only after PLACE CUBE.
echo FIRST SPACE starts EMPTY preparation. Keep cube outside hand and camera until PLACE CUBE.
echo Same reached grasp endpoints; no cube rotation or SAC policy.
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_slow_grasp_demo.py" --execute
set "LEAP_SLOW_GRASP_DEMO_EXIT_CODE=%errorlevel%"
pause
exit /b %LEAP_SLOW_GRASP_DEMO_EXIT_CODE%
