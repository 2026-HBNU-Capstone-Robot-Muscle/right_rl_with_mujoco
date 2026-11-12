@echo off
setlocal
echo CONTACT VIEW: existing grasp path only. No rotation or stronger grip.
echo After Approach ended / REMOVE CUBE appears, FIRST show INDEX/MIDDLE gaps for 5-10s.
echo Move only the camera outside finger paths, then REMOVE cube and SPACE for OFF.
call "%~dp0leap_cube_grasp_trial.cmd"
exit /b %errorlevel%
