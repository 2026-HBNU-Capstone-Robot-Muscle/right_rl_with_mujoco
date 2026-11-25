@echo off
echo This extra capture is no longer needed: middle tip-to-root order 9,11,10,12 confirmed by user.
echo No camera or motor port will be opened. Offline mapping/path review is in progress.
pause
exit /b 0
rem Original procedure retained below for history; unreachable.
set PYTHONIOENCODING=utf-8
echo READ ONLY: no target, torque, gain or calibration writes. No cube.
echo Close other motor programs. USB + motor power ON, torque already OFF.
echo Keep the current pose. Fixed side camera, show entire MIDDLE finger. FIRST SPACE saves baseline.
echo AFTER BASELINE SAVED: MOTOR POWER OFF before touching fingers.
echo Bend the SECOND hinge from the fingertip inward about 10-15 degrees.
echo Move the TIP PAD and its MOTOR HOUSING together. Keep their relative angle unchanged.
echo Support the lower link; keep root still. No force or contact. This is NOT tip-only motion.
echo Restore motor power with torque OFF, hands away; SECOND SPACE saves the new pose. Q cancels.
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_pose_pair_readonly.py" --capture --reference middle_pip_order
pause
