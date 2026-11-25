@echo off
set PYTHONIOENCODING=utf-8
echo READ ONLY: no motor targets, torque changes, gain changes or calibration writes.
echo Fixed SIDE camera. Current pose first; FIRST SPACE saves baseline.
echo After BASELINE SAVED: motor power OFF; gently flex only middle TIP inward 10-20 degrees.
echo Do not force. Restore power with torque OFF; hands away; SECOND SPACE saves changed pose.
echo One window, two photographs with encoder reads. Q cancels. Close other motor programs.
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_pose_pair_readonly.py" --capture --reference middle_tip_direction
pause
