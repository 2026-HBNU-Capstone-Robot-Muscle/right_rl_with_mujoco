@echo off
set PYTHONIOENCODING=utf-8
echo MIDDLE REFERENCE MEASUREMENT ONLY. No powered positioning or calibration writes.
echo First view the reference image and prepare gently with motor power OFF.
echo Other fingers need not match the image. Do not force a joint.
echo Then connect power with torque OFF for PING/READ, and press SPACE in the preview once.
start "" "%~dp0paired_pose_audit_20261017\middle_reference_upright.jpg"
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_pose_pair_readonly.py" --capture --reference middle_upright
pause
