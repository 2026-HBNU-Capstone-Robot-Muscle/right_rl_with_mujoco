@echo off
set PYTHONIOENCODING=utf-8
echo SIDE PHOTO + ENCODERS ONLY. No motor targets or torque changes.
echo Keep the existing upright middle pose. Move the camera only.
echo Show palm side edge and entire middle finger, root to fingertip, from the side.
echo USB and motor power connected; torque must remain OFF. Close other motor programs.
echo Rest camera on stable support. SPACE once starts angle reads and a 1-second photo burst.
echo Keep camera still until SAVED. Best-detail frame and burst images are saved. No markers required. Q exits.
"%~dp0.runtime\python\python.exe" -B "%~dp0leap_pose_pair_readonly.py" --capture --reference middle_upright_side
pause
