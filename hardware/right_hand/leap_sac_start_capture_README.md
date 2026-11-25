# Revised SAC start: one-command placement capture

Purpose: the new simulated starting hand pose has now completed an empty hardware
trial. Previous cube-placement images belong to the old pose. Capture the actual
cube on the revised pose before claiming its placement matches the learned reset.
This does not execute the flip or call the SAC policy.

Run `leap_sac_start_capture.cmd`:

1. Cube off the hand. Keep the hand base and ID41 rigid; show palm ID40 and ID41
   together. SPACE in the camera preview starts checks and a 3-second countdown.
2. The same root-relief, revised-start and short-hold path already tested is used.
   A fresh photo of the FINAL pose is saved separately. Fingers may cover ID40:
   no marker visibility gate is applied after preparation. The earlier ID40+41
   reference is retained, but its rigid relation is not certified at the final
   pose. No extra movement is inserted and no 3D registration is approved.
3. At PLACE CUBE, place green-up/orange-front without forcing a fit. Clear your
   hands. SPACE in camera or terminal saves the current image and encoder sample.
   Check that the entire cube, including its top edge, is inside the live frame;
   the 21:17 capture cut off the top and cannot independently resolve its small
   marker's two planar pose solutions. Keep ID41 visible; ID40 may be covered.
   There is no cube-marker count gate for this diagnostic photo: a missing or
   single marker is recorded, not misrepresented as a validated 3D pose.
4. The fingers keep the same goals. Remove the cube, then SPACE in the terminal
   turns torque OFF and restores settings. Q/Esc/Ctrl+C or faults stop immediately.

Results: `leap_sac_start_capture_result.json`, timestamped archive, and a separate
`sequence_vision_*` image folder. The empty test result is not overwritten.

Shares 450mA/original sequence gains, health checks and `fresh_confirm_v1` voltage
handling with the tested start script. The assistant has not run this on hardware.
Default `python leap_sac_start_capture.py` without `--execute` validates only the
pinned target and does not open camera or serial port.
