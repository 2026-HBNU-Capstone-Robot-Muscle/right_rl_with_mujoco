# Current-pose empty thumb preparation

Run `leap_thumb_prepare_current_empty.cmd` once with the cube removed and the
hand left as it is. Close other motor programs, secure the base, show the thumb
and middle finger in the webcam, and clear fingers/cables. Press SPACE once in
the terminal. Q/Esc/Ctrl+C stops.

The program reads three torque-OFF encoder snapshots and checks that their
position spread is at most two ticks. It then screens the remaining thumb
preparation from that exact pose using the installed WSL simulator. Expect a
pause of roughly 30 seconds for the screen on this PC. A three-second countdown
precedes movement. One command records the camera and motors. There is no cube
placement or flipping during this trial.

Roots 8/12/16 rise away from the support if necessary. Thumb 1/2/3/4 then move
slowly to `[2601,3100,3074,2708]` ticks, with at most two ticks per goal update.
Other finger bending/rotation goals hold their current readings. The longer
remaining trajectory has a 55-second ON limit. Current/PID, voltage, tracking,
fixed-joint drift, measured-progress and camera stops are retained. Goals are
not retried. Torque OFF and original setting restoration follow the final hold.

This supports the measured family of open non-thumb postures, rather than
arbitrary hand configurations. Changing finger shapes may require a new review.
Historical ON angles are not used as a mandatory OFF starting pose. Any start
outside the admitted family, model limits, clearance screen or fresh-read checks
stops before motion; do not manually force the hand into an archived pose.

The current-start check uses 67 perturbations within four ticks and three gain
scales in native MuJoCo. Joint/control ranges are preserved. The result is tied
to the actual request, scheduled path, mapping and checker/planner files. Before
configuration, the hand must still match the screened neighborhood. This is a
model screen, not proof of physical clearance or calibrated actuator dynamics.

Python without `--execute` only prints the description. It opens no camera,
motor port, or simulator process. The normal launcher requires the existing WSL
Python `/home/star1209042/venv/bin/python`. Missing WSL, failed simulation,
cancelled checks, stale/incorrect proof or changed encoders cannot enable torque.

User-run output is a timestamped `thumb_prepare_current_empty_*` folder, including
`fresh_off_request.json`, `current_start_screen.json`, its log, camera frames,
and `result.json`. Latest report: `leap_thumb_prepare_current_empty_result.json`.
The stored simulation fixture intentionally uses the earlier unsettled OFF
sample and is never treated as a fresh physical starting pose.
