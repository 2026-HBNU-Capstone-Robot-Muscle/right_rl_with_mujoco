# Empty hand preparation after the reached thumb target

Run `leap_hand_prepare_current_empty.cmd` with no cube, leaving the hand as it
is. Show thumb and middle finger in the webcam and clear fingers/cables. Press
SPACE once in the terminal. The program reads stable OFF angles, checks the
actual path in WSL, then gives a three-second countdown before movement.
Q/Esc/Ctrl+C stops. One launcher records camera and encoders and verifies OFF
and original setting restoration after the final hold.

This is the next preparation stage: root clearance, a small thumb adjustment,
then the index, middle and ring fingers move to the modeled preparation pose.
The thumb stays at its preparation target during finger movement. Keep the cube
out for the whole trial. This is not a cube rotation or a live SAC rollout.

The preceding 00:20 trial reached all 608 targets and four final-goal samples.
It stopped during the final hold because its extra 50ms sleep was added to
camera, read and write time, making most cycles about 94ms. The archived result
stays marked stopped; it is not relabeled a completed one-second hold.

The revised scheduler includes processing in each 50ms cycle. It never skips
targets or sends a catch-up burst: every target still follows a new health
read and advances by at most two ticks. The ON limit remains 55 seconds. The
current/PID, voltage, tracking, measured-progress, held-joint drift, camera and
OFF/restoration checks remain active. The new finger phase also stops a joint
that makes no measured progress with a growing goal error.

Startup is supported near the measured open-finger posture with the thumb
near its reached preparation target. Each actual start is checked again; a
model screen is not proof of physical geometry or calibrated motor dynamics.
The stored native-model fixture passed 67 start perturbations and three gain
scales with no modeled self/support overlap or joint/control-range violation.

Without `--execute`, Python only prints the program description and opens no
camera, motor or simulator process. User output is archived under
`hand_prepare_current_empty_*`, with the latest report in
`leap_hand_prepare_current_empty_result.json`.
