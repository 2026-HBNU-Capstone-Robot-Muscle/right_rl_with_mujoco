# Next empty thumb preparation segment

Run `leap_thumb_continuation_empty.cmd` once. Leave the fingers in the pose left
by the completed small-thumb trial. Remove the cube, secure the base, close
other motor programs, and show the thumb and middle finger in the webcam.
Press SPACE in the terminal when fingers and cables are clear. Q/Esc/Ctrl+C
stops. One launcher records the camera and encoders and turns torque OFF after
the final hold. No additional phone or second terminal is needed.

This continues preparation toward the cube holding pose: roots first, then
thumb joints up to 384 ticks (33.75 degrees). Other bending and rotation goals
remain at the fresh starting readings. Do not place a cube during this trial.

Revision 2 uses the actual read-only OFF readings saved at 19:24, after revision
1 stopped before any write. Fresh readings must be within 32 ticks of this OFF
reference, then pass separate four-tick stable OFF/ON checks. The tolerance is
unchanged. A mismatch stops before motion; do not manually force an old pose.

After motion and verified OFF/settings restoration, the runtime also saves two
fresh OFF encoder snapshots. They record whether the OFF endpoints were stable
within two ticks, and the change from the last ON position. These readings do
not automatically approve a new starting neighborhood.

The runtime retains the current, tracking, voltage, measured-progress, camera,
and 35-second guards. No goal retry, raw angle wrap, EEPROM change, gain
increase, or simulated range change is deployed.

The 67-case static screen passed. The native model still cannot hold the
measured initial ID3 angle within its original limit. Dynamics using an
isolated observed-range diagnostic model passed the tested gain scales, but
this is not a physical clearance or actuator calibration guarantee. See
`right_hand_preparation_20261021/thumb_continuation_screen.json`.

`leap_thumb_continuation_empty.py` without `--execute` only validates and prints
the nominal plan. It opens no camera or motor port. Actual user trials save a
timestamped `thumb_continuation_empty_*` folder, including camera frames and
`result.json`. The latest report is `leap_thumb_continuation_empty_result.json`.
