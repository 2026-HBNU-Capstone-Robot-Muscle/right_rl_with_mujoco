# Cube close-up — no motor control

Run `leap_cube_closeup.cmd`. Keep motor power OFF, put cube on a table outside
the robot hand, and face the green ID5 side toward the external webcam.
Move only the camera/cube as needed to show the whole face with its white rim,
roughly head-on and without glare. ID40 and ID41 are NOT required.

The preview shows READY when the detected ID5's short edge is at least 80px,
foreshortening is modest, and its estimated whole-face extent fits in the image.
Press SPACE once in camera or terminal and hold still while ten fresh raw frames
are saved. Q/Esc/Ctrl+C stops; partial captures are preserved. A recording that
cannot collect the frames in 15 seconds ends with an explicit incomplete result.

Outputs: `cube_closeup_<timestamp>/result.json` and raw PNGs, plus
`leap_cube_closeup_result.json`. Capture status is explicitly UNVALIDATED.
There is no motor dependency, serial port, goal, torque or policy action.
Running the Python script without `--capture` performs check-only, no devices.

Purpose: archived placed-cube images contain at most a 40px short ID5 edge.
Different corner refinements move the inferred camera depth by roughly 14mm.
This larger, unobstructed reference is to diagnose the image measurement, not
another robot pose trial. Current reported dimensions are cube 55mm and ID5
approximately 10.7mm. Do not shrink the physical dimensions to fit simulation.
