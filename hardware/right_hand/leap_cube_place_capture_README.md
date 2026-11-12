# Cube placement on the prepared hand

Run `leap_cube_place_capture.cmd` with the cube removed. Leave the fingers as
they are and keep the whole hand/cube area visible in the webcam.

1. FIRST SPACE in the terminal reads stable current OFF angles, screens the
   recovery path and starts a three-second countdown. Preparation moves empty.
2. Wait for `PLACE CUBE NOW`. Put the cube into the space above the palm between
   the fingers, green up and orange toward the camera as a reference. Do not
   force it. Clear your hand, then SECOND SPACE saves the photo. Motor goals
   continue holding; the fingers do not execute a flip.
3. When `REMOVE CUBE` appears, remove the cube, then THIRD SPACE confirms it and
   turns torque OFF. Original settings are restored and OFF is verified.

Q/Esc/Ctrl+C stops. Placement and removal confirmation each have a 90-second
limit with fresh motor health reads throughout. Release SPACE between prompts;
holding it cannot confirm the following stage.

This is a qualitative placement and loaded-hold capture. ID40 can be occluded;
there is no marker prerequisite and no claim of 3D registration. Save the whole
cube in frame. The run writes `prepared_empty.jpg`, `cube_placed.jpg`, camera
frames, numeric placement/goal/current snapshots and the final result in a
`prepared_placement_capture_*` folder. Latest report is
`leap_prepared_placement_capture_result.json`.

The preparation admission envelope uses the completed hand trial and its OFF
readings. Those settling readings are not assumed to be today's stable pose.
The actual current state is read and its exact recovery path screened at run
time before any setting write. Native model limits and existing current/PID,
voltage, tracking and movement checks are preserved. No goal changes are sent
during placement or removal. Model screening does not prove physical clearance.
