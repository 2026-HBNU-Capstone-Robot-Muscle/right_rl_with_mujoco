# ID40 / ID41: three-view reference capture

Purpose: separate monocular planar-pose ambiguity from the fixed relation between
the palm and wrist markers. The last eight archived paired-marker photographs
span only 0.139 degrees in estimated camera normal; a low 0.292px joint image fit
alone does not validate their depth. This is a different measurement from repeating
the revised-start motion or cube-placement photograph.

1. Remove the cube and keep motor power OFF. Close other camera programs.
   Keep the hand base and the attached markers stationary throughout.
2. Run `leap_anchor_multiview.cmd`. This program imports no motor driver,
   never opens a serial port, and never changes torque, goals or settings.
3. Show both ID40 and ID41. Briefly press SPACE in the camera window or terminal.
   Hold the camera still while five fresh paired-marker frames are saved.
4. Move **only the camera** around the hand to one side by roughly 15–25 degrees,
   keeping both markers visible. Press SPACE and hold still again.
5. Move the camera to the other side, keeping both markers visible. SPACE saves
   the third view and the program closes. Do not merely rotate the image in place;
   the camera needs to see the marker planes from a different direction.

No cube, checkerboard, new hand pose or motor command is involved. Do not alter
focus, zoom, marker mounting or resolution. A live angle estimate is shown as a
guide; it does not certify calibration. Q/Esc/Ctrl+C exits and preserves saved data.

Outputs: `anchor_multiview_TIMESTAMP/result.json`, 15 raw PNGs, and
`leap_anchor_multiview_result.json`. The report deliberately says *captured,
unvalidated*. ID40=28.2mm and ID41 approximately 27mm are recorded; both physical
size uncertainty and existing camera calibration remain relevant.

Offline processing: `sac_anchor_bundle.py INPUT_JSON` in the existing WSL
NumPy/SciPy environment. It jointly fits the rigid ID40-to-ID41 transform and
camera poses with fixed intrinsics/sizes, without using a desired cube/model
location as a target. Fits with less than 12 degrees of camera-normal diversity
or high reprojection error are rejected. Even a passing result is a candidate;
the separate marker-to-hand-model transform is not calibrated by this procedure.
No live calibration is automatically overwritten.

Validation: OpenCV-generated synthetic fixtures independently test the NumPy
distortion projection, recovery of a known multi-view transform, and rejection
of duplicate views. All three tests pass. The launcher's default/check-only mode
was executed without opening any device; physical capture has not been run by
the assistant.
