# Multiview anchor analysis — 2026-10-14

Source: `anchor_multiview_20261014_030307_761969/result.json`, three views,
15 raw frames, no motor port. Original images/corners preserved.

Two ID41 contour detections (`view_3_2.png`, `view_3_5.png`) disagree by
15.00/15.30 pixels with the stationary burst. Gray-image affine alignment
shows only 0.087/0.056 pixel motion and correlations above 0.999. The exact
physical border represented by the alternate contour is not certified.
This is evidence of a corner-detection jump, not a moving wrist mount.

`anchor_burst_consistency.py` rejects frames only after independent gray-image
alignment and comparison against a burst medoid. It does not replace corners
or fit them to a simulated pose. Requires at least three coherent frames per
view, image correlation >=0.98, image movement <=2px and corner disagreement
<=3px. Does not detect a bias shared by all frames. Raw ArUco detector and motor
runtime are unchanged. Three tests pass, including real failures, permitted
small camera motion, and insufficient-burst rejection.

All-frame bundle: RMS 2.095px, rejected. Screened 13-frame bundle: RMS 0.548px,
per-view 0.379/0.416/0.880px, camera-normal span 39.06 degrees. This passes the
offline image-fit diagnostic. Fixed approximate physical sizes: ID40 28.2mm,
ID41 27mm. It does not validate these dimensions or marker-to-hand-model pose.

Comparison against the archived 21:31 capture: relative-frame differences
1.03–1.30mm / 0.66–0.80deg, projected ID40 error 1.96–2.03px. Applying the new
relation hypothetically changes the previous cube center by only
(+0.12,+0.20,-1.48)mm. The approximately 10mm height discrepancy remains;
no cross-session transform has been installed in live code.

Additional independent image diagnostic: manual intersections of the outer
white-plastic top face, assuming the configured 54.5mm cube, imply the detected
ID5 black-square sides are approximately 9.20–9.74mm (mean 9.45mm), rather
than the configured 10.6mm. The outer-face PnP and small-marker PnP estimate
camera depths 237.8mm and 256.1mm. This is a dimensional inconsistency clue,
not a measurement certificate: corner annotations, rounded plastic, upper
image clipping, marker detection and non-coplanarity may bias the result.
The +/-2px annotation sensitivity is NOT a physical confidence interval.

`anchor_pattern_diagnostic.py` was an ECC/template experiment only. It did not
resolve all contour ambiguities; `anchor_pattern_diagnostic_input.json` is
explicitly unvalidated and is not the accepted bundle input.

Next necessary physical information: measured cube side and black ID5 width /
height, preferably 0.1mm precision. User's earlier approximate 10.6mm answer
remains recorded; no arbitrary replacement has been made. No robot replay or
new placement photo is requested. Continue offline geometry/pose and policy
integration after resolving the scale discrepancy. The learned physical flip
has not been demonstrated.
