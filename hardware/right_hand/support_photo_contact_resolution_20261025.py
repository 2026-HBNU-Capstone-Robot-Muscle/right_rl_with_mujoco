"""Resolve two initial thumb/palm box seams at 0.01mm; contact forces unchanged."""
NUMERICAL_ZERO_MM=.01
PAIRS=frozenset(('palm_collision_7|th_tip','palm_collision_8|th_tip'))
def classify(pairs,*,initial_release):
    return {k:v for k,v in pairs.items() if not (initial_release and k in PAIRS and 0<=v<=NUMERICAL_ZERO_MM)}
def metadata():return dict(version='support_photo_initial_thumb_seam_resolution_v1',pairs=sorted(PAIRS),numerical_zero_mm=NUMERICAL_ZERO_MM,
    scope='static starting pose and initial hold/ring release only; strict raw classification afterwards',
    contact_forces_preserved=True,actual_tracking_current_voltage_stops_unchanged=True,physical_contact_calibration_verified=False)
