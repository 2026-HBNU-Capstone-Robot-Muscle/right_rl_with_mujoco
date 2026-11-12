"""Numerical10micrometre classification for one observed palm-box seam.

Only validation bookkeeping changes. The MuJoCo contacts and reactions are
not removed or changed, and this is not a physical clearance calibration.
"""
PAIR='palm_collision_7|th_tip'
NUMERICAL_ZERO_MM=.01
def classify(pairs):
    return {p:d for p,d in pairs.items() if not(p==PAIR and 0<=d<=NUMERICAL_ZERO_MM)}
def metadata():
    return dict(revision='slow_known_thumb_palm_seam_resolution_v1_20261026',
        pairs=[PAIR],numerical_zero_mm=NUMERICAL_ZERO_MM,
        scope='known palm7/thumb-tip seam only, including post-release poses',
        collision_parameters_or_contact_forces_disabled=False,raw_discarded_depths_recorded=True,
        physical_clearance_calibration_verified=False,actual_motor_health_stops_changed=False)
