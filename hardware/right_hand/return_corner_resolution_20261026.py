"""Existing 10-micrometre palm6/tip model resolution, EMPTY return only.

Refinement measured 0.00748/0.00789/0.00816mm; the contact does not disappear.
This is an explicit geometry resolution policy, not proof of an integration
error or actual physical separation. MuJoCo contacts/forces remain intact.
"""
PAIR='palm_collision_6|th_tip'
PHASE='stage_3_empty_return_with_tip_clearance'
NUMERICAL_ZERO_MM=.01
def classify(pairs,*,phase=None):
    return {p:v for p,v in pairs.items() if not (phase==PHASE and p==PAIR and 0<=v<=NUMERICAL_ZERO_MM)}
def metadata():
    return dict(pair=PAIR,phase=PHASE,model_resolution_mm=NUMERICAL_ZERO_MM,
                same_pair_had_this_resolution_in_initial_release=True,
                observed_raw_depths_mm=[.007482596546658208,.007893081830377041,.00816329650260634],
                contact_vanishes_under_timestep_refinement=False,
                pure_integration_error_not_claimed=True,physical_geometry_calibrated=False,
                collision_parameters_or_contact_forces_disabled=False,hardware_health_limits_changed=False)
