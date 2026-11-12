"""Grade a recorded free-cube90 continuation independently of regrasp-only yaw.

Passing this filter is a checkpoint result. Full initial placement and physical
motion remain separate proof obligations. No physical adapters are imported.
"""
import numpy as np

def quarter_reasons(verification,reference_center,constant_tail,orientation_error_deg):
    if verification is None:return ['recorded_goal_replay_incomplete']
    v=verification;t=v.get('settled_tail',{});reference=np.asarray(reference_center,float)
    if reference.shape!=(3,) or not np.isfinite(reference).all():raise ValueError('finite common-grip reference center required')
    fields=[v.get('final_yaw_from_placement_deg'),v.get('final_up_tilt_deg'),orientation_error_deg,t.get('seconds'),t.get('yaw_drift_deg'),t.get('center_shift_mm'),t.get('minimum_separating_plane_gap_1ms_mm'),t.get('opposed_lower_body_contact_fraction')]
    if any(value is None or not np.isfinite(value) for value in fields):return ['missing_or_nonfinite_replay_evidence']
    center=np.asarray(v.get('final_center_world_m'),float)
    if center.shape!=(3,) or not np.isfinite(center).all():return ['missing_or_nonfinite_replay_center']
    why=[]
    if v.get('safety_passed') is not True or v.get('completed_goals')!=v.get('total_goals'):why.append('original_dynamic_safety_or_incomplete')
    if not constant_tail:why.append('last134_goals_not_constant')
    if abs((v['final_yaw_from_placement_deg']-90+180)%360-180)>=5:why.append('whole_cube_not_at90')
    if v['final_up_tilt_deg']>=5 or orientation_error_deg>=5:why.append('working_top_or_target_orientation')
    if np.linalg.norm(center-reference)*1000>=2:why.append('common_center_not_restored')
    if t['seconds']<4.05-1e-9 or t['yaw_drift_deg']>=3 or t['center_shift_mm']>=2:why.append('settled_hold_drift_or_duration')
    if t.get('separation_certificate_bad_steps_1ms')!=0 or t.get('certificate_unqueried_steps_after_first_failure')!=0 or t.get('certificate',{}).get('queries')!=4050 or t['minimum_separating_plane_gap_1ms_mm']<=.3:why.append('exact_upper_space_not_proved_at_every_tail_state')
    if t['opposed_lower_body_contact_fraction']<=.8:why.append('required_lower_contact_presence')
    return why
