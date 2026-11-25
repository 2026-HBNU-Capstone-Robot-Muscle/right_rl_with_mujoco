"""Qualify saved evidence without changing any dynamics or collision bounds."""
def task_reasons(d):
    if not d:return ['continuous_replay_missing']
    why=[]
    if not d['safety_passed']:why.append('original_dynamic_safety')
    if d['regrasp_yaw_change_deg'] is None or abs(d['regrasp_yaw_change_deg'])>=5:why.append('regrasp_yaw')
    if d['final_up_tilt_deg']>=5:why.append('terminal_tilt')
    t=d['settled_tail'];cert=t.get('certificate')
    if cert:
        expected=round(t['seconds']/.001)
        if (t.get('separation_certificate_bad_steps_1ms')!=0 or t.get('certificate_unqueried_steps_after_first_failure',0)!=0 or cert.get('queries')!=expected or t.get('minimum_separating_plane_gap_1ms_mm',-.1)<=.3):why.append('exact_cylinder_separation_not_certified_at_every_1ms_state')
    elif t['minimum_upper_gap_sampled_75ms_mm']<=.3:why.append('sampled_layer_clearance')
    if t['yaw_drift_deg']>=3:why.append('settled_yaw_drift')
    if t['center_shift_mm']>=2:why.append('settled_translation')
    if t['opposed_lower_body_contact_fraction']<=.8:why.append('lower_body_contact')
    return why
