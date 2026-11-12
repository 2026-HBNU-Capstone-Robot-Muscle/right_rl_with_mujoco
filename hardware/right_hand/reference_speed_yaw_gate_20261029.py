"""Existing27-case loaded review plus the actually completed motor sequence."""
import json
from pathlib import Path
import numpy as np
import calibrated_quarter_gate_20261028 as base
from reference_speed_yaw_plan_20261029 import ReferenceSpeedYawPlan,ROOT,OUT,REQUEST,PROOF,REFERENCE,hashes,loaded_rows
from retained_slow_yaw_gate_20261029 import readonly_recovery_review,fast_read_review,serial_link_review
from calibrated_quarter_gate_20261028 import PRELOAD_POLICY,GOAL_POLICY,ROOT_POLICY,EPISODE_POLICY,CLOCK_POLICY
from combined_quiet_reserve_20261029 import POLICY as RECEIVE_POLICY
from fast_health_window_20261029 import WINDOW_POLICY
from paced_retained_yaw_gate_20261029 import PASSIVE_POLICY
from review_empty_hold_comm_20261029 import review as communication_evidence
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
from hand_motor_mapping import SIM_TO_MOTOR_IDS
def completed_reference():
    r=json.loads(REFERENCE.read_bytes())
    if r['status']!='calibrated_quarter_observation_complete' or not r['all_torque_off_verified'] or not r['settings_restored']:raise RuntimeError('Completed physical reference unavailable')
    if r['requested_sample_period_s']!=.075:raise RuntimeError('Physical reference timing differs')
    goals=[q.tolist() for phase,q in loaded_rows()]
    written=[e['requested'] for e in r['write_events'] if e['address']==116]
    if written[-len(goals):]!=goals:raise RuntimeError('Loaded targets differ from the completed physical goal writes')
    # Verify applied settings, not the original OFF/preflight settings.
    for address,values in ((84,[600 if i not in (8,12,16) else 450 for i in range(1,17)]),(80,[200 if i not in (8,12,16) else 150 for i in range(1,17)]),(102,[450]*16)):
        events=[e for e in r['write_events'] if e['address']==address]
        if not events or events[0]['requested']!=values:raise RuntimeError('Physical reference active drive settings differ')
    return dict(path=REFERENCE.relative_to(ROOT).as_posix(),sha256=__import__('hashlib').sha256(REFERENCE.read_bytes()).hexdigest(),
        original_loaded_targets_reproduced=True,sample_period_s=.075,initial_orientation='GREEN up; ORANGE wristward',
        orientation_measurement='previous same-marker camera estimate approximately88deg; not robot-axis certified',
        changed_motor_force_or_health_thresholds=False,stability_and90deg_not_certified=True)
def make_review(p,empty,before):
    loaded,static=base.validate_loaded();base.validate_empty(p,empty)
    ref=completed_reference();after=hashes()
    if before!=after:raise RuntimeError('Source changed during reference-speed review')
    nominal=next(c for c in loaded['cases'] if (c['book_front_mm'],c['gain_scale'],c['friction'])==(-10.,1.,.3))
    rows=p.plan(p.source_position())
    return dict(status='supervised_completed_speed_reproduction_review',accepted_for_operator_trial=True,
        plan_version=p.PLAN_VERSION,source_sha256=p.SOURCE_SHA,start_ticks=p.source_position().tolist(),raw_off_ticks=p.raw_off_ticks.tolist(),
        review_center_reused=p.review_center_reused,reviewed_start_radius_ticks=4,plan_sha256=p.plan_digest(p.source_position()),
        last_goal_ticks=rows[-1][1].tolist(),mapping=list(SIM_TO_MOTOR_IDS),code_hashes=after,code_unchanged=True,
        empty_recovery=empty,initial_self_separation=base.separation_review(p,empty),empty_ring_pip_margin=base.margin_review(p),
        held_thumb_book_numerical_policy=base.thumb_book_review(p,empty),completed_physical_reference=ref,
        loaded_model_cases=len(loaded['cases']),loaded_model_all_passed=True,loaded_model_nominal_angle_deg=nominal['yaw_before_release_deg'],
        loaded_model_is_not_a90degree_certificate=True,loaded_geometry_cases=len(static['cases']),
        off_preload_confirmation_policy=dict(PRELOAD_POLICY),goal_readback_confirmation_policy=dict(GOAL_POLICY),
        empty_root_motion_policy=dict(ROOT_POLICY),read_resynchronization_policy=dict(RECEIVE_POLICY),sync_receive_window_policy=dict(WINDOW_POLICY),
        read_recovery_episode_policy=dict(EPISODE_POLICY),precise_clock_policy=dict(CLOCK_POLICY),verified_serial_link_policy=serial_link_review(),
        readonly_recovery_evidence=readonly_recovery_review(),fast_health_read_evidence=fast_read_review(),
        passive_observation_policy=dict(PASSIVE_POLICY),fixed_hold_hardware_evidence=communication_evidence(),
        nominal_motion_seconds=len(rows)*.075,sample_period_s=.075,distinct_yaw_goal_period_s=.075,goal_step_ticks=2,
        active_motion_limit_s=300,current_ma=450,hardware_tracking_stop_rad=.18,hardware_health_thresholds_unchanged=True,
        motor_gain_change=False,automatic_goal_or_setting_resend=False,no_live_policy=True,physical_rotation_verified=False,assistant_motor_writes=0)
def validate_review(p,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Reference-speed proof/source/limit mismatch')
    return r
def run_screen(request,output,**kwargs):
    p=ReferenceSpeedYawPlan(request);reference=ReferenceSpeedYawPlan(REQUEST);r=validate_review(reference,PROOF)
    if np.array_equal(p.source_position(),reference.source_position()):
        r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=p.review_center_reused)
        Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
    process_screen(request,output,checker=ROOT/'screen_reference_speed_yaw_20261029.py',failure_details=lambda p:'reference-speed empty recovery rejected',**kwargs)
    return validate_review(p,output)
