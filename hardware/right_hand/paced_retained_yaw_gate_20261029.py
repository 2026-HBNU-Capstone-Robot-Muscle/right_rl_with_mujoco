"""Bind the unchanged motion evidence and the new passive-observation schedule."""
import json
from pathlib import Path
import numpy as np
from paced_retained_yaw_plan_20261029 import PacedRetainedYawPlan,ROOT,REQUEST,PROOF,hashes
import retained_slow_yaw_gate_20261029 as base
from review_empty_hold_comm_20261029 import review as communication_evidence
PASSIVE_POLICY=dict(version='passive_observation75ms_v1_20261029',period_s=.075,
    scope='placement, countdown, photos, removal: no new goals',processing_included=True,
    catch_up_bursts=False,cancel_before_and_after_pacing=True,health_checker_unchanged=True,
    recovery_budget_ms=200,health_thresholds_unchanged=True)
def make_review(plan,empty,before):
    r=base.make_review(plan,empty,base.hashes());r['code_hashes']=hashes()
    if before!=r['code_hashes']:raise RuntimeError('Source changed during paced-yaw review')
    r.update(status='supervised_paced_retained_red_yaw_review',passive_observation_policy=PASSIVE_POLICY,
        fixed_hold_hardware_evidence=communication_evidence(),
        OFF_setting_readback='exact EmptyBus implementation used in completed45s experiment',
        motion_targets_and_step_timing_unchanged=True,automatic_goal_or_setting_resend=False)
    return r
def validate_review(plan,path):
    r=base.read(path);expected=make_review(plan,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Paced-yaw proof/source/limits mismatch')
    return r
def run_screen(request,output,**kwargs):
    p=PacedRetainedYawPlan(request);reference=PacedRetainedYawPlan(REQUEST);r=validate_review(reference,PROOF)
    if np.array_equal(p.source_position(),reference.source_position()):
        r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=p.review_center_reused)
        Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
    base.process_screen(request,output,checker=ROOT/'screen_paced_retained_yaw_review_20261029.py',
        failure_details=lambda p:'fresh empty recovery or unchanged loaded-yaw proof rejected',**kwargs)
    return validate_review(p,output)
