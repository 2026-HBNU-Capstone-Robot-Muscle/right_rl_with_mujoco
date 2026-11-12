"""Bind the new EMPTY proof, source, queue, code and operator request."""
import json,math,shutil
from pathlib import Path
import numpy as np
from fresh_empty_grasp_plan_20261027 import FreshEmptyGraspPlan,ROOT,REQUEST,PROOF,hashes
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from thumb_root_off_screen_process_20261026 import run_screen as process_screen
from book_grasp_trial_plan import RING_SELF_PAIRS

def _finite(value):return isinstance(value,(int,float)) and math.isfinite(value) and value>=0
def validate_proof(plan,path):
    r=json.loads(Path(path).read_bytes());a=plan.source_position();rows=plan.plan(a)
    if r.get('accepted_for_empty_only') is False:
        reasons=[]
        for case in r.get('static_screens',[]):
            if case.get('passed') is False and case.get('initial_ring_step_increase_mm',0)>1e-6:reasons.append('initial ring release re-contacts palm')
        for case in r.get('dynamic_sensitivity',[]):
            if case.get('passed') is False:
                if case.get('native_range_excess_rad',0)>1e-6:reasons.append('modeled joint moves beyond native range')
                if case.get('initial_support_bound_excess_mm',0)>1e-6:reasons.append('modeled initial support contact increases')
        raise RuntimeError('Current-pose model screen rejected: '+('; '.join(dict.fromkeys(reasons)) or 'geometry/response conditions not passed')+'; no motion started')
    checks={'accepted_for_empty_only':True,'diagnostic_nominal_only':False,'code_unchanged':True,'cube_disabled_entire_screen':True,'physical_calibration_verified':False,'loaded_motion_allowed':False}
    if any(r.get(k) is not v for k,v in checks.items()) or r.get('hardware_commands_sent')!=0:raise RuntimeError('Incomplete EMPTY-only proof; no writes')
    if (r.get('plan_version')!=plan.PLAN_VERSION or r.get('source_sha256')!=plan.SOURCE_SHA or r.get('plan_sha256')!=plan.plan_digest(a) or r.get('start_ticks')!=a.tolist() or r.get('last_goal_ticks')!=rows[-1][1].tolist() or r.get('mapping')!=list(SIM_TO_MOTOR_IDS) or r.get('code_hashes')!=hashes()):raise RuntimeError('Current source/queue/code proof mismatch; no writes')
    if (r.get('goal_step_ticks')!=2 or r.get('sample_period_s')!=.075 or r.get('root_escape_goals')!=70 or r.get('root_escape_seconds')!=3.5 or r.get('initial_support_bound_mm')!=5 or r.get('self_contact_tolerance_mm')!=1e-6 or r.get('support_after_release_tolerance_mm')!=.15 or r.get('actual_tracking_stop_rad')!=.18):raise RuntimeError('EMPTY bounds changed; no writes')
    if (r.get('root_initial_average_step_ticks')!=1.5 or r.get('root_initial_travel_ticks')!=120 or r.get('root_tail_goal_step_ticks')!=1):raise RuntimeError('Reduced-rate root proof required; no writes')
    if r.get('initial_ring_self_bound_mm')!=4 or r.get('initial_ring_pairs')!=list(RING_SELF_PAIRS):raise RuntimeError('Existing initial ring separation scope changed; no writes')
    s=r.get('static_screens',[]);d=r.get('dynamic_sensitivity',[]);c=r.get('camera_gap_hold_sensitivity',[])
    if len(s)!=201 or len(d)!=9 or len(c)!=9:raise RuntimeError('Missing geometry/response cases; no writes')
    for row in s:
        fields=('max_self_after_ring_mm','initial_ring_excess_mm','initial_ring_step_increase_mm','max_support_after_release_mm','unapproved_support_increase_mm','maximum_step_increase_mm')
        if any(not _finite(row.get(k)) for k in fields) or row.get('passed') is not True or row['max_self_after_ring_mm']>1e-6 or row['initial_ring_excess_mm']>1e-6 or row['initial_ring_step_increase_mm']>1e-6 or row['max_support_after_release_mm']>.15 or row['unapproved_support_increase_mm']>1e-6 or row['maximum_step_increase_mm']>1e-6:raise RuntimeError('Static clearance failed; no writes')
        if not set(row.get('initial_ring_self_mm',{}))<=set(RING_SELF_PAIRS) or max(row.get('initial_ring_self_mm',{}).values(),default=0.)>4 or not isinstance(row.get('ring_clear_sample'),int) or not 0<=row['ring_clear_sample']<=40 or not isinstance(row.get('ring_clear_goal_ticks'),int) or not isinstance(row.get('ring_clear_dip_goal_ticks'),int):raise RuntimeError('Initial ring contact did not separate; no writes')
        at=row.get('support_clear_sample')
        # Existing policy: static geometry uses70 advancing goals; the3.5s
        # deadline is checked against independently simulated actual response.
        if not isinstance(at,int) or at<0 or at>70 or max(row.get('initial_support_mm',{}).values(),default=0)>5:raise RuntimeError('Initial support did not clear in bounds; no writes')
    for row in d+c:
        fields=('max_self_after_ring_mm','initial_ring_excess_mm','max_support_after_release_mm','max_tracking_rad','tracking_after_root_release_rad','native_range_excess_rad','initial_support_bound_excess_mm','support_clear_time_excluding_image_pause_s')
        if any(not _finite(row.get(k)) for k in fields) or row.get('passed') is not True or row.get('finite') is not True or row['max_self_after_ring_mm']>1e-6 or row['initial_ring_excess_mm']>1e-6 or row['max_support_after_release_mm']>.15 or row['max_tracking_rad']>.16 or row['tracking_after_root_release_rad']>.12 or row['native_range_excess_rad']>1e-6 or row['initial_support_bound_excess_mm']>1e-6 or row['support_clear_time_excluding_image_pause_s']>3.575:raise RuntimeError('Dynamic/hold response failed; no writes')
        if not set(row.get('initial_ring_envelope_mm',{}))<=set(RING_SELF_PAIRS) or max(row.get('initial_ring_envelope_mm',{}).values(),default=0.)>4:raise RuntimeError('Dynamic initial ring envelope outside existing scope; no writes')
    if any(row.get('camera_hold_seconds')!=0 for row in d) or any(row.get('camera_hold_seconds')!=10 for row in c):raise RuntimeError('Missing complete image-gap hold stress; no writes')
    return r
def run_screen(request,output,**kwargs):
    # Exact same measured start can use its already completed proof. Different
    # fresh values trigger a new complete screen rather than being rewritten.
    candidate=FreshEmptyGraspPlan(request);cached=FreshEmptyGraspPlan(REQUEST)
    if np.array_equal(candidate.source_position(),cached.source_position()):
        r=validate_proof(cached,PROOF)
        if candidate.plan_digest(candidate.source_position())!=r['plan_sha256']:raise RuntimeError('Cached queue differs')
        r=dict(r);r['source_sha256']=candidate.SOURCE_SHA;r['exact_start_cached_proof_used']=True;r['cached_proof_source']=str(PROOF);Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_proof(candidate,output)
    process_screen(request,output,checker=ROOT/'screen_fresh_empty_grasp_20261027.py',failure_details=lambda p:'new EMPTY route geometry/response did not pass',**kwargs)
    return validate_proof(candidate,output)
