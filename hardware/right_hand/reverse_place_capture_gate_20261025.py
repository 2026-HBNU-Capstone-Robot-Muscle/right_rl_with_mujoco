"""Independently verify bounded initial-contact release and the complete route."""
import json
from pathlib import Path
import numpy as np
from book_empty_preparation_plan import FRONTS,LIFT,ALLOWED_SUPPORT_PAIRS,offsets,code_hashes
from book_grasp_trial_plan import (ROOT,extra_hashes,THUMB_RELEASE,RING_RELEASE,SELF_PAIRS,
    MAX_INITIAL_THUMB_SELF_MM,THUMB_ESCAPE_GOALS,SUPPORT_ROOT_ESCAPE_GOALS,INITIAL_CONTACT_TRACKING_RAD,
    MAX_INITIAL_SUPPORT_MM,ROOT_ESCAPE_SECONDS)
from book_passive_support_reentry_policy import policy as support_reentry_policy
from hand_motor_mapping import SIM_TO_MOTOR_IDS
from thumb_live_start_gate import run_screen as run_native_screen
from observed_book_model_policy import policy,SOFT_JOINT_RESIDUAL_RAD


from reverse_place_capture_plan_20261025 import extra_hashes
REVISION='CAD_tip_contact_diagnostic_v1_20261025'


from reverse_place_capture_plan_20261025 import BASE_RELEASE,CASING_PAIR,CASING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE
THUMB_RELEASE=BASE_RELEASE
SELF_PAIRS=SELF_PAIRS+(CASING_PAIR,)


def screen_failure_details(path):
    try:
        r=json.loads(Path(path).read_bytes());rows=r['static_screens']+r['dynamic_sensitivity'];notes=[]
        if any(s['self_escape_max_unapproved_overlap_mm']>1e-6 or not s['self_escape_complete'] for s in rows):
            notes.append('initial thumb contact did not separate without a new/deeper self-contact')
        if any(s['support_escape_max_unapproved_overlap_mm']>1e-6 or not s['support_escape_complete'] for s in rows):
            notes.append('initial support contact did not separate without a new/deeper contact')
        if (any(s['native_range_violation_rad']>1e-6 for s in r['static_screens'])
            or any(s['native_range_violation_rad']>SOFT_JOINT_RESIDUAL_RAD
                or s.get('unmodified_joint_range_violation_rad',float('inf'))>1e-6 for s in r['dynamic_sensitivity'])):
            notes.append('observed recovery/model range check failed')
        if any(s['max_tracking_rad']>INITIAL_CONTACT_TRACKING_RAD or s['max_tracking_after_clearance_rad']>.12 for s in r['dynamic_sensitivity']):
            notes.append('model tracking check failed')
        return '; '.join(notes) or 'the full book grasp route did not pass'
    except (OSError,KeyError,ValueError,TypeError):return 'no readable complete book grasp screen'


def run_screen(request,output,**kwargs):
    return run_native_screen(request,output,checker=ROOT/'screen_reverse_place_capture_20261025.py',
        failure_details=screen_failure_details,**kwargs)


def validate_proof(plan,path):
    r=json.loads(Path(path).read_bytes());raw=plan.source_position();rows=plan.plan(raw)
    checkpoint=next(i for i,(phase,_) in enumerate(rows) if phase==plan.CAPTURE_BEFORE_PHASE)
    if checkpoint==0 or any(not np.array_equal(q,rows[checkpoint-1][1]) for _,q in rows[checkpoint:]):
        raise RuntimeError('Placement capture contains post-placement motion; no writes')
    required={'accepted_for_supervised_empty_preparation':True,'accepted_for_supervised_grasp_approach':False,
        'empty_hand_only':False,'placement_capture_only':True,'joint_goals_fixed_after_placement':True,
        'casing_release_motor_id':4,'no_contact_outside_reviewed_initial_envelope':True,'full_CAD_tip_contacts':True,'cube_disabled_entire_screen':True,'code_unchanged':True,
        'extra_code_unchanged':True,'cube_contact_verified':False,'placement_checkpoint_required':True,
        'rotation_included':bool(getattr(plan,'ROTATION_INCLUDED',False)),
        'native_ranges_unchanged':not policy(raw)['model_range_extended'],'free_space_certificate':False,
        'physical_calibration_verified':False,'initial_thumb_contact_operator_observed':False}
    if (any(r.get(k) is not v for k,v in required.items())
        or r.get('extra_code_hashes')!=getattr(plan,'proof_hashes',extra_hashes)() or r.get('code_hashes')!=code_hashes()
        or r.get('observed_model_policy')!=policy(raw)
        or r.get('passive_support_reentry_policy')!=support_reentry_policy()
        or r.get('native_placement_hold_samples')!=40 or r.get('hardware_commands_sent')!=0
        or r.get('initial_self_depth_limit_mm')!=MAX_INITIAL_THUMB_SELF_MM
        or r.get('initial_support_depth_limit_mm')!=MAX_INITIAL_SUPPORT_MM
        or r.get('initial_contact_model_tracking_limit_rad')!=INITIAL_CONTACT_TRACKING_RAD
        or r.get('actual_tracking_stop_rad')!=.18
        or r.get('initial_casing_contact_limit_mm')!=CASING_INITIAL_LIMIT_MM or r.get('initial_support_scope')!=INITIAL_SUPPORT_SCOPE
        or r.get('plan_version')!=plan.PLAN_VERSION or r.get('source_sha256')!=plan.SOURCE_SHA
        or r.get('plan_sha256')!=plan.plan_digest(raw) or r.get('start_ticks')!=raw.tolist()
        or r.get('last_goal_ticks')!=rows[-1][1].tolist()
        or r.get('mapping')!=list(SIM_TO_MOTOR_IDS)):
        raise RuntimeError('Book grasp release/path/code/checkpoint proof mismatch; no writes')
    manifests=r.get('model_manifests',[])
    if len(manifests)!=3 or any(
        s.get('fingertip_collision_model')!=REVISION or not s.get('CAD_collision_and_visual_compiled_transforms_equal')
        or len(s.get('middle_ring_proxies',[]))!=10 for s in manifests):
        raise RuntimeError('Incomplete full CAD geometry evidence; no writes')
    static=r.get('static_screens',[]);dynamic=r.get('dynamic_sensitivity',[])
    if len(static)!=201 or len(dynamic)!=9:raise RuntimeError('Incomplete book grasp sensitivity cases; no writes')
    tip_count=sum(p==THUMB_RELEASE for p,_ in rows);lift_count=sum(p==LIFT for p,_ in rows)
    ring_count=sum(p==RING_RELEASE for p,_ in rows)
    counts_cache={}
    def phase_counts(s,dynamic):
        if dynamic:return ring_count,tip_count,lift_count
        key=tuple(s['offset_ticks'])
        if key not in counts_cache:
            case_rows=plan.plan(raw+np.asarray(key))
            counts_cache[key]=tuple(sum(p==phase for p,q in case_rows) for phase in (RING_RELEASE,THUMB_RELEASE,LIFT))
        return counts_cache[key]
    def contact_valid(s,kind,dynamic):
        ring_count,tip_count,lift_count=phase_counts(s,dynamic)
        prefix=kind+'_escape';pairs=s.get('initial_'+kind+'_contacts_mm')
        allowed=SELF_PAIRS if kind=='self' else ALLOWED_SUPPORT_PAIRS
        limit=MAX_INITIAL_THUMB_SELF_MM if kind=='self' else MAX_INITIAL_SUPPORT_MM
        if not isinstance(pairs,dict) or not set(pairs).issubset(allowed):return False
        if kind=='self' and pairs.get(CASING_PAIR,0.)>CASING_INITIAL_LIMIT_MM:return False
        depths=np.asarray(list(pairs.values()),dtype=float)
        if not np.isfinite(depths).all() or np.any(depths<0) or np.any(depths>limit):return False
        if kind=='self':deadline=(.5+(ring_count+tip_count)*.05 if dynamic else ring_count+min(tip_count,THUMB_ESCAPE_GOALS))
        else:deadline=(.5+(ring_count+tip_count)*.05+min(lift_count*.05,ROOT_ESCAPE_SECONDS) if dynamic
            else ring_count+tip_count+min(lift_count,SUPPORT_ROOT_ESCAPE_GOALS))
        keys=(prefix+'_max_unapproved_overlap_mm',prefix+'_max_step_increase_mm',
            prefix+'_complete_at',prefix+'_deadline')
        if any(s.get(k) is None for k in keys):return False
        a=np.asarray([s[k] for k in keys],dtype=float)
        if not np.isfinite(a).all() or np.any(a<0):return False
        return (a[0]<=1e-6 and (dynamic or a[1]<=1e-6) and a[2]<=deadline and a[3]==deadline
            and s.get(prefix+'_complete') is True)
    def valid(s,dynamic):
        if not contact_valid(s,'self',dynamic) or not contact_valid(s,'support',dynamic):return False
        keys=('max_self_after_release_mm','max_support_after_lift_mm','native_range_violation_rad')
        try:a=np.array([s[k] for k in keys],dtype=float)
        except (KeyError,TypeError,ValueError):return False
        unchanged=s.get('unmodified_joint_range_violation_rad',float('nan'))
        ring_after=s.get('max_ring_self_after_release_mm',float('nan'))
        limit=SOFT_JOINT_RESIDUAL_RAD if dynamic else 1e-6
        if (not np.isfinite(a).all() or np.any(a<0) or a[0]>1e-6 or a[1]>.15 or a[2]>limit
            or not np.isfinite(unchanged) or unchanged<0 or unchanged>1e-6
            or not np.isfinite(ring_after) or ring_after<0 or ring_after>1e-6):return False
        if dynamic:
            a=np.array([s.get('max_tracking_rad',np.nan),s.get('max_tracking_after_clearance_rad',np.nan)])
            return s.get('finite') is True and np.isfinite(a).all() and np.all(a>=0) and a[0]<=INITIAL_CONTACT_TRACKING_RAD and a[1]<=.12
        ring_count,_,_=phase_counts(s,False)
        step=s.get('max_goal_step_ticks',float('nan'))
        threshold=s.get('thumb_clear_goal_ticks')
        expected_threshold=raw[3]+s['offset_ticks'][3]-2*max(0,s['self_escape_complete_at']-ring_count)
        ring_at=s.get('ring_clear_complete_at');ring_goal=s.get('ring_clear_goal_ticks')
        if (not isinstance(ring_at,int) or ring_at<0 or ring_at>ring_count
            or not isinstance(ring_goal,int) or ring_goal!=raw[14]+s['offset_ticks'][14]-2*ring_at):return False
        return (np.isfinite(step) and step==round(step) and 0<=step<=2
            and isinstance(threshold,int) and threshold==expected_threshold)
    def envelope_bound(front):
        envelope={}
        for s in static:
            if s.get('book_front_mm')==front:
                for pair,depth in s['initial_support_contacts_mm'].items():envelope[pair]=max(envelope.get(pair,0.),depth)
        return envelope
    def passive_hold_valid(s,front):
        bound=envelope_bound(front);e=s.get('passive_enable_hold_support',{})
        if e.get('duration_seconds')!=.5 or e.get('goal_ticks')!=raw.tolist() or e.get('new_pair_depth_limit_mm')!=.15:return False
        pairs=e.get('contact_maximums_mm')
        if not isinstance(pairs,dict) or not set(pairs).issubset(ALLOWED_SUPPORT_PAIRS):return False
        for pair,depth in pairs.items():
            if not np.isfinite(depth) or depth<0 or depth>(bound[pair] if pair in bound else .15)+1e-6:return False
            if pair not in bound:bound[pair]=depth
        return s.get('initial_support_contacts_mm')==bound
    expected=sorted(tuple(v.tolist()) for v in offsets())
    for front in FRONTS:
        sr=[s for s in static if s.get('book_front_mm')==front];dr=[s for s in dynamic if s.get('book_front_mm')==front]
        actual=[]
        for s in sr:
            a=np.asarray(s.get('offset_ticks'))
            if a.shape!=(16,) or not np.isfinite(a).all() or np.any(a!=np.rint(a)):
                raise RuntimeError('Invalid start neighborhood; no writes')
            actual.append(tuple(a.tolist()))
        if (sorted(actual)!=expected or sorted(s.get('gain_scale') for s in dr)!=[.5,1.,1.5]
            or not all(valid(s,False) for s in sr) or not all(valid(s,True) for s in dr)
            or not all(passive_hold_valid(s,front) for s in dr)
            or any(s.get('nominal_initial_support_contacts_mm')!=next(c['initial_support_contacts_mm'] for c in sr if c['offset_ticks']==[0]*16) for s in dr)):
            raise RuntimeError('Contact separation/native range/dynamics check rejected; no writes')
    return r

