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


from loaded_quarter_trial_plan_20261025 import extra_hashes,START_HOLD,TURN_PHASE,RETURN_PHASE,motion_goals,empty_return_goals,EXTRA_RING_PAIR,EXTRA_RING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE,validate_experiment_basis,THUMB_NEIGHBOR_PAIR,THUMB_NEIGHBOR_LIMIT_MM,INITIAL_SELF_SCOPE
from loaded_quarter_reentry_20261025 import cmc_native_min
from loaded_quarter_model_policy_20261025 import policy
from quarter_finish_empty_plan_20261025 import TURN_PHASE,FINISH_PHASE,END_HOLD,empty_basis,finish_goals
REVISION='CAD_tip_contact_diagnostic_v1_20261025'


from reverse_place_capture_plan_20261025 import BASE_RELEASE,CASING_PAIR,CASING_INITIAL_LIMIT_MM
THUMB_RELEASE=BASE_RELEASE
SELF_PAIRS=SELF_PAIRS+(CASING_PAIR,EXTRA_RING_PAIR,THUMB_NEIGHBOR_PAIR)


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
    except (OSError,KeyError,ValueError,TypeError):
        try:
            lines=Path(path).with_suffix('.log').read_text(encoding='utf-8',errors='replace').splitlines()
            error=next(line for line in reversed(lines) if line.startswith(('ValueError:','RuntimeError:')))
            return 'initial-state/path check: '+error[:240]
        except (OSError,StopIteration):return 'no complete recovery screen; no writes'


def run_screen(request,output,**kwargs):
    return run_native_screen(request,output,checker=ROOT/'screen_quarter_finish_empty_20261025.py',
        failure_details=screen_failure_details,**kwargs)


def validate_proof(plan,path):
    r=json.loads(Path(path).read_bytes());raw=plan.source_position();rows=plan.plan(raw)
    if plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE is not None:
        raise RuntimeError('EMPTY finish must have no PLACE checkpoint; no writes')
    finish=np.array([q for p,q in rows if p==FINISH_PHASE])
    holds=np.array([q for p,q in rows if p==END_HOLD])
    if not np.array_equal(finish,finish_goals()[1:]) or holds.shape!=(60,16) or not np.all(holds==finish_goals()[-1]) or r.get('empty_root_finish_goal_count')!=64:
        raise RuntimeError('EMPTY root finish/hold mismatch; no writes')
    anchor=int(np.argmin(np.max(abs(motion_goals()-raw),axis=1)))
    actual_return=np.array([q for p,q in rows if p==RETURN_PHASE])
    expected_return=empty_return_goals(anchor)[1:]
    if actual_return.size==0:actual_return=np.empty((0,16),dtype=int)
    if not np.array_equal(actual_return,expected_return):raise RuntimeError('EMPTY return differs from the derived tip-clearance reference; no writes')
    motion=np.array([q for p,q in rows if p==TURN_PHASE])
    if not np.array_equal(motion,motion_goals()[1:]) or sum(p==START_HOLD for p,q in rows)!=40:
        raise RuntimeError('Whole EMPTY motion/hold mismatch; no writes')
    required={'accepted_for_supervised_empty_preparation':True,'accepted_for_supervised_grasp_approach':False,
        'empty_hand_only':True,'placement_capture_only':False,'joint_goals_fixed_after_placement':False,'full_quarter_reference_path':True,'full_empty_return_and_finish_hand_route_checked':True,'interrupted_suffix_verified':False,'experimental_loaded_trial':False,
        'casing_release_motor_id':4,'no_contact_outside_reviewed_initial_envelope':True,'full_CAD_tip_contacts':True,'cube_disabled_entire_screen':True,'code_unchanged':True,
        'extra_code_unchanged':True,'cube_contact_verified':False,'placement_checkpoint_required':False,
        'rotation_included':bool(getattr(plan,'ROTATION_INCLUDED',False)),
        'native_ranges_unchanged':not policy(raw)['model_range_extended'],'free_space_certificate':False,
        'physical_calibration_verified':False,'initial_thumb_contact_operator_observed':False}
    if (any(r.get(k) is not v for k,v in required.items())
        or r.get('extra_code_hashes')!=getattr(plan,'proof_hashes',extra_hashes)() or r.get('code_hashes')!=code_hashes()
        or r.get('observed_model_policy')!=policy(raw)
        or r.get('passive_support_reentry_policy')!=support_reentry_policy()
        or r.get('passive_ID4_monotonic_reentry') is not True or r.get('combined_motion_time_limit_seconds')!=plan.ON_TIMEOUT
        or r.get('native_placement_hold_samples')!=0 or r.get('empty_motion_basis')!=empty_basis() or r.get('hardware_commands_sent')!=0
        or r.get('initial_self_depth_limit_mm')!=MAX_INITIAL_THUMB_SELF_MM
        or r.get('initial_support_depth_limit_mm')!=MAX_INITIAL_SUPPORT_MM
        or r.get('initial_contact_model_tracking_limit_rad')!=INITIAL_CONTACT_TRACKING_RAD
        or r.get('actual_tracking_stop_rad')!=.18
        or r.get('initial_casing_contact_limit_mm')!=CASING_INITIAL_LIMIT_MM or r.get('initial_support_scope')!=INITIAL_SUPPORT_SCOPE or r.get('initial_ring_neighbor_limit_mm')!=EXTRA_RING_INITIAL_LIMIT_MM or r.get('initial_thumb_neighbor_limit_mm')!=THUMB_NEIGHBOR_LIMIT_MM or r.get('initial_self_scope')!=INITIAL_SELF_SCOPE
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
    counts_cache={};goal_rows_cache={}
    def clear_goal(s,at,j):
        key=tuple(s['offset_ticks'])
        if key not in goal_rows_cache:goal_rows_cache[key]=plan.plan(raw+np.asarray(key))
        if not isinstance(at,int) or at<0 or at>len(goal_rows_cache[key]):return None
        return int((raw+np.asarray(key))[j] if at==0 else goal_rows_cache[key][at-1][1][j])
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
        if kind=='self' and (pairs.get(CASING_PAIR,0.)>CASING_INITIAL_LIMIT_MM or pairs.get(EXTRA_RING_PAIR,0.)>EXTRA_RING_INITIAL_LIMIT_MM or pairs.get(THUMB_NEIGHBOR_PAIR,0.)>THUMB_NEIGHBOR_LIMIT_MM):return False
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
        cmc=s.get('native_cmc_after_release_min_ticks',float('nan'))
        if not np.isfinite(cmc) or cmc<cmc_native_min()-(1. if dynamic else 1e-6):return False
        if dynamic:
            a=np.array([s.get('max_tracking_rad',np.nan),s.get('max_tracking_after_clearance_rad',np.nan)])
            return s.get('finite') is True and np.isfinite(a).all() and np.all(a>=0) and a[0]<=INITIAL_CONTACT_TRACKING_RAD and a[1]<=.12
        ring_count,_,_=phase_counts(s,False)
        step=s.get('max_goal_step_ticks',float('nan'))
        expected_tip=clear_goal(s,s['self_escape_complete_at'],0)
        if s.get('thumb_tip_clear_goal_ticks')!=expected_tip:return False
        threshold=s.get('thumb_clear_goal_ticks')
        expected_threshold=clear_goal(s,s['self_escape_complete_at'],3)
        ring_at=s.get('ring_clear_complete_at');ring_goal=s.get('ring_clear_goal_ticks')
        if (not isinstance(ring_at,int) or ring_at<0 or ring_at>ring_count
            or not isinstance(ring_goal,int) or ring_goal!=clear_goal(s,ring_at,14)):return False
        return (np.isfinite(step) and step==round(step) and 0<=step<=2
            and isinstance(threshold,int) and threshold==expected_threshold)
    def envelope_bound(front):
        envelope={}
        for s in static:
            if s.get('book_front_mm')==front:
                for pair,depth in s['initial_support_contacts_mm'].items():envelope[pair]=max(envelope.get(pair,0.),depth)
        return envelope
    def passive_hold_valid(s,front):
        bound=envelope_bound(front);e=s.get('passive_enable_hold_support',{});nominal=s.get('nominal_initial_support_contacts_mm',{})
        if e.get('duration_seconds')!=.5 or e.get('goal_ticks')!=raw.tolist() or e.get('new_pair_depth_limit_mm')!=.15:return False
        pairs=e.get('contact_maximums_mm')
        if not isinstance(pairs,dict) or not set(pairs).issubset(ALLOWED_SUPPORT_PAIRS):return False
        for pair,depth in pairs.items():
            if not np.isfinite(depth) or depth<0 or depth>(bound[pair] if pair in nominal else .15)+1e-6:return False
            if pair not in nominal:bound[pair]=max(bound.get(pair,0.),depth)
        return s.get('initial_support_contacts_mm')==bound
    def initial_self_valid(s,front):
        bound={}
        for c in static:
            if c['book_front_mm']==front:
                for pair,depth in c['initial_self_contacts_mm'].items():bound[pair]=max(bound.get(pair,0.),depth)
        nominal=next(c['initial_self_contacts_mm'] for c in static if c['book_front_mm']==front and c['offset_ticks']==[0]*16)
        return s.get('initial_self_contacts_mm')==bound and s.get('nominal_initial_self_contacts_mm')==nominal
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
            or not all(passive_hold_valid(s,front) for s in dr) or not all(initial_self_valid(s,front) for s in dr)
            or any(s.get('nominal_initial_support_contacts_mm')!=next(c['initial_support_contacts_mm'] for c in sr if c['offset_ticks']==[0]*16) for s in dr)):
            raise RuntimeError('Contact separation/native range/dynamics check rejected; no writes')
    from support_photo_contact_resolution_20261025 import metadata,PAIRS,NUMERICAL_ZERO_MM
    if r.get('photo_initial_thumb_contact_resolution')!=metadata() or r.get('initial_resolution_forces_preserved') is not True:
        raise RuntimeError('Initial numerical resolution metadata mismatch; no writes')
    discarded=r.get('initial_resolution_discarded_maximums_by_front')
    if not isinstance(discarded,list) or len(discarded)!=3 or any(not isinstance(v,dict) or not set(v).issubset(PAIRS) or any(not isinstance(x,(int,float)) or not 0<=x<=NUMERICAL_ZERO_MM for x in v.values()) for v in discarded):
        raise RuntimeError('Initial numerical resolution evidence mismatch; no writes')
    return r

