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
from camera_gap_native_policy_20261026 import expected_holds
from loaded_thumb_rotation_camera_20261026 import metadata as gap_metadata
from loaded_thumb_voltage_sample_20261026 import metadata as voltage_metadata
from thumb_root_off_screen_process_20261026 import run_screen as run_native_screen
from observed_book_model_policy import policy,SOFT_JOINT_RESIDUAL_RAD


from loaded_quarter_trial_plan_20261025 import extra_hashes,START_HOLD,TURN_PHASE,RETURN_PHASE,motion_goals,empty_return_goals,EXTRA_RING_PAIR,EXTRA_RING_INITIAL_LIMIT_MM,INITIAL_SUPPORT_SCOPE,validate_experiment_basis,THUMB_NEIGHBOR_PAIR,THUMB_NEIGHBOR_LIMIT_MM,INITIAL_SELF_SCOPE
from loaded_quarter_reentry_20261025 import cmc_native_min
from loaded_quarter_model_policy_20261025 import policy
from loaded_thumb_rotation_plan_20261026 import TURN_PHASE,FINISH_PHASE,END_HOLD,loaded_basis,finish_goals,THUMB_LIFT,THUMB_CLEAR,ROOT_UP,tail_goals,final_goal,proposal_metadata,empty_return_goals
from slow_thumb_seam_resolution_20261026 import metadata as slow_seam_metadata,PAIR as SLOW_SEAM_PAIR,NUMERICAL_ZERO_MM as SLOW_ZERO_MM
from loaded_thumb_rest_policy_20261026 import metadata as interrupted_contact_policy,PAIR as CONFIRMED_PALM_PAIR,MAX_INITIAL_MM as CONFIRMED_PALM_MAX_MM
from loaded_thumb_return_model_resolution_20261026 import metadata as return_corner_metadata,accepted_discard
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
    return run_native_screen(request,output,checker=ROOT/'screen_loaded_thumb_rotation_20261026.py',
        failure_details=screen_failure_details,**kwargs)


def validate_proof(plan,path):
    r=json.loads(Path(path).read_bytes());raw=plan.source_position();rows=plan.plan(raw)
    corner=r.get('return_corner_discarded_maximums_by_front')
    if r.get('return_corner_resolution')!=return_corner_metadata() or not isinstance(corner,list) or len(corner)!=3 or any(not isinstance(v,dict) or any(not accepted_discard(p,d) for p,d in v.items()) for v in corner):
        raise RuntimeError('Return corner resolution evidence mismatch; no writes')
    if not getattr(plan,'CAMERA_GAP_HOLD_EARLY_EMPTY',False) or r.get('camera_gap_hold_policy')!=gap_metadata() or r.get('support_escape_dynamic_extra_seconds')!=plan.SAMPLE_PERIOD:
        raise RuntimeError('Camera hold/support cycle policy mismatch; no writes')
    if not getattr(plan,'IMMEDIATE_HEALTHY_VOLTAGE_RECHECK',False) or r.get('fresh_voltage_read_policy')!=voltage_metadata():
        raise RuntimeError('Fresh voltage confirmation policy mismatch; no writes')
    cycle=plan.SAMPLE_PERIOD
    if (r.get('control_cycle_seconds')!=cycle or r.get('simulation_timestep_seconds')!=.005 or r.get('control_sim_step_rounding')!='cumulative_nearest_native_step' or not 0<=r.get('maximum_control_boundary_rounding_s',float('inf'))<=.0025+1e-12 or r.get('initial_hold_seconds')!=.5 or r.get('native_placement_hold_seconds')!=4. or r.get('native_final_hold_seconds')!=20*cycle or r.get('voltage_supply_model_identified') is not False):
        raise RuntimeError('Slow-cycle timing proof mismatch; no writes')
    seam=r.get('slow_thumb_seam_discarded_maximums_by_front')
    if r.get('slow_thumb_seam_resolution')!=slow_seam_metadata() or not isinstance(seam,list) or len(seam)!=3 or any(not isinstance(v,dict) or any(p!=SLOW_SEAM_PAIR or not 0<=d<=SLOW_ZERO_MM for p,d in v.items()) for v in seam):
        raise RuntimeError('Slow seam numerical-resolution evidence mismatch; no writes')
    contact_scope=interrupted_contact_policy(raw)
    THUMB_NEIGHBOR_LIMIT_MM=contact_scope['initial_pair_limit_mm']
    SELF_PAIRS=globals()['SELF_PAIRS']+tuple(contact_scope['additional_initial_pairs'])
    if r.get('recorded_interrupted_contact_policy')!=contact_scope:
        raise RuntimeError('Recorded OFF-contact scope/proof mismatch; no writes')
    if not plan.CAPTURE_PLACEMENT or plan.CAPTURE_BEFORE_PHASE!=TURN_PHASE:
        raise RuntimeError('Loaded proposal needs explicit PLACE/REMOVE; no writes')
    expected=tail_goals();actual=[(p,q) for p,q in rows if p in (THUMB_LIFT,THUMB_CLEAR,ROOT_UP)]
    if r.get('thumb_root_proposal')!=proposal_metadata() or [p for p,q in actual]!=[p for p,q in expected] or not np.array_equal([q for p,q in actual],[q for p,q in expected]):
        raise RuntimeError('New thumb/root goal sequence mismatch; no writes')
    finish=np.array([q for p,q in rows if p==FINISH_PHASE])
    holds=np.array([q for p,q in rows if p==END_HOLD])
    if not np.array_equal(finish,finish_goals()[1:]) or holds.shape!=(60,16) or not np.all(holds==final_goal()) or r.get('empty_root_finish_goal_count')!=64:
        raise RuntimeError('EMPTY root finish/hold mismatch; no writes')
    anchor=int(np.argmin(np.max(abs(motion_goals()-raw),axis=1)))
    actual_return=np.array([q for p,q in rows if p==RETURN_PHASE])
    expected_return=empty_return_goals(anchor)[1:]
    if actual_return.size==0:actual_return=np.empty((0,16),dtype=int)
    if not np.array_equal(actual_return,expected_return):raise RuntimeError('EMPTY return differs from the derived tip-clearance reference; no writes')
    motion=np.array([q for p,q in rows if p==TURN_PHASE])
    if not np.array_equal(motion,motion_goals()[1:]) or sum(p==START_HOLD for p,q in rows)!=40:
        raise RuntimeError('Whole EMPTY motion/hold mismatch; no writes')
    required={'accepted_for_supervised_empty_preparation':True,'accepted_for_supervised_grasp_approach':True,
        'empty_hand_only':False,'placement_capture_only':False,'joint_goals_fixed_after_placement':False,'full_quarter_reference_path':True,'full_empty_return_and_new_thumb_root_route_checked':True,'interrupted_suffix_verified':False,'experimental_loaded_trial':True,
        'casing_release_motor_id':4,'no_contact_outside_reviewed_initial_envelope':True,'full_CAD_tip_contacts':True,'cube_disabled_entire_screen':True,'code_unchanged':True,
        'extra_code_unchanged':True,'cube_contact_verified':False,'placement_checkpoint_required':True,
        'rotation_included':bool(getattr(plan,'ROTATION_INCLUDED',False)),
        'native_ranges_unchanged':not policy(raw)['model_range_extended'],'free_space_certificate':False,
        'physical_calibration_verified':False,'initial_thumb_contact_operator_observed':False}
    if (any(r.get(k) is not v for k,v in required.items())
        or r.get('extra_code_hashes')!=getattr(plan,'proof_hashes',extra_hashes)() or r.get('code_hashes')!=code_hashes()
        or r.get('observed_model_policy')!=policy(raw)
        or r.get('passive_support_reentry_policy')!=support_reentry_policy()
        or r.get('passive_ID4_monotonic_reentry') is not True or r.get('combined_motion_time_limit_seconds')!=plan.ON_TIMEOUT
        or r.get('native_placement_hold_samples')!=80 or r.get('loaded_motion_basis')!=loaded_basis() or r.get('hardware_commands_sent')!=0
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
    for model in manifests:
        identity=model.get('initial_palm_body_identity',{})
        names=identity.get('geom_names',[])
        if (identity.get('body')!='palm' or identity.get('pair')!=CONFIRMED_PALM_PAIR
            or identity.get('verified_same_rigid_body') is not True
            or identity.get('geometry_or_contact_forces_modified') is not False
            or not {'palm_collision_3','palm_collision_4','palm_collision_6','palm_collision_7'}.issubset(names)):
            raise RuntimeError('Unverified compound palm body; no writes')
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
        if kind=='self' and pairs.get(CONFIRMED_PALM_PAIR,0.)>CONFIRMED_PALM_MAX_MM:return False
        if kind=='self' and (pairs.get(CASING_PAIR,0.)>CASING_INITIAL_LIMIT_MM or pairs.get(EXTRA_RING_PAIR,0.)>EXTRA_RING_INITIAL_LIMIT_MM or pairs.get(THUMB_NEIGHBOR_PAIR,0.)>THUMB_NEIGHBOR_LIMIT_MM):return False
        depths=np.asarray(list(pairs.values()),dtype=float)
        if not np.isfinite(depths).all() or np.any(depths<0) or np.any(depths>limit):return False
        if kind=='self':deadline=(.5+(ring_count+tip_count)*cycle if dynamic else ring_count+min(tip_count,THUMB_ESCAPE_GOALS))
        else:deadline=(.5+(ring_count+tip_count)*cycle+min(lift_count*cycle,ROOT_ESCAPE_SECONDS)+cycle if dynamic
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
    stress=r.get('camera_gap_hold_sensitivity',[])
    if len(stress)!=9:raise RuntimeError('Incomplete camera-hold response proof; no writes')
    for front in FRONTS:
        dr=[s for s in stress if s.get('book_front_mm')==front]
        if (sorted(s.get('gain_scale') for s in dr)!=[.5,1.,1.5] or not all(valid(s,True) for s in dr)
            or not all(passive_hold_valid(s,front) and initial_self_valid(s,front) for s in dr)
            or any(s.get('camera_hold_events')!=expected_holds(rows) for s in dr)):
            raise RuntimeError('Camera-hold response/health geometry proof rejected; no writes')
    if any(s.get('camera_hold_events')!=[] for s in dynamic):raise RuntimeError('Nominal response must not contain camera holds')
    return r
