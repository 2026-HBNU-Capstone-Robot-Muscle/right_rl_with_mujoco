"""Admit only the exact two-leg composition with unchanged transport/limits."""
import json,math
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as transport
from calibrated_quarter_gate_20261028 import bounded
from gapped_inverse_gate_20261029 import physical_empty_review,additional_path_review
from book_empty_preparation_plan import FRONTS
from yaw_roundtrip_plan_20261030 import YawRoundtripPlan,ROOT,OUT,REQUEST,PROOF,LOADED,HAND_PATH,MODEL_SOURCES,hashes,sha
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
def composition_review():
    from calibrated_quarter_plan_20261028 import LOADED_PATH
    from gapped_inverse_plan_20261029 import LOADED as INVERSE
    f=np.load(LOADED_PATH);i=np.load(INVERSE);z=np.load(LOADED)
    end=int(np.flatnonzero(f['phases']=='release')[0]);prefix=f['goals'][:end];reverse=f['goals'][f['phases']=='yaw'][::-1]
    release=i['goals'][i['phases']=='stage_9_loaded_open'];goal=np.vstack([prefix,reverse,np.repeat(reverse[-1][None,:],40,0),release,np.repeat(f['start'][None,:],20,0)])
    pm={'grasp':'stage_5_loaded_grasp','grasp_hold':'stage_6_grasp_hold','yaw':'stage_7_horizontal_yaw','yaw_hold':'stage_8_horizontal_hold'}
    phases=np.r_[[pm[str(p)] for p in f['phases'][:end]],['stage_7_horizontal_yaw']*638,['stage_8_horizontal_hold']*40,['stage_9_loaded_open']*len(release),['stage_10_remove_hold']*20]
    if len(reverse)!=638 or not np.array_equal(prefix[-1],reverse[0]) or not np.array_equal(z['start'],f['start']) or not np.array_equal(z['goals'],goal) or not np.array_equal(z['phases'],phases):
        raise RuntimeError('Continuous composition changed or Cube release/replacement inserted')
    if int(z['forward_end_sample'])!=end or not np.array_equal(z['forward_hold_goal'],prefix[-1]) or not np.array_equal(z['inverse_hold_goal'],reverse[-1]):raise RuntimeError('Leg capture boundaries changed')
    if np.max(abs(np.diff(np.vstack([z['start'],goal]),axis=0)))>2:raise RuntimeError('Two-tick bound exceeded')
    hand=np.load(HAND_PATH)
    if not np.array_equal(hand['start'],z['start']) or not np.array_equal(hand['goals'],goal):raise RuntimeError('Hand dynamics path differs from composed Cube path')
    covered={tuple(q) for q in np.vstack([f['start'],f['goals']])};extra={tuple(q) for q in goal if tuple(q) not in covered}
    additional=additional_path_review();from gapped_inverse_plan_20261029 import GEOMETRY_PATH
    if not extra<={tuple(q) for q in np.load(GEOMETRY_PATH)['goals']}:raise RuntimeError('Composed goal lacks historical201-condition geometry coverage')
    return dict(forward_yaw_goals=638,inverse_yaw_goals=638,loaded_goals=len(goal),first_leg_through_hold=end,source_forward_sha256=sha(LOADED_PATH),
        source_inverse_sha256=sha(INVERSE),candidate_sha256=sha(LOADED),new_joint_goal_vectors=0,additional_source_pose_vectors=len(extra),
        additional_geometry=additional,physical_reverse_empty=physical_empty_review(),full_composed_cycle_not_yet_physically_tested=True)
def hand_review():
    path=OUT/'loaded_hand_dynamics/review.json';r=json.loads(path.read_bytes())
    if r.get('source_sha256')!=sha(HAND_PATH) or not r.get('all_passed') or not r.get('cube_disabled') or not r.get('finite_actuators') or r.get('hand_qpos_prescribed') is not False:raise RuntimeError('Composed finite hand review missing or unbound')
    if len(r['cases'])!=9 or {(c['book_front_mm'],c['gain_scale']) for c in r['cases']}!={(f,g) for f in FRONTS for g in (.5,1.,1.5)}:raise RuntimeError('Missing composed hand conditions')
    for c in r['cases']:
        if not c['passed'] or c['stop'] is not None or not bounded(c,{'max_self_mm':1e-6,'raw_max_self_mm':1e-6,'max_support_mm':.15,'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6}):raise RuntimeError('Composed hand exceeds original bounds')
    return dict(path=path.relative_to(ROOT).as_posix(),sha256=sha(path),cases=9,all_passed=True,hand_path_sha256=sha(HAND_PATH))
def cube_models():
    cases=[];sources=[]
    for path in MODEL_SOURCES:
        r=json.loads(path.read_bytes())
        if not r.get('all_passed') or r.get('candidate_sha256')!=sha(LOADED) or r.get('simulation_timestep_s')!=.001 or not r.get('initial_placement_also_checked'):raise RuntimeError('Composed Cube review rejected or unbound')
        if r.get('model_source_sha256')!=sha(ROOT/'screen_yaw_roundtrip_model_20261030.py'):raise RuntimeError('Composed Cube model implementation changed')
        cases.extend(r['cases']);sources.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=sha(path)))
    if len(cases)!=27 or {(c['book_front_mm'],c['gain_scale'],c['friction']) for c in cases}!={(f,g,mu) for f in FRONTS for g in (.5,1.,1.5) for mu in (.2,.3,.5)}:raise RuntimeError('Composed Cube27 incomplete')
    for c in cases:
        if not c['passed'] or c['stop'] is not None or not c['initial_placement_passed'] or c['external_cube_contact'] or not c['final_cube_supported_by_hand']:raise RuntimeError('Composed Cube retention rejected')
        if not c.get('vertical_seating_line_upper_verified_free'):raise RuntimeError('Free upper Cube seating endpoint not verified')
        if not bounded(c,{'max_self_mm':1e-6,'max_support_mm':.15,'max_cube_mm':1,'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6,'center_translation_mm':80,'longest_no_hand_contact_seconds':.25}):raise RuntimeError('Composed Cube exceeds original physics bounds')
        forward,inverse=c['forward_from_placement_deg'],c['inverse_from_forward_hold_deg']
        if not all(isinstance(a,(int,float)) and math.isfinite(a) for a in (forward,inverse)) or forward<15 or inverse>-15 or not 0<=c['yaw_tilt_change_deg']<=15:raise RuntimeError('Both modeled directions not supported')
        if not bounded(c['initial_placement_probe'],{'max_self_mm':1e-6,'max_support_mm':.15,'max_cube_mm':1,'max_tracking_rad':.18}) or c['initial_placement_probe']['external_cube_contact']:raise RuntimeError('Initial Cube placement violates original bounds')
    nominal=next(c for c in cases if (c['book_front_mm'],c['gain_scale'],c['friction'])==(-10,1.,.3))
    return dict(sources=sources,cases=27,nominal_forward_deg=nominal['forward_from_placement_deg'],nominal_inverse_deg=nominal['inverse_from_forward_hold_deg'],
        inverse_range_deg=[min(c['inverse_from_forward_hold_deg'] for c in cases),max(c['inverse_from_forward_hold_deg'] for c in cases)],
        physical_inverse90_verified=False,initial_cube_pose_not_registered=True)
def make_review(p,empty,before):
    if p.EMPTY_RELEASE_ONLY or p.CAPTURE_BEFORE_PHASE!='stage_5_loaded_grasp' or p.SAMPLE_PERIOD!=.075 or p.ON_TIMEOUT!=300:raise RuntimeError('Scope, checkpoint or unchanged timing altered')
    r=transport.make_review(p,empty,transport.hashes());r['historical_forward_reference']=r.pop('completed_physical_reference')
    for key in ('loaded_model_cases','loaded_model_all_passed','loaded_model_nominal_angle_deg','loaded_geometry_cases'):r['historical_forward_'+key]=r.pop(key)
    composed=composition_review();hand=hand_review();cube=cube_models();after=hashes()
    if before!=after:raise RuntimeError('Composed controller/model source changed during review')
    r.update(status='supervised_continuous_forward_inverse_observation_review',code_hashes=after,composed_path=composed,composed_hand_models=hand,composed_cube_models=cube,
        placement_orientation='GREEN up; ORANGE wristward',keep_cube_grasped_between_legs=True,operator_placement_checkpoints_programmed=1,physical_roundtrip_verified=False,
        inverse90_ready=False,no_live_policy=True,health_thresholds_unchanged=True)
    return r
def validate_review(p,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Roundtrip proof/source/scope mismatch')
    return r
def run_screen(request,output,**kwargs):
    p=YawRoundtripPlan(request);ref=YawRoundtripPlan(REQUEST);r=validate_review(ref,PROOF)
    if np.array_equal(p.source_position(),ref.source_position()):
        r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=p.review_center_reused)
        Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
    process_screen(request,output,checker=ROOT/'screen_yaw_roundtrip_20261030.py',failure_details=lambda p:'continuous yaw current EMPTY recovery rejected',**kwargs)
    return validate_review(p,output)
