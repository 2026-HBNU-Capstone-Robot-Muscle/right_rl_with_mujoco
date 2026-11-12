"""Existing physical source plus strict free seating and exact scope checks."""
import json,math
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as transport
from calibrated_quarter_gate_20261028 import bounded
from book_empty_preparation_plan import FRONTS
from single_forward_plan_20261030 import SingleForwardPlan,ROOT,OUT,REQUEST,PROOF,LOADED,HAND_PATH,MODEL_SOURCES,hashes,sha,validate_candidate
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
def hand_review():
 path=OUT/'loaded_hand_dynamics/review.json';r=json.loads(path.read_bytes())
 if r.get('source_sha256')!=sha(HAND_PATH) or not r.get('all_passed') or not r.get('cube_disabled') or not r.get('finite_actuators') or r.get('hand_qpos_prescribed') is not False:raise RuntimeError('Single-forward hand evidence missing or unbound')
 if len(r['cases'])!=9 or {(c['book_front_mm'],c['gain_scale']) for c in r['cases']}!={(f,g) for f in FRONTS for g in (.5,1.,1.5)}:raise RuntimeError('Single-forward hand conditions incomplete')
 for c in r['cases']:
  if not c['passed'] or c['stop'] is not None or not bounded(c,{'max_self_mm':1e-6,'raw_max_self_mm':1e-6,'max_support_mm':.15,'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6}):raise RuntimeError('Single-forward hand exceeds original bounds')
 return dict(path=path.relative_to(ROOT).as_posix(),sha256=sha(path),cases=9,all_passed=True)
def validate_model_cases(cases):
 if len(cases)!=27 or {(c['book_front_mm'],c['gain_scale'],c['friction']) for c in cases}!={(f,g,mu) for f in FRONTS for g in (.5,1.,1.5) for mu in (.2,.3,.5)}:raise RuntimeError('Single-forward Cube27 incomplete')
 for c in cases:
  if not c['passed'] or c['stop'] is not None or not c.get('initial_placement_passed') or c['external_cube_contact'] or not c['final_cube_supported_by_hand'] or not c.get('vertical_seating_line_upper_verified_free'):raise RuntimeError('Single-forward free seating/retention rejected')
  if not bounded(c,{'max_self_mm':1e-6,'max_support_mm':.15,'max_cube_mm':1,'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6,'center_translation_mm':80,'longest_no_hand_contact_seconds':.25}):raise RuntimeError('Single-forward Cube exceeds original bounds')
  a=c['forward_from_placement_deg']
  if not isinstance(a,(int,float)) or not math.isfinite(a) or a<15 or c['inverse_from_forward_hold_deg'] is not None or not 0<=c['yaw_tilt_change_deg']<=15:raise RuntimeError('Single-forward scope/rotation diagnostic rejected')
  if not bounded(c['initial_placement_probe'],{'max_self_mm':1e-6,'max_support_mm':.15,'max_cube_mm':1,'max_tracking_rad':.18}) or c['initial_placement_probe']['external_cube_contact']:raise RuntimeError('Initial placement violates original bounds')
def cube_models():
 cases=[];sources=[]
 for path in MODEL_SOURCES:
  r=json.loads(path.read_bytes())
  if not r.get('all_passed') or r.get('candidate_sha256')!=sha(LOADED) or r.get('simulation_timestep_s')!=.001 or not r.get('initial_placement_also_checked') or r.get('model_source_sha256')!=sha(ROOT/'screen_yaw_roundtrip_model_20261030.py'):raise RuntimeError('Single-forward Cube proof rejected or unbound')
  cases.extend(r['cases']);sources.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=sha(path)))
 validate_model_cases(cases);nominal=next(c for c in cases if (c['book_front_mm'],c['gain_scale'],c['friction'])==(-10,1.,.3))
 return dict(sources=sources,cases=27,nominal_forward_deg=nominal['forward_from_placement_deg'],forward_range_deg=[min(c['forward_from_placement_deg'] for c in cases),max(c['forward_from_placement_deg'] for c in cases)],physical90_not_certified=True,initial_cube_pose_not_registered=True)
def make_review(p,empty,before):
 if p.EMPTY_RELEASE_ONLY or p.CAPTURE_BEFORE_PHASE!='stage_5_loaded_grasp' or p.SAMPLE_PERIOD!=.075 or p.ON_TIMEOUT!=300:raise RuntimeError('Single-forward checkpoint or original timing altered')
 validate_candidate();r=transport.make_review(p,empty,transport.hashes());hand=hand_review();cube=cube_models();after=hashes()
 if before!=after:raise RuntimeError('Single-forward source changed during review')
 for key in ('loaded_model_cases','loaded_model_all_passed','loaded_model_nominal_angle_deg','loaded_geometry_cases'):r['historical_forward_'+key]=r.pop(key)
 r.update(status='supervised_same_forward_reproduction_strict_seating_review',code_hashes=after,strict_single_forward_cube_models=cube,single_forward_hand_models=hand,
    loaded_reference_goals=2032,new_joint_goal_vectors=0,automatic_inverse=False,automatic_open=True,operator_placement_checkpoints_programmed=1,
    placement_orientation='GREEN up; ORANGE wristward',physical_repeatability_verified=False,physical_exact90_verified=False,inverse90_ready=False,no_live_policy=True,health_thresholds_unchanged=True)
 return r
def validate_review(p,path):
 r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
 if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Single-forward proof/source/scope mismatch')
 return r
def run_screen(request,output,**kwargs):
 p=SingleForwardPlan(request);ref=SingleForwardPlan(REQUEST);r=validate_review(ref,PROOF)
 if np.array_equal(p.source_position(),ref.source_position()):
  r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=p.review_center_reused);Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
 process_screen(request,output,checker=ROOT/'screen_single_forward_20261030.py',failure_details=lambda p:'single-forward current EMPTY recovery rejected',**kwargs)
 return validate_review(p,output)
