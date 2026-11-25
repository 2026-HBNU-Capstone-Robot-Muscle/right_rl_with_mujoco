"""Evidence for a new natural-recess grasp. Never inherits an old yaw claim."""
import json
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as transport
from calibrated_quarter_gate_20261028 import bounded
from natural_cradle_grasp_plan_20261030 import NaturalCradleGraspPlan,ROOT,OUT,REQUEST,PROOF,LOADED,SEED,hashes,sha,validate_candidate
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
def loaded_models():
 hpath=OUT/'loaded_hand_dynamics/review.json';h=json.loads(hpath.read_bytes())
 if h.get('source_sha256')!=sha(LOADED) or len(h['cases'])!=9 or not h.get('cube_disabled') or not h.get('finite_actuators') or h.get('hand_qpos_prescribed') is not False:raise RuntimeError('Natural grasp hand dynamics missing or unbound')
 if {(c['book_front_mm'],c['gain_scale']) for c in h['cases']}!={(f,g) for f in (-15,-10,-5) for g in (.5,1,1.5)}:raise RuntimeError('Hand condition coverage incomplete')
 for c in h['cases']:
  if not c['passed'] or c['stop'] is not None or c['samples_completed']!=1838 or not bounded(c,dict(max_self_mm=1e-6,raw_max_self_mm=1e-6,max_support_mm=.15,max_tracking_rad=.18,unapproved_range_excess_rad=1e-6)):raise RuntimeError('Natural hand dynamics exceeds unchanged limits')
 cases=[];sources=[]
 for f in (-15,-10,-5):
  path=OUT/f'free_front_{f}/review.json';r=json.loads(path.read_bytes())
  if r.get('candidate_sha256')!=sha(LOADED) or r.get('model_source_sha256')!=sha(ROOT/'screen_natural_grasp_hold_20261030.py') or r.get('cube_seed_sha256')!=sha(SEED) or r['simulation_timestep_s']!=.001 or len(r['cases'])!=9:raise RuntimeError('Natural free Cube evidence missing or unbound')
  cases.extend(r['cases']);sources.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=sha(path)))
 if {(c['book_front_mm'],c['gain_scale'],c['friction']) for c in cases}!={(f,g,mu) for f in (-15,-10,-5) for g in (.5,1,1.5) for mu in (.2,.3,.5)} or len(cases)!=27:raise RuntimeError('Natural free Cube27 incomplete')
 for c in cases:
  if not c['passed'] or c['stop'] is not None or c['last_sample']!=1808 or not c['initial_placement_passed'] or c['external_cube_contact'] or not c['final_cube_supported_by_hand'] or not c['vertical_seating_line_upper_verified_free']:raise RuntimeError('Natural free placement/path rejected')
  if not bounded(c,dict(max_self_mm=1e-6,max_support_mm=.15,max_cube_mm=1,max_tracking_rad=.18,unapproved_range_excess_rad=1e-6,longest_no_hand_contact_seconds=.25,center_translation_mm=80)):raise RuntimeError('Natural free Cube safety limit exceeded')
  hold=c['grasp_hold_observation']
  if hold['evaluated_steps']<4000 or set(hold['opposing_contact_fraction'])!={'mf_tip','th_tip'} or not np.isfinite([hold['max_yaw_drift_deg'],hold['max_center_shift_mm'],*hold['opposing_contact_fraction'].values()]).all():raise RuntimeError('Natural grasp observation incomplete')
  # This is a bounded physical observation experiment, not a reliable grasp
  # claim. Every safety condition above remains mandatory in all27 cases.
  # Retain the exact hold task criterion; require it at nominal gain/friction,
  # and disclose every nonnominal task failure rather than marking it passed.
  if c['gain_scale']==1. and c['friction']==.3:
   if not hold['passed'] or hold['max_yaw_drift_deg']>=3 or hold['max_center_shift_mm']>=2 or min(hold['opposing_contact_fraction'].values())<=.8:raise RuntimeError('Nominal natural Cube not held by both fingertips')
 failures=[dict(front=c['book_front_mm'],gain=c['gain_scale'],friction=c['friction'],observation=c['grasp_hold_observation']) for c in cases if not c['grasp_hold_observation']['passed']]
 nominal=[c['grasp_hold_observation'] for c in cases if c['gain_scale']==1. and c['friction']==.3]
 return dict(cube_safety_cases=27,hand_cases=9,nominal_grasp_task_cases=3,grasp_task_passed_cases=27-len(failures),nonnominal_grasp_task_failures=failures,robust_grasp_verified=False,admission_scope='supervised grasp observation only; all27 safety cases mandatory; nominal hold task mandatory; nonnominal task failures disclosed',sources=sources,hand_proof_sha256=sha(hpath),body_marker_to_robot_registration_verified=False,physical_grasp_verified=False,nominal_hold_max_yaw_deg=max(c['max_yaw_drift_deg'] for c in nominal),nominal_hold_max_center_shift_mm=max(c['max_center_shift_mm'] for c in nominal))
def make_review(p,empty,before):
 if p.EMPTY_RELEASE_ONLY or p.CAPTURE_BEFORE_PHASE!='stage_5_loaded_grasp' or p.CONTACT_RELEASE_BEFORE_PHASE is not None or p.SAMPLE_PERIOD!=.075 or p.ON_TIMEOUT!=300:raise RuntimeError('Natural grasp scope or timing changed')
 validate_candidate();models=loaded_models();r=transport.make_review(p,empty,transport.hashes());after=hashes()
 if before!=after:raise RuntimeError('Natural grasp sources changed during review')
 for key in ('loaded_model_cases','loaded_model_all_passed','loaded_model_nominal_angle_deg','loaded_geometry_cases','completed_physical_reference'):r['historical_transport_'+key]=r.pop(key)
 r.update(status='supervised_natural_cradle_grasp_hold_open_review',code_hashes=after,natural_cradle_models=models,scope='EMPTY -> natural PLACE -> new grasp ->5s hold ->OPEN ->REMOVE/OFF',rotation_targets=0,physical_rotation_verified=False,physical_grasp_verified=False,grasp_only_loaded_goals=1808,automatic_inverse=False,no_live_policy=True,loaded_motion_authorized=True,elevated_reference_matching_required=False)
 return r
def validate_review(p,path):
 r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
 if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Natural grasp proof/source/scope mismatch')
 return r
def run_screen(request,output,**kwargs):
 p=NaturalCradleGraspPlan(request);ref=NaturalCradleGraspPlan(REQUEST);r=validate_review(ref,PROOF)
 if np.array_equal(p.source_position(),ref.source_position()):
  r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=False);Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
 process_screen(request,output,checker=ROOT/'screen_natural_cradle_grasp_current_20261030.py',failure_details=lambda p:'natural-grasp current EMPTY recovery rejected',**kwargs)
 return validate_review(p,output)
