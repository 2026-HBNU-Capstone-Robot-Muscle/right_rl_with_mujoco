"""Admission for a bounded model-validation experiment, never a90deg claim."""
import json
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as transport
from calibrated_quarter_gate_20261028 import bounded
from raised_cradle_plan_20261104 import RaisedCradlePlan,ROOT,OUT,REQUEST,PROOF,LOADED,SEED,hashes,sha,validate_candidate
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
def loaded_models():
 validate_candidate();hpath=OUT/'loaded_hand_dynamics/review.json';h=json.loads(hpath.read_bytes());expected={(f,g) for f in (-15,-10,-5) for g in (.5,1,1.5)}
 if h.get('source_sha256')!=sha(LOADED) or len(h['cases'])!=9 or {(c['book_front_mm'],c['gain_scale']) for c in h['cases']}!=expected or not h.get('cube_disabled') or not h.get('finite_actuators') or h.get('hand_qpos_prescribed') is not False:raise RuntimeError('Raised cradle hand evidence missing/unbound')
 for c in h['cases']:
  if not c['passed'] or c['stop'] is not None or c['samples_completed']!=1500 or not bounded(c,dict(max_self_mm=1e-6,raw_max_self_mm=1e-6,max_support_mm=.15,max_tracking_rad=.18,unapproved_range_excess_rad=1e-6)):raise RuntimeError('Raised cradle hand clearance failed')
 cases=[];sources=[]
 for f in (-15,-10,-5):
  path=OUT/f'material_front_{f}/review.json';r=json.loads(path.read_bytes())
  if r.get('candidate_sha256')!=sha(LOADED) or r.get('model_source_sha256')!=sha(ROOT/'screen_yaw_roundtrip_model_20261030.py') or r.get('cube_seed_sha256')!=sha(SEED) or r['simulation_timestep_s']!=.001 or len(r['cases'])!=9:raise RuntimeError('Free model evidence incomplete or changed')
  cases.extend(r['cases']);sources.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=sha(path)))
 if len(cases)!=27 or {(c['book_front_mm'],c['gain_scale'],c['friction']) for c in cases}!={(f,g,mu) for f in (-15,-10,-5) for g in (.5,1,1.5) for mu in (.2,.3,.5)}:raise RuntimeError('Missing free model case')
 for c in cases:
  if not c['passed'] or c['stop'] is not None or c['last_sample']!=1470 or not c['initial_placement_passed'] or c['external_cube_contact'] or not c['final_cube_supported_by_hand'] or not c['vertical_seating_line_upper_verified_free']:raise RuntimeError('Free seating/turn safety failed')
  if not bounded(c,dict(max_self_mm=1e-6,max_support_mm=.15,max_cube_mm=1,max_tracking_rad=.18,unapproved_range_excess_rad=1e-6,longest_no_hand_contact_seconds=.25,center_translation_mm=80)):raise RuntimeError('Free cube model limit exceeded')
 nominal=[c for c in cases if c['gain_scale']==1 and c['friction']==.3]
 if len(nominal)!=3 or any(not np.isfinite(c['final_yaw_from_placement_deg']) or c['final_yaw_from_placement_deg']<45 for c in nominal):raise RuntimeError('Nominal candidate has no useful turn to observe')
 return dict(safety_cases=27,hand_cases=9,scope='supervised prepared-support reference observation; nominal model65degrees;90degree target remains unmet',nominal_rotation_deg=[c['final_yaw_from_placement_deg'] for c in nominal],all_model_angles_deg=[c['final_yaw_from_placement_deg'] for c in cases],model90_task_verified=False,physical90_verified=False,robot_axis_registration_verified=False,hand_sha256=sha(hpath),sources=sources)
def make_review(p,empty,before):
 validate_candidate();models=loaded_models();r=transport.make_review(p,empty,transport.hashes());after=hashes()
 if before!=after:raise RuntimeError('Raised cradle sources changed during review')
 for key in ('loaded_model_cases','loaded_model_all_passed','loaded_model_nominal_angle_deg','loaded_geometry_cases','completed_physical_reference'):r['historical_transport_'+key]=r.pop(key)
 if p.SAMPLE_PERIOD!=.075 or p.ON_TIMEOUT!=300 or p.EMPTY_RELEASE_ONLY or p.CONTACT_RELEASE_BEFORE_PHASE is not None:raise RuntimeError('Runtime timing/scope changed')
 r.update(status='supervised_raised_cradle_reference_observation',code_hashes=after,raised_cradle_models=models,scope='EMPTY raised support -> natural PLACE -> full reference turn/fixed5s -> user supports cube/SPACE OFF; no inverse/no automatic OPEN',loaded_motion_authorized=True,loaded_goals=1470,automatic_inverse=False,automatic_OPEN=False,physical90_verified=False,no_live_policy=True,assistant_motor_writes=0);return r
def validate_review(p,path):
 r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
 if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Raised cradle proof/source/scope mismatch')
 return r
def run_screen(request,output,**kwargs):
 p=RaisedCradlePlan(request);reference=RaisedCradlePlan(REQUEST);r=validate_review(reference,PROOF)
 if np.array_equal(p.source_position(),reference.source_position()):
  r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=False);Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
 process_screen(request,output,checker=ROOT/'screen_raised_cradle_current_20261104.py',failure_details=lambda p:'raised-cradle current EMPTY recovery rejected',**kwargs);return validate_review(p,output)
