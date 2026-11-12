"""Bind EMPTY-only model evidence and preserve existing bus/health policies."""
import json
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as transport
from placement_alignment_plan_20261030 import PlacementAlignmentPlan,ROOT,OUT,REQUEST,PROOF,REFERENCE_RESULT,REFERENCE_IMAGE,hashes,sha,validate_scope
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
def reference_review():
 r=json.loads(REFERENCE_RESULT.read_bytes())
 if not r['all_torque_off_verified'] or not r['settings_restored'] or not REFERENCE_IMAGE.exists():raise RuntimeError('Actual placement reference unavailable')
 return dict(result=REFERENCE_RESULT.relative_to(ROOT).as_posix(),result_sha256=sha(REFERENCE_RESULT),photo=REFERENCE_IMAGE.relative_to(ROOT).as_posix(),photo_sha256=sha(REFERENCE_IMAGE),observed_front_change='ORANGE to WHITE later in this run',precise90_not_verified=True,orientation='GREEN up; ORANGE toward wrist ID41')
def make_review(p,empty,before):
 if not p.CAPTURE_PLACEMENT or p.CAPTURE_BEFORE_PHASE is not None or p.CONTACT_RELEASE_BEFORE_PHASE is not None or p.EMPTY_RELEASE_ONLY or p.SAMPLE_PERIOD!=.075 or p.ON_TIMEOUT!=300:raise RuntimeError('Placement-only capture or original timing scope altered')
 rows=validate_scope(p,p.source_position());r=transport.make_review(p,empty,transport.hashes());after=hashes()
 if before!=after:raise RuntimeError('Placement-only sources changed during model review')
 for k in ('loaded_model_cases','loaded_model_all_passed','loaded_model_nominal_angle_deg','loaded_geometry_cases'):r['historical_reference_'+k]=r.pop(k)
 r.update(status='supervised_empty_recovery_then_fixed_goal_placement_photo',code_hashes=after,scope='EMPTY prefix only; fixed goals during cube placement and removal',post_placement_goal_writes=0,grasp_targets=0,rotation_targets=0,physical_rotation_verified=False,body_grip_verified=False,reference_placement=reference_review(),loaded_motion_authorized=False,nominal_motion_seconds=len(rows)*p.SAMPLE_PERIOD)
 return r
def validate_review(p,path):
 r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
 if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Placement-only source/scope/proof mismatch')
 return r
def run_screen(request,output,**kwargs):
 p=PlacementAlignmentPlan(request);ref=PlacementAlignmentPlan(REQUEST);r=validate_review(ref,PROOF)
 if np.array_equal(p.source_position(),ref.source_position()):
  r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=False);Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
 process_screen(request,output,checker=ROOT/'screen_placement_alignment_20261030.py',failure_details=lambda p:'placement-only current EMPTY recovery rejected',**kwargs)
 return validate_review(p,output)
