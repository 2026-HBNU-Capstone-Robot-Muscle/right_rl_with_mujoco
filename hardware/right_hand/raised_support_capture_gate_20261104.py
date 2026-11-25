import json
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as transport
from raised_support_capture_plan_20261104 import RaisedSupportCapturePlan,ROOT,OUT,REQUEST,PROOF,TURN_OUT,hashes,sha
from raised_cradle_plan_20261104 import validate_candidate
from calibrated_quarter_plan_20261028 import PREFIX_PHASES
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
def make_review(p,empty,before):
 rows=p.plan(p.source_position());held=[q for phase,q in rows if phase not in PREFIX_PHASES];goal=validate_candidate()['start']
 if len(held)!=27 or not np.all(np.asarray(held)==goal):raise RuntimeError('Placement capture contains an angle change')
 r=transport.make_review(p,empty,transport.hashes());after=hashes()
 if before!=after:raise RuntimeError('Capture code changed during review')
 for k in ('loaded_model_cases','loaded_model_all_passed','loaded_model_nominal_angle_deg','loaded_geometry_cases','completed_physical_reference'):r['historical_transport_'+k]=r.pop(k)
 r.update(status='raised_support_static_capture_review',code_hashes=after,scope='EMPTY raised-support preparation; static natural placement photo2.025s; removal/OFF;no grasp/turn',loaded_motion_authorized=True,post_placement_angle_changes=0,static_observation_goals=27,rotation_targets=0,grasp_targets=0,physical_rotation_verified=False,physical_grasp_verified=False,raised_turn_candidate_admitted=False,no_live_policy=True,assistant_motor_writes=0);return r
def validate_review(p,path):
 r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
 if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Static capture proof/source/scope mismatch')
 return r
def reuse_identical_prefix():
 p=RaisedSupportCapturePlan(REQUEST);old=TURN_OUT/'empty_recovery_screen';z=np.load(old/'prefix_path.npz');prefix=[(phase,q) for phase,q in p.plan(p.source_position()) if phase in PREFIX_PHASES]
 if not np.array_equal(z['start'],p.source_position()) or not np.array_equal(z['goals'],np.asarray([q for phase,q in prefix])) or not np.array_equal(z['phases'],np.asarray([phase for phase,q in prefix])):raise RuntimeError('Capture prefix is not exactly the screened prefix')
 r=make_review(p,json.loads((old/'review.json').read_bytes()),hashes());r['identical_prefix_source']=dict(path=(old/'review.json').relative_to(ROOT).as_posix(),sha256=sha(old/'review.json'),arrays_sha256=sha(old/'prefix_path.npz'),exact_goals_and_phases_verified=True);PROOF.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');validate_review(p,PROOF);print('STATIC CAPTURE REVIEW READY; no post-placement movement')
def run_screen(request,output,**kwargs):
 p=RaisedSupportCapturePlan(request);reference=RaisedSupportCapturePlan(REQUEST);r=validate_review(reference,PROOF)
 if np.array_equal(p.source_position(),reference.source_position()):
  r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=False);Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
 process_screen(request,output,checker=ROOT/'screen_raised_support_capture_current_20261104.py',failure_details=lambda p:'current empty support preparation rejected',**kwargs);return validate_review(p,output)
if __name__=='__main__':reuse_identical_prefix()
