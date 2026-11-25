"""EMPTY-only admission; new geometry and dynamics never certify a cube turn."""
import json
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as transport
from supported_inverse_empty_plan_20261029 import SupportedInverseEmptyPlan,ROOT,OUT,REQUEST,PROOF,HAND_PATH,hashes,sha
from calibrated_quarter_gate_20261028 import bounded
from book_empty_preparation_plan import FRONTS,offsets
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen

def new_empty_review():
    geometry=OUT/'whole_geometry/review.json';g=json.loads(geometry.read_bytes())
    if not g.get('all_passed') or not g.get('all_offsets_requested') or g.get('source_sha256')!=sha(HAND_PATH):raise RuntimeError('New static route rejected or unbound')
    if g['source']!=HAND_PATH.relative_to(ROOT).as_posix() or len(g['cases'])!=201:raise RuntimeError('Whole static coverage incomplete')
    if {(c['book_front_mm'],tuple(c['offset_ticks'])) for c in g['cases']}!={(f,tuple(o)) for f in FRONTS for o in offsets()}:raise RuntimeError('Static conditions differ')
    if any(not c['passed'] or not bounded(c,{'max_self_mm':1e-6,'max_support_mm':.15}) for c in g['cases']):raise RuntimeError('Static collision bound violated')
    dynamic=OUT/'loaded_hand_dynamics/review.json';d=json.loads(dynamic.read_bytes())
    if not d.get('all_passed') or not d.get('cube_disabled') or not d.get('finite_actuators') or d.get('source_sha256')!=sha(HAND_PATH):raise RuntimeError('New finite EMPTY route rejected')
    if len(d['cases'])!=9 or {(c['book_front_mm'],c['gain_scale']) for c in d['cases']}!={(f,s) for f in FRONTS for s in (.5,1.,1.5)}:raise RuntimeError('Finite conditions incomplete')
    if any(not c['passed'] or c['stop'] is not None or not bounded(c,{'max_self_mm':1e-6,'raw_max_self_mm':1e-6,'max_support_mm':.15,'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6}) for c in d['cases']):raise RuntimeError('Finite EMPTY bounds violated')
    return dict(static_cases=201,finite_cases=9,path_sha256=sha(HAND_PATH),geometry_sha256=sha(geometry),dynamics_sha256=sha(dynamic),cube_disabled=True)
def make_review(p,empty,before):
    if not p.EMPTY_RELEASE_ONLY or p.CAPTURE_BEFORE_PHASE is not None:raise RuntimeError('EMPTY scope changed')
    r=transport.make_review(p,empty,transport.hashes())
    r['historical_forward_reference']=r.pop('completed_physical_reference')
    for key in ('loaded_model_cases','loaded_model_all_passed','loaded_model_nominal_angle_deg','loaded_geometry_cases'):
        r['historical_forward_'+key]=r.pop(key)
    reviewed=new_empty_review();after=hashes()
    if before!=after:raise RuntimeError('New EMPTY source changed during review')
    r.update(status='EMPTY_supported_inverse_operator_review',code_hashes=after,new_empty_hand_route=reviewed,
        cube_stays_OUT=True,no_PLACE_or_REMOVE=True,no_placement_confirmation_fabricated=True,
        loaded_model_evidence_is_historical_only=True,physical_new_path_completed=False,physical_inverse_90_verified=False,
        inverse_cube_trial_ready=False,no_new_wait_controller=True)
    return r
def validate_review(p,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('New EMPTY proof/source mismatch')
    return r
def run_screen(request,output,**kwargs):
    p=SupportedInverseEmptyPlan(request);ref=SupportedInverseEmptyPlan(REQUEST);r=validate_review(ref,PROOF)
    if np.array_equal(p.source_position(),ref.source_position()):
        r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=p.review_center_reused)
        Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
    process_screen(request,output,checker=ROOT/'screen_supported_inverse_empty_20261029.py',failure_details=lambda p:'new supported inverse EMPTY recovery rejected',**kwargs)
    return validate_review(p,output)
