"""Source-bound admission for the new inverse observation, no relaxed health stops."""
import json,math
from pathlib import Path
import numpy as np
import four_reply_yaw_gate_20261029 as transport
from gapped_inverse_plan_20261029 import (GappedInversePlan,ROOT,OUT,REQUEST,PROOF,
    LOADED,SEED,HAND_PATH,GEOMETRY_PATH,hashes,sha,extra_rows,loaded_rows)
from calibrated_quarter_gate_20261028 import bounded
from book_empty_preparation_plan import FRONTS,offsets
from pitch_handoff_offscreen_process_20261028 import run_screen as process_screen
from calibrated_quarter_plan_20261028 import LOADED_PATH as FORWARD_PATH

PHYSICAL_EMPTY=ROOT/'reverse_yaw_empty_20261029_164720_101745/result.json'
PHYSICAL_EMPTY_REVIEW=PHYSICAL_EMPTY.parent/'offline_review/review.json'

def physical_empty_review():
    r=json.loads(PHYSICAL_EMPTY_REVIEW.read_bytes());raw=json.loads(PHYSICAL_EMPTY.read_bytes())
    if r.get('source_sha256')!=sha(PHYSICAL_EMPTY) or not r.get('all_actual_goal_writes_match_ON_rebased_queue'):
        raise RuntimeError('Actual EMPTY reverse evidence changed')
    if raw.get('status')!='empty_reverse_yaw_path_complete' or not raw.get('all_torque_off_verified') or not raw.get('settings_restored') or raw.get('cube_present') is not False:
        raise RuntimeError('Actual EMPTY reverse run incomplete')
    original=np.load(FORWARD_PATH);new=np.load(LOADED)
    reverse=original['goals'][::-1]
    opened=original['start'];distance=int(max(abs(opened-reverse[-1])))
    count=max(1,int(np.ceil(distance/2)))
    join=np.rint([reverse[-1]+(opened-reverse[-1])*i/count for i in range(1,count+1)]).astype(int)
    expected=np.vstack([reverse,join,np.repeat(opened[None,:],20,axis=0)]).tolist()
    written=[e['requested'] for e in raw['write_events'] if e['address']==116]
    if written[-len(expected):]!=expected:
        raise RuntimeError('Saved EMPTY run did not write the bound forward cycle in reverse')
    if not np.array_equal(original['goals'][original['phases']=='yaw'][::-1],new['goals'][new['phases']=='stage_7_horizontal_yaw']):
        raise RuntimeError('Inverse yaw differs from EMPTY-tested goals')
    return dict(path=PHYSICAL_EMPTY.relative_to(ROOT).as_posix(),sha256=sha(PHYSICAL_EMPTY),
        review_sha256=sha(PHYSICAL_EMPTY_REVIEW),yaw_goals=638,physical_cube_rotation_verified=False)

def additional_path_review():
    original=np.load(FORWARD_PATH);covered={tuple(q) for q in np.vstack([original['start'],original['goals']])}
    added={tuple(q) for p,q in extra_rows()+loaded_rows() if tuple(q) not in covered}
    data=np.load(GEOMETRY_PATH)
    if added!={tuple(q) for q in data['goals']} or len(added)!=32:
        raise RuntimeError('New geometry coverage does not match all added goals')
    static_path=OUT/'additional_geometry/review.json';static=json.loads(static_path.read_bytes())
    if not static.get('all_passed') or not static.get('all_offsets_requested') or static.get('source')!=GEOMETRY_PATH.relative_to(ROOT).as_posix():
        raise RuntimeError('Additional geometry review missing')
    if len(static['cases'])!=201 or {(r['book_front_mm'],tuple(r['offset_ticks'])) for r in static['cases']}!={(f,tuple(o)) for f in FRONTS for o in offsets()}:
        raise RuntimeError('Missing additional geometry offset cases')
    if any(not r['passed'] or not bounded(r,{'max_self_mm':1e-6,'max_support_mm':.15}) for r in static['cases']):
        raise RuntimeError('New placement gap geometry rejected')
    path=OUT/'loaded_hand_dynamics/review.json';dynamic=json.loads(path.read_bytes())
    if dynamic.get('source_sha256')!=sha(HAND_PATH) or not dynamic.get('all_passed') or not dynamic.get('cube_disabled') or not dynamic.get('finite_actuators'):
        raise RuntimeError('Whole EMPTY hand route dynamics rejected')
    if len(dynamic['cases'])!=9 or {(r['book_front_mm'],r['gain_scale']) for r in dynamic['cases']}!={(f,g) for f in FRONTS for g in (.5,1.,1.5)}:
        raise RuntimeError('Missing hand dynamics conditions')
    if any(not r['passed'] or r['stop'] is not None or not bounded(r,{'max_self_mm':1e-6,'raw_max_self_mm':1e-6,'max_support_mm':.15,'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6}) for r in dynamic['cases']):
        raise RuntimeError('New hand route violates existing bounds')
    return dict(new_pose_vectors=32,new_geometry_cases=201,finite_empty_cases=9,
        geometry_sha256=sha(static_path),dynamic_sha256=sha(path),hand_path_sha256=sha(HAND_PATH),
        previous_pose_coverage='unchanged original reverse cycle; historical201-case geometry remains bound by parent hashes')

def inverse_model_review():
    path=OUT/'inverse_material_review.json';r=json.loads(path.read_bytes())
    if not r.get('all_passed') or r.get('candidate_sha256')!=sha(LOADED) or r.get('cube_seed_sha256')!=sha(SEED):
        raise RuntimeError('New loaded inverse tape or seed not reviewed')
    if r.get('model_source_sha256')!=sha(ROOT/'screen_material_checked_reverse_cube_20261029.py') or r.get('material_policy_sha256')!=sha(ROOT/'cube_contact_material_policy_20261029.py'):
        raise RuntimeError('Inverse physics implementation differs from review')
    if not r.get('actual_contact_material_varied') or r.get('physical_inverse_cube_rotation_verified') is not False or r.get('inverse_90_ready') is not False:
        raise RuntimeError('Inverse experiment scope changed')
    for source in r['sources']:
        if sha(ROOT/source['path'])!=source['sha256']:raise RuntimeError('Inverse case source changed')
    cases=r['cases']
    if len(cases)!=27 or {(c['book_front_mm'],c['gain_scale'],c['friction']) for c in cases}!={(f,g,mu) for f in FRONTS for g in (.5,1.,1.5) for mu in (.2,.3,.5)}:
        raise RuntimeError('Missing inverse cube conditions')
    for c in cases:
        if not c['passed'] or c['stop'] is not None or not c['initial_placement_passed'] or c['external_cube_contact'] or not c['final_cube_supported_by_hand']:
            raise RuntimeError('Inverse cube retention rejected')
        if not bounded(c,{'max_self_mm':1e-6,'max_support_mm':.15,'max_cube_mm':1,'max_tracking_rad':.18,'unapproved_range_excess_rad':1e-6,'center_translation_mm':80,'longest_no_hand_contact_seconds':.25}):
            raise RuntimeError('Inverse cube exceeds unchanged safety bounds')
        if not math.isfinite(c['placed_to_turn_deg']) or c['placed_to_turn_deg']>-15 or not 0<=c['yaw_tilt_change_deg']<=15:
            raise RuntimeError('Counter-direction observation not supported in every model case')
        probe=c['initial_placement_probe']
        if not bounded(probe,{'max_self_mm':1e-6,'max_support_mm':.15,'max_cube_mm':1,'max_tracking_rad':.18}) or probe['external_cube_contact']:
            raise RuntimeError('Initial placement violates unchanged bounds')
    nominal=next(c for c in cases if (c['book_front_mm'],c['gain_scale'],c['friction'])==(-10.,1.,.3))
    if not -34<=nominal['placed_to_turn_deg']<=-29:raise RuntimeError('Nominal inverse response changed')
    return dict(path=path.relative_to(ROOT).as_posix(),sha256=sha(path),cases=27,
        nominal_placed_to_turn_deg=nominal['placed_to_turn_deg'],
        range_deg=r['placed_to_turn_range_deg'],initial_cube_pose_not_registered=True,
        physical_inverse_verified=False,inverse_90_ready=False)

def make_review(p,empty,before):
    # Retain all original transport, OFF/ON, health, initial separation and
    # current-prefix checks. Forward object evidence is explicitly historical.
    r=transport.make_review(p,empty,transport.hashes())
    r['historical_forward_reference']=r.pop('completed_physical_reference')
    for field in ('loaded_model_cases','loaded_model_all_passed','loaded_model_nominal_angle_deg','loaded_geometry_cases'):
        r['historical_forward_'+field]=r.pop(field)
    added=additional_path_review();inverse=inverse_model_review();physical=physical_empty_review()
    after=hashes()
    if before!=after:raise RuntimeError('Inverse controller/model sources changed during review')
    r.update(status='supervised_gapped_inverse_observation_review',code_hashes=after,
        experimental_inverse_only=True,additional_hand_path=added,inverse_model=inverse,
        actual_empty_reverse_source=physical,placement_orientation='GREEN up; WHITE wristward',
        empty_preposition_before_PLACE=True,post_PLACE_close_goals=32,inverse_yaw_goals=638,
        physical_inverse_cube_rotation_verified=False,inverse_90_ready=False,
        no_new_wait_controller=True,loaded_model_evidence_is_historical_only=False)
    return r

def validate_review(p,path):
    r=json.loads(Path(path).read_bytes());expected=make_review(p,r['empty_recovery'],hashes())
    if any(r.get(k)!=v for k,v in expected.items()):raise RuntimeError('Gapped inverse proof/source mismatch')
    return r

def run_screen(request,output,**kwargs):
    p=GappedInversePlan(request);ref=GappedInversePlan(REQUEST);r=validate_review(ref,PROOF)
    if np.array_equal(p.source_position(),ref.source_position()):
        r=dict(r);r.update(source_sha256=p.SOURCE_SHA,raw_off_ticks=p.raw_off_ticks.tolist(),review_center_reused=p.review_center_reused)
        Path(output).write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8');return validate_review(p,output)
    process_screen(request,output,checker=ROOT/'screen_gapped_inverse_20261029.py',
        failure_details=lambda p:'current inverse EMPTY recovery rejected',**kwargs)
    return validate_review(p,output)
