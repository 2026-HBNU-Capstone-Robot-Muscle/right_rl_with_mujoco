"""Summarize whole-path model evidence without authorizing a physical trial."""
import json,hashlib,itertools
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/held_contact_release'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
candidate=np.load(O/'candidate.npz');checksum=sha(O/'candidate.npz');cases=[];sources={}
for front in (-15,-10,-5):
 p=O/f'full_front_{front}/review.json';r=json.loads(p.read_bytes());assert r['candidate_sha256']==checksum and len(r['cases'])==9
 assert {(c['gain_scale'],c['friction']) for c in r['cases']}==set(itertools.product((.5,1.,1.5),(.2,.3,.5)))
 for c in r['cases']:
  assert c['book_front_mm']==front and c['passed'] and c['last_sample']==len(candidate['goals']) and c['body_hold_observation']['hold_task_passed'] and c['max_self_mm']<=1e-6 and not c['external_cube_contact']
 cases+=r['cases'];sources[p.relative_to(R).as_posix()]=sha(p)
hand_path=O/'loaded_hand_dynamics/review.json';hand=json.loads(hand_path.read_bytes())
assert hand['source_sha256']==checksum and len(hand['cases'])==9 and all(c['passed'] and c['raw_max_self_mm']<=1e-6 and c['samples_completed']==len(candidate['goals'])+30 for c in hand['cases'])
release_path=O/'free_release_review.json';release=json.loads(release_path.read_bytes());assert release['candidate_sha256']==checksum and release['safety_passed'] and release['release_task_passed']
source=np.load(R/'right_hand_rotation_20261030/lower_body_regrasp/candidate.npz');assert np.array_equal(candidate['goals'][:1597],source['goals'][:1597])
summary=dict(status='model_research_complete_physical_trial_not_ready',candidate_sha256=checksum,goals=len(candidate['goals']),loaded_model_cases=len(cases),all_loaded_safety_and_body_hold_cases_passed=True,hand_model_cases=9,hand_raw_self_mm=max(c['raw_max_self_mm'] for c in hand['cases']),forward_model_rotation_range_deg=[min(c['forward_from_placement_deg'] for c in cases),max(c['forward_from_placement_deg'] for c in cases)],body_hold_max_yaw_drift_deg=max(c['body_hold_observation']['max_yaw_drift_deg'] for c in cases),body_hold_max_center_shift_mm=max(c['body_hold_observation']['max_center_shift_mm'] for c in cases),nominal_release_task=release,release_unclamped_all_27_conditions_not_measured=True,nominal_release_source_is_evolved_held_state=True,first1597_goals_byte_identical_to_previous_held_prefix=True,upper_layer_tool_and_physical_torque_not_verified=True,physical_rotation90_verified=False,physical_rotation_hold_trial_ready=False,motor_writes=0,controller_limits_or_forces_increased=False,sources={**sources,hand_path.relative_to(R).as_posix():sha(hand_path),release_path.relative_to(R).as_posix():sha(release_path)})
(O/'research_summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in summary.items() if k not in ('sources','nominal_release_task')}))
