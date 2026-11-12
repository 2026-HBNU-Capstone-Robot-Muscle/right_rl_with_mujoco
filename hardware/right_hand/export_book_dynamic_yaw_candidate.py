"""Freeze the selected measured-baseline dynamic candidate, not hardware approval."""
import hashlib
import json
from pathlib import Path
root=Path(__file__).resolve().parent
folder=root/'right_hand_rotation_20261024/book_yaw_measured_response'
r=json.loads((folder/'dynamic_contact_search.json').read_bytes())
if len(r['candidates'])!=1:raise ValueError('No single direction-consistent dynamic candidate')
c=r['candidates'][0]
sources=('book_small_yaw_trial_20261024_215736_594865/result.json',
    'book_small_yaw_trial_20261024_215736_594865/actual_yaw_review.json',
    'book_small_yaw_trial_20261024_215736_594865/operator_correction.json',
    'right_hand_rotation_20261024/book_yaw_measured_response/measured_baseline_source.json',
    'right_hand_rotation_20261024/book_yaw_measured_response/relative_candidate_replay.json',
    'right_hand_rotation_20261024/book_yaw_measured_response/dynamic_contact_search.json',
    'search_book_yaw_dynamics.py','right_hand_rotation_20261023/native_motor_limits.json')
deltas=[s['delta_yaw_deg'] for s in c['runs']]
out=dict(version='book_measured_dynamic_clockwise_candidate_v2',**c,
    source_hashes={s:hashlib.sha256((root/s).read_bytes()).hexdigest() for s in sources},
    desired_physical_yaw_degrees=-3.,motion_direction='clockwise_viewed_from_above',
    simulated_action_minus_same_closure_control_yaw_deg=[min(deltas),max(deltas)],
    optimizer='bounded_cross_entropy_search_not_SAC',experimental_trial_only=True,
    physical_cube_rotation_verified=False,live_sac=False,robust_policy=False,
    marker_to_model_registration_verified=False,cube_friction_measured=False,
    fitted_joint_zero_offsets=False,following_error_used_only_in_offline_simulation=True,
    current_or_PID_changed=False,hardware_execution_allowed=False,
    source_baseline_precedes_operator_removal_and_OFF=True,
    source_removal_flag_is_not_physical_removal_proof=True)
blob=json.dumps(out,indent=2).encode();path=folder/'dynamic_clockwise_candidate_v2.json'
if path.exists() and path.read_bytes()!=blob:raise ValueError('Different frozen candidate exists')
path.write_bytes(blob)
print(json.dumps(dict(path=str(path),sha256=hashlib.sha256(blob).hexdigest(),
    samples=len(c['goal_ticks']),closure_samples=c['closure_samples'],rotation_samples=c['rotation_samples'],
    delta_yaw_deg=out['simulated_action_minus_same_closure_control_yaw_deg'],
    action_yaw_deg=[s['action']['yaw_deg'] for s in c['runs']])))
