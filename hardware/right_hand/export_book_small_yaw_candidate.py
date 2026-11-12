"""Freeze the smaller contact-error clockwise candidate, without hardware approval."""
import hashlib
import json
from pathlib import Path
root=Path(__file__).resolve().parent;out=root/'right_hand_rotation_20261024/book_grasp_yaw_diagnostic'
source=out/'contact_search_clockwise.json';r=json.loads(source.read_bytes())
if not r['candidates']:raise RuntimeError('No direction-consistent candidate')
c=min(r['candidates'],key=lambda c:(abs(c['summary']['kinematic_angle_deg']),c['summary']['max_contact_target_error_mm']))
sources=('book_cube_grasp_trial_20261024_212855_824047/result.json',
    'right_hand_rotation_20261024/book_grasp_yaw_diagnostic/contact_search_clockwise.json',
    'right_hand_rotation_20261024/book_grasp_yaw_diagnostic/relative_candidate_replay.json',
    'right_hand_rotation_20261023/native_motor_limits.json',
    'search_book_contact_yaw.py','replay_book_relative_yaw_candidate.py')
delta=[x['delta_yaw_deg'] for x in c['summary']['runs']+c['summary']['held_out']]
candidate=dict(version='book_completed_grasp_clockwise_candidate_v1',**c,
    source_hashes={s:hashlib.sha256((root/s).read_bytes()).hexdigest() for s in sources},
    desired_physical_yaw_degrees=-3.,motion_direction='clockwise_viewed_from_above',
    simulated_action_minus_control_yaw_range_deg=[min(delta),max(delta)],
    experimental_trial_only=True,physical_cube_rotation_verified=False,live_sac=False,
    robust_policy=False,full_three_degree_amplitude_verified=False,
    marker_to_model_registration_verified=False,cube_friction_measured=False,
    following_error_used_only_in_offline_simulation=True,fitted_joint_zero_offsets=False,
    current_or_PID_changed=False,hardware_execution_allowed=False)
path=out/'book_small_yaw_candidate.json';blob=json.dumps(candidate,indent=2).encode()
if path.exists() and path.read_bytes()!=blob:raise RuntimeError('Refuse to overwrite another candidate')
path.write_bytes(blob)
print(json.dumps(dict(path=str(path),sha256=hashlib.sha256(blob).hexdigest(),
    samples=len(c['goal_ticks']),closure_samples=c['closure_samples'],
    simulated_delta_yaw_deg=candidate['simulated_action_minus_control_yaw_range_deg'])))
