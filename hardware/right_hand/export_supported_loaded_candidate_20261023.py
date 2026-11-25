"""Freeze one bounded loaded observation after independent model checks."""
import hashlib
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261023'
screen=OUT/'supported_loaded_prefix_256_scale_085.json'
limits=OUT/'native_motor_limits.json'
empty=ROOT/'supported_yaw_empty_20261023_030552_675209/result.json'
review=empty.parent/'completion_review.json'
r=json.loads(screen.read_bytes());physical=json.loads(review.read_bytes())
assert r['clearance_tracking_screen_passed'] and r['total_motion_cases']==154 and r['total_control_cases']==154
assert r['native_ranges_unchanged'] and not r['live_sac'] and r['hardware_commands_sent']==0
assert r['completed_empty_source_sha256']==hashlib.sha256(empty.read_bytes()).hexdigest()
assert physical['exact_previous_targets'] and physical['final_target_exact']
assert physical['all_torque_off_verified'] and physical['settings_restored']
base=np.array(r['base_goal_ticks']);q=np.array(r['goal_ticks'])
assert q.shape==(256,16) and np.max(abs(np.diff(np.vstack([base,q]),axis=0)))<=2
assert np.max(abs(q-base))<=256 and np.all(q[:,[12,14,15]]<=base[[12,14,15]])
record=dict(version='supported_loaded_observation_v1',base_goal_ticks=base.tolist(),goal_ticks=q.tolist(),
    sample_period_s=.05,pre_rotation_hold_samples=0,hold_samples=20,motion_scale=.85,
    independent_screen_sha256=hashlib.sha256(screen.read_bytes()).hexdigest(),
    empty_rehearsal_result_sha256=hashlib.sha256(empty.read_bytes()).hexdigest(),
    empty_review_sha256=hashlib.sha256(review.read_bytes()).hexdigest(),
    native_limits_sha256=hashlib.sha256(limits.read_bytes()).hexdigest(),
    clearance_tracking_cases_passed=154,loaded_observation_ready=True,
    physical_rotation_verified=False,requested_rotation_guaranteed=False,live_sac=False,
    algorithm='offline_CEM_fixed_path_NOT_SAC',
    scope='One operator-supervised loaded observation; not a verified 15deg or 90deg controller.')
target=OUT/'supported_loaded_candidate.json'
target.write_text(json.dumps(record,indent=2),encoding='utf-8')
print('CANDIDATE_SHA',hashlib.sha256(target.read_bytes()).hexdigest())
