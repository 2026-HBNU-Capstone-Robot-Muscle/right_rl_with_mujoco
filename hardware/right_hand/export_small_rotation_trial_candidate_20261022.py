"""Freeze a bounded commissioning candidate, without hardware I/O."""
import hashlib
import json
from pathlib import Path
root=Path(__file__).resolve().parent
out=root/'right_hand_rotation_20261022'
candidate=out/'small_yaw_refined_result.json'
replay=out/'small_yaw_refined_result_independent.json'
c=json.loads(candidate.read_bytes());r=json.loads(replay.read_bytes())
assert r['candidate_sha256']==hashlib.sha256(candidate.read_bytes()).hexdigest()
assert r['clearance_tracking_screen_passed'] is True and r['total_cases']==77
assert r['hardware_commands_sent']==0 and c['hardware_commands_sent']==0
assert all(row['fault'] is None for row in r['trials'])
assert c['motion_samples']==80 and c['goal_period_s']==.05
data=dict(version='bounded_small_yaw_commissioning_v1',
    source_sha256=c['source_sha256'],candidate_sha256=r['candidate_sha256'],
    independent_replay_sha256=hashlib.sha256(replay.read_bytes()).hexdigest(),
    base_goal_ticks=c['base_goal_ticks'],goal_ticks=c['candidate_goal_ticks'],
    sample_period_s=.05,pre_rotation_hold_samples=0,rotation_goal_count=80,
    clearance_tracking_cases_passed=77,requested_10deg_rotation_cases_passed=r['successful_cases'],
    requested_rotation_passed=False,physical_rotation_verified=False,
    algorithm='offline_CEM_fixed_path_NOT_SAC',live_sac=False,
    purpose='One operator-triggered small rotation observation. No success or force-closure certification.',
    caveats=['Provisional synthetic cube positions, not certified camera registration.',
             'Native servo and friction models approximate the real hand.'])
path=out/'small_rotation_trial_candidate.json'
path.write_text(json.dumps(data,indent=2),encoding='utf-8')
print(path.name,hashlib.sha256(path.read_bytes()).hexdigest())
