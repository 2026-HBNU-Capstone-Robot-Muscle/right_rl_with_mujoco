"""Offline calibration candidate from an observed90-degree point, no devices."""
import hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
source=ROOT/'right_hand_rotation_20261027/yaw90_clearance_t8_early_root16/candidate.npz'
loaded_source=ROOT/'right_hand_rotation_20261027/yaw90_clearance_t8_early_root16/full_loaded_27/loaded_path.npz'
trace_source=ROOT/'right_hand_rotation_20261028/completed_quarter_045326/rotation_trace.json'
trace=json.loads(trace_source.read_bytes());seen=trace['closest_observed_targets']['90']
count=seen['yaw_goals_already_observed'];assert count==638 and abs(seen['angle_deg']-90)<2
z=np.load(source);loaded=np.load(loaded_source)
assert np.array_equal(z['goals'],loaded['goals'][loaded['phases']=='yaw'])
assert np.array_equal(z['start'],loaded['grasp_goal'])
out=ROOT/'right_hand_rotation_20261028/observed90_prefix638';out.mkdir(exist_ok=True)
np.savez_compressed(out/'candidate.npz',start=z['start'],goals=z['goals'][:count],desired_angles=z['desired_angles'][:count],bias_ticks=z['bias_ticks'])
report=dict(status='offline_physical_timing_calibration_candidate',source=source.relative_to(ROOT).as_posix(),
    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),original_yaw_goals=len(z['goals']),kept_yaw_goals=count,
    observed_source='horizontal_quarter_trial_20261028_045326_636152/result.json',
    trace_source=trace_source.relative_to(ROOT).as_posix(),observed_angle_deg=seen['angle_deg'],observed_image=seen['image'],
    measured_frame_command_ticks=seen['command_ticks'],
    candidate_final_goal_ticks=z['goals'][count-1].tolist(),
    final_goal_matches_measured_frame_command=bool(np.array_equal(z['goals'][count-1],seen['command_ticks'])),
    automatic_open_and_hold_replay_required=True,new_operator_program_ready=False,
    actual_shortened_rotation_not_verified=True,one_run_calibration_not_general_feedback=True,
    original_path_file_modified=False,native_limits_gains_current_unchanged=True,
    camera_angle_is_not_certified_robot_frame=True,hardware_execution_allowed=False,motor_writes=0)
(out/'calibration_source.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
print(json.dumps(report))
