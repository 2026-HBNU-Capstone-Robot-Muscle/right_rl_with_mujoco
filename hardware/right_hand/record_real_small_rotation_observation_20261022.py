"""Record user observation separately from motion telemetry and noisy vision."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'cube_small_rotation_trial_20261022_151241_962114'
review=json.loads((FOLDER/'completion_review.json').read_bytes())
review.update(user_observation_pending=False,
    user_statement='다시 해봤는데 손가락만 조금 움직이고 큐브는 그대로네 로그 남았을거야',
    user_reports_no_visible_cube_rotation=True,physical_desired_rotation_success=False,
    physical_cube_rotation_verified=False,
    note='Exact 80-goal sequence completed. User reports fingers moved but cube stayed. Small marker relative-pose estimate is a noisy local-normal diagnostic, not proof of the desired world yaw.')
(FOLDER/'completion_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8')
out=ROOT/'right_hand_rotation_20261022'
status=dict(status='small_path_complete_but_desired_physical_rotation_not_demonstrated',
    source_trial=FOLDER.name,source_sha256=review['source_sha256'],
    camera_content_usable=True,command_path_completed=True,
    exact_rotation_targets_confirmed=True,user_reports_no_visible_cube_rotation=True,
    physical_desired_rotation_success=False,physical_cube_rotation_verified=False,
    camera_only_recapture_needed=False,repeated_identical_motor_trial_requested=False,
    minimum_voltage_v=review['minimum_voltage_v'],peak_current_ma=review['peak_current_ma'],
    maximum_tracking_rad=review['max_tracking_rad'],all_torque_off_verified=True,settings_restored=True,
    offline_next_action='Search a slower, larger visible rotation trajectory; preserve measured health trips.')
(out/'latest_physical_observation.json').write_text(json.dumps(status,indent=2),encoding='utf-8')
print(json.dumps(status,indent=2))
