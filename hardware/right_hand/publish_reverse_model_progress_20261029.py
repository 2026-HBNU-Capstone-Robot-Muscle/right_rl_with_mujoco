import datetime,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
evidence=json.loads((ROOT/'reverse_yaw_empty_20261029_164720_101745/offline_review/review.json').read_bytes())
now=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat()
for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name;r=json.loads(path.read_bytes());r.update(status='EMPTY_reverse_complete_inverse_cube_model_in_progress',current_status='EMPTY_reverse_complete_inverse_cube_model_in_progress',timestamp=now,
        latest_reverse_empty_evidence=evidence,reverse_empty_yaw_ready=False,reverse_empty_yaw_complete=True,
        next_command=None,next_user_command=None,next_user_command_ready=False,next_physical_command_ready=False,next_user_action_required=False,
        next_user_action='No motion program yet. Keep cube OUT; inverse cube contact model is being evaluated.',
        next_step='Find a genuinely negative cube rotation; reversed joint motion alone did not reproduce a negative90deg turn',
        inverse_cube_trial_ready=False,inverse_cube_rotation_verified=False,
        motors_off_verified=True,original_settings_restored_now=True,assistant_motor_writes=0,assistant_motor_or_setting_writes=0)
    path.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('Saved completed EMPTY evidence; inverse cube execution remains unready.')
