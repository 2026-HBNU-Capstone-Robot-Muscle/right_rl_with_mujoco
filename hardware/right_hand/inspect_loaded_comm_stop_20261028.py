"""Offline review of the loaded stop; no device access."""
import json,datetime as dt
from pathlib import Path
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'horizontal_quarter_trial_20261028_000157_905217/result.json'
r=json.loads(SOURCE.read_bytes());samples=r.get('samples',[])
review={k:r.get(k) for k in ('status','failed_phase','primary_motion_error','primary_failed_phase',
    'error','final_check_error','all_torque_off_verified','settings_restored','read_failures',
    'read_recovery_confirmation_events','placement_settle_confirmation','placement_photo')}
review.update(source=SOURCE.relative_to(ROOT).as_posix(),sample_count=len(samples),
    loaded_samples=sum(v['phase']=='stage_5_loaded_grasp' for v in samples),
    minimum_measured_voltage_v=min(min(v['voltage_v']) for v in samples),
    peak_measured_current_ma=max(max(abs(x) for x in v['current_ma']) for v in samples),
    maximum_tracking_rad=max(max(v['tracking_rad']) for v in samples),
    latest_write_events=r.get('write_events',[])[-4:],
    first_missing_id_is_not_proof_that_all_later_ids_are_offline=True,
    next_user_task='Cut motor supply and confirm. No retry. No terminal/cable handling while energized.',
    physical_horizontal_90_verified=False,assistant_motor_writes=0)
out=ROOT/'right_hand_rotation_20261028/loaded_comm_stop_000157';out.mkdir(parents=True,exist_ok=True)
(out/'review.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
ready=ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v8_receive_resync/readiness.json'
data=json.loads(ready.read_bytes());data.update(status='power_off_confirmation_required_after_loaded_communication_loss',
    latest_trial=SOURCE.relative_to(ROOT).as_posix(),next_user_command_ready=False,
    pending_user_input='motor power OFF confirmation',all_torque_off_verified=False,
    latest_stop_review=(out/'review.json').relative_to(ROOT).as_posix())
ready.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
             'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name
    if path.exists():
        item=json.loads(path.read_bytes());item.update(current_status=data['status'],
            next_user_command_ready=False,pending_user_input=data['pending_user_input'],
            current_hand_review=ready.relative_to(ROOT).as_posix())
        path.write_text(json.dumps(item,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:review[k] for k in ('primary_motion_error','error','all_torque_off_verified',
    'minimum_measured_voltage_v','peak_measured_current_ma','maximum_tracking_rad','loaded_samples',
    'read_failures','read_recovery_confirmation_events','latest_write_events')},ensure_ascii=False))
