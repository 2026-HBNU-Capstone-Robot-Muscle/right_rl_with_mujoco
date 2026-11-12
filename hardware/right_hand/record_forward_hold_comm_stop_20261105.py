"""Record why the last run stopped without changing any control criteria."""
import json,hashlib
from pathlib import Path
R=Path(__file__).resolve().parent;F=R/'forward_hold_observe_20261105_034338_763669';OFF=R/'resume_off_readonly_20261105_034940_727997/result.json'
def main():
    d=json.loads((F/'result.json').read_bytes());off=json.loads(OFF.read_bytes());events=d['read_recovery_confirmation_events'];failures=d['read_failures']
    assert d['status']=='stopped' and d['failed_phase']=='stage_3_empty_open' and d['all_torque_off_verified'] and d['settings_restored']
    assert off['status']=='readonly_data_saved' and off['all_torque_off_read'] and off['servo_writes']==0 and off['torque_changes']==0 and not off.get('pose_health_validation_errors')
    report=dict(status='complete_health_read_not_confirmed_twice_within_existing200ms',source=F.name,source_sha256=hashlib.sha256((F/'result.json').read_bytes()).hexdigest(),
        stopped_before_cube_placement=True,failed_phase=d['failed_phase'],last_sample_t=d['samples'][-1]['t'],
        initial_group_ids=failures[0]['requested_ids'],first_packet_bytes_received=failures[0]['received_packet_bytes'],recovered_full_read_ms=failures[0]['total_elapsed_ms'],
        clean_snapshots_confirmed=events[0]['clean_reads'],required_clean_snapshots=2,shared_budget_ms=200,confirmation_elapsed_ms=events[0]['elapsed_ms'],remaining_reply_window_ms=failures[-1]['receive_window_ms'],last_packet_received_bytes=failures[-1]['received_packet_bytes'],last_packet_expected_bytes=failures[-1]['expected_packet_bytes'],
        minimum_voltage_v=min(min(s['voltage_v']) for s in d['samples']),maximum_tracking_rad=max(max(s['tracking_rad']) for s in d['samples']),camera_blank_frames=d['camera_recording']['image_signal_blank_frames'],
        all_torque_off_verified=True,settings_restored=True,goal_advance_during_recovery=0,goal_resends=0,
        poststop_readonly=OFF.relative_to(R).as_posix(),poststop_individual_responses=16,poststop_three_snapshots_drift_ticks=off['maximum_drift_ticks'],poststop_torque_off=True,
        host_known_adapter_latency_timer_ms=1,physical_link_root_cause_identified=False,transient_missing_reply_is_not_proof_of_a_broken_motor=True,
        no_health_or_communication_thresholds_changed=True,assistant_motor_writes=0,next_user_command_ready=False)
    (F/'communication_stop_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    for rel in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json','right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        path=R/rel;v=json.loads(path.read_bytes());v.update(status=report['status'],latest_actual_run=F.name,latest_stop_review=(F/'communication_stop_review.json').relative_to(R).as_posix(),latest_readonly=OFF.relative_to(R).as_posix(),next_user_command=None,next_user_command_ready=False,next_user_action_required=False,current_pose_requires_fresh_path_review=True,next_user_action='No immediate rerun requested; partial status reception in shared200ms is under diagnosis. Keep posture unchanged.');path.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
