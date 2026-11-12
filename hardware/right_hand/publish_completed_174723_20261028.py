"""Saved-run review of the completed precise-clock/margin trial; no devices."""
import datetime as dt,hashlib,json
import numpy as np
import calibrated_quarter_plan_20261028 as p
ROOT=p.ROOT;run=ROOT/'calibrated_quarter_trial_20261028_174723_390729';out=ROOT/'right_hand_rotation_20261028/completed_calibrated_174723'
r=json.loads((run/'result.json').read_bytes());measure=json.loads((out/'same_marker_rotation_review.json').read_bytes())
frames=json.loads((run/'camera/frames.json').read_bytes())['frames'];events=r.get('read_recovery_confirmation_events',[])
assert r['status']=='calibrated_quarter_observation_complete' and r['all_torque_off_verified'] and r['settings_restored'] and r['cube_removal_confirmed']
assert len(r.get('read_failures',[]))==2 and all(v['recovered'] for v in r['read_failures'])
assert len(events)==2 and all(e['status']=='confirmed' and e['clean_reads']>=2 and e['elapsed_ms']<200 and e['goal_writes']==0 and e['goals_advanced']==0 for e in events)
held=measure['comparisons']['placement_to_turn']['twist_total_tilt_p05_median_p95'][1][0]
opened=measure['comparisons']['placement_to_open']['twist_total_tilt_p05_median_p95'][1][0]
review=dict(status='completed_full_trial_with_near90_hold_and_bounded_read_recovery',source_run=run.relative_to(ROOT).as_posix(),
    source_result_sha256=hashlib.sha256((run/'result.json').read_bytes()).hexdigest(),plan_version=r['plan_version'],timestamp=r['timestamp'],
    preparation_grasp_yaw_open_and_removal_complete=True,commanded_yaw_goals=638,
    all_torque_off_verified=True,settings_restored=True,cube_removal_confirmed=True,
    camera_frames=len(frames),unusable_frames=sum(f.get('image_signal_usable') is not True for f in frames),
    communications_failure_events=2,all_read_failures_recovered=True,
    recovery_confirmation_elapsed_ms=[e['elapsed_ms'] for e in events],
    full_healthy_snapshot_counts=[e['clean_reads'] for e in events],no_goal_advance_or_resend_during_recovery=True,
    min_voltage_v=min(min(s['voltage_v']) for s in r['samples']),peak_abs_current_ma=max(max(abs(v) for v in s['current_ma']) for s in r['samples']),
    max_tracking_rad=max(max(s['tracking_rad']) for s in r['samples']),
    held_camera_relative_yaw_estimate_deg=held,after_open_camera_relative_yaw_estimate_deg=opened,
    apparent_release_unwind_deg=held-opened,angle_reference='cube placement before loaded grasp',
    exact_robot_frame90_certified=False,near90_held_rotation_observed=True,
    static_palm_feature_check=measure['static_palm_feature_check'],small_marker_planar_ambiguity_and_angle_uncertainty_remain=True,
    first_v4_full_trial_completed=True,other_rotation_axes_verified=False,release_angle90_verified=False,
    communications_root_cause_permanently_resolved=False,physical_read_recovery_demonstrated_in_this_trial=True,
    no_live_SAC=True,assistant_motor_writes=0,original_recording_modified=False,
    video_preview_4x=(out/'cube_rotation_preview_4x.mp4').relative_to(ROOT).as_posix(),video_realtime=(out/'full_run_realtime.mp4').relative_to(ROOT).as_posix(),
    repeated_operator_trial_not_requested=True)
(out/'physical_review.json').write_bytes((json.dumps(review,indent=2)+'\n').encode())
state=dict(status='latest_trial_complete_saved_video_reviewed_no_repeat_requested',timestamp=dt.datetime.now().astimezone().isoformat(),
    latest_run=review['source_run'],latest_run_time='2026-10-28 17:47 Asia/Seoul',
    physical_review=(out/'physical_review.json').relative_to(ROOT).as_posix(),
    recorded_command_sequence_complete=True,held_angle_approx_deg=round(held,1),released_angle_approx_deg=round(opened,1),
    read_errors_recovered=2,all_torque_off_verified=True,settings_restored=True,cube_removal_confirmed=True,
    repeat_current_command_required=False,user_next_action='None for confirming this completed run. Do not repeat just to resend a log.',
    remaining_goal_work=['Keep the target orientation through release or handover.','Verify other required rotation axes and full cube-solving integration.'],
    video_preview=review['video_preview_4x'],motor_writes_by_assistant=0)
(p.OUT/'latest_physical_review.json').write_bytes((json.dumps(state,indent=2)+'\n').encode())
archive=out/'prior_pointers';archive.mkdir(exist_ok=True)
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name;saved=archive/path.name
    if path.exists() and not saved.exists():saved.write_bytes(path.read_bytes())
    path.write_bytes((json.dumps(state,indent=2)+'\n').encode())
text='2026-10-28 17:47 실물 실행 완료 확인: 사용자가 보완판 실행 여부를 물었고 새 기록 `calibrated_quarter_trial_20261028_174723_390729`를 확인했다. 약지 초기 모으기, 빈손 복귀, 큐브 배치/파지,638개 회전 목표, 고정, 자동OPEN, 큐브 제거와 토크OFF 및 설정복구까지 완료했다. 저장2162프레임에 검은 화면0. 통신 오류2건은 목표 재전송/이동 없이 완전한 정상 상태두번을200ms 안에 확인해 복구했고 끝까지 진행했다. 통신 오류0 또는 근본 원인 영구 해결로 기록하지 않는다.\n\n같은 초록ID5 카메라 기준 추정은 파지 고정 중 약88도, 손을 편 뒤 약82도다. 정확한 로봇 좌표계90도 인증은 아니고 작은 마커의 불확실성이 남는다. 보완판의 수평90도 근처 회전과 실제 읽기 복구/전체 완주는 관찰됐으나 다른 회전축, 해제 후90도 유지와 전체 큐브풀기는 완료되지 않았다. 실제 영상은 `right_hand_rotation_20261028/completed_calibrated_174723/cube_rotation_preview_4x.mp4` 및 `full_run_realtime.mp4`에 보존했다. 사용자는 이번 명령을 다시 실행할 필요가 없으며 모터 구동이나 설정 쓰기를 추가로 보내지 않았다. 현재 근거 `right_hand_rotation_20261028/completed_calibrated_174723/physical_review.json`.\n\n<!-- END COMPLETED174723 PHYSICAL REVIEW 20261028 -->\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_text(encoding='utf-8-sig')
    if '<!-- END COMPLETED174723 PHYSICAL REVIEW 20261028 -->' not in old:path.write_bytes((text+old).encode())
print(json.dumps({k:review[k] for k in ('status','held_camera_relative_yaw_estimate_deg','after_open_camera_relative_yaw_estimate_deg','recovery_confirmation_elapsed_ms','min_voltage_v','peak_abs_current_ma','max_tracking_rad','all_torque_off_verified','settings_restored')}))
