"""Review all three attempts and publish the smaller-reply operator candidate."""
import datetime,hashlib,json
import numpy as np
from four_reply_yaw_plan_20261029 import ROOT,OUT,REQUEST,PROOF,FourReplyYawPlan,BENCH
from four_reply_yaw_gate_20261029 import validate_review
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
p=FourReplyYawPlan(REQUEST);proof=validate_review(p,PROOF)
names=('reference_speed_yaw_20261029_050610_114985','reference_speed_yaw_20261029_050741_131414','reference_speed_yaw_20261029_050854_082496')
attempts=[]
for index,name in enumerate(names):
    path=ROOT/name/'result.json';r=read(path);failures=r.get('read_failures',[])
    confirmations=r.get('read_recovery_confirmation_events',[]);samples=r.get('samples',[])
    events=[]
    for e in confirmations:
        indices=e.get('sample_indices',[]);events.append(dict(phase=e['phase'],elapsed_ms=e['elapsed_ms'],
            first_sample_t=samples[indices[0]]['t'] if indices and indices[0]<len(samples) else None,
            complete_healthy_reads=e['clean_reads'],status=e['status']))
    attempt=dict(run=name,result_sha256=sha(path),status=r['status'],failed_phase=r.get('failed_phase'),error=r.get('error'),
        torque_OFF_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],write_events=len(r['write_events']),
        settings_restoration_required=bool(r['write_events']),user_explicitly_reported_accidental_cancel=index==1,
        stop_signal_in_log=r.get('error')=='User stop',loaded_motion_started=any(s['phase']=='stage_5_loaded_grasp' for s in samples),
        yaw_started=any(s['phase']=='stage_7_horizontal_yaw' for s in samples),read_failures=len(failures),recoveries=events,
        last_failure=failures[-1] if failures else None,camera=r.get('camera_recording'),
        minimum_voltage_v=min((min(s['voltage_v']) for s in samples),default=None),
        peak_tracking_rad=max((max(s['tracking_rad']) for s in samples),default=None))
    if index==2:
        assert failures[-1]['retry_requested'] is False and failures[-1]['elapsed_ms']<200
        attempt.update(repeated_receive_failure_rate_limit_is_stop_cause=True,not_a200ms_deadline_expiry=True,
            current_cube_removed_confirmed_by_user=True)
    attempts.append(attempt)
review=dict(status='three_reference_speed_attempts_reviewed',attempts=attempts,new_yaw_evidence_from_these_attempts=False,
    current_cube_removed_confirmed_by_user=True,latest_torque_OFF_verified=True,latest_settings_restored=True,assistant_motor_writes=0)
(OUT/'three_attempt_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
fresh_path=ROOT/'fast_off_readonly_20261029_052248_574397/result.json';fresh=read(fresh_path)
assert fresh['status']=='off_snapshots_saved' and fresh['maximum_drift_ticks']<=2 and not fresh['servo_writes'] and not fresh['torque_changes']
delta=int(np.max(abs(np.asarray([s['position'] for s in fresh['snapshots']])-p.source_position())))
assert delta<=4,'Current pose requires a new model review'
for name,digest in {'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
    'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
    'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
    'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d'}.items():assert sha(ROOT/name)==digest,name
now=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat()
ready=dict(status='four_reply_reference_speed_operator_trial_ready',timestamp=now,proof=PROOF.relative_to(ROOT).as_posix(),proof_sha256=sha(PROOF),
    current_readonly=fresh_path.relative_to(ROOT).as_posix(),current_pose_difference_ticks=delta,empty_recovery_cases=210,
    original_loaded_contact_tracking_cases=27,loaded_geometry_cases=201,regression_checks_passed=11,
    test_source_sha256=sha(ROOT/'test_four_reply_yaw_20261029.py'),read_only_benchmark=proof['four_reply_readonly_evidence'],
    full_health_reply_group_size=4,reply_size_bytes=356,previous_reply_size_bytes=704,
    shared_health_budget_ms=200,normal_full_healthy_snapshots_after_failure=2,retry_rate_limit_unchanged=True,
    current_gain_voltage_tracking_temperature_stops_unchanged=True,OFF_verification_and_configuration_readbacks_unchanged=True,
    target_vectors_and_period_unchanged=True,initial_orientation='GREEN up; ORANGE toward WRIST/ID41',
    nominal_motion_seconds=proof['nominal_motion_seconds'],active_motion_limit_s=300,link_stability_under_torque_not_yet_demonstrated=True,
    motion_source_of_interruption_not_yet_identified=True,assistant_motor_writes=0)
(OUT/'readiness.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
guide='''작게 나눠 읽는 수평 회전 시험

기존 파일을 반복 실행할 필요는 없습니다. leap_four_reply_yaw.cmd를 한 번 실행합니다.
손가락과 받침은 현재 그대로 두고 큐브는 손과 카메라 시야 밖에 둡니다.
터미널에서 첫 SPACE를 누릅니다. 카메라 창은 미리보기입니다.
PLACE CUBE가 뜬 뒤에만 초록 위/주황 손목ID41 쪽으로 놓고 손을 뺀 뒤 터미널에서 두 번째 SPACE를 누릅니다.
수평 회전과 자동 OPEN을 기다립니다. 명목 동작 약3분14초이며 배치 시간은 별도입니다.
REMOVE가 뜨면 큐브를 빼고 터미널에서 세 번째 SPACE를 누릅니다.
Q/Esc/Ctrl+C나 카메라 창 닫기는 중단으로 처리됩니다. 끝나거나 멈추면 했어라고 알려주면 됩니다.

직전 세 기록은 회전까지 진행하지 않았습니다.
첫 번째는PLACE 대기 중 User stop 신호, 두 번째는쓰기0개/동작 전 사용자 취소,
세 번째는PLACE 대기 중 반복 부분 수신에 따른 기존 재시도 횟수 제한으로 중단됐습니다.
세 번째는200ms 만료가 원인이 아니며, 토크OFF와 원래 설정 복구는 확인됐습니다.

새 파일은 같은 회전 목표와75ms 주기를 사용하고, 건강 상태만4개씩356바이트 응답으로 나눠 받습니다.
16개 모터 모두의 새 데이터, 순서/개별 오류/CRC 확인과200ms 안의 정상 상태2회 확인을 유지합니다.
모터 설정이나EEPROM, 통신속도, 힘, 온도/전압/추종 및 반복 수신 실패 기준을 바꾸지 않았습니다.
토크OFF 확인과OFF 설정 복구 확인도 이전 구현을 사용합니다.
카메라 미리보기와JPEG 저장을 켠OFF 비교에서8개/4개 방식 모두300회씩 정상 수신됐습니다.
호스트에서만CRC를 깨뜨린3건도 두 번의 전체 새 상태로56~59ms에 회복됐습니다.
구동 중 개선은 아직 확인되지 않았고, 회전 성공도 이번 영상으로 확인해야 합니다.
'''
(OUT/'실행안내.md').write_text(guide,encoding='utf-8')
command='& "'+str(ROOT/'leap_four_reply_yaw.cmd')+'"'
for filename in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/filename;r=read(path)
    r.update(status=ready['status'],current_status=ready['status'],timestamp=now,
        next_command=command,next_user_command=command,next_user_command_ready=True,next_physical_command_ready=True,
        next_user_action_required=True,next_user_action='New four-reply command once: EMPTY terminal SPACE; PLACE GREEN up/ORANGE wristward SPACE; REMOVE SPACE',
        next_physical_command_kind='FOUR_REPLY_GREEN_UP_REFERENCE_SPEED_YAW',next_step='Same completed-speed trajectory; smaller strict health replies',
        trajectory='EMPTY -> PLACE GREEN up/ORANGE wristward -> same75ms yaw -> OPEN -> REMOVE -> OFF',
        cube_orientation_at_place=ready['initial_orientation'],nominal_motion_seconds=ready['nominal_motion_seconds'],active_motion_limit_s=300,
        source_off_readonly=ready['current_readonly'],latest_readonly=ready['current_readonly'],latest_pose_difference_from_review_ticks=delta,
        proof=ready['proof'],proof_sha256=ready['proof_sha256'],software_checks_passed=11,full_health_reply_group_size=4,
        latest_three_attempts_review=review,next_yaw_readiness=ready,current_cube_removed_confirmed_by_user=True,
        latest_reference_speed_status='three attempts stopped before loaded motion',motors_off_verified=True,original_settings_restored_now=True,
        physical_combined_rotation_verified=False,assistant_motor_writes=0,assistant_motor_or_setting_writes=0)
    path.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
notice=('2026-10-29 05:22 현재 단계: 세 번의 기준 속도 실행은 회전 전 중단됐고,4개씩 응답을 읽는 버전 준비. '
    '05:06 기록은PLACE 대기 중 User stop 신호,05:07 기록은쓰기0개/동작 전 취소(사용자 실수 취소 확인), '
    '05:08 기록은PLACE 대기 중 반복 부분 수신으로 기존3회/10초 재시도 제한에 걸렸다. '
    '마지막 실패 자체는40.366ms에 발생했고 재시도가 허용되지 않았으므로200ms 만료로 설명하면 안 된다. '
    '마지막 토크OFF/설정 복구와 사용자 큐브 제거가 확인됐다. '
    'READ-only A/B 비교는 실제 카메라 미리보기와JPEG 저장을 켠채8/4/4/8개 순서로150회씩, '
    '각방식300/300회 정상 수신됐다. 정상 중앙값은8개18.35ms/4개20.45ms였다. '
    '4개 방식은호스트CRC 손상3건 후16개 전체 새 상태2회를56~59ms에 확인했다. '
    '모터OFF에서둘다정상이므로4개 방식의 구동 중 개선이나 원인 확정은 주장하지 않는다. '
    '새 leap_four_reply_yaw.cmd는동일2032개 loaded 목표와75ms주기를 사용한다. '
    '응답을704에서356바이트씩으로 나눴으며 기존엄격CRC/ID/개별오류 파서,200ms,정상2회,재시도횟수,힘/건강 기준은 유지한다. '
    'OFF 검증과설정확인은기존구현이다. 현재복귀201정적+9동적 통과,11개 새 버전 테스트 통과. '
    '05:22 현재각도와 모델시작점 차이최대'+str(delta)+'틱,명목193.5초/기존300초. '
    '새버전은아직실물로실행하지않았다.\n\n')
for filename in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/filename;previous=path.read_text(encoding='utf-8')
    if notice not in previous:path.write_text(notice+previous,encoding='utf-8')
print(json.dumps(dict(status=ready['status'],current_pose_difference_ticks=delta,nominal_motion_s=ready['nominal_motion_seconds'],command=command,assistant_motor_writes=0),ensure_ascii=False))
