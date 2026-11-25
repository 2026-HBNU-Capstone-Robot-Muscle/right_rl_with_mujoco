"""Publish measured READ-only recovery and the source-bound next trial."""
import json,hashlib
from retained_slow_yaw_plan_20261029 import ROOT,OUT,PROOF,REQUEST,hashes
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
proof=read(PROOF);ready=read(OUT/'readiness.json')
assert proof['code_hashes']==hashes() and ready['review_sha256']==sha(PROOF)
run=ROOT/'retained_slow_yaw_20261029_012108_810204';result=read(run/'result.json')
record=dict(status='stopped_before_cube_placement',path=run.relative_to(ROOT).as_posix(),result_sha256=sha(run/'result.json'),
    error=result['error'],phase=result['failed_phase'],cube_placement_reached=False,physical_horizontal_rotation_executed=False,
    all_torque_off_verified=result['all_torque_off_verified'],settings_restored=result['settings_restored'],
    min_voltage_v=4.9,camera_usable_frames=270,camera_blank_frames=0,
    source_archive=(run/'offline_review/execution_source_v1').relative_to(ROOT).as_posix(),
    limitation='Old quiet loop left no byte/drain trace; delayed serial data and scheduler delay cannot be distinguished retrospectively')
(run/'offline_review/stop_review.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
protected={
 'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b',
 'right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716',
 'exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0',
 'camera_gap_motion_runner_20261026.py':'7efbf942c38e290dd59d24162f34ff3bba83480e004f31d1a06a13fa7b96de7d'}
for path,digest in protected.items():assert sha(ROOT/path)==digest,path
checks=dict(status='recovery_v2_checks_passed',transport_regression_tests=16,operator_tests=9,original_health_confirmation_tests=12,
    tests_total=37,test_sources={n:sha(ROOT/n) for n in ('test_combined_quiet_reserve_20261029.py',
        'test_retained_slow_yaw_operator_20261029.py','test_precise_two_full_20261028.py')},
    actual_readonly_recovery=proof['readonly_recovery_evidence'],
    fresh_recovery_model_cases=210,unchanged_isolated_loaded_hand_cases_reused=9,
    protected_hashes=protected,drive_health_stops_unchanged=True,ON_exact_goal_consistency_unchanged=True,
    physical_rotation_verified=False,assistant_motor_writes=0)
(OUT/'verification.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
guide='''수평 구간 재확인: 통신 복구 v2

손가락과 받침은 현재 상태로 둡니다. USB, 기존5V 전원, Streaming Webcam을 유지합니다.

1. 큐브를 손 밖에 두고 leap_retained_slow_yaw.cmd를 실행합니다. 터미널에서 첫 SPACE는 빈손으로 누릅니다.
2. PLACE CUBE 뒤 빨강이 위, 초록이 손목 ID41 쪽으로 놓습니다.
   위치 사진: ../../pitch_slow_yaw_20261029_004532_520638/cube_transfer_hold.jpg
3. 큐브가 스스로 놓이면 사람 손을 뺀 뒤 두 번째 SPACE를 누릅니다.
4. 잡기와 느린 수평 회전 뒤 자동 OPEN을 기다립니다. REMOVE CUBE가 뜨면 큐브를 빼고 세 번째 SPACE로 OFF합니다.

움직임은 명목241.65초이며 약4~5분입니다. Q/Esc/Ctrl+C로 중단합니다.
이번에는 뒤집기를 생략합니다. 수평 회전량은 저장 영상으로 확인합니다.

v2는 수신 정리와 정상 전체 상태2회 확인을 같은200ms 안에 마치게 합니다.
별도의20ms 정리 제한과100ms 예측 예약을 없앴고, 전체200ms와 정상 상태2회 요건은 유지합니다.
관절 범위,2틱 목표 간격,450mA, 기존PID, 전압/온도/전류/추종 기준, 읽기 재시도 횟수와 빈도 제한도 유지합니다.
목표나 설정 명령을 다시 보내지 않습니다. 미완성 상태와 오류 있는 응답으로 다음 목표를 진행하지 않습니다.
실물 모터를 움직이지 않는 실제 읽기와 고의적인 호스트 오류/지연 검사, 원래 건강 검사 테스트를 통과했습니다.
정지 상태 통신 검사가 움직임 중 통신 안정성까지 보증하지는 않습니다. 새 복구 기록은 수신 바이트와 시간을 남깁니다.

현재는 감독하에 확인하는 실험이며 정확90도나 연속2축 완주를 확인한 상태는 아닙니다.
넓은 모델 조건에서의 실패 기록은 보존했습니다. 실험이 끝나면 했어라고 알려주면 저장 파일을 확인합니다.
'''
(OUT/'실행안내.md').write_text(guide,encoding='utf-8')
command='& "'+str(ROOT/'leap_retained_slow_yaw.cmd')+'"'
for name in ('right_hand_rotation_20261028/two_axis_connection/current_progress.json',
    'right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    p=ROOT/name;r=read(p)
    r.update(status='ready_retained_slow_yaw_read_recovery_v2_20261029',current_status='ready_retained_slow_yaw_read_recovery_v2_20261029',
        latest_retained_slow_yaw_run=record,next_command=command,next_physical_command_ready=True,
        next_step='Same command with revised READ recovery; EMPTY first SPACE, PLACE RED up/GREEN wrist ID41, second SPACE, REMOVE then third SPACE',
        next_ready=ready,next_verification=checks,next_review=PROOF.relative_to(ROOT).as_posix(),
        latest_readonly='prefix638_off_readonly_20261029_013538_791704/result.json',
        physical_horizontal_rotation_this_run=False,read_recovery_version='shared200ms_quiet_and_two_complete_QPC_v2_20261029')
    for k in list(r):
        if 'next' in k.lower() and 'command' in k.lower():r[k]=command if isinstance(r[k],str) else r[k]
    p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
notice=('2026-10-29 통신 복구 v2 준비 완료. 01:21 실행은 큐브를 놓기 전 빈손 준비에서 수신 정리20ms 제한으로 멈췄다. '
    '토크 OFF와 설정 복구를 확인했고 전압 최저4.9V와 카메라270장 모두 정상이었다. '
    '정지 상태 전체 상태160회 읽기는 성공했다. 실제 READ에 고의적인 호스트 오류와 지연을 넣은 검사에서 '
    '정상 상태2회 확인이 약144/166/184ms에 성공했고, 큰 지연은 원래200ms 제한에서 중단됐다. '
    '별도의20ms 정리 제한과100ms 예측 예약을 없애되 같은200ms 안에 수신 정리와 정상 상태2회를 마치도록 수정했다. '
    '모터 건강 기준과 ON 상태 정확한 목표 대조, 재시도 횟수와 빈도 제한은 유지한다. '
    'OFF 상태에서는 목표 레지스터가 현재 각도와 함께1틱 변하는 것이 관찰되어 OFF 진단을 분리했고 ON 검사는 수정하지 않았다. '
    '회귀16개, 실행 절차9개, 기존 건강 확인12개 테스트와 현재 자세 복귀 모델210개 조건을 통과했다. '
    '새 명령은 이전과 같은 leap_retained_slow_yaw.cmd이며 첫 SPACE는 빈손, PLACE 뒤 빨강 위/초록 손목 ID41, '
    '두 번째 SPACE로 수평 구간, REMOVE 뒤 큐브 제거와 세 번째 SPACE다. 명목241.65초와 기존300초 한도를 사용한다. '
    '정지 상태 검사만으로 움직임 중 통신의 영구적인 안정성을 인증하지 않는다. 수평 회전은 아직 실행 전이다.\n\n')
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8')
    if notice not in old:p.write_text(notice+old,encoding='utf-8')
print(json.dumps(dict(status='v2_ready_published',command=command,tests_passed=37,fresh_recovery_cases=210,
    nominal_seconds=ready['nominal_motion_seconds'],assistant_motor_writes=0),ensure_ascii=False))
