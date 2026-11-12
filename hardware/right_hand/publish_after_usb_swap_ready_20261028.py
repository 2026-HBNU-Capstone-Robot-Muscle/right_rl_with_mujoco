"""Record bounded clean observations and the renewed user-started trial."""
import datetime as dt,hashlib,json
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,HorizontalQuarterPlan,hashes
from horizontal_quarter_gate_20261027 import validate_review
def read(path):return json.loads(path.read_bytes())
def save(path,value):path.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode('utf-8'))
source=ROOT/'usb_swap_readonly_20261028_044716_809381/result.json';raw=source.read_bytes();r=json.loads(raw)
assert r['status']=='swapped_usb_readonly_observation_complete' and not r['errors'] and r['blank_frames']==0
assert r['reads']==787 and r['camera_frames']==r['usable_frames']==1173 and len(r['individual_checks'])==16
validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
v=read(OUT/'verification.json');assert v['tests_passed']==150 and v['code_hashes']==hashes()
dest=ROOT/'right_hand_rotation_20261028/after_usb_swap_044716';dest.mkdir(exist_ok=True)
report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='swapped_usb_60s_clean_v11_operator_trial_ready',
    user_recalled_prior_webcam_blackout_as_reason_for_previous_port_change=True,
    user_reported_current_usb_exchange_connected=True,
    source=source.relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(raw).hexdigest(),
    device='COM7',expected_usb_serial_number='FTBENPSAA',baud=1000000,responding_ids=list(range(1,17)),
    duration_s=r['camera_observed_duration_s'],read_period_s=.075,complete_fresh_snapshots=787,read_errors=0,
    camera_frames=1173,camera_usable_frames=1173,camera_blank_frames=0,
    camera_maximum_frame_gap_s=r['maximum_frame_gap_s'],
    median_read_ms=r['median_read_ms'],maximum_read_ms=r['maximum_read_ms'],
    first_and_last_saved_scene_images_inspected=True,
    hand_and_cube_observation='Images show robot hand with no cube; palm/roots/thumb visible.',
    last_observed_torque_off=True,last_observed_error_watchdog_zero=True,
    operator_request=REQUEST.relative_to(ROOT).as_posix(),operator_proof=PROOF.relative_to(ROOT).as_posix(),
    current_pose_model_conditions=210,software_tests_passed=150,
    physical_loaded_rotation_after_usb_exchange_verified=False,
    long_duration_or_under_motion_link_stability_proven=False,
    underlying_hardware_or_host_root_cause_proven=False,
    hardware_health_limits_unchanged=True,loaded_goals_unchanged=True,
    assistant_motor_or_setting_writes=0,motion_retry_on_hold=False,
    prior_observed_quarter='right_hand_rotation_20261028/completed_quarter_012305/physical_review.json',
    spare_usb_data_cable_available=False,keep_current_usb_port_layout=True,
    next_user_command='leap_horizontal_quarter_trial.cmd',next_user_command_ready=True,pending_user_input=None,
    next_user_action='Keep current USB ports and hand posture. EMPTY first SPACE. PLACE green up/orange toward wrist ID41, hands clear then second SPACE. After automatic OPEN remove cube, third SPACE for OFF. Send done only.')
save(dest/'review.json',report);save(OUT/'readiness.json',report)
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
             'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name;value=read(path)
    value.update(status=report['status'],current_status=report['status'],timestamp=report['timestamp'],
        current_hand_review=(dest/'review.json').relative_to(ROOT).as_posix(),
        horizontal_loaded_execution_ready=True,next_user_command=report['next_user_command'],
        next_user_command_ready=True,next_user_action=report['next_user_action'],pending_user_input=None,
        motion_retry_on_hold=False,all16_at_1Mbps=True,keep_current_usb_port_layout=True)
    save(path,value)
old_path=ROOT/'right_hand_rotation_20261028/verified_1m_usb_isolation/review.json'
old=read(old_path);old.update(current_status=report['status'],superseded_by=(dest/'review.json').relative_to(ROOT).as_posix(),
    current_action_no_longer_usb_swap=True);save(old_path,old)
marker='<!-- END SWAPPED USB 60S CLEAN READY 20261028 -->'
header='''2026-10-28 USB 교환 후 재개 준비: 사용자는 원래 웹캠 검은 화면으로 포트를 바꿨던 것이라고 설명하고 이번 포트 교환을 연결했다고 알렸다. 같은 USB 어댑터 FTBENPSAA는 COM7로 유지됐고, 16개 모두 1Mbps, 모드5, 토크 OFF, 오류와 워치독 0을 개별 확인했다. 모터를 움직이지 않고 기존 75ms 주기에 맞춰 카메라와 상태 읽기를 1분 관찰했다. 전체 상태 787회 오류0, 카메라1173프레임 전부 정상 화면, 검은 화면0, 프레임 최대 간격 약0.079초, 읽기 중앙값32.708ms/최대36.961ms였다. 처음과 마지막 저장 사진에서도 빈손과 화면을 확인했다. 이 기간에 문제가 재현되지 않은 것이며 장시간이나 구동 중 안정성 또는 원인 해결을 입증한 것은 아니다.

이때 읽은 현재 자세를 새 출발 근거로 삼아 복귀 모델210조건과 제어 테스트150개를 통과했다. 이전의 성공한 회전 목표, 전류와PID, 전압과 추종오차 등 건강 기준은 유지했다. 모터 구동과 추가 설정 쓰기는 하지 않았다. 현재 USB 포트를 유지하고 같은 `leap_horizontal_quarter_trial.cmd`로 실제 동작 확인 한 번을 진행한다. 손은 그대로, 빈손 첫 SPACE, PLACE에서 초록 위/주황 손목ID41 방향 후 손을 빼고 두 번째 SPACE, 자동OPEN 후 큐브제거/세 번째 SPACE. ‘했어’만 보내면 저장 기록을 검토한다. 1Mbps 설정 프로그램은 다시 실행할 필요가 없다. 실행 보류를 해제하고 현재 자세/소스/정상 관찰에 맞춘 심사 근거로 교체했다. 현재 근거 `right_hand_rotation_20261028/after_usb_swap_044716/review.json`.

'''+marker+'\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_bytes()
    if marker.encode('utf-8') not in old:path.write_bytes(header.encode('utf-8')+old)
(OUT/'operator_guide.md').write_bytes(('''# 현재 USB 연결에서 회전 관측

현재 포트 배치를 유지한다. 1분 읽기 관찰은 모터 오류0과 정상 카메라 화면만 기록했지만, 장시간 및 구동 중 안정성은 이번 실행에서 확인한다. 1Mbps 설정 프로그램은 다시 실행하지 않는다.

1. 손가락은 그대로, 큐브를 밖에 두고 `leap_horizontal_quarter_trial.cmd`를 실행한다. 미리보기와 손 주변을 확인한 뒤 빈손으로 첫 SPACE를 누른다.
2. PLACE 때 엄지와 중지 사이 뿌리 쪽에 초록 위, 주황 손목 ID41 방향으로 큐브를 놓는다. 사람 손을 뺀 뒤 두 번째 SPACE를 누른다.
3. 회전과 자동 OPEN이 끝나면 큐브를 빼고 세 번째 SPACE를 누른다. ‘했어’만 보내면 된다.

원래 목표 경로와 건강 기준을 유지한다. 각 다음 목표는 16개 모터의 새 정상 상태가 확인돼야 전송한다. 통신은 1Mbps에서 한 개씩 읽어 전체 상태를 모으며, 전체200ms 제한과 복구 후 연속 두 정상 읽기를 유지한다.
''').encode('utf-8'))
assert source.read_bytes()==raw and v['code_hashes']==hashes()
validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
print(json.dumps(dict(status=report['status'],bus_reads=787,bus_errors=0,camera_frames=1173,black_frames=0,
    model_conditions=210,tests=150,motor_writes=0,next_user_command=report['next_user_command'])))
