"""Update current handoff after successful user-applied USB latency change."""
import datetime as dt,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'right_hand_rotation_20261027'/'usb_baud_latency_review'
applied=json.loads((folder/'applied_review.json').read_text(encoding='utf-8'))
assert applied['status']=='usb_latency_applied_reading_faster_v5_proof_valid'
assert applied['all_torque_off_verified'] and applied['unchanged_operator_proof_valid']
camera=ROOT/'rewired_contact_readonly_20261027_192743_277914'/'result.json'
current=json.loads((folder/'review.json').read_text(encoding='utf-8'))
current.update(status='driver_latency_1ms_applied_and_measured',current_driver_latency_ms=1,driver_latency_changed=True,
    application_review='right_hand_rotation_20261027/usb_baud_latency_review/applied_review.json',
    median_after_ms=applied['median_after_ms'],read_time_reduction_percent=applied['read_time_reduction_percent'],
    unchanged_v5_operator_proof_valid=True,next_user_command='leap_horizontal_quarter_trial.cmd',
    next_user_action=applied['next_task'],windows_admin_step_complete=True,
    latest_camera_review=str(camera.relative_to(ROOT)),latest_camera_hand_empty_visually_verified=True,
    previous_windows_error=current.pop('windows_error',None))
(folder/'review.json').write_text(json.dumps(current,indent=2)+'\n',encoding='utf-8')
(folder/'review.md').write_text('''# USB 통신 설정 적용 완료 2026-10-27

사용자가 관리자 실행 명령을 완료했다. COM7 FTDI LatencyTimer를16ms에서1ms로 변경했고, 포트를 다시 열었을 때 바로 효과가 확인돼 장치 재시작은 필요하지 않았다. 모터16개의 통신 속도3Mbps와 토크OFF는 그대로다.

전체 상태100회 읽기에서 오류0회. 읽기 중앙값26.0036ms →14.9918ms(42.35% 단축),적용 후95백분위15.0568ms,최대15.0763ms,최저 정지 전압5.0V였다. 관절값은 v5의 검토된 출발점과 최대1틱 차이다. 현재 설정 유지.

v5의 저장된 모델 승인과 코드 해시를 다시 확인했고 통과했다. 최신 웹캠5장과 전후 토크OFF 읽기도 확보해 빈손임을 확인했다. 손가락 자세를 다시 맞출 필요는 없다.

다음 사용자 작업: 위자드를 닫고 `leap_horizontal_quarter_trial.cmd` 한 번 실행. 빈손 첫SPACE,PLACE에서 초록 위/주황 손목ID41쪽으로 놓고 손을 뺀 뒤 두 번째SPACE. 자동OPEN 후 큐브를 빼고 세 번째SPACE로OFF. 결과는 ‘했어’만 보내면 된다.

이번 변경은 통신 대기 시간을 줄인 것이다. 재배선 후 모터 구동 중 전압과 실제 수평90도 회전은 다음 실물 시험에서 확인해야 한다. 모터 목표,힘,건강 기준과 v5 경로는 바꾸지 않았다.
''',encoding='utf-8')
header='''USB 통신 개선 적용 완료(2026-10-27): COM7 LatencyTimer1ms,모터16개3Mbps 유지. 상태 읽기100회 오류0,중앙값26.00→14.99ms(42.35% 단축),최대15.08ms,정지 최저5.0V. 관리자 작업 완료. 현재 자세는 검토 출발점과 최대1틱 차이이고 빈손 웹캠도 확인. v5 모델 승인/코드 해시 재확인 통과. 다음 사용자 명령은 `leap_horizontal_quarter_trial.cmd`. 빈손 첫SPACE→PLACE에서 초록위/주황손목쪽 후SPACE→자동OPEN 후 큐브제거/SPACE. 실제 수평90도와 구동 중 전압은 미검증.

<!-- END USB BAUD REVIEW 20261027 -->

'''
marker='<!-- END USB BAUD REVIEW 20261027 -->'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_text(encoding='utf-8')
    if marker in old[:2500]:old=old.split(marker,1)[1].lstrip('\n')
    path.write_text(header+old,encoding='utf-8')
ready_path=ROOT/'right_hand_rotation_20261027'/'horizontal_quarter_trial_v5_rewired'/'readiness.json'
ready=json.loads(ready_path.read_text(encoding='utf-8'))
ready.update(usb_latency_ms=1,usb_latency_applied_review=str((folder/'applied_review.json').relative_to(ROOT)),
    latest_camera_review=str(camera.relative_to(ROOT)),usb_latency_read_median_ms=applied['median_after_ms'],
    pending_user_input=None,next_user_command='leap_horizontal_quarter_trial.cmd',next_user_command_ready=True)
ready_path.write_text(json.dumps(ready,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(status=current['status'],next_user_command=current['next_user_command'],model_approval_verified=True,
    hand_motion_sent_by_assistant=False),ensure_ascii=False))
