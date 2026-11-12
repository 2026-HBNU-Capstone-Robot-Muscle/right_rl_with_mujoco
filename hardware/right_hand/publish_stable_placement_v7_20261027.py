"""Record the measured stop, completed checks and next operator task."""
import datetime as dt,json
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,HorizontalQuarterPlan,hashes
from horizontal_quarter_gate_20261027 import validate_review
proof=validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
verification=json.loads((OUT/'verification.json').read_bytes())
assert verification['tests_passed']==97 and verification['code_hashes']==hashes()
source=ROOT/'repeat_off_readonly_20261027_224731_974099/result.json'
off=json.loads(source.read_bytes());state=off['snapshots'][-1]
evidence=proof['stable_placement_review'];weak_path=ROOT/'right_hand_rotation_20261027/open_settle_221539/weaker_root_matrix/review.json'
weak=json.loads(weak_path.read_bytes())
ready=dict(status='stable_measured_placement_v7_ready',timestamp=dt.datetime.now().astimezone().isoformat(),
    latest_trial='horizontal_quarter_trial_20261027_221539_291945/result.json',
    latest_stop='stable ID12 error67 exceeded scalar placement limit64; no PLACE or loaded motion',
    latest_trial_torque_off_verified=True,latest_trial_settings_restored=True,
    latest_off_readonly=source.relative_to(ROOT).as_posix(),off_position_ticks=state['position'],
    off_pose_drift_ticks=off['maximum_drift_ticks'],responding_motor_ids=list(range(1,17)),
    off_voltage_range_v=[min(state['voltage'])/10,max(state['voltage'])/10],
    current_source=REQUEST.relative_to(ROOT).as_posix(),operator_proof=PROOF.relative_to(ROOT).as_posix(),
    placement_policy=evidence['policy'],tests_passed=97,current_empty_static_cases=201,current_empty_dynamic_cases=9,
    placement_geometry_cases=207,additional_weaker_response_cases=27,all_additional_weaker_cases_passed=weak['all_passed'],
    required_weaker_response_cases=9,weaker_response_yaw_range_deg=evidence['weaker_response']['yaw_range_deg'],
    weaker_response_is_calibration=False,loaded_goals_unchanged=True,
    current_initial_self_exception_enabled=proof['initial_self_separation']['enabled'],
    held_thumb_book_numerical_allowance_enabled=proof['held_thumb_book_numerical_policy']['enabled'],
    physical_horizontal_90_verified=False,assistant_motor_writes=0,hardware_health_thresholds_unchanged=True,
    next_user_command='leap_horizontal_quarter_trial.cmd',next_user_command_ready=True,pending_user_input=None,
    next_user_task='Leave hand as it is. Close Wizard. EMPTY first SPACE. PLACE: green up, orange toward wrist/ID41; hands away, second SPACE. After automatic OPEN, remove cube and third SPACE. Send done only.')
(OUT/'readiness.json').write_text(json.dumps(ready,indent=2)+'\n',encoding='utf-8')
(OUT/'operator_guide.md').write_text('''# 안정된 준비 자세에서 수평 회전 시험

손가락은 현재대로 두고, 위자드와 다른 모터 프로그램을 닫은 뒤 `leap_horizontal_quarter_trial.cmd`를 한 번 실행한다.

1. 큐브를 손과 카메라 밖에 두고 터미널에서 첫 SPACE를 누른다.
2. PLACE CUBE가 나오면 초록 면을 위, 주황 면을 손목 ID41 쪽으로 놓는다. 손을 뺀 뒤 두 번째 SPACE를 누른다.
3. 회전 목표와 자동 OPEN이 끝나면 큐브를 뺀 뒤 세 번째 SPACE로 토크를 끈다. 결과는 ‘했어’만 보내면 된다.

22:15 실행은 큐브 배치 전 중지 뿌리 ID12의 안정된 오차67틱이 기존64틱을 넘어서 멈췄다. 10회 측정에서 최대 변화1틱, 최저전압4.8V, 최대전류144mA였고 OFF와 설정 복구가 확인됐다. v7은 ID12만72틱, 다른 관절64틱을 적용하며 10회 연속 새 측정, 최대 변화2틱, 정확한 목표 읽기, 모델 각도 범위 안의 실제 위치를 요구한다. 전류, PID, 전압 및 추종오차 차단과 기존2742개 실린 큐브 목표는 그대로다.

현재 꺼진 자세의 복귀210조건, 관측 자세와72틱 경계207조건이 통과했다. 더 약한 뿌리 응답을 적용한 추가27물리조건도 기존 접촉과 추종오차 기준을 통과했다. 그중 약한 응답9조건의 회전각은 약61~84도였다. 이는 실측 처짐을 참고한 가정이며 정확한 모델 보정이나 실물90도 성공을 의미하지 않는다. 다음 영상에서 실제 회전각과 유지 여부를 검토한다.

제어와 오류 처리 테스트97개 통과. 모델 형상, 모터 대응, 원래 시퀀스, 원래 각도 제한, 전시 문구 파일을 변경하지 않았다.
''',encoding='utf-8')
marker='<!-- END STABLE PLACEMENT V7 20261027 -->'
header='''준비 자세의 안정성 검사 수정 완료(2026-10-27): 22:15 실행은 큐브 배치 전 안정된 ID12 오차67틱이 기존64틱을 넘어 중단됐고, 토크OFF와 설정복구는 정상이다. v7은 ID12만72틱, 다른 관절64틱을 적용하고10회 새 측정의 변화가2틱 이내여야 PLACE를 허용한다. 현재 자세 복귀210조건, 실측 경계207조건, 추가 약한 응답27조건과 오류 처리 테스트97개 통과. 모터 힘, PID, 전압 및 추종오차 기준, 실린 큐브 경로는 그대로다. 추가 약한 응답 모델은 약61~84도이며 실물 수평90도는 아직 검증 전이다. 다음은 같은 `leap_horizontal_quarter_trial.cmd` 한 번: 빈손 첫SPACE, PLACE에서 초록 위와 주황 손목ID41쪽으로 놓고 손을 뺀 뒤 SPACE, 자동OPEN 후 큐브를 빼고 마지막SPACE. ‘했어’만 보내면 저장 파일을 검토한다. 현재 근거: `right_hand_rotation_20261027/horizontal_quarter_trial_v7_stable_placement/readiness.json`.

'''+marker+'\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_text(encoding='utf-8')
    if marker not in old:path.write_text(header+old,encoding='utf-8')
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
             'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name
    if path.exists():
        data=json.loads(path.read_bytes());data.update(current_status=ready['status'],
            current_hand_review=(OUT/'readiness.json').relative_to(ROOT).as_posix(),
            next_user_command=ready['next_user_command'],next_user_command_ready=True,pending_user_input=None,
            physical_horizontal_90_verified=False)
        path.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
oldpath=ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v6_current/readiness.json'
if oldpath.exists():
    old=json.loads(oldpath.read_bytes());old.update(status='superseded_by_v7_stable_placement',
        next_user_command_ready=False,superseded_by=(OUT/'readiness.json').relative_to(ROOT).as_posix())
    oldpath.write_text(json.dumps(old,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(status=ready['status'],tests=97,motor_writes=0,next_user_command=ready['next_user_command']),ensure_ascii=False))
