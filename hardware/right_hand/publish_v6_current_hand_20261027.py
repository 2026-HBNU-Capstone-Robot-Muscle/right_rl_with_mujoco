"""Current handoff after user reshaped fingers and fresh v6 validation passed."""
import datetime as dt,hashlib,json
import numpy as np
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,HorizontalQuarterPlan,LOADED_PATH
from horizontal_quarter_gate_20261027 import validate_review
plan=HorizontalQuarterPlan(REQUEST);proof=validate_review(plan,PROOF)
verification=json.loads((OUT/'verification.json').read_text(encoding='utf-8'))
assert verification['tests_passed']==80 and verification['code_hashes']==proof['code_hashes']
fresh=ROOT/'repeat_off_readonly_20261027_210618_019260'/'result.json'
read=json.loads(fresh.read_text(encoding='utf-8'))
assert read['status']=='off_snapshots_saved' and read['servo_writes']==0 and read['maximum_drift_ticks']<=2
delta=int(np.max(abs(np.asarray(read['snapshots'][-1]['position'])-plan.source_position())))
assert delta<=4
old=ROOT/'horizontal_quarter_trial_20261027_203555_910246'
failed=json.loads((old/'empty_recovery_screen'/'review.json').read_text(encoding='utf-8'))
review=dict(status='current_changed_hand_v6_ready_for_operator',timestamp=dt.datetime.now().astimezone().isoformat(),
    prior_user_trial=str(old.relative_to(ROOT)),prior_trial_motion_started=False,
    prior_failed_static_cases=sum(not r['passed'] for r in failed['static_cases']),
    prior_failed_dynamic_cases=sum(not r['passed'] for r in failed['dynamic_cases']),
    prior_stop_reason='Old low-root posture: model support contact during ring-side stage; some perturbed starts did not select that stage. This old posture has not been approved.',
    user_changed_hand_acknowledged=True,cube_removed_user_confirmed=True,
    latest_off_readonly=str(fresh.relative_to(ROOT)),latest_camera='rewired_contact_readonly_20261027_204752_229664/result.json',
    latest_source_delta_ticks=delta,three_read_drift_ticks=read['maximum_drift_ticks'],
    current_source=str(REQUEST.relative_to(ROOT)),operator_proof=str(PROOF.relative_to(ROOT)),
    current_empty_static_cases=201,current_empty_dynamic_cases=9,tests_passed=80,tests_skipped=0,
    current_initial_self_exception_enabled=proof['initial_self_separation']['enabled'],
    held_thumb_book_policy=proof['held_thumb_book_numerical_policy'],
    commanded_static_thumb_book_depth_mm=max(v['commanded_thumb_book_mm'] for v in proof['empty_recovery']['static_cases']),
    original_hardware_health_stops_unchanged=True,physics_shapes_and_forces_unchanged=True,
    prepared_physical_target_path_unchanged=True,
    loaded_path_sha256=hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest(),
    validated_loaded_static_cases=201,validated_loaded_physics_cases=27,
    nominal_model_yaw_before_open_deg=proof['nominal_model_yaw_deg'],model_yaw_range_deg=proof['model_yaw_range_deg'],
    model_responses_vary=True,physical_horizontal_90_verified=False,physical_loaded_voltage_verified=False,
    usb_latency_ms=1,motor_baud_bps=3000000,assistant_motor_writes=0,
    operator_default_opens_no_devices=True,next_user_command='leap_horizontal_quarter_trial.cmd',
    next_user_command_ready=True,hardware_execution_requires_user_command_and_spaces=True,
    next_user_task='Keep the current fingers; EMPTY firstSPACE. PLACE greenup/orange toward wristID41, handsaway then secondSPACE. Automatic OPEN; REMOVE cube before thirdSPACE. Send done only.',
    pending_user_input=None)
(OUT/'readiness.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
(OUT/'operator_guide.md').write_text('''# 현재 바뀐 손 자세에서 실행

사용자가 손 모양을 바꾼 뒤 관절값과 웹캠을 새로 확인했다. 큐브는 제거됨을 사용자에게 확인했다. 이전 자세로 맞출 필요 없이 현재 빈손 그대로 시작한다.

`leap_horizontal_quarter_trial.cmd` 한 번 실행. 다른 모터 프로그램과 위자드는 닫는다. 첫SPACE는 빈손,PLACE에서 초록위와 주황손목ID41 방향으로 놓고 손을 뺀 뒤 두 번째SPACE. 기록된 큰 수평 회전과 자동OPEN 후 큐브를 빼고 세 번째SPACE로OFF. 저장 결과는 ‘했어’만 보내면 확인한다.

현재 복귀는201기하 조건과9유한PD 조건을 통과했다. 단위/오류 차단 테스트80개 통과. 이전20:35 자세의 받침 간섭은 여전히 승인하지 않았으며 현재 바뀐 자세를 기준으로 한다.

현재 자세에서는 약한PD 조건3개에서 고정 엄지 끝 링크와 책받침 사이 약0.014445mm 접촉이 계산됐다. 이 한 부위의 빈손 초기고정/뿌리이동 계산에만0.02mm 수치 여유를 명시했다. 모든201기하 조건에서는 그 부위의 명령상 겹침이0이어야 하며,엄지목표가 움직이거나 다른 부위이거나 범위를 넘으면 이 여유를 쓸 수 없다. 원래 접촉력과 물성은 유지했다. 이는 물리적 접촉을 측정한 결과는 아니다.

실제 명령 경로와 큰 회전 데이터는 변경하지 않았다. 모터450mA와 기존PID,전압/전류/추종오차 중단 기준도 동일하다. USB지연1ms와3Mbps를 유지한다. 실제 수평90도와 재배선 후 구동 중 전압은 다음 실물 시험에서 확인해야 한다.
''',encoding='utf-8')
header='''현재 바뀐 빈손 자세 반영 완료(2026-10-27). 사용자 큐브 제거 확인,20:47관절값/사진과21:06읽기 재확인(차이1틱 이내). v6복귀201기하+9유한PD 조건과 오류차단 테스트80개 통과. 엄지끝링크-책받침의 고정 빈손 계산 한 부위에만0.02mm 수치 여유를 명시했고,명령상 겹침은0을 요구한다. 물성/접촉력/실제 경로/건강 기준은 유지. USB1ms/3Mbps. 다음 사용자 명령 `leap_horizontal_quarter_trial.cmd` — 현재 손 그대로,빈손첫SPACE→PLACE 초록위/주황손목쪽 후SPACE→자동OPEN 뒤 큐브제거/SPACE. 실제 수평90도는 미검증. 자세한 근거는 `right_hand_rotation_20261027/horizontal_quarter_trial_v6_current/readiness.json`.

<!-- END V6 CURRENT HAND READY 20261027 -->

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;oldtext=path.read_text(encoding='utf-8')
    if '<!-- END V6 CURRENT HAND READY 20261027 -->' not in oldtext:path.write_text(header+oldtext,encoding='utf-8')
for relative in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
                 'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/relative
    if path.exists():
        data=json.loads(path.read_text(encoding='utf-8'));data.update(current_hand_review=str((OUT/'readiness.json').relative_to(ROOT)),
            current_status=review['status'],next_user_command=review['next_user_command'],next_user_command_ready=True,
            physical_horizontal_90_verified=False,pending_user_input=None)
        path.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(status=review['status'],model_cases=210,tests=80,latest_delta_ticks=delta,
    nominal_model_yaw_deg=proof['nominal_model_yaw_deg'],physical_horizontal_90_verified=False,next_user_command=review['next_user_command']),ensure_ascii=False))
