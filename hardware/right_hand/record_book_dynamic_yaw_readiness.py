"""Verify and record the revised production entry, proof and actual EMPTY read."""
import datetime as dt
import hashlib
import json
from book_dynamic_yaw_plan import ROOT,BookDynamicYawPlan,CANDIDATE_SHA,load_candidate
from leap_book_dynamic_yaw_trial import require_offline_review
from leap_book_small_yaw_trial import require_offline_review as old_v1_review

folder=ROOT/'right_hand_rotation_20261024/book20_current_review/dynamic_yaw_fixture_v2'
proof=require_offline_review();plan=BookDynamicYawPlan(folder/'request.json')
tests=json.loads((folder/'device_free_validation.json').read_bytes())
if not tests['successful'] or tests['tests_run']!=95:raise ValueError('Current test suite incomplete')
read_folder=ROOT/'book_settled_off_readonly_20261024_221942_286109'
read=json.loads((read_folder/'result.json').read_bytes());request=json.loads((read_folder/'request.json').read_bytes())
if (read['status']!='stable_current_off_read' or read.get('motion_admission_error')
    or read['servo_write_commands_sent']!=0 or request['off_state']['position']!=plan.source_position().tolist()):
    raise ValueError('Latest confirmed EMPTY state changed')
old_rejected=False
try:old_v1_review()
except RuntimeError:old_rejected=True
if not old_rejected:raise ValueError('Obsolete entry/proof unexpectedly remained valid')
candidate,_=load_candidate();runs=candidate['runs']
record=dict(review_time=dt.datetime.now().astimezone().isoformat(),ready_for_supervised_experiment=True,
    command='leap_book_small_yaw_trial.cmd',production_python='leap_book_dynamic_yaw_trial.py',
    plan_version=plan.PLAN_VERSION,old_v1_certificate_rejected_after_entry_redirect=True,
    latest_confirmed_EMPTY_OFF_source=read_folder.relative_to(ROOT).as_posix(),
    latest_off_ticks_equal_native_replay_source=True,assistant_servo_write_commands_sent=0,
    static_cases=len(proof['static_screens']),dynamic_cases=len(proof['dynamic_sensitivity']),
    device_free_tests=tests,proof_sha256=hashlib.sha256((folder/'screen.json').read_bytes()).hexdigest(),
    candidate_sha256=CANDIDATE_SHA,moving_motor_ids=candidate['moving_motor_ids'],
    maximum_goal_delta_ticks=64,maximum_goal_step_ticks=2,current_cap_ma=450,
    pid_voltage_tracking_current_temperature_and_no_progress_stops_unchanged=True,
    total_goal_samples=len(plan.plan(plan.source_position())),
    delayed_cycle_budget_seconds=(len(plan.plan(plan.source_position()))+40)*.06+2,
    active_cube_sensitivity_cases=7,action_minus_same_closure_control_yaw_deg=candidate['simulated_action_minus_same_closure_control_yaw_deg'],
    predicted_action_yaw_range_deg=[min(s['action']['yaw_deg'] for s in runs),max(s['action']['yaw_deg'] for s in runs)],
    exploration_method='bounded_cross_entropy_search_not_SAC',
    measured_baseline_prescribed_response_is_not_force_prediction=True,
    marker_registration_verified=False,friction_measured=False,physical_rotation_verified=False,
    final_SPACE_records_operator_acknowledgement_not_physical_removal_verification=True,
    fresh_runtime_stable_OFF_read_and_full_native_replay_required=True,
    next_user_action='Same command. EMPTY/FIRST SPACE; PLACE GREEN up ORANGE toward camera/SECOND SPACE; '
        'grasp -> 3s hold -> revised contact/turn 6.3s -> 2s hold; physically REMOVE cube BEFORE THIRD SPACE/OFF.')
(folder/'readiness_review.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
entry=f'''## CURRENT {dt.datetime.now().strftime('%Y-%m-%d %H:%M')}: 실제 파지 각도 반영 작은 회전 v2 실행 준비

- 직전 실물 `book_small_yaw_trial_20261024_215736_594865`는 회전 명령과 정지 구간을 완료했고 OFF 및 설정 복원을 확인했다. 영상1117프레임, 최소4.0V, 최대104mA, 최대 추종0.084369rad다. 큐브 윗면 특징점에는 약2픽셀의 변화가 있지만3도 목표 방향 회전은 입증되지 않았다. 사용자가 마지막 SPACE 전에 큐브를 빼지 않았다는 설명은 `operator_correction.json`에 보존했다. OFF 때의 큐브 움직임은 성공 근거가 아니다. 사용자는 이후 큐브를 뺐다고 확인했다.
- 같은 실제 파지 각도를 사용한 기존 손끝 회전 후보들은 세 손끝 접촉 또는 큐브 자체의 방향 조건을 만족하지 못해 제외했다. 새 `dynamic_clockwise_candidate_v2.json`은 실제 기준 정지20개 샘플의 관절값으로 시작하고 큐브 자체의 동역학 움직임을 점수로 삼아 탐색했다. SAC가 아니며 손을 지정한 궤적으로 움직이는 모델, 미확인 마찰과 배치 가정이므로 실물 성공 증명이 아니다.
- 후보는 접촉 보완26개와 회전100개 목표이며 검지 뿌리 ID8 움직임을 추가했다. 전체 관절 목표는 도달한 파지 기준64틱 이내이고 단계는2틱 이하다. 엄지 뿌리 ID4와 중지 뿌리 ID12, 약지 전체 목표는 고정한다. 전류450mA 및 PID와 실제 전압 및 추종0.18rad 및 전류와 온도 및 무진행 중단 기준은 변경하지 않았다.
- 탐색의7개 마찰과 배치 조건에서 큐브의 전체 예측 변화는 시계방향 약2.18~3.55도, 같은 접촉 보완 뒤 정지한 비교 조건보다 시계방향1.06~2.02도 더 변했다. 이 조건들에서 방향이 같았다는 뜻이며 강건한 정책이나 실제3도 달성을 뜻하지 않는다. 전체 빈손 복귀와 파지 및 회전 경로는201개 정적 조건과9개 동역학 조건을 통과했고, 장치 없는 검사95개도 통과했다.
- 최신 직접 읽기 `book_settled_off_readonly_20261024_221942_286109`는 사용자 큐브 제거 확인 이후의 안정 빈손 OFF이며 토크와 오류 및 watchdog은 모두0이다. 각도는 새 경로의 원본 `220357_083205` 읽기와 모두 동일하다. 모터 쓰기는0회다. 실행은 다시 그때의 안정 OFF 각도와 전체 경로를 검사한다.
- **다음 사용자 작업:** 같은 `leap_book_small_yaw_trial.cmd`를 실행한다. 이 명령은 이제 `leap_book_dynamic_yaw_trial.py`로 연결된다. 첫 SPACE는 빈손, PLACE 안내에서만 같은 위치에 초록 위 및 주황 카메라 방향으로 큐브를 놓고 손을 뺀 뒤 두 번째 SPACE다. 파지 뒤3초 정지,6.3초 접촉 보완 및 작은 회전,2초 정지한다. REMOVE 안내 뒤 큐브를 실제로 먼저 빼고 세 번째 SPACE로 OFF한다. SPACE 기록은 사람의 제거 확인 입력으로 구분하며 실제 카메라 검증이나 큐브 부재 확인으로 기록하지 않는다.
- 근거: `right_hand_rotation_20261024/book20_current_review/dynamic_yaw_fixture_v2/screen.json`, `device_free_validation.json`, `readiness_review.json`. 옛 v1 진입점은 변경된 명령 해시 때문에 장치를 열기 전에 이전 증명을 거부함을 확인했다. 전시 문구는 변경하지 않았다.

'''
for name in ('sac_current_status.md','professor_progress_20261015.md'):
    path=ROOT/name;prior=path.read_text(encoding='utf-8')
    if prior.startswith('## CURRENT'):prior=prior.replace('## CURRENT','## PREVIOUS',1)
    path.write_text(entry+prior,encoding='utf-8')
print(json.dumps({k:v for k,v in record.items() if k!='device_free_tests'},ensure_ascii=False))
