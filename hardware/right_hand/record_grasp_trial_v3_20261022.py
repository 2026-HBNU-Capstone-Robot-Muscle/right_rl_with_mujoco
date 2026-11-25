"""Keep actual tracking failures separate from the revised opening evidence."""
import json
from pathlib import Path
from grasp_place_plan import ROOT,GraspPlacePlan,LOADED_PHASE,endpoints
from thumb_live_start_gate import validate_proof

out=ROOT/'prepared_cube_review_20261022'
folders=('grasp_trial_v3_fixture','grasp_trial_v3_after_stop_fixture','grasp_trial_v3_first_stop_fixture')
screens=[]
for name in folders:
    folder=out/name;plan=GraspPlacePlan(folder/'request.json');proof=validate_proof(plan,folder/'screen.json')
    rows=plan.plan(plan.source_position());empty=[q for phase,q in rows if phase!=LOADED_PHASE]
    screens.append(dict(fixture=name,static_cases=len(proof['static_screens']),
        native_ranges_unchanged=proof['native_ranges_unchanged'],motion_goals=len(rows),
        nominal_motion_seconds=len(rows)*.05,preplacement_max_id12_goal=max(int(q[11]) for q in empty),
        max_unapproved_self_overlap_mm=max(r['self_escape_max_unapproved_overlap_mm'] for r in proof['static_screens']),
        max_unapproved_native_range_violation_rad=max(r['unapproved_native_range_violation_rad'] for r in proof['static_screens']),
        dynamics=proof['dynamic_sensitivity'],source_fixture_only=True))
trials=[json.loads((ROOT/name/'stop_review.json').read_text(encoding='utf-8')) for name in
    ('cube_grasp_trial_20261022_023904_356527','cube_grasp_trial_20261022_024003_945968')]
task='큐브를 빼고 손은 그대로 두고 같은 leap_cube_grasp_trial.cmd 실행. v3 배너 확인 후 첫 터미널 SPACE. 약지 분리/엄지 여유/열기 완료와 PLACE까지 큐브는 올리지 않기. PLACE 후 초록 위·주황 카메라 쪽으로 놓고 손 제거/두 번째 SPACE. REMOVE 후 큐브 제거/세 번째 SPACE. 걸리거나 이상 소리면 Q. 로그 전달.'
ready=dict(status='direct_opening_cube_trial_v3_ready',plan_version=GraspPlacePlan.PLAN_VERSION,
    launcher='leap_cube_grasp_trial.cmd',user_task=task,user_confirmed_ring_tip_palm_contact=True,
    observed_id12_stall_not_a_voltage_or_start_pose_gate=True,
    physical_root_cause_established=False,previous_simulator_failed_to_predict_physical_id12_stall=True,
    removed_closed_waypoint_before_placement=True,recovery_order=['ring_contact_release','thumb_clearance','open_preparation'],
    preplacement_id12_goal=2472,loaded_id12_goal=int(endpoints()[1][11]),
    loaded_approach_goals_unchanged=True,checkpoint_settle_reads=20,checkpoint_max_error_ticks=64,
    planned_motion_and_settle_budget_s=50,live_motion_timeout_s=55,
    current_pid_voltage_tracking_stops_unchanged=True,tests_passed=97,test_duration_s=31.105,
    measured_id12_plateau_regression_test_passed=True,windows_wsl_fixtures=screens,
    latest_user_trials=trials,hardware_commands_sent_by_assistant=0,
    physical_grasp_verified=False,physical_rotation_verified=False,live_sac=False,
    limitation='Geometry and native dynamics are not calibrated enough to predict every real contact, cable load or static friction. The revised physical opening still requires an operator-run observation.')
(out/'grasp_trial_v3_ready.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
p=out/'current_status.json';state=json.loads(p.read_text(encoding='utf-8'))
state.update(status=ready['status'],next_user_task=task,next_motion_launcher=ready['launcher'],
    latest_user_tracking_stops=trials,next_trial_evidence='prepared_cube_review_20261022/grasp_trial_v3_ready.json',
    user_confirmed_ring_tip_palm_contact=True,physical_id12_stall_root_cause_established=False)
p.write_text(json.dumps(state,ensure_ascii=False,indent=2),encoding='utf-8')
p=ROOT/'right_hand_start_20261019/objective_and_schedule.json';state=json.loads(p.read_text(encoding='utf-8'))
state.update(current_user_task=task,
    current_blocker='Actual ID12 stalled during restoring the closed grasp in empty recovery. v3 removes that waypoint and opens after ring release/thumb clearance. Physical root cause and new recovery still require observation; cube grasp/rotation unverified.',
    latest_user_hardware_trial=trials[-1],latest_user_tracking_trials=trials,
    latest_offline_review='prepared_cube_review_20261022/grasp_trial_v3_fixture/windows_integration.json',
    next_trial_review='prepared_cube_review_20261022/grasp_trial_v3_ready.json')
p.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

note='''## CURRENT 2026-10-22 02:39/02:40 ID12 실제 추종 실패 / 닫힌 경유 제거 v3

- 사용자 보고: 약지 끝과 손바닥이 실제로 닿아 있음. 이제 이는 모델 추정만이 아닌 사용자 확인 접촉이다. 두 실행은 시작 검사·native 화면을 통과한 뒤 실제 빈손 복귀 도중 중단했으며, 과거 자세 gate/저전압 중단과 구분한다. 원본 stopped 기록은 유지한다.
- 02:39: ID12 마지막 목표2575/실제2477, 최종 오차0.1503rad. 약20개 읽기 동안 실측 거의 고정이고 전류가128→171mA로 증가해 실측 진행 guard에서 중단. 전체 최저4.2V/최대171mA, OFF/복원 확인.
- 02:40: ID12 목표2578/실제2460, 오차0.1810rad/207mA로 추종 한계에서 중단. 전체 최저4.1V, OFF/복원 확인. 통신 실패/전류 상한/저전압 실패가 아니다. 로그와 처음/마지막 원본 사진 검토. 사진만으로 특정 케이블/하우징/엄지 접촉을 정확한 원인으로 확정하지 않는다.
- v2가 빈손 초기화에서 먼저 닫힌 파지 목표(ID12=2613)를 복원하고 다시 열게 만든 경유는 불필요했다. 이 경로가 모델에서는 통과했어도 실물에서 ID12 부하를 예측하지 못한 사실을 기록한다. v3는 닫힌 경유를 삭제하고, ID15를 먼저 펴 약지 끝 접촉을 분리(엄지 뿌리는 필요한 작은 범위 복귀만), 엄지에최대128틱 여유, 나머지 열린 준비 순서로 변경. 이 초기화에서 ID12를2472보다 높이지 않음. 큐브를 놓고 확인한 후322개 loaded 접근 목표만 종전과 동일하게 수행한다.
- 열린 준비 뒤20개 정착 읽기를 수행하고16관절 오차가64틱 이내여야 PLACE callback에 들어간다. 과거 자세 일치 대신 열린/닫힌 검토 지점 중 하나까지512틱 이내인 시작을 접수하여 열다가 중단한 자세도 다시 검사할 수 있게 함. 전체 계획 이동/정착50초 예산을 쓰기 전에 확인하며, 기존55초 live 제한/전압/전류/PID/추종/쓰기 재시도 금지/OFF 복원은 변경하지 않음.
- 두 실행의 실제 preflight 및 마지막 OFF, 총3개 저장 자세를 실제 Windows→WSL로 검사: 각각67개 ±4틱 경로와gain0.5/1/1.5 동역학 통과. 초기 약지 접촉은 그대로 보고하며 지정된 초기 접촉의 단조 분리 조건만 적용. 이후 새 자기 접촉/허용되지 않은 범위 위반0, 받침 겹침0. 새 경로 명목 이동43.00/41.20/42.95초, 정착 읽기는 별도 약2초. 현재 안정 자세가 검증됐다는 의미는 아니며 실행 때 다시 읽어 검사한다.
- **97개 테스트 통과**(31.105초). 실측을 반영해 ID12를2460틱에서 못 올라가게 만든 fake plant에서도 PLACE까지 도달하며 초기화 목표가2472이하임을 검증. 초기 접촉 먼저 해제/엄지 우선 여유/닫힌 경유 없음/열린 상태의 재접수/PLACE·REMOVE·쓰기 실패·Q·OFF 복원 등을 확인. 카메라/모터 포트 없는 CLI 확인 완료. 조수 실제 모터 명령0.
- 아직 새 초기화의 실물 성공과 파지/회전은 미검증이다. 정확한 ID12 정지 원인도 확정되지 않았다. 초기화는 이후 회전 실행에서도 반복되므로 테스트 종료를 이유로 고장을 무시하거나 중단 한계를 올리지 않는다.
- 다음 사용자 작업: 큐브 없이 같은 `leap_cube_grasp_trial.cmd`, v3 배너 확인 후 첫 SPACE. PLACE까지 빈손. 초록 위·주황 카메라 쪽 배치 후 손 제거/두 번째 SPACE. REMOVE 후 큐브 제거/세 번째 SPACE. 이상 접촉·소리는 Q. 로그 전달. 같은 v2를 반복하거나 자세를 수동 복원할 필요 없음.
- 근거: 두 실행 폴더 `stop_review.json`, `grasp_recovery_plan.py`, `prepared_cube_review_20261022/grasp_trial_v3_ready.json`, 3개 `grasp_trial_v3*_fixture/windows_integration.json`, `test_grasp_place_trial.py`.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;content=p.read_text(encoding='utf-8')
    if note.splitlines()[0] not in content:p.write_text(note+content,encoding='utf-8')
print(json.dumps(dict(status=ready['status'],tests=ready['tests_passed'],fixtures=len(screens),
    preplacement_id12_goal=ready['preplacement_id12_goal'],physical_root_cause_established=False),indent=2))
