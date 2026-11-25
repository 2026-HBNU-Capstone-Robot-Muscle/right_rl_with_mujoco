"""Record the failed model-depth gate and short separation certificate."""
import json
from pathlib import Path
from grasp_place_plan import ROOT,GraspPlacePlan,LOADED_PHASE
from thumb_live_start_gate import validate_proof

out=ROOT/'prepared_cube_review_20261022'
stop=json.loads((ROOT/'cube_grasp_trial_20261022_025316_928334/stop_review.json').read_text(encoding='utf-8'))
assert stop['write_events']==0 and stop['all_torque_off_verified']
cases=[]
for name in ('grasp_trial_v4_fixture','grasp_trial_v4_prior_fixture','grasp_trial_v4_after_stop_fixture','grasp_trial_v4_first_stop_fixture'):
    folder=out/name;plan=GraspPlacePlan(folder/'request.json');proof=validate_proof(plan,folder/'screen.json')
    contacts=[s for s in proof['static_screens'] if s['initial_self_contacts_mm']]
    threshold=(min(s['contact_clear_pip_goal_ticks'] for s in contacts)-plan.CONTACT_RELEASE_MARGIN_TICKS
               if contacts else None)
    rows=plan.plan(plan.source_position())
    release=[q for phase,q in rows if phase=='stage_0_empty_recover'][-1]
    if threshold is not None:assert release[14]<=threshold
    cases.append(dict(fixture=name,static_cases=len(proof['static_screens']),
        native_ranges_unchanged=proof['native_ranges_unchanged'],motion_goals=len(rows),
        nominal_motion_seconds=len(rows)*.05,
        maximum_geometric_release_pip_travel_ticks=max(s['contact_clear_pip_travel_ticks'] for s in proof['static_screens']),
        measured_release_pip_threshold_ticks=threshold,
        first_stage_pip_goal=int(release[14]),
        first_stage_total_pip_travel_ticks=int(plan.source_position()[14]-release[14]),
        maximum_unapproved_overlap_mm=max(s['self_escape_max_unapproved_overlap_mm'] for s in proof['static_screens']),
        dynamics=proof['dynamic_sensitivity']))
task='큐브 없이 같은 leap_cube_grasp_trial.cmd, v4 배너 확인 후 첫 SPACE. RING RELEASE CHECK에서 약지 끝이 손바닥과 떨어진 것을 눈으로 확인한 경우만 두 번째 SPACE로 빈손 열기. 닿아 있거나 안 보이면 Q. PLACE 이후 큐브 배치/사람 손 제거/세 번째 SPACE. REMOVE 이후 큐브 제거/네 번째 SPACE. 로그 전달.'
ready=dict(status='contact_release_observation_cube_trial_v4_ready',plan_version=GraspPlacePlan.PLAN_VERSION,
    launcher='leap_cube_grasp_trial.cmd',source_stop=stop,user_task=task,
    replaced_initial_model_overlap_cap_mm=2.0,
    model_overlap_is_not_a_measured_physical_penetration=True,
    separation_certificate_max_pip_travel_ticks=64,
    certificate_limits_travel_to_first_separation_not_entire_first_stage=True,
    release_measurement_margin_ticks=16,operator_release_confirmation_required=True,
    new_or_increasing_self_contact_still_rejected=True,native_model_ranges_unchanged=True,
    current_pid_voltage_tracking_stops_unchanged=True,loaded_approach_goals_unchanged=True,
    tests_passed=103,test_duration_s=41.419,windows_wsl_saved_cases=cases,
    hardware_commands_sent_by_assistant=0,physical_release_verified=False,
    physical_grasp_verified=False,physical_rotation_verified=False,live_sac=False,
    limitation='Approximate hand geometry does not measure contact pressure. The new release must still be observed on hardware; failed release never advances to thumb/opening or cube motion.')
(out/'grasp_trial_v4_ready.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
p=out/'current_status.json';state=json.loads(p.read_text(encoding='utf-8'))
state.update(status=ready['status'],next_user_task=task,next_motion_launcher=ready['launcher'],
    latest_user_screen_stop=stop,next_trial_evidence='prepared_cube_review_20261022/grasp_trial_v4_ready.json')
p.write_text(json.dumps(state,ensure_ascii=False,indent=2),encoding='utf-8')
p=ROOT/'right_hand_start_20261019/objective_and_schedule.json';state=json.loads(p.read_text(encoding='utf-8'))
state.update(current_user_task=task,
    current_blocker='v3 stopped with no writes at an arbitrary 2mm initial model-overlap cap. v4 requires short monotone separation, actual encoder threshold, and operator confirmation. Physical release and cube rotations remain unverified.',
    latest_user_hardware_trial=dict(source='cube_grasp_trial_20261022_025316_928334/result.json',
        status='stopped_before_writes',write_attempts=0,all_torque_off_verified=True),
    latest_offline_review='prepared_cube_review_20261022/grasp_trial_v4_fixture/windows_integration.json',
    next_trial_review='prepared_cube_review_20261022/grasp_trial_v4_ready.json')
p.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
note='''## CURRENT 2026-10-22 02:53 모터 쓰기 전 모델 접촉 gate 중단 / 분리 확인 v4

- `cube_grasp_trial_20261022_025316_928334`: 설정/목표 쓰기0회, 전체 토크 OFF 확인. 최초 OFF 세 읽기는1틱 이내로 안정. 중단은 시작 잠금/ID12 실물 정지/전압 실패가 아니라 시뮬레이션의 초기 접촉2mm 상한이었다. settings_restored=False는 복원할 쓰기가 없었던 실행이다. 원본 failed screen과 stopped 결과는 보존.
- 이번 약지 회전 ID14가2196틱으로 변한 자세에서 약지 끝/손바닥 모델 겹침 nominal2.169mm, ±4틱67개 사례 최대2.505mm. 실제 눌림 깊이를 측정한 값이 아니다. 모든 목표의 접촉 깊이는 단조 감소하고 새 접촉/범위/받침 위반은 없으며, 가장 늦게30개 목표 안에 접촉이 사라짐. 동역학 세 강도 분리는1.85/1.82/1.79초, 추종 최대0.05466rad. 원래2mm 상한만으로 이 상태를 거부했다.
- v4는 이 근사 모델 깊이의 단일 상한을 ‘짧은 분리 검증’으로 대체. 지정된 약지 끝/손바닥 초기3쌍만 대상. 깊이 단조 감소, 새 접촉·깊어짐 금지, 최초 분리까지 약지 PIP64틱(5.625°) 이내/40개 목표 이내, native 동역학의 초기 깊이 초과 금지 및2.5초 내 분리 조건을 모두 요구. 모델의 관절/제어 범위·질량·마찰·접촉 도형을 수정하지 않음. 이64틱은 첫 접촉 해제에 필요한 각도 상한이며, 첫 준비 단계 전체 이동량의 상한은 아니다. 전체 첫 단계는 종전대로 안전 여유가 있는 약지 목표까지 이동하며 이번 표본에서218틱이다.
- 그 첫 단계 이후 **엄지/나머지 열기 전**에 실제 ID15가67개 사례의 가장 보수적인 분리 위치보다16틱 더 편 위치인지 읽어 확인. 통과하면 `RING RELEASE CHECK`에서 멈춰 사용자가 약지 끝과 손바닥의 실제 분리를 관찰한 SPACE 확인을 해야만 나머지 빈손 열기가 가능. 아직 닿거나 안 보이면Q. `ring_released.jpg` 자동 저장. 기다릴 때도 건강 상태와 실제 분리 기준을 계속 확인하고90초 제한. 확인 누락/Q/기준 밖/쓰기 실패는 다음 목표 없이OFF/복원. SPACE가 배치/제거 동작으로 연달아 넘어가지 않도록 기존 키 해제/새 누름 조건 유지.
- 현재와 이전 실제 OFF 자세4개 각각67개 정적 시작 사례·세 강도 동역학 Windows→WSL 실제 검사 통과. 초기 겹침을0으로 바꾸지 않고 원래값 그대로 보고. 허용되지 않은 이후 자기 겹침/받침 위반0. 물리 교정이나 실물 분리 성공으로 표시하지 않음. 기존4개 준비 경로도 실제 호출로 해시/결과 검증. 전류450mA/PID/전압/추종0.18rad/실측 진행/55초 이동 제한/자동 재전송 금지는 유지. loaded 접근 목표 변경 없음.
- **103개 테스트 통과**(41.419초). 분리 SPACE/Q/확인 누락/목표 큐 유지/새 사진/짧은 분리 travel·threshold 관계 검증/64틱 초과·누락·NaN 거부와 기존 ID12 plateau 회귀 포함. 실패 검사 메시지에는 단순 log 경로 외에 실패 항목도 표시하도록 개선. 조수 실제 카메라·모터 접근 없음.
- 다음 사용자 작업은 SPACE4회: 큐브 없이 같은 `leap_cube_grasp_trial.cmd`, v4 → 첫 SPACE. 약지 끝이 손바닥에서 떨어졌을 때만 분리 확인 두 번째 SPACE(큐브는 계속 밖). PLACE 후 초록 위·주황 카메라 쪽 배치/손 제거/세 번째 SPACE. REMOVE 후 큐브 제거/네 번째 SPACE. 분리 안 되거나 확인이 안 되면Q. 로그 전달. 새 실물 분리/파지/회전은 아직 미검증이다.
- 근거: 중단 폴더 `stop_review.json`, `prepared_cube_review_20261022/grasp_trial_v4_ready.json`, 4개 `grasp_trial_v4*_fixture/windows_integration.json`, `test_grasp_place_trial.py`, `test_screen_failure_diagnostics.py`.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;content=p.read_text(encoding='utf-8')
    if note.splitlines()[0] not in content:p.write_text(note+content,encoding='utf-8')
print(json.dumps(dict(status=ready['status'],tests=ready['tests_passed'],
    cases=[{k:c[k] for k in ('fixture','maximum_geometric_release_pip_travel_ticks',
                           'measured_release_pip_threshold_ticks','first_stage_pip_goal')} for c in cases]),indent=2))
