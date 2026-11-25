"""Record the diagnosed user failures and verified v2 EMPTY trial readiness."""
import json
from pathlib import Path
from grasp_approach_plan import GraspApproachPlan,RECOVERY_SOURCES
from thumb_live_start_gate import validate_proof
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'prepared_cube_review_20261022'
fixture=OUT/'empty_approach_v2_fixture'
plan=GraspApproachPlan(fixture/'request.json');proof=validate_proof(plan,fixture/'screen.json')
integration=json.loads((fixture/'windows_integration.json').read_bytes())
assert integration['actual_windows_wsl_subprocess_passed']
first=json.loads((ROOT/RECOVERY_SOURCES[0][0]).read_bytes())
latest=json.loads((ROOT/RECOVERY_SOURCES[1][0]).read_bytes())
last=first['samples'][-1];previous=first['samples'][-2]
diagnosis=dict(first_trial=RECOVERY_SOURCES[0][0],first_failed_phase=first['failed_phase'],
    original_status_unchanged=first['status'],
    fixed_roots=[dict(id=i+1,actual=last['actual'][i],goal=last['goal'][i],
        prior_goal_error_ticks=last['actual'][i]-last['goal'][i],
        actual_change_from_previous_read_ticks=last['actual'][i]-previous['actual'][i]) for i in (7,11)],
    minimum_voltage_v=min(min(s['voltage_v']) for s in first['samples']),
    peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in first['samples']),
    second_trial=RECOVERY_SOURCES[1][0],second_id15_ticks=latest['preflight_state']['position'][14],
    second_write_attempts=len(latest['write_events']),
    first_torque_off_verified=first['all_torque_off_verified'],first_settings_restored=first['settings_restored'],
    second_torque_off_verified=latest['all_torque_off_verified'],
    interpretation='First failure classified existing tracking offsets as movement. Second failure rejected a changed OFF posture before writes.')
ready=dict(status='empty_grasp_approach_v2_ready',plan_version=GraspApproachPlan.PLAN_VERSION,
    launcher='leap_grasp_approach_empty.cmd',tests_passed=79,actual_windows_wsl_subprocess_passed=True,
    static_cases=67,native_ranges_unchanged=True,dynamics=proof['dynamic_sensitivity'],
    fixture_motion_samples=len(plan.plan(plan.source_position())),
    source_is_saved_read_only_pose_not_current_verified_stable=True,
    fresh_start_screen_required_each_run=True,held_joint_guard='last_command_settling_interval_v1',
    additional_held_motion_margin_ticks=20,absolute_tracking_trip_rad=.18,
    current_and_pid_unchanged=True,voltage_policy_unchanged=True,no_goal_or_enable_retry=True,
    diagnosis=diagnosis,hardware_commands_sent_by_assistant=0,physical_grasp_verified=False,
    physical_rotation_verified=False,live_sac=False,
    user_task='Remove cube; leave hand as it is. Same leap_grasp_approach_empty.cmd, confirm v2 banner, one terminal SPACE. Camera records. Stop with Q for contact; send result log.')
(OUT/'empty_grasp_v2_ready.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
p=OUT/'current_status.json';state=json.loads(p.read_text(encoding='utf-8'))
state.update(status=ready['status'],next_user_task=ready['user_task'],next_motion_launcher=ready['launcher'],
    latest_user_trials=diagnosis,empty_grasp_ready_evidence='prepared_cube_review_20261022/empty_grasp_v2_ready.json')
p.write_text(json.dumps(state,ensure_ascii=False,indent=2),encoding='utf-8')
p=ROOT/'right_hand_start_20261019/objective_and_schedule.json';state=json.loads(p.read_text(encoding='utf-8'))
state.update(current_user_task=ready['user_task'],
    current_blocker='v1 stopped from an offset/drift classification bug and a changed OFF posture; v2 verified offline, physical empty approach still pending.',
    latest_user_hardware_trial=dict(timestamp=latest['timestamp'],status=latest['status'],
        evidence=RECOVERY_SOURCES[1][0],all_torque_off_verified=True,write_attempts=0,cube_rotation_verified=False),
    latest_offline_review='prepared_cube_review_20261022/empty_approach_v2_fixture/windows_integration.json',
    next_trial_review='prepared_cube_review_20261022/empty_grasp_v2_ready.json')
p.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
note='''## CURRENT 2026-10-22 01:50/01:51 중단 분석 / 빈손 접근 v2 준비

- 01:50 `grasp_approach_current_empty_20261022_015038_969787`: stage0/1/2 준비 274개 목표 뒤 stage3의 첫 읽기에서 중단. ID8 실제2443/목표2472(-29틱), ID12 실제2447/목표2472(-25틱). 직전 읽기 대비 실제 변화는 각각1틱 이내였다. 따라서 기존 목표 오차를 ‘고정 관절이20틱 넘게 이동’으로 간주한 소프트웨어 기준 오류. 최저4.3V/최대58mA, 토크 OFF 및 설정 복원 확인. 새로운 약지/접근 목표는 아직 보내지 않았고 원본 stopped 기록 유지.
- 01:51 `grasp_approach_current_empty_20261022_015140_792784`: 목표/설정 쓰기0회, 토크 OFF 확인. ID15=2700틱이 예전 admission 범위2593..2660을 벗어나 preflight에서 중단. 앞 OFF 표본에서도 ID15가2634→2647로 계속 변함. 전원을 끈 뒤 자세가 고정된다고 가정하지 않음. settings_restored=False는 이 실행에서 복원할 쓰기를 하지 않은 결과이며, 이전 실행의 복원이 실패했다는 뜻은 아님.
- 새 `EMPTY GRASP APPROACH v2`: held guard는 각 관절의 마지막 목표 변경 **직전** 실측값과 현재 목표 사이 범위에 양쪽20틱을 더해 추가 이탈을 감시. 목표가 변하지 않는 관절의 기준을 단계 전환/다른 관절 이동 때 다시 잡지 않음. 기존 오차와 목표 쪽 정착을 새 이탈로 오인하지 않으며, 실제 추가 이탈의 ID/실제/기준/목표/허용 범위를 기록. 별도0.18rad 추종, 전류, 전압, 실측 진행, read-before-write 및 OFF/복원은 유지. 기존 실행기는 opt-in 없는 경우 원래 동작 유지.
- 시작 자세 접수에 검토된 두 중단 기록 추가. OFF 후 계속 움직이는 손가락 회전/PIP 6관절은 도달한 준비 목표에서 최대512틱(45°) 이내만 접수. 이 접수 범위가 안전 승인이라는 주장은 아니며 실제 안정 OFF 상태의 native67개 ±4틱 경로 검사/동역학/한계 확인을 모두 통과해야만 쓰기가 가능. 엄지·손끝·뿌리와 raw 회전수의 기존 범위는 유지. 초기 값으로 수동 복원하거나 읽기만 다시 하도록 사용자에게 요구하지 않음.
- 저장된 최신 read-only 자세의616개 목표 경로 Windows→WSL 실제 검사 통과. 67개 사례 자기 겹침/원래 범위 위반0, gain0.5/1/1.5 최대 추종0.06804/0.03427/0.02302rad, 자기/받침 겹침0. 저장 표본은 현재 안정 시작각이 아니며 실제 실행 때 새로 검사한다.
- 실제 중단 로그 전체 재생에서 잘못된 stage3 중단 해소 확인. 기준을 다른 관절 변화로 재설정하지 않음, 추가21틱 이탈/순서 오류/큐 변조/진행 없음/Q/쓰기 실패 뒤 재전송 없음·OFF 복원 포함 **79개 테스트 통과**. 검사 대상에 새 guard와 runner 코드 해시도 고정. 조수 실제 하드웨어 접근 없음.
- 사용자 다음 작업: 큐브 빼고 손 모양 그대로 **같은 `leap_grasp_approach_empty.cmd`**, v2 배너 확인 후 터미널 SPACE 한 번. 여전히 빈손 접근 시험이며 큐브를 올리지 않음. 자동 OFF 후 로그/자동 저장 카메라 확인. 목표는 오른손 SAC 수직/수평 실물 회전이며 아직 파지/뒤집기 성공으로 표시하지 않음.
- 근거: 두 실행 폴더 `stop_review.json`, `held_joint_guard.py`, `test_held_joint_guard.py`, `prepared_cube_review_20261022/empty_approach_v2_fixture/windows_integration.json`, `empty_grasp_v2_ready.json`.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;content=p.read_text(encoding='utf-8')
    if note.splitlines()[0] not in content:p.write_text(note+content,encoding='utf-8')
print(json.dumps({k:ready[k] for k in ('status','tests_passed','static_cases','fixture_motion_samples')},indent=2))
