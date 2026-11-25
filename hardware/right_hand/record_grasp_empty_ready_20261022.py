"""Record offline verification and the next user-owned EMPTY trial."""
import json
from pathlib import Path
from grasp_approach_plan import GraspApproachPlan,target
from thumb_live_start_gate import validate_proof
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'prepared_cube_review_20261022'
fixture=OUT/'empty_approach_fixture'
plan=GraspApproachPlan(fixture/'request.json')
proof=validate_proof(plan,fixture/'screen.json')
integration=json.loads((fixture/'windows_integration.json').read_bytes())
assert integration['actual_windows_wsl_subprocess_passed']
# Fix the unused helper-generated fixture's provenance label as well. It is
# never a live capture; the proof is pinned to request.json independently.
unused=fixture/'fresh_off_request.json'
if unused.exists():
    doc=json.loads(unused.read_bytes());doc['origin']='saved_relaxing_off_fixture_not_live'
    unused.write_text(json.dumps(doc,indent=2),encoding='utf-8')
ready=dict(status='empty_grasp_approach_ready_for_user_trial',launcher='leap_grasp_approach_empty.cmd',
    tests_passed=74,windows_wsl_integration_passed=True,static_cases=len(proof['static_screens']),
    maximum_static_self_overlap_mm=max(s['max_self_overlap_mm'] for s in proof['static_screens']),
    dynamic_sensitivity=proof['dynamic_sensitivity'],native_ranges_unchanged=True,
    actual_start_measured_and_screened_again_at_runtime=True,fixture_is_saved_relaxing_off_sample=True,
    fixture_motion_samples=len(plan.plan(plan.source_position())),
    target_ticks=target().tolist(),goal_step_max_ticks=2,requested_period_s=.05,on_timeout_s=55,
    original_current_and_pid_preserved=True,cube_permitted=False,live_sac=False,
    physical_grasp_verified=False,physical_rotation_verified=False,hardware_executed_by_assistant=False,
    user_task='Cube removed. Leave fingers as they are; secure base; show thumb/index/middle with webcam. Run launcher and press terminal SPACE once. New empty approach, hold, automatic OFF. Send log; video is saved automatically.')
(OUT/'empty_grasp_ready.json').write_text(json.dumps(ready,indent=2),encoding='utf-8')
p=OUT/'current_status.json';state=json.loads(p.read_text(encoding='utf-8'))
state.update(status=ready['status'],next_user_task=ready['user_task'],next_motion_launcher=ready['launcher'],
    empty_grasp_ready_evidence='prepared_cube_review_20261022/empty_grasp_ready.json')
p.write_text(json.dumps(state,ensure_ascii=False,indent=2),encoding='utf-8')
p=ROOT/'right_hand_start_20261019/objective_and_schedule.json';state=json.loads(p.read_text(encoding='utf-8'))
state.update(current_user_task=ready['user_task'],
    current_blocker='Need physical empty approach tracking/clearance observation before a loaded grasp trial; cube reorientation and live SAC remain unverified.',
    latest_offline_review='prepared_cube_review_20261022/empty_approach_fixture/windows_integration.json',
    next_trial_review='prepared_cube_review_20261022/empty_grasp_ready.json')
p.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
note='''## CURRENT 2026-10-22: 뿌리 홈 기울어짐 반영한 새 빈손 접근 점검 준비

- 다음 사용자 작업: **큐브를 빼고 손 모양 그대로 `leap_grasp_approach_empty.cmd` 실행, 터미널 SPACE 한 번.** 현재 OFF 안정 읽기/전체 경로 검사 → 3초 뒤 준비 회복 → 약지 방향 여유 → 검지·중지·엄지의 새 접근 → 유지 → 자동 OFF/설정 복원. 끝날 때까지 큐브를 올리지 않는다. 로그와 자동 카메라 자료로 실제 추종/접촉을 검토한다. 같은 배치 사진을 반복하는 작업이 아니다.
- 파지 후보 계산에서 검지/중지 목표점을 둘 다 같은 먼 면 중앙에 잡으면 원래 관절/받침 제약 아래 도달이 불충분했다. 서로 다른 면/위쪽 모서리를 향하는 후보로 변경하고, 중지-약지 간섭을 피하려고 약지 회전 ID14에32틱(2.81°) 여유를 두는 경로를 검사했다. 이 후보의 명목 관절 목표/중간 경로 자기 겹침과 받침 겹침0. 7개 합성 큐브 시작 동역학에서도 자기/받침 겹침0, 추종 약0.0307rad. 일부 시작은 큐브가 약14.5mm 이동하거나 세 손끝 파지가 성립하지 않아 loaded rollout이나 회전 성공으로 보고하지 않음.
- 새 실행기는 기존 proof-before-write, 정확한 목표 큐, read-before-every-goal, OFF/설정 복원 코드를 재사용. 450mA/PID/전압/추종/실측 진행/55초 조건 변경 없음. 목표2틱 이하·50ms 처리 포함, 새 약지/접근 단계에 고정 관절 감시와 진행 검사 적용. 카메라/마커 gate 없이 빈손 영상 저장; Cube PLACE callback 없음. 새 목표 출처 및 코드 해시를 실제 시작 proof에 고정.
- 저장된 마지막 OFF 표본(현재/정지 상태라는 주장이 아님)의513개 목표 경로: 시작 ±4틱67개 사례 자기/받침/원래 관절·제어 범위 위반0. Native gain0.5/1/1.5 추종 최대0.06722/0.03391/0.02278rad, 자기/받침 겹침0. 실제 실행은 fresh OFF 값으로 다시 검사해야 쓰기가 가능. 중지/엄지 형상과 외부 받침의 근사는 실물 교정이 아니며 케이블은 모델에 없음.
- Windows→WSL 실제 호출과 해시 검증 통과, 기존 세 준비 경로도 실제 subprocess 재검증. 신규 단계의 Q/진행 없음/고정 약지 이탈/목표 쓰기 실패 후 재전송 없음·OFF 복원/변조된 출처 차단 등 포함 **74개 테스트 통과**. 지연 모형에서도55초 안에 유지/복원 완료. CLI check-only는 카메라·모터·시뮬레이터를 열지 않음을 확인. 조수는 실제 모터를 구동하지 않음.
- 근거: `prepared_cube_review_20261022/grasp_approach_screen.json`, `empty_approach_fixture/windows_integration.json`, `empty_grasp_ready.json`, `leap_grasp_approach_empty_README.md`. 이 동작은 파지 준비를 위한 기하 계획이며 새 학습 SAC 정책이나 실물 뒤집기 성공이 아님. SAC 수직/수평 실물 회전 목표는 그대로 유지한다.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;content=p.read_text(encoding='utf-8')
    if note.splitlines()[0] not in content:p.write_text(note+content,encoding='utf-8')
print(json.dumps({k:ready[k] for k in ('status','launcher','tests_passed','static_cases','fixture_motion_samples')},indent=2))
