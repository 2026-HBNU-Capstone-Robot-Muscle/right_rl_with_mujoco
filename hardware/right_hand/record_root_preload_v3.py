"""Offline evidence and v3 notes; no hardware access."""
from pathlib import Path
import json,hashlib
root=Path(__file__).resolve().parent
out=root/'right_hand_start_20261019_1811/preload_v3_review';out.mkdir(exist_ok=True)
evidence=[]
for name in ('leap_roots_empty_20261019_181501_566207.json','leap_roots_empty_20261019_181518_400116.json'):
    b=(root/name).read_bytes();r=json.loads(b);(out/name).write_bytes(b)
    enabled=any(e['address']==64 and any(e['requested']) for e in r['write_events'])
    assert not enabled and r['all_torque_off_verified'] and r['settings_restored']
    evidence.append(dict(file=name,error=r['error'],torque_enabled=False,all_off_verified=True,settings_restored=True))
review=dict(version=3,evidence=evidence,root_tests_passed=15,source_pose_tolerance_ticks=16,
    preload_and_enable_absolute_shift_ticks=4,consecutive_snapshot_stability_ticks=1,
    requested_goal_retries=0,enable_retries=0,after_enable_goal_checks='exact',
    actual_hardware_run_by_assistant=False,full_policy_released=False,
    limitation='Second failure did not log failed snapshot; exact mismatched field is unknown.',
    runner_sha256=hashlib.sha256((root/'leap_roots_empty.py').read_bytes()).hexdigest())
(out/'review.json').write_text(json.dumps(review,indent=2),encoding='utf-8')
note='''## CURRENT 2026-10-19: 根 준비 v3 — OFF/ON 전환의 작은 변화 확인, 자동 준비 목표 명시

- 사용자 요구: 최종 실행은 특정 각도를 수동으로 재현하지 않고, 유효한 현재 빈손 자세를 읽어 충돌 검사를 거친 준비 경로를 생성해야 한다. 현재 소스 자세 고정은 번호·방향 검증용 임시 시험이다. 임의의 충돌/범위 밖 자세까지 무조건 구동하거나 SAC가 모든 초기 상태를 다룬다고 주장하지 않는다.
- 두 실패 모두 torque ON 없음, OFF 및 설정 복원 확인. 첫 실패는 116번 목표 쓰기/읽기 간 1틱 차이. 둘째는 실패 순간 snapshot이 없어 어느 필드인지 특정 불가. 이번 코드의 완전 일치 preload 요구와 진단 부족을 수정했다.
- OFF preload는 목표 재전송 없이 전체 새 snapshot 2개로 확인. 원래 각도 대비 목표/위치 차이 최대 4틱, 목표-위치 차이 최대 4틱, 연속 goal/position 변화 최대 1틱. PID·전류·프로파일은 정확히 일치해야 함.
- ON 직후에도 2개 snapshot에서 같은 한계를 확인한 뒤 관측된 유지 목표를 한 번만 채택. 그 기준으로 뿌리 경로를 생성하고 나머지 관절 목표를 고정. 이후 goal readback은 정확히 일치해야 함. 모듈러 변환·모드 변경·전류 증가 없음. 시작 허용16틱/추종/전압 중단 조건은 그대로.
- 모든 OFF preload/enable snapshot을 판정 전에 기록하며 PID·프로파일 불일치는 ID/필드/기대/관측값을 출력. 실패 시 OFF와 설정 복원 경로 유지. 15개 관련 테스트 통과, 실제 모터 실행은 사용자 차례.
- 다음 사용자 동작: 큐브 없이 현재 자세를 유지하고 같은 `leap_roots_empty.cmd` 1회, v3 표시 확인 후 SPACE. Q/Esc/Ctrl+C 중단. 작은 뿌리 상승 및 실행 로그 확인 후 자동 준비 경로 작업으로 이어간다.

'''
for p in (root/'professor_progress_20261015.md',root/'sac_current_status.md'):
    old=p.read_text(encoding='utf-8-sig')
    if not old.startswith(note.splitlines()[0]):p.write_text(note+old,encoding='utf-8')
(out/'README.md').write_text(note,encoding='utf-8')
p=root/'right_hand_start_20261019/objective_and_schedule.json'
s=json.loads(p.read_text(encoding='utf-8'))
s['current_user_task']='Run the same leap_roots_empty.cmd, v3, EMPTY once; observe roots. No manual reshaping or extra capture requested.'
s['startup_product_requirement']='Read valid current empty-hand state and plan a collision-checked transition to preparation; do not require exact manual reproduction of archived angles. Not yet implemented for arbitrary starts.'
p.write_text(json.dumps(s,ensure_ascii=False,indent=2),encoding='utf-8')
print('Saved v3 evidence, limitations and automatic-preparation requirement. No hardware access.')
