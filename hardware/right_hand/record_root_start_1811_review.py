"""Record reviewed preflight rebase; no robot access."""
from pathlib import Path
import json,hashlib
root=Path(__file__).resolve().parent
out=root/'right_hand_start_20261019_1811'
screen=json.loads((out/'integer_root_lift_screen.json').read_text())
assert screen['worst_self_overlap_mm']==screen['worst_final_support_overlap_mm']==0
assert screen['all_support_depth_nonincreasing']
review=dict(status='operator_empty_check_ready_after_offline_rebase',
    source_timestamp='2026-10-19T18:11:01.488208+09:00',
    source_sha256=screen['source_sha256'],tests_passed=12,
    source_tolerance_ticks=16,source_tolerance_increased=False,
    root_ids=[8,12,16],start_ticks=[2402,2410,2411],target_ticks=[2472]*3,
    nominal_root_travel_degrees=[6.15234375,5.44921875,5.361328125],
    no_new_user_capture_needed=True,original_stop_write_count=0,
    original_stop_all_torque_off_verified=True,assistant_motor_commands=0,
    full_sac_execution_enabled=False,
    runner_sha256=hashlib.sha256((root/'leap_roots_empty.py').read_bytes()).hexdigest())
(out/'review.json').write_text(json.dumps(review,indent=2),encoding='utf-8')
note='''## CURRENT 2026-10-19 18:11: 現 자세 재검토 후 빈손 뿌리 점검 v2

- 직전 중단은 아침 07:00 각도와 현재 각도의 차이 때문. ID8/12/16은 각각 +366/+391/+420틱(약 32/34/37도). 명령 쓰기 0회, 16개 토크 OFF 확인. 통신/전압 고장이나 실제 충돌 판정이 아니다.
- 현재 18:11 preflight의 16개 각도가 이미 결과에 있어 추가 측정 요청 없이 보존했다: `right_hand_start_20261019_1811/source_preflight.json`. 과거 아침 기준과 검사 결과는 그대로 보존했다.
- 현 각도에서 목표 2472까지 뿌리 움직임은 6.15/5.45/5.36도. 정확한 정수 경로와 32개 ±16틱 시작각 교란을 정적 MuJoCo로 재검토: 자체/최종 받침 겹침 0, 받침 겹침 증가 없음. 실제 접촉/부하 검증은 아니다.
- `leap_roots_empty.cmd`는 같은 명령으로 v2 실행. 허용 오차를 넓히지 않고 검토 기준을 현 로그로 갱신했다. 기준 불일치 시 ID별 전후 틱과 차이를 출력하며, 그 실행 결과에 현재 각도가 있음을 알린다. 재측정하라는 불필요한 문구를 제거했다.
- 관련 테스트 12개 통과. 현 4069틱 유지와 종전 4098틱 비래핑, 구동 전 mode5 검사/불일치 중단, 고정 목표/비정상 종료 OFF 복원 확인. 조수의 실제 모터 실행 없음. 엄지 범위 확장이나 전체 SAC 해제 없음.
- 사용자 할 일: 손을 아침 자세로 다시 펴지 말고 지금 상태에서 큐브 없이 같은 명령 1회. SPACE, 세 뿌리의 작은 상승을 관찰, Q로 중단 가능. 결과 로그 공유.

'''
for p in (root/'professor_progress_20261015.md',root/'sac_current_status.md',out/'README.md'):
    old=p.read_text(encoding='utf-8-sig') if p.exists() else ''
    if not old.startswith(note.splitlines()[0]):p.write_text(note+old,encoding='utf-8')
p=root/'right_hand_start_20261019/objective_and_schedule.json'
s=json.loads(p.read_text(encoding='utf-8'))
s['current_user_task']='Run leap_roots_empty.cmd v2 EMPTY once from unchanged 18:11 posture; SPACE. No reshaping or additional read-only capture needed.'
s['current_blocker']='Root direction/physical contact awaits test; old-source mismatch resolved through a separately reviewed fresh preflight archive. Thumb range and full SAC remain unverified.'
p.write_text(json.dumps(s,ensure_ascii=False,indent=2),encoding='utf-8')
print('Recorded v2 review and the next operator action; no motor access.')
