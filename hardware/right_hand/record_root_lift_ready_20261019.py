"""Record user clarification and offline-only preparation status."""
import json
from pathlib import Path
import hashlib
root=Path(__file__).resolve().parent
out=root/'right_hand_start_20261019'
confirmation=dict(date='2026-10-19',question='Current open-hand reconstruction, top-row red thumb direction and bend',
                  user_answer='방향·굽힘이 대체로 비슷함',
                  conclusion='Qualitative thumb pose agreement; no offset, wrapping or range calibration justified.',
                  physical_range_verified=False)
(out/'thumb_visual_confirmation.json').write_text(json.dumps(confirmation,ensure_ascii=False,indent=2),encoding='utf-8')
schedule=json.loads((out/'objective_and_schedule.json').read_text(encoding='utf-8'))
schedule['current_user_task']='Run leap_roots_empty.cmd once, EMPTY and unchanged open pose; SPACE arms. Observe root IDs8/12/16 lift; share result and any contact/unexpected direction.'
schedule['current_blocker']='Real root direction/clearance remains unverified. Thumb visually agrees, but ID3 remains outside native model range; no thumb sweep or full SAC execution released.'
(out/'objective_and_schedule.json').write_text(json.dumps(schedule,ensure_ascii=False,indent=2),encoding='utf-8')
verification=dict(status='ready_for_operator_empty_root_check_not_full_policy',
    unit_tests_passed=51,root_specific_tests_passed=10,
    test_command='.runtime\\python\\python.exe -B -X utf8 -m unittest test_leap_roots_empty test_hand_motor_mapping test_pose_pair_readonly test_readonly_port_failures test_leap_sac_start_empty test_leap_sac_path_empty',
    offline_screen='integer_root_lift_screen.json',source='source_readonly.json',
    operator_launcher='leap_roots_empty.cmd',hardware_runs_by_assistant=0,
    moving_ids=[8,12,16],goal_ticks=2472,frozen_goal_ids=[1,2,3,4,5,6,7,9,10,11,13,14,15],
    changes_to_modes_offsets_limits=False,legacy_sac_guards_preserved=True,
    file_sha256={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in
                 ('leap_roots_empty.py','leap_roots_empty.cmd','test_leap_roots_empty.py')})
(out/'root_check_verification.json').write_text(json.dumps(verification,indent=2),encoding='utf-8')
note='''## CURRENT 2026-10-19: 엄지 시각 확인 완료 — 새 매핑의 빈손 뿌리 상승 점검 준비

- 사용자가 현재 편 자세 그림의 빨간 엄지 방향·굽힘이 대체로 비슷하다고 확인했다. 엄지 영점·부호·가동범위를 정량 검증한 것은 아니므로 수정하지 않았다.
- `leap_roots_empty.cmd`: 현재 편 자세와 가까운 새 OFF 측정값에서 시작한다. 뿌리 ID8/12/16만 2472틱까지 약 6초 이상 천천히 상승, 약 1초 유지, 토크 OFF 및 설정 복원. 나머지 13개 목표는 새 측정값 그대로 고정한다. ID3=4098 부근의 값은 mode5에서 보존하며 모듈러 변환·모드·홈 오프셋 변경이 없다.
- 소스는 `right_hand_start_20261019/source_readonly.json` 해시 고정. 시작각 차이 >16틱, 카운트다운 드리프트 >4틱, OFF preload 불일치면 enable 이전 중단. 구동 중 기존 전압·전류·추종 검사와 고정 관절 드리프트 검사를 적용한다. 목표·enable 자동 재전송 없음.
- 실제 정수 120단계 및 시작각 ±16틱 무작위 32개 경우의 정적 MuJoCo 검사: 손가락 자체 겹침 0, 마지막 받침 겹침 0, 받침 겹침 깊이 비증가. 초기 받침 겹침은 근사 형상 결과이며 실제 침투 깊이가 아니다. 동역학/실물 안전 인증도 아니다.
- 51개 테스트 통과(새 실행기 10개 포함). 통신 실패·저전압·고정 관절 이동·토크 ON 시 각도 리셋·목표 쓰기 실패의 OFF/복원 경로 검증. 조수는 실제 모터를 실행하지 않았다.
- 사용자 다음 할 일: 큐브 없이 현재 편 자세를 유지하고 `leap_roots_empty.cmd` 1회 실행, SPACE. 세 손가락 뿌리가 올라가는 방향과 받침/다른 손가락 접촉 여부를 관찰하고 로그를 전달한다. Q/Esc/Ctrl+C 중단. 예전 SAC 경로는 계속 차단 상태.
- 목표는 10/24까지 오른손 수직·수평 회전 시연이며 일정은 목표다. 실물 회전 완료로 보고하지 않는다.

'''
for path in (root/'professor_progress_20261015.md',root/'sac_current_status.md',out/'README.md'):
    old=path.read_text(encoding='utf-8-sig') if path.exists() else ''
    if not old.startswith(note.splitlines()[0]):path.write_text(note+old,encoding='utf-8')
print('Recorded thumb confirmation, offline validation and the single next operator task. No motor access.')
