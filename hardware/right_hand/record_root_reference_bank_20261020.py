"""Persist today's offline verification and next operator action."""
from pathlib import Path
import json,hashlib
import leap_roots_empty as roots
root=Path(__file__).resolve().parent
out=root/'right_hand_start_20261019_thumb_shift'
screen=json.loads((out/'integer_root_lift_screen.json').read_text())
assert screen['source_sha256']=='a0274a7b108f9a866c7ae680cfe321b41b3f65f5b1f0cd3f9d03303cf2531350'
assert screen['worst_self_overlap_mm']==0 and screen['worst_final_support_overlap_mm']==0
assert screen['all_support_depth_nonincreasing']
report=dict(review_date_local='2026-10-20',timezone='Asia/Seoul',runner_version=4,
    status='ready_for_operator_empty_root_check',root_tests_passed=18,
    approval_review_failure_from_previous_turn_resolved=True,
    hardware_executions_by_assistant=0,all_pose_startup_implemented=False,
    full_sac_execution_released=False,root_target=2472,source_tolerance_ticks=16,
    reviewed_starts=[dict(path=x['path'],sha256=x['sha256'],position=x['position'].tolist()) for x in roots.reviewed_starts()],
    validation='33 kinematic sweeps of latest source: nominal and 32 randomized +/-16-tick starts; no hand self-overlap; final support overlap zero. Not a physical clearance or dynamic guarantee.',
    implementation='Choose one matching full 16-joint reference neighborhood; never combine joint bounds from separate poses. Actual measured angles form the ramp start; other 13 goals frozen after bounded enable hold.',
    runner_sha256=hashlib.sha256((root/'leap_roots_empty.py').read_bytes()).hexdigest())
(out/'review_20261020.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
note='''## CURRENT 2026-10-20: 엄지 ID2 변화 검토 완료, 시작 자세 자동 선택 v4

- 전날 사용량 한도로 자동 승인 검토가 실패했던 오프라인 MuJoCo 실행을 오늘 재요청하여 완료했다. 실제 모터는 구동하지 않았다.
- 10/19 18:23 로그의 엄지 ID2=2056(기존1967 대비 +89틱, 약7.82도) 자세와 ±16틱 교란 32개를 포함한 33개 정적 경로 검사: 자체 겹침 0, 최종 받침 겹침 0, 받침 겹침 깊이 비증가. 근사 모델의 검사이며 실물/동역학 인증이 아니다.
- `leap_roots_empty.cmd` v4는 아침 편 자세 / 18:11 자세 / 18:23 엄지 변경 자세 중 현재 전체16개 각도와 맞는 검토 기준을 자동 선택한다. 관절별 범위를 서로 섞지 않고, 각 기준의 기존16틱 허용 범위를 그대로 적용한다. 새 미검토 자세를 자동 승인하지 않는다.
- 선택한 기준의 파일/해시/차이를 기록한다. 실제 새 측정값에서 뿌리 ID8/12/16을2472틱까지 올리고, 나머지13개 목표를 유지한다. OFF/ON4틱 절대변화 확인, 연속1틱 안정성 확인, 구동 중 정확한 목표 검사와 전류·전압·추종 중단은 유지한다.
- 관련 테스트18개 통과. 세 자세 모두 별도 출처 기록/나머지 목표 고정, 다른 자세의 관절을 섞은 미검토 자세 차단, 출처 해시 변경 차단, 과거4098틱 보존 및 실패 시 OFF/복원 확인.
- 다음 사용자 할 일: 큐브 없이 손을 그대로 두고 같은 명령1회. v4 표시 확인 후 SPACE. 작은 뿌리 상승 방향·접촉 여부와 로그 전달. Q/Esc/Ctrl+C 중단. 이전 각도로 손을 억지로 맞추거나 추가 사진/측정할 필요 없음.
- 임의의 유효한 빈손 자세에서 준비 자세로 자동 경로를 만드는 최종 사용 방식은 아직 미완료. 현재는 검토된 세 시작 영역을 지원하는 단계이며 전체 SAC/큐브 회전 실물 성공으로 보고하지 않는다.

'''
for p in (root/'professor_progress_20261015.md',root/'sac_current_status.md',out/'README.md'):
    old=p.read_text(encoding='utf-8-sig') if p.exists() else ''
    if not old.startswith(note.splitlines()[0]):p.write_text(note+old,encoding='utf-8')
p=root/'right_hand_start_20261019/objective_and_schedule.json'
s=json.loads(p.read_text(encoding='utf-8'))
s['current_user_task']='Run leap_roots_empty.cmd v4 EMPTY once from current posture; SPACE. Three reviewed start regions supported; no manual reshaping requested. Report log and root direction/contact.'
s['current_blocker']='Offline latest thumb-shift review completed; physical root-lift direction/contact still awaits the user test. Arbitrary-start planning and full SAC motion remain unverified.'
p.write_text(json.dumps(s,ensure_ascii=False,indent=2),encoding='utf-8')
print('Saved 2026-10-20 v4 review and operator task. No hardware access.')
