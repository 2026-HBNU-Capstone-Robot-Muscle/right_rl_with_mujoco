from pathlib import Path
import json,hashlib
root=Path(__file__).resolve().parent;out=root/'right_hand_root_drift_20261020'
s=json.loads((out/'screen.json').read_text())['summary']
assert s['trials']==202 and s['max_self_overlap_mm']==s['max_final_support_overlap_mm']==s['max_support_depth_increase_mm']==0
assert s['max_command_step_ticks']<=5
r=dict(date_local='2026-10-20',version=5,start_tolerance_ticks=32,
       previous_start_tolerance_ticks=16,tests_passed=20,kinematic_paths_checked=202,
       latest_failed_source_sha256=s['latest_source_sha256'],
       live_guard_changes=False,assistant_hardware_execution=False,
       physical_motion_verified=False,reason='Observed inter-session drift 22/23/27 ticks; offline sensitivity review supports a larger provisional start neighborhood.',
       runner_sha256=hashlib.sha256((root/'leap_roots_empty.py').read_bytes()).hexdigest())
(out/'v5_review.json').write_text(json.dumps(r,indent=2),encoding='utf-8')
note='''## CURRENT 2026-10-20 21:22 이후: 시작각 소폭 변화 검토, v5 준비

- 사용자 로그의 ID2/10/14 차이는22/23/27틱(1.93/2.02/2.37도). 날짜만으로 각도가 변한다고 판단하지 않는다. OFF 중 처짐/접촉 등은 가능하지만 원인은 로그로 확인되지 않는다. 이번 결과는 쓰기0회·토크OFF 확인.
- `right_hand_root_drift_20261020/source_preflight.json`에 현재16개 각도 보존. 세 기준마다 원점/전체±32틱/단일관절±32틱32개/무작위32개, 그리고 실제 현재값까지 총202개 정적 경로 검사. 자체 겹침·마지막 받침 겹침·받침 겹침 증가 모두0, 목표증분최대5틱. 샘플링 검사이며 실제 접촉·동역학 또는 모든 조합의 인증은 아니다.
- 시작 자세 허용 차이만16→32틱(2.8125도)로 변경한 `leap_roots_empty.cmd` v5. 출처 전체16개 기준 선택, 새 실측값에서 보간 및 나머지13개 목표고정은 유지. OFF/ON4틱 절대변화, 연속1틱 안정성, 구동 중 전압/전류/추종/정확한 목표 검사는 변경하지 않았다.
- 관련테스트20개통과. 실제22/23/27틱 로그로 가짜버스 실행, 기준각도로 순간이동하지 않는 것과 고정목표 보존 확인. 시작허용32틱이 enable허용4틱을 넓히지 않는 것 검증. 조수는 실물 모터를 구동하지 않았다.
- 사용자 할 일: 큐브 없이 손을 그대로 두고 같은명령1회, v5확인후SPACE. 작은뿌리상승을 관찰하고 로그전달. 접촉시Q. 추가측정·수동각도맞춤 불필요. 모든시작자세나 전체SAC실물회전은 여전히 검증되지 않았다.

'''
for p in (root/'professor_progress_20261015.md',root/'sac_current_status.md',out/'README.md'):
    old=p.read_text(encoding='utf-8-sig') if p.exists() else ''
    if not old.startswith(note.splitlines()[0]):p.write_text(note+old,encoding='utf-8')
p=root/'right_hand_start_20261019/objective_and_schedule.json';v=json.loads(p.read_text(encoding='utf-8'))
v['current_user_task']='Run leap_roots_empty.cmd v5 EMPTY from current posture; SPACE. Start tolerance 32 ticks after 202 offline sensitivity paths; live guards unchanged. Report motion/contact and log.'
p.write_text(json.dumps(v,ensure_ascii=False,indent=2),encoding='utf-8')
print('Recorded v5 review; no hardware access.')
