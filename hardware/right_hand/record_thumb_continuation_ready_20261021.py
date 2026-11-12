"""Record the reviewed next operator task; no device access."""
import json
import numpy as np
import thumb_continuation_plan as p
from leap_thumb_prefix_empty import validate_offline_review

if p.PLAN_VERSION != 'thumb_continuation_empty_v1':
    raise SystemExit('Archived v1 recorder. Use record_thumb_continuation_v2_ready_20261021.py.')

validate_offline_review(p)
r=json.loads(p.REVIEW_PATH.read_bytes())
raw=p.source_position();end=p.plan(raw)[-1][1]
ready=dict(plan_version=p.PLAN_VERSION,source_sha256=p.SOURCE_SHA,
    source_kind='Last measured ON pose from completed recorded prefix, NOT a new OFF measurement.',
    source_ticks=raw.tolist(),final_nominal_ticks=end.tolist(),
    nominal_thumb_goal_travel_deg=((end[:4]-raw[:4])*360/4096).tolist(),
    static_cases=len(r['static_screens']),max_goal_step_ticks=2,
    tests_passed=22,launcher='leap_thumb_continuation_empty.cmd',
    native_dynamics_accepted=False,isolated_diagnostic_dynamics_only=True,
    current_user_task='Empty hand: leave fingers as they are, show thumb/middle in camera, terminal SPACE once.',
    remaining_after_this_task='Whole preparation and physical cube rotation still require validation.',
    hardware_executed=False,physical_contact_verified=False,policy_used=False)
(p.REVIEW_PATH.parent/'thumb_continuation_ready.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
note='''## CURRENT 2026-10-21: 다음 엄지 빈손 구간 준비됨

- 이전 작은 엄지 시험 완료를 기준으로 `leap_thumb_continuation_empty.cmd` 준비. 손 모양을 바꾸지 않고 빈손으로 실행, 터미널 SPACE 한 번. 카메라·모터 기록을 한 명령으로 저장, 고정 유지 후 자동 OFF 및 설정 복원. 큐브는 올리지 않는다.
- 직전 실제 ON 도착각을 기준으로 fresh OFF 시작각 ±32틱을 확인한다. OFF 자세가 같다고 가정하지 않으며, 범위를 벗어나면 쓰기 전에 현재 읽기를 기록하고 멈춘다. 수동으로 이전 자세를 강제로 만들 필요 없음.
- 엄지 다음 구간 최대 384틱(33.75°), 목표 갱신 최대 2틱. 명목 엄지 목표 변화 [334,357,-384,275]틱. 뿌리 ID8/12/16은 2472보다 낮을 때만 올리고 그보다 높으면 현재값 유지. 나머지 9개 관절 목표 고정. 원래 전류/PID, 전압·추종·실측 진행·카메라·35초 제한 유지.
- 67개 시작 자세 표본에서 자기 겹침/받침 겹침 증가/엄지 구간 받침 겹침 0. 격리된 관찰 범위 진단 모델 gain 0.5/1/1.5의 최대 추종 오차 0.0632/0.0324/0.0220rad. 실제 PID/마찰 교정이나 실물 무접촉 보증은 아니다.
- 원래 모델은 ID3 시작각 범위 불일치로 유지 명령만으로 약 0.528rad 이동하므로 동역학 통과로 취급하지 않는다. 실제 모터/배포 모델의 범위·영점 변경 없음. 명목 다음 구간 목표 ID3=3601틱은 원래 모델 범위 안으로 진입하지만, 이 사실이 영점 교정을 증명하지는 않는다.
- 기존 빈손/카메라 회귀 및 새 구간 오류 주입 테스트 22개 통과. 오프라인 검사·시뮬레이션만 실행. 전체 준비·큐브 회전·live SAC는 아직 미검증.
- 근거: `right_hand_preparation_20261021/thumb_continuation_screen.json`, `thumb_continuation_ready.json`, `thumb_continuation_views.jpg`.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    file=p.ROOT/name;text=file.read_text(encoding='utf-8')
    if note.splitlines()[0] not in text:file.write_text(note+text,encoding='utf-8')
file=p.ROOT/'right_hand_start_20261019/objective_and_schedule.json'
state=json.loads(file.read_text(encoding='utf-8'))
state['current_user_task']='Run leap_thumb_continuation_empty.cmd once with empty hand, unchanged current pose, thumb/middle in camera; terminal SPACE. Do not repeat old prefix.'
state['current_blocker']='Next EMPTY thumb segment prepared; physical completion pending. Native thumb range/model mismatch remains. Full preparation and cube reorientation unvalidated.'
state['next_trial_review']='right_hand_preparation_20261021/thumb_continuation_ready.json'
file.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(ready,ensure_ascii=False,indent=2))
