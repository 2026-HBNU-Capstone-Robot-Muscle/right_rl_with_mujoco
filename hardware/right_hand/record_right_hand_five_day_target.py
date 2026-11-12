"""Record narrowed user objective and current offline evidence; no hardware."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_start_20261019'
r=json.loads((OUT/'analysis.json').read_text(encoding='utf-8'))
rows=r['screens']['roots_thumb_others']['rows']
rootrows=[x for x in rows if x['segment']=='roots']
depths=[max([c['overlap_mm'] for c in x['contacts'] if c['support']]or[0]) for x in rootrows]
root_review=dict(samples=len(rootrows),max_support_overlap_sequence_mm=depths,
    max_support_overlap_monotone_nonincreasing=all(b<=a+1e-6 for a,b in zip(depths,depths[1:])),
    support_overlap_at_root_stage_end_mm=depths[-1],
    max_self_overlap_mm=max([c['overlap_mm'] for x in rootrows for c in x['contacts'] if not c['support']]or[0]),
    caveat='Aggregate overlap trend in a provisional static model, not force or real clearance verification.')
objective=dict(user_confirmed_scope='Right hand physically reorients the cube in requested vertical and horizontal directions.',
    started_local_date='2026-10-19',target_local_date='2026-10-24',timezone='Asia/Seoul',
    deadline_is_target_not_guarantee=True,
    minimum_demo='Actual cube orientation changes in each requested maneuver, stays in hand, no human correction during maneuver, before/after visible in video.',
    sac_requirement='Keep SAC integration as the approach; label fixed SAC-derived trajectory replay separately from live feedback policy.',
    excluded_from_this_five_day_scope=['Left-hand layer turns','Full scrambled-cube solve'],
    milestones=[
        dict(target='2026-10-19..2026-10-20',deliverable='Resolve physical joint conversion and validate empty preparation/trajectory',status='in_progress'),
        dict(target='2026-10-21',deliverable='First verified physical vertical reorientation',status='not_verified'),
        dict(target='2026-10-22',deliverable='First verified physical horizontal reorientation',status='not_verified'),
        dict(target='2026-10-23..2026-10-24',deliverable='Direction selection, SAC integration check and complete maneuver demonstration video',status='not_verified')],
    current_user_task='Compare red thumb in top row of current_and_target_views.jpg with current open hand; no motion or new encoder read needed.',
    current_blocker='ID3 decoded angle 3.14466 rad lies 40.202deg beyond model th_axl upper bound. Turn wrapping does not resolve. Physical pose/model correspondence unconfirmed.',
    hardware_commands_sent=0)
(OUT/'objective_and_schedule.json').write_text(json.dumps(objective,indent=2,ensure_ascii=False),encoding='utf-8')
(OUT/'root_relief_review.json').write_text(json.dumps(root_review,indent=2),encoding='utf-8')
note='''## CURRENT 2026-10-19: 사용자 목표를 오른손 실물 회전으로 한정 — 10/24 목표

사용자가 “오른손으로 원하는 방향 회전,5일이면 되는거지 해보자 난 이게 목표야”라고
범위를 정했다. **이번 목표는 오른손의 수직 뒤집기와 수평 방향 전환 실물 재현**이다.
5일은 목표 일정이며 보장이 아니다. 전체 큐브 풀이/왼손 층 회전은 이번 범위에서
분리한다. SAC 적용은 유지하되,SAC 유래 고정 경로와 live SAC 구동을 구분한다.
최소 시연은 사람 손으로 동작을 보정하지 않고 큐브를 유지하며,지정한 방향 변화가
실제 전후 영상에서 확인되는 것이다. 고정된 높은 반복 성공률을 새 필수조건으로 추가하지 않는다.

일정 목표:10/19~20 변환·빈손 준비,10/21 첫 수직 회전,10/22 첫 수평 회전,
10/23~24 방향 선택·SAC 연결 검증과 시연 영상. 실제 검증 결과에 따라 갱신한다.

최신 읽기 저장은2026-10-19 07:00:09+09:00. 사용자가 손을 편 뒤 측정한 값이다.
16모터 토크OFF/오류0. raw ID3=4098,mode5,homing_offset0,drive_mode0.
현재 모델 th_axl=3.14466rad,상한2.443rad보다40.202° 크다. ±1회전 등가각도
모델 범위에 들어오지 않는다. 모터 고장이나 영점 오류로 단정하지 않으며,
4096나머지 연산·클리핑·영점 변경·한계 확장을 자동 적용하지 않는다.

제조사 e-manual:torqueOFF에서 Present Position은 연속32bit값이며,mode5는
다회전 위치를 지원한다. 단일회전 Min/Max Position Limit은 mode5에 적용되지 않는다.
따라서4098 자체가 센서 오류는 아니다. mode3토크ON의 위치reset 설명을
mode5토크ON에 그대로 확대 적용하지 않는다.
https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#present-position132
https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#operating-mode11

기존표시 ID7/11/15의 -0.44rad 전후 PIP는 정책의 -0.2rad 하한 밖이지만,
native기하모델의 -0.506rad 하한 안이다. 이 셋과 엄지 ID3는 다른 문제다.
표시만으로 실제 관절이 걸렸다고 결론내리지 않는다.

오프라인 준비 경로3개를 새 매핑으로 검사했다. direct127샘플/받침겹침97,
엄지먼저235샘플/209,뿌리→엄지→나머지271샘플/39. 현재 열린 자세 자체가
잠정 받침 모델과 최대14.89mm 겹치므로 이를 실제 침투량으로 해석하지 않는다.
새 순서의 뿌리ID8/12/16을 먼저 올리는 후보에서 겹침이 빠르게 감소한다.
이 수치는 정적 검사이며 구동 승인 또는 안전한 복귀 경로 확정이 아니다.

자료:`right_hand_start_20261019/analysis.json`,`source_readonly.json`,
`current_and_target_views.jpg`,`root_relief_review.json`,`objective_and_schedule.json`.
현재 사용자에게 윗줄 빨간 엄지가 실제 편 엄지와 비슷한지 질문한 상태다.
**지금 추가 사진/각도읽기/수동조작/실물 구동을 요청하지 않는다.**
이미 확정된 네 손가락 ID 질문을 반복하지 않는다. 그림과 명백히 다르면 그때
현재 사진과 동시읽기로 원인을 구분한다. 기존 잘못된 SAC 실행 차단은 유지한다.

---

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8-sig')
    if not old.startswith('## CURRENT 2026-10-19: 사용자 목표를 오른손 실물 회전으로'):
        p.write_text(note+old,encoding='utf-8')
(OUT/'README.md').write_text(note,encoding='utf-8')
print(json.dumps(root_review,indent=2))
