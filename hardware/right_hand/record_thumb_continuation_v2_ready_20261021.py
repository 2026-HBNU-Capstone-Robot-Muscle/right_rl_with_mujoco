"""Record the re-screened actual-OFF reference; never accesses devices."""
import json
import numpy as np
import thumb_continuation_plan as p
from leap_thumb_prefix_empty import validate_offline_review

validate_offline_review(p)
source=json.loads(p.SOURCE.read_bytes())
screen=json.loads(p.REVIEW_PATH.read_bytes())
old=p.ROOT/'thumb_prefix_empty_20261021_190731_591017/result.json'
previous=json.loads(old.read_bytes())
raw=p.source_position();end=p.plan(raw)[-1][1]
delta=raw-np.array(previous['final_on_state']['position'])
review=dict(source_sha256=p.SOURCE_SHA,status=source['status'],
    no_configuration_or_goal_writes=source['write_events']==[],
    all_torque_off_verified=source['all_torque_off_verified'],
    camera_frames=source['camera_recording']['frame_count'],
    reviewed_image='camera/frame_00010.jpg',
    visual_observation='Empty hand with thumb/middle visible; some remote fingertips clipped. No new photo needed for this segment.',
    actual_off_ticks=raw.tolist(),difference_from_previous_on_ticks=delta.tolist(),
    id4_difference_degrees=float(delta[3]*360/4096),
    cause_of_position_change_established=False)
(p.SOURCE.parent/'review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2),encoding='utf-8')
ready=dict(plan_version=p.PLAN_VERSION,source_sha256=p.SOURCE_SHA,
    source_kind='Actual torque-OFF read-only preflight, 2026-10-21 19:24; zero writes.',
    source_ticks=raw.tolist(),final_nominal_ticks=end.tolist(),
    nominal_thumb_goal_travel_deg=((end[:4]-raw[:4])*360/4096).tolist(),
    start_tolerance_ticks=p.TOLERANCE,static_cases=len(screen['static_screens']),
    max_goal_step_ticks=p.MAX_GOAL_STEP,tests_passed=26,
    launcher='leap_thumb_continuation_empty.cmd',native_dynamics_accepted=False,
    isolated_diagnostic_dynamics_only=True,
    current_user_task='Leave fingers unchanged, cube OUT, thumb/middle in camera. Run the same launcher with v2 banner; terminal SPACE once.',
    hardware_executed=False,policy_used=False,physical_contact_verified=False,
    whole_preparation_verified=False,physical_cube_rotation_verified=False)
(p.REVIEW_PATH.parent/'thumb_continuation_ready.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')
note='''## CURRENT 2026-10-21 19:24 중단 검토 / 엄지 구간 v2 준비

- 사용자 실행 `thumb_continuation_empty_20261021_192403_090594`는 preflight에서 중단. 쓰기 0회, 16개 토크 OFF, watchdog/error 0. 카메라 21개 프레임 저장. 저장 사진에서 빈손 및 엄지·중지 배치 확인, 추가 사진/수동 자세 복원 요청 없음.
- v1 기준은 이전 시험의 토크 ON 도착각이었다. 실제 OFF 읽기에서 ID4=2001틱으로 ON 도착 2055틱보다 54틱(4.746°) 작음. 뿌리 ID8/12/16은 -22/-19/-17틱. 원인을 토크 해제 이완이나 사용자 접촉으로 단정하지 않음.
- v2는 방금 저장된 실제 read-only OFF 각도로 시작 기준을 바꾸고 그 주변 ±32틱을 다시 검사. 허용오차 확대 없음. 같은 `leap_thumb_continuation_empty.cmd` 명령, 시작 배너에 v2. 손 모양 그대로 큐브 없이 SPACE 한 번.
- 다음 구간 명목 엄지 목표 변화 [334,356,-384,298]틱, 최대 33.75°. 67개 시작 자세 표본에서 자기 겹침/받침 겹침 증가/엄지 받침 겹침 0. 격리된 범위 진단 gain 0.5/1/1.5의 최대 추종 오차 약 0.0634/0.0324/0.0220rad. 원래 모델은 ID3 범위 불일치로 유지 시 약 0.530rad 이동하므로 통과로 취급하지 않음. 실물 무접촉이나 전체 준비 성공은 미검증.
- 다음 사용자 실행 후에는 토크 OFF·설정 복원이 끝난 상태의 fresh 각도 2회를 자동 저장. 두 OFF 끝점 변화 ≤2틱일 때만 참고값 유효로 표시하고 ON→OFF 변화도 기록. 자동으로 시작 허용 범위를 넓히거나 새 기준을 승인하지 않음. 읽기 실패는 진단 누락으로 기록하며 어떤 쓰기도 추가하지 않는다.
- 원래 전류/PID, 4틱 OFF/ON 확인, 전압/추종/실측 진행/카메라/35초 제한 유지. 새 OFF 참고 저장과 기준 검증을 포함해 26개 테스트 통과. v2 실물 실행은 아직 안 함.
- 근거: 중단 폴더 `review.json`, `right_hand_preparation_20261021/thumb_continuation_screen.json`, `thumb_continuation_ready.json`. 이전 v1 화면·검사는 `_v1_` 파일로 보관.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    file=p.ROOT/name;content=file.read_text(encoding='utf-8')
    if note.splitlines()[0] not in content:file.write_text(note+content,encoding='utf-8')
file=p.ROOT/'right_hand_start_20261019/objective_and_schedule.json'
state=json.loads(file.read_text(encoding='utf-8'))
last=state.get('latest_user_hardware_trial')
if last and last.get('status')=='empty_thumb_prefix_complete':state['last_completed_user_hardware_trial']=last
state['latest_user_hardware_trial']=dict(timestamp=source['timestamp'],status='stopped_before_any_write',
    evidence=str(p.SOURCE.relative_to(p.ROOT)).replace('\\','/'),all_torque_off_verified=True)
state['current_user_task']=ready['current_user_task']
state['current_blocker']='Continuation v2 re-screened from actual OFF preflight; user execution pending. Full preparation and physical cube reorientation unverified.'
state['next_trial_review']='right_hand_preparation_20261021/thumb_continuation_ready.json'
file.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(plan_version=p.PLAN_VERSION,no_writes=review['no_configuration_or_goal_writes'],
    id4_difference_ticks=int(delta[3]),static_cases=67,tests_passed=26,
    nominal_final=end.tolist(),next_user_task=ready['current_user_task']),ensure_ascii=False,indent=2))
