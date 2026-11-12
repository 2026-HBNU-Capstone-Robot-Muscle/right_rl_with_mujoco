"""Record current read-only connection, replay and tests before operator trial."""
import hashlib
import json
from book_grasp_trial_plan import ROOT,BookGraspPlan
from leap_book_cube_grasp_trial import require_offline_review

folder=ROOT/'right_hand_rotation_20261024/book20_current_review/grasp_route_fixture_v6'
proof=require_offline_review();plan=BookGraspPlan(folder/'request.json')
tests=json.loads((folder/'device_free_validation.json').read_bytes())
read_folder=ROOT/'book_settled_off_readonly_20261024_212026_419273'
read=json.loads((read_folder/'result.json').read_bytes())
request=json.loads((read_folder/'request.json').read_bytes())
if not tests['successful'] or read['status']!='stable_current_off_read' or read.get('motion_admission_error'):
    raise RuntimeError('Current measured-source review incomplete')
if request['off_state']['position']!=plan.source_position().tolist():
    raise RuntimeError('Development replay changed measured encoders')
record=dict(implementation_reviewed=True,plan_version=plan.PLAN_VERSION,
    actual_read_source=str(read_folder.relative_to(ROOT)),actual_off_ticks=plan.source_position().tolist(),
    fresh_runtime_read_and_full_screen_still_required=True,
    source_replay_is_not_stale_runtime_approval=True,
    static_cases=len(proof['static_screens']),dynamic_cases=len(proof['dynamic_sensitivity']),
    proof_sha256=hashlib.sha256((folder/'screen.json').read_bytes()).hexdigest(),
    passive_support_model_policy=proof['passive_support_reentry_policy'],
    maximum_initial_support_model_overlap_mm=max(max(s['initial_support_contacts_mm'].values(),default=0.) for s in proof['static_screens']),
    maximum_model_tracking_rad=max(s['max_tracking_rad'] for s in proof['dynamic_sensitivity']),
    maximum_model_tracking_after_clearance_rad=max(s['max_tracking_after_clearance_rad'] for s in proof['dynamic_sensitivity']),
    actual_tracking_stop_rad=.18,actual_goal_current_ma=450,
    actual_voltage_current_temperature_no_progress_stops_unchanged=True,
    camera_exposure_control='preserve existing settings; frame quality and freshness required',
    device_free_validation=tests,command_count=len(plan.plan(plan.source_position())),
    physical_grasp_verified=False,physical_rotation_verified=False,assistant_motor_write_commands_sent=0,
    next_user_action='Keep hand EMPTY and fingers/base/book unchanged. Run leap_book_cube_grasp_trial.cmd. '
        'SPACE prepares EMPTY, PLACE/SPACE starts the reached grasp prefix, REMOVE/SPACE confirms removal and OFF.')
(folder/'readiness_review.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
entry='''## CURRENT 2026-10-24 21:20: USB 재연결 정상 / 현재 OFF 자세의 받침 분리 모델 검토 완료 / 파지 실행 대기

- 사용자 "연결했어" 뒤 최초 ID1 PING은 시간 초과였다. 다른 모터를 가리는 첫 ID 실패를 구분하기 위해 기존 읽기 전용 검사로16개 ID를 한 번씩 확인했고 모두1240 모델과 정상 OFF 상태를 읽었다. `reconnected_bus_readonly_20261024_212016_703296`이다. 설정과 토크 및 목표 쓰기는0회다.
- 이어 `book_settled_off_readonly_20261024_212026_419273`에서3개 안정 OFF 스냅샷을 직접 읽었다. 원본 ID1..16 값은 `[2612,3116,3075,2743,2855,2100,1758,1872,2852,2085,1758,1886,3234,2154,2833,3073]`이고 포트는 닫았다. 엄지 값이 읽히지 않거나 과거 자세와 달라 막힌 상태가 아니다. 사용자에게 추가 수동 자세 조정을 요청하지 않는다.
- 기존 초기 구조물 모델 겹침4mm와 분리 구간 모델 추종0.14rad 기준은 보수적인 책 앞끝-5mm 가정의 안정 OFF 자세를 처리하지 못했다. 저장 실패 분석에서 초기 겹침은 최대4.739mm, 모델 추종은0.151858rad였다. 이는 실제 침투량이나 접촉 힘의 측정값이 아니다. `book_passive_support_reentry_policy.py`에 이 초기 분리 경우만 별도 명시해 기존 접촉의 모델 깊이 상한5mm, 초기 구간 모델 추종0.16rad, 뿌리 상승70목표/3.5초 내 분리로 검토했다. 동작 중 새롭거나 더 깊어지는 접촉은 계속 금지하고 분리 뒤 모델 추종0.12rad 및 받침 겹침0.15mm 기준은 유지한다. 실제 전류450mA와 PID, 실물 추종0.18rad, 전압 및 온도와 무진행 중단은 변경하지 않았다. 원래 다른 준비 프로그램의4mm 기준도 바꾸지 않았다.
- v6는 현재 실제 각도를 바꾸지 않은 입력과 +/-4틱 주변67개 및 세 책 앞끝 가정에서201개 정적 조건, 반/기준/1.5배 응답의9개 동역학 조건을 통과했다. 최대 초기 모델 추종 약0.151859rad이며 분리 후에는 별도0.12rad 이내다. 새로운 접촉과 더 깊어지는 접촉, 초기 깊이5mm 초과와 모델 정책 변조 및 분리 실패가 차단되는 장치 없는 검사86개도 통과했다. 실제 복귀 목표를 더 깊게 밀거나 빠르게 만들지 않았으며 이번 현재 자세는 총865개 목표다.
- Camera는 전 턴에 확인한 노출 속성 GET/SET 없는 프레임 모드를 유지한다. 직접65초 촬영502프레임 모두 유효함을 확인한 방식이다. 실제 프레임 유효성, 최신성, 검은 화면 및 단절과 Q 중단 및 시작 전 큐브 마커 증거를 계속 확인한다. 복원이 필요 없는 상태를 검증된 복원으로 표기하지 않는다.
- **다음 사용자 작업:** 손가락과 책 및 본체는 그대로 두고 빈손 상태에서 `leap_book_cube_grasp_trial.cmd` 실행한다. 첫 SPACE는 약지와 엄지 분리 후 빈손 복귀, PLACE에서만 큐브를 놓고 두 번째 SPACE로279개 기존 도달 파지 접근을 실행한다. REMOVE에서 큐브를 뺀 뒤 세 번째 SPACE로 OFF한다. 실행 프로그램은 그때의 새 안정 OFF 각도와 전체 경로를 다시 검증한다. 이번 실행에는 큐브 회전이 포함되지 않는다.
- 근거: `right_hand_rotation_20261024/book20_current_review/grasp_route_fixture_v6/screen.json`, `device_free_validation.json`, `readiness_review.json`. 과거 실제 결과와 실패 증명 및 전시 문구는 보존했고 이번 개발 중 실물 모터 쓰기는0회다.

'''
for name in ('sac_current_status.md','professor_progress_20261015.md'):
    path=ROOT/name;prior=path.read_text(encoding='utf-8')
    if not prior.startswith(entry.splitlines()[0]):
        if prior.startswith('## CURRENT'):prior=prior.replace('## CURRENT','## PREVIOUS',1)
        path.write_text(entry+prior,encoding='utf-8')
print(json.dumps({k:v for k,v in record.items() if k not in ('device_free_validation','actual_off_ticks')},ensure_ascii=False))
