"""Preserve evidence and the next physical checkpoint without claiming rotation."""
import datetime as dt
import hashlib
import json
from book_small_yaw_plan import ROOT,BookSmallYawPlan,load_candidate,CANDIDATE_SHA
from book_small_yaw_gate import validate_proof
from leap_book_small_yaw_trial import require_offline_review
from leap_book_cube_grasp_trial import require_offline_review as grasp_review

development=require_offline_review();grasp_review()
folder=ROOT/'right_hand_rotation_20261024/book20_current_review/small_yaw_current_215126'
plan=BookSmallYawPlan(folder/'request.json');proof=validate_proof(plan,folder/'screen.json')
tests=json.loads((folder.parent/'small_yaw_fixture_v1/device_free_validation.json').read_bytes())
if not tests['successful'] or tests['tests_run']!=95:raise RuntimeError('Incomplete device-free verification')
source=ROOT/'book_settled_off_readonly_20261024_215126_125157'
read=json.loads((source/'result.json').read_bytes());actual=json.loads((source/'request.json').read_bytes())
if (read['status']!='stable_current_off_read' or read.get('motion_admission_error')
    or read['servo_write_commands_sent']!=0 or actual['off_state']['position']!=plan.source_position().tolist()):
    raise RuntimeError('Latest stable source not preserved')
candidate,_=load_candidate()
trial_folder=ROOT/'book_cube_grasp_trial_20261024_212855_824047'
trial=json.loads((trial_folder/'result.json').read_bytes())
samples=trial['samples']
tracking=max(max(s['tracking_rad']) for s in samples)
current=max(max(abs(v) for v in s['current_ma']) for s in samples)
record=dict(review_time=dt.datetime.now().astimezone().isoformat(),implementation_ready_for_supervised_trial=True,
    plan_version=plan.PLAN_VERSION,command='leap_book_small_yaw_trial.cmd',
    completed_grasp_source=trial_folder.relative_to(ROOT).as_posix(),
    latest_actual_off_read=source.relative_to(ROOT).as_posix(),actual_off_ticks=plan.source_position().tolist(),
    servo_write_commands_sent_by_assistant=0,fresh_runtime_read_and_full_screen_still_required=True,
    source_replay_is_not_runtime_approval=True,static_cases=len(proof['static_screens']),
    dynamic_cases=len(proof['dynamic_sensitivity']),device_free_tests=tests,
    candidate_sha256=CANDIDATE_SHA,proof_sha256=hashlib.sha256((folder/'screen.json').read_bytes()).hexdigest(),
    simulated_action_minus_same_closure_control_yaw_deg=candidate['simulated_action_minus_control_yaw_range_deg'],
    exploratory_active_cube_scenarios=7,simulation_registration_verified=False,
    hand_prescribed_in_active_cube_search=True,friction_measured=False,
    goal_samples=len(plan.plan(plan.source_position())),
    delayed_cycle_budget_seconds=(len(plan.plan(plan.source_position()))+40)*.06+2,
    actual_tracking_stop_rad=.18,actual_current_cap_ma=450,actual_pid_and_voltage_stops_unchanged=True,
    desired_physical_yaw_degrees=-3.,direction='clockwise_viewed_from_above',
    physical_cube_rotation_verified=False,physical_three_degree_amplitude_verified=False,live_sac=False,
    observation_before_cube_removal_required=True,
    next_user_action='EMPTY/FIRST SPACE; PLACE GREEN up ORANGE toward camera/SECOND SPACE; '
        'automatic grasp + 3s baseline + 4.5s small turn + 2s hold; REMOVE/THIRD SPACE -> OFF')
(folder/'readiness_review.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
stamp=dt.datetime.now().strftime('%Y-%m-%d %H:%M')
entry=f'''## CURRENT {stamp}: 새 파지 접근 실물 완료 / 작은 시계방향 회전 실행 준비

- `book_cube_grasp_trial_20261024_212855_824047`에서 빈손 복귀, PLACE, 기존에 도달한279개 파지 목표, REMOVE와 OFF 및 설정 복원이 모두 완료됐다. 영상937프레임도 모두 유효하다. 큐브가 손가락 사이에 유지됐으나 파지 힘이나 목표 방향 회전은 아직 입증되지 않았다. 로그의 최소 전압4.1V, 최대 전류{current:g}mA, 최대 추종 오차{tracking:.6f}rad다.
- OFF 뒤 엄지 뿌리 ID4가 추가로 내려오는 현상을 새 안정 읽기에서 확인했다. 이전1778틱 고정 모델 하한을 출발 각도에 맞춰 계산하도록 바꿨다. 허용하는 수동 출발 범위는 원래 하한 아래 최대10도이며, 각 실제 경로의 모델 하한은 그때 읽은 ID4보다4틱 아래까지만 적용한다. 복귀 명령은 출발점보다 더 내려가지 않고 원래 범위로 돌아온다. 영점이나 다른15개 관절 범위와 실물 전류 및 PID 및 중단 기준을 바꾼 것이 아니다.
- 기존 회전 경로를 새로 실제 도달한 파지 자세로 옮긴 결과는 같은 접촉 보완 후 정지하는 비교 조건과 거의 차이가 없어 제외했다. 새 후보는 접촉 보완20개 목표와 작은 시계방향 회전70개 목표다. 움직이지 않는 뿌리와 약지는 그대로 유지하고 한 단계2틱 및 전체64틱 이내다. 전류450mA, 기존 PID와 실제 추종0.18rad 및 전압 및 무진행 중단 기준을 유지한다.
- 활성 큐브의 탐색 시뮬레이션7조건에서는 같은 접촉 보완 후 정지하는 비교 조건보다 시계방향으로 약0.61~1.50도 더 변했다. 목표는 위에서 보아 시계방향3도지만 실제3도 달성을 확인한 것은 아니다. 손 궤적을 지정한 탐색이며 마커 좌표 정합과 실제 마찰은 미확인이다. 이를 SAC 정책이나 강건한 회전 성공으로 표기하지 않는다.
- 마지막 직접 읽기 `book_settled_off_readonly_20261024_215126_125157`는 토크와 오류 및 watchdog 모두0, 안정 OFF였다. 쓰기는0회다. 이 실제 원본 각도를 바꾸지 않은 새 전체 경로가201개 정적 조건과9개 동역학 조건을 통과했다. 기존 파지 v7 경로도 별도로 통과했고, 장치 없는 검사95개가 통과했다. 자세나 코드 및 증거 파일이 바뀌면 승인하지 않으며 실행 시 새 안정 OFF 각도와 전체 경로를 다시 검사한다.
- **다음 사용자 작업:** `leap_book_small_yaw_trial.cmd` 실행. 책과 본체 및 손가락은 그대로 둔다. 빈손에서 첫 SPACE, PLACE에서 초록 위 및 주황 카메라 방향으로 같은 위치에 큐브를 놓고 손을 뺀 뒤 두 번째 SPACE. 파지 후3초 정지,4.5초 작은 회전,2초 정지한다. REMOVE 안내 뒤 큐브를 빼고 세 번째 SPACE로 OFF한다. 큐브를 빼기 전까지의 영상만 회전 증거로 사용한다. 완료 뒤에는 "했어"만 알려주면 저장 결과를 직접 검토한다.
- 근거: `right_hand_rotation_20261024/book20_current_review/small_yaw_current_215126/screen.json`, `readiness_review.json`, `small_yaw_fixture_v1/device_free_validation.json`. 전시 문구는 변경하지 않았다.

'''
for name in ('sac_current_status.md','professor_progress_20261015.md'):
    path=ROOT/name;prior=path.read_text(encoding='utf-8')
    if not prior.startswith(entry.splitlines()[0]):
        if prior.startswith('## CURRENT'):prior=prior.replace('## CURRENT','## PREVIOUS',1)
        path.write_text(entry+prior,encoding='utf-8')
print(json.dumps({k:v for k,v in record.items() if k not in ('device_free_tests','actual_off_ticks')},ensure_ascii=False))
