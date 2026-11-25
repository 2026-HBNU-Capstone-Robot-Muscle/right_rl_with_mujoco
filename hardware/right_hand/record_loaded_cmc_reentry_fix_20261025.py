"""Record the concrete zero-write failure and matched recovery verification."""
import datetime,hashlib,json
from pathlib import Path
import numpy as np
from loaded_quarter_trial_plan_20261025 import ROOT,LoadedQuarterTrialPlan,TURN_PHASE,END_HOLD
from loaded_quarter_trial_gate_20261025 import validate_proof
from loaded_quarter_reentry_20261025 import cmc_native_min,limits
OUT=ROOT/'right_hand_rotation_20261024/loaded_quarter_trial_20261025'
FIXTURE=OUT/'fixture_v5'
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,r):Path(p).write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
def main():
    log=(FIXTURE/'test_completion.log').read_text(encoding='utf-8');assert 'Ran 21 tests' in log and log.rstrip().endswith('OK')
    summaries=[]
    for folder in (FIXTURE,OUT/'fixture_v5_regression_0915'):
        p=LoadedQuarterTrialPlan(folder/'request.json');proof=validate_proof(p,folder/'screen.json')
        summaries.append(dict(fixture=folder.relative_to(ROOT).as_posix(),source_sha256=p.SOURCE_SHA,
            source_id4=int(p.source_position()[3]),static_cases=len(proof['static_screens']),native_cases=len(proof['dynamic_sensitivity']),
            minimum_native_cmc_after_release_ticks=min(s['native_cmc_after_release_min_ticks'] for s in proof['dynamic_sensitivity']),
            maximum_self_after_release_mm=max(s['max_self_after_release_mm'] for s in proof['dynamic_sensitivity']),
            maximum_support_after_clearance_mm=max(s['max_support_after_lift_mm'] for s in proof['dynamic_sensitivity']),
            elapsed_seconds=proof['elapsed_seconds'],accepted=True))
    p=LoadedQuarterTrialPlan(FIXTURE/'request.json');proof=validate_proof(p,FIXTURE/'screen.json');rows=p.plan(p.source_position())
    actual_path=ROOT/'loaded_quarter_trial_20261025_165036_298713/result.json';actual=json.loads(actual_path.read_bytes())
    assert actual['write_events']==[] and actual['all_torque_off_verified']
    baseline=json.loads((ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json').read_bytes())
    assert digest(ROOT/'exhibition_20261023/족자_붙여넣기.txt')==baseline['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    assert digest(Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'))=='008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'
    old=json.loads((ROOT/'right_hand_rotation_20261024/reverse_quarter_path_empty/fixture_v1/screen.json').read_bytes())
    for name,sha in {**old['code_hashes'],**old['extra_code_hashes']}.items():assert digest(ROOT/name)==sha,name
    ready_path=OUT/'readiness_review.json';ready=json.loads(ready_path.read_bytes());backup=OUT/'readiness_before_cmc_fix.json'
    if not backup.exists():backup.write_bytes(ready_path.read_bytes())
    failure=dict(source=actual_path.relative_to(ROOT).as_posix(),source_sha256=digest(actual_path),motor_writes=0,
        torque_off_verified=True,id4_start_ticks=int(p.source_position()[3]),native_original_lower_ticks=float(limits()[0][3]),
        original_integer_native_min_ticks=cmc_native_min(),deficit_ticks=cmc_native_min()-int(p.source_position()[3]),
        cause='Existing admission allowed observed passive OFF extension, but path validation rejected the starting and monotonic re-entry values.',
        generated_goal_step_limit_was_not_the_cause=True)
    save(OUT/'id4_reentry_failure_review.json',failure)
    now=datetime.datetime.now().astimezone().isoformat()
    ready.update(timestamp=now,user_command='leap_loaded_quarter_trial.cmd',supervised_experimental_loaded_trial_ready=True,
        robust_loaded_execution_ready=False,physical_cube_rotation_verified=False,fixture=FIXTURE.relative_to(ROOT).as_posix(),
        source_stable_off='loaded_quarter_trial_20261025_165036_298713/fresh_off_request.json',source_request_sha256=p.SOURCE_SHA,
        plan_version=p.PLAN_VERSION,plan_sha256=p.plan_digest(p.source_position()),
        total_recorded_goals=len(rows),nominal_motor_on_seconds=(len(rows)+20)*.05,
        nominal_empty_before_place_seconds=sum(phase not in (TURN_PHASE,END_HOLD) for phase,q in rows)*.05,
        combined_motion_limit_seconds=p.ON_TIMEOUT,previous_combined_motion_limit_seconds=100,
        combined_time_limit_change_reason='Longer bounded EMPTY recovery from the newly observed OFF posture; current/rate/voltage/tracking/temperature limits unchanged.',
        native_ranges_widened_in_limits_file=False,only_existing_observed_id4_passive_reentry_used=True,
        id4_monotonic_reentry_target_ticks=1872,measured_id4_native_min_before_next_phase=cmc_native_min(),
        early_roots_for_passive_cmc_profile=True,loaded_reference_goal_count=760,loaded_reference_seconds=38.,
        loaded_reference_identical_to_previous_empty_test=True,device_free_tests_passed=21,
        full_hand_route_static_cases=201,full_hand_route_native_cases=9,recovery_regressions=summaries,
        code_hashes=proof['code_hashes'],extra_code_hashes=proof['extra_code_hashes'],
        assistant_motor_goal_writes=0,assistant_torque_changes=0,assistant_read_only_acquisition_this_fix=False,
        poster_unchanged=True,original_reference_unchanged=True)
    save(ready_path,ready)
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json'):
        path=ROOT/name;r=json.loads(path.read_bytes())
        r.update(timestamp=now,next_user_command=ready['user_command'],next_user_command_ready=True,next_user_command_empty_only=False,
            next_user_command_photo_only=False,rotation_in_next_command=True,supervised_experimental_loaded_trial_ready=True,
            reverse_loaded_execution_ready=False,robust_loaded_execution_ready=False,physical_ninety_degree_rotation_verified=False,
            loaded_trial_readiness=ready,loaded_id4_reentry_failure_review=failure,source_readonly=ready['source_stable_off'],
            new_device_free_tests_passed=21,new_agent_motor_goal_writes=0,new_agent_torque_changes=0,
            next_action='Same loaded trial command, fixed passive ID4 re-entry. FIRST SPACE EMPTY; about 48s EMPTY preparation, PLACE green UP/orange toward WRIST then SECOND SPACE for 38s turn plus 4s hold; REMOVE cube before THIRD SPACE/OFF. First physical Cube rotation remains unverified.')
        save(path,r)
    text='''## CURRENT 2026-10-25: OFF 엄지 뿌리 복귀 누락 수정 / 같은 전체 회전 실험 재준비

- 16:50 실행은 경로 검사에서 중단됐으며 실제 모터 쓰기 0회와 토크 OFF를 확인했다. 2틱 이동 제한에 걸린 것이 아니다. ID4 출발값 1804틱이 설정 하한 1820.49틱보다 낮았고, 이미 허용된 수동 OFF 처짐을 경로 코드가 이동 목표와 동일하게 처리해 거부한 오류다. 실제 관절 범위 파일이나 EEPROM은 변경하지 않았다.
- 초기 빈손 단계에서 엄지 뿌리를 원래 범위 안쪽 1872틱으로 천천히 돌린다. 복귀 동안 더 낮아지는 목표를 금지하고, 그 단계 이후 모든 목표는 원래 관절 범위 안에 둔다. 다음 단계는 신선한 실제 ID4 읽기가 정수 하한 1821틱 이상일 때만 허용한다. 다른 관절의 원래 범위와 2틱/50ms, 전류 및 PID, 실제 전압 및 추종 중단은 그대로다.
- 이번 자세에서 복귀 관절 이동이 더 커졌으므로 전체 동작 시간 상한만 100초에서 120초로 조정했다. 준비와 회전의 명목 합계는 약 90.4초다. 빈손 준비에서는 세 손가락 뿌리를 먼저 올려 받침에서 분리하고, 다른 관절의 긴 보간 때문에 이 분리가 늦어지지 않게 했다. 접촉 상한이나 분리 기한을 늘려 통과시킨 것은 아니다. 적재 회전의 760개 목표와 38초 경로는 바꾸지 않았다.
- 현재 자세의 `fixture_v5`와 이전 정상 자세의 `fixture_v5_regression_0915`가 각각 201개 정적 조건과 9개 동역학 조건을 통과했다. 독립 검증과 장치 없는 검사 21개도 통과했다. 실패한 개발 결과와 이전 v3 실행 코드는 보존했다. 이번 수정에서 실제 장치는 열지 않았고 모터 및 토크 쓰기는 없다.
- 다음 사용자 작업은 이전과 같은 **`leap_loaded_quarter_trial.cmd`**다. 손가락과 받침은 그대로 둔다. 첫 SPACE는 큐브를 손과 화면 밖에 둔 빈손 시작이다. 검사 후 약 48초 준비 복귀를 수행한다. PLACE에서 검지와 중지의 평평한 링크 위에 초록 위와 주황 손목 ID41 방향으로 큐브를 놓고 손을 뺀 뒤 두 번째 SPACE를 누른다. 전체 38초 경로와 4초 정지 뒤 REMOVE에서 큐브를 먼저 빼고 세 번째 SPACE로 OFF한다. Q와 Esc 및 Ctrl+C로 중단할 수 있다. 끝나면 했어만 알리면 저장 파일을 직접 확인한다.
- 실제 큐브 90도 회전은 아직 미검증이며 다음 명령은 감독 실험이다. 기존 조건부 시뮬레이션을 실물 성공으로 표시하지 않는다. 원본과 실제 기록 및 전시 문구는 변경하지 않았다.

<!-- END PASSIVE ID4 REENTRY FIX 20261025 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;oldtext=path.read_text(encoding='utf-8')
        if oldtext.startswith('## CURRENT 2026-10-25: OFF 엄지 뿌리 복귀 누락'):
            oldtext=oldtext.split('<!-- END PASSIVE ID4 REENTRY FIX 20261025 -->',1)[1].lstrip()
        path.write_text(text+oldtext.replace('## CURRENT','## PREVIOUS'),encoding='utf-8')
    (OUT/'README.md').write_text(text,encoding='utf-8')
    print(json.dumps(dict(ready=True,command=ready['user_command'],tests=21,nominal_motor_on_seconds=ready['nominal_motor_on_seconds'],regressions=summaries),ensure_ascii=False))
if __name__=='__main__':main()
