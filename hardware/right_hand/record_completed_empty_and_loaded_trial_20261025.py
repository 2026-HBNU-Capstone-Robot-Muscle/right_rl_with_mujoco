"""Record completion and separately mark the first supervised loaded attempt."""
import datetime,hashlib,json
from pathlib import Path
import numpy as np
from loaded_quarter_trial_plan_20261025 import ROOT,LoadedQuarterTrialPlan,RETURN_PHASE,TURN_PHASE,START_HOLD,validate_experiment_basis
from loaded_quarter_trial_gate_20261025 import validate_proof
OUT=ROOT/'right_hand_rotation_20261024/loaded_quarter_trial_20261025'
REVIEW=ROOT/'right_hand_rotation_20261024/completed_empty_path_review_20261025'
FIXTURE=OUT/'fixture_v3'
def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,value):Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')

def main():
    p=LoadedQuarterTrialPlan(FIXTURE/'request.json');proof=validate_proof(p,FIXTURE/'screen.json')
    assert 'Ran 15 tests' in (FIXTURE/'test_completion.log').read_text(encoding='utf-8')
    actual=json.loads((REVIEW/'completion_review.json').read_bytes());basis=validate_experiment_basis()
    original=json.loads((ROOT/'right_hand_rotation_20261024/original_sequence_review/review_summary.json').read_bytes())
    assert digest(ROOT/'exhibition_20261023/족자_붙여넣기.txt')==original['production_hashes_recorded_without_reissuing_certificates']['exhibition_20261023/족자_붙여넣기.txt']
    assert digest(Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/sequence_flip.py'))=='008f5a6b09b9f3231af6bff1c21a754ec8463b96a7efaa521dffd6bc88081656'
    assert digest(ROOT/'sequence_flip_poses.json')=='cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b'
    oldproof=json.loads((ROOT/'right_hand_rotation_20261024/reverse_quarter_path_empty/fixture_v1/screen.json').read_bytes())
    for name,sha in {**oldproof['code_hashes'],**oldproof['extra_code_hashes']}.items():assert digest(ROOT/name)==sha,name
    rows=p.plan(p.source_position());now=datetime.datetime.now().astimezone().isoformat()
    ready=dict(timestamp=now,user_command='leap_loaded_quarter_trial.cmd',supervised_experimental_loaded_trial_ready=True,
        robust_loaded_execution_ready=False,physical_cube_rotation_verified=False,prior_empty_completion=actual,
        whole_path_continuous_actual_execution_verified=False,experimental_model_review=basis,
        full_hand_route_static_cases=len(proof['static_screens']),full_hand_route_native_cases=len(proof['dynamic_sensitivity']),
        full_hand_route_native_cube_enabled=False,full_hand_route_screen_seconds=proof['elapsed_seconds'],device_free_tests_passed=15,
        loaded_reference_goal_count=sum(phase==TURN_PHASE for phase,q in rows),loaded_reference_seconds=38.,
        total_recorded_goals=len(rows),nominal_motor_on_seconds=(len(rows)+20)*.05,
        nominal_empty_before_place_seconds=sum(phase not in (TURN_PHASE,'stage_7_loaded_observation') for phase,q in rows)*.05,
        source_stable_off='completed_empty_readonly_20261025_091531_043679/request.json',operator_terminal_space_count=3,
        loaded_reference_identical_to_previous_empty_test=True,empty_return_thumb_clearance_ticks=16,
        current_PID_speed_and_actual_health_stops_unchanged=True,initial_same_pad_thumb_neighbor_limit_mm=.5,
        initial_self_scope=proof['initial_self_scope'],initial_self_force_geometry_unchanged=True,
        source_request_sha256=p.SOURCE_SHA,plan_sha256=p.plan_digest(p.source_position()),
        code_hashes=proof['code_hashes'],extra_code_hashes=proof['extra_code_hashes'],
        assistant_motor_goal_writes=0,assistant_torque_changes=0,assistant_read_only_acquisition=True,
        poster_unchanged=True,original_reference_unchanged=True)
    save(OUT/'readiness_review.json',ready)
    save(REVIEW/'replay_boundary_method_review.json',dict(
        excluded_finite_inertia_cube_diagnostic='actual_geometry_and_cube_replay.json',
        excluded_reason='Resetting joint positions with finite hand inertia does not impose a faithful prescribed-contact boundary; do not interpret its dropping Cube as a valid loaded result.',
        current_conditional_diagnostic='actual_geometry_and_cube_replay_kinematic_v2.json',
        current_method='Measured EMPTY qpos/qvel imposed in the infinite effective hand-inertia limit. Hypothetical loaded tracking only; not a native actuator/force or hardware certificate.',
        measured_pose_geometry_check_valid=True,physical_cube_rotation_verified=False,hardware_commands_sent=0))
    for name in ('right_hand_rotation_20261024/current_handoff_20261025.json','right_hand_rotation_20261024/middle_ring_side_review/progress_20261025.json'):
        path=ROOT/name;r=json.loads(path.read_bytes())
        r.update(timestamp=now,next_user_command=ready['user_command'],next_user_command_ready=True,next_user_command_empty_only=False,
            next_user_command_photo_only=False,rotation_in_next_command=True,supervised_experimental_loaded_trial_ready=True,
            reverse_loaded_execution_ready=False,robust_loaded_execution_ready=False,physical_ninety_degree_rotation_verified=False,
            empty_quarter_path_completed_across_two_runs=True,whole_reference_continuously_completed=False,
            empty_continuation_physically_complete=True,empty_continuation_completion=actual,empty_continuation_readiness_current=False,
            loaded_trial_readiness=ready,source_readonly=ready['source_stable_off'],new_device_free_tests_passed=15,
            new_agent_motor_goal_writes=0,new_agent_torque_changes=0,additional_side_photo_needed=False,
            next_action='User runs the first full supervised loaded reference: FIRST SPACE EMPTY; about 37s EMPTY return; PLACE green UP/orange toward WRIST then SECOND SPACE for whole 38s reference and fixed 4s; REMOVE cube before THIRD SPACE/OFF. Review saved video; no physical success claim yet.')
        save(path,r)
    text='''## CURRENT 2026-10-25: 빈손 후반 완료 / 큐브를 올린 전체 38초 경로의 첫 감독 실험 준비

- `reverse_quarter_resume_empty_20261025_090335_740374`가 정상 완료됐다. 실제 477개 읽기와 명령 끝점, 약지 분리 읽기, OFF 및 설정 복원을 확인했다. 최대 추종 0.0982rad, 최대 전류 125mA, 최저 전압 3.7V, 최대 온도 24도다. 카메라 496장이 모두 유효했다. 저장 영상을 직접 확인했고 보이는 사람 손은 본체 쪽이다. 가려진 부분의 실물 접촉이 전혀 없다고 단정하지 않는다.
- 이전 472구간과 이번 288구간을 합쳐 원본 후보의 760개 관절 이동 구간을 모두 빈손으로 확인했다. OFF를 사이에 둔 두 실행이므로 연속 전체 실행 성공으로 표시하지 않는다. 두 기록의 실제 관절값 762개를 CAD에 넣었을 때 손끼리 및 받침 겹침은 0이었다. 접합점 실제 값 차이는 최대 14틱이며 연속 측정으로 가장하지 않았다.
- 실제 빈손 관절 궤적을 적재 때도 따른다고 처방한 별도 진단의 기준 마찰 조건에서는 약 100.2도 회전, 최종 목표 오차 12.0도, 마지막 4초 최대 오차 13.1도와 손 접촉 유지가 나왔다. 정지 대조군은 목표에 도달하지 않았다. 낮거나 높은 마찰의 유지 오차가 커지는 조건도 있다. 이 계산은 고정에 가까운 손 관성으로 처방된 위치와 속도를 구현한 가정이며 실제 적재 모터 식별이나 물리 성공 증거가 아니다. 초기 유한 손 관성의 단순 위치 재설정 계산은 제외 사유와 함께 보존했다.
- 종료 직후 OFF 값은 계속 내려가고 있어 출발 기준으로 쓰지 않았다. `completed_empty_readonly_20261025_091531_043679`에서 현재 안정 OFF 값을 읽었으며 모터 쓰기 및 토크 변경은 0회다. 새 빈손 복귀는 가장 가까운 기존 경로 지점으로 들어간 뒤 시작점을 되짚는다. 엄지와 약지 끝을 먼저 펴고, 빈손 복귀에서만 엄지 끝 목표를 16틱 더 펴서 낮은 모델 응답의 손바닥 모서리 접촉을 제거했다. 적재 때 760개 목표는 이전 빈손 실행과 정확히 같다.
- 초기 엄지 패드와 이웃 손바닥 도형의 +/-4틱 범위를 별도로 명시하고 새 이웃 쌍의 모델 상한은 0.5mm로 제한했다. 그 범위가 분리 후에도 허용되는 것은 아니며 이후 새 접촉 및 깊어짐을 거부한다. 실제 분리 읽기 전에는 다음 목표를 보내지 않는다. 형상과 접촉 힘, 실제 전류와 PID, 2틱/50ms 및 전압과 추종 중단 기준은 그대로다.
- 다음 사용자 작업은 **`leap_loaded_quarter_trial.cmd`**다. 첫 SPACE는 빈손이며 큐브는 손과 화면 밖에 둔다. 검사 후 약 37초 빈손 복귀를 수행한다. PLACE에서 이전처럼 검지와 중지의 평평한 링크 위에 초록 위와 주황 손목 ID41 방향으로 큐브를 놓고 손을 뺀 뒤 두 번째 SPACE를 누른다. 3초 뒤 전체 38초 경로와 4초 정지를 한 번 수행한다. REMOVE에서 먼저 큐브를 빼고 세 번째 SPACE로 OFF한다. 현재 명목 전체 ON 시간은 78.7초이며 모델 검사 및 사용자 기다림은 별도다. Q와 Esc 및 Ctrl+C는 중단이며 자동 재시도는 없다. 끝나면 했어만 알리면 된다.
- `loaded_quarter_trial_20261025/fixture_v3`의 전체 손 경로 201개 정적 조건과 9개 동역학 조건 및 장치 없는 검사 15개를 통과했다. 모델 검사는 약 25초다. 이 native 검사는 큐브 없는 손 경로를 검증하고 적재 접촉은 별도 조건부 진단이다. 다음 명령은 최초 감독 적재 실험으로만 준비됐으며 강건한 적재 제어기나 실제 90도 성공으로 표시하지 않는다. 원본 및 과거 실제 기록과 전시 문구는 변경하지 않았다.

<!-- END COMPLETED EMPTY AND LOADED TRIAL 20261025 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if old.startswith('## CURRENT 2026-10-25: 빈손 후반 완료'):
            old=old.split('<!-- END COMPLETED EMPTY AND LOADED TRIAL 20261025 -->',1)[1].lstrip()
        path.write_text(text+old.replace('## CURRENT','## PREVIOUS'),encoding='utf-8')
    (OUT/'README.md').write_text(text+'\n원본 실제 기록과 빈손 및 적재 가정 계산은 각각 별도 폴더에 보존했다. fixture_v1과 v2의 실패는 허용치 확대 없이 복귀 순서를 수정한 근거다. 조건부 계산 영상은 `../completed_empty_path_review_20261025/measured_quarter_diagnostic.gif`에 저장한다.\n',encoding='utf-8')
    print(json.dumps({k:ready[k] for k in ('supervised_experimental_loaded_trial_ready','robust_loaded_execution_ready','user_command','loaded_reference_seconds','nominal_motor_on_seconds','device_free_tests_passed')},ensure_ascii=False))

if __name__=='__main__':main()
