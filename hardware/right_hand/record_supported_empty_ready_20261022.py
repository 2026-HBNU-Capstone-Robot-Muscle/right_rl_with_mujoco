"""Consolidate completed checks and the next required operator action."""
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261022'


def read(name):return json.loads((OUT/name).read_bytes())


def main():
    physical=read('latest_physical_observation.json')
    native=read('supported_empty_integration_validation.json')
    regression=read('camera_signal_fix_tests.json');extra=read('supported_empty_tests.json')
    assert physical['user_reports_no_visible_cube_rotation']
    assert native['passed'] and len(native['fixtures'])==3
    assert regression['passed'] and regression['tests_run']==147
    assert extra['passed'] and extra['tests_run']==6
    from supported_yaw_empty_plan import SupportedYawEmptyPlan,candidate
    from thumb_live_start_gate import validate_proof
    for row in native['fixtures']:
        folder=ROOT/row['fixture'];validate_proof(SupportedYawEmptyPlan(folder/'request.json'),folder/'screen.json')
    source=OUT/'supported_yaw_result.json';model=json.loads(source.read_bytes())
    result=dict(status='EMPTY_operator_rehearsal_ready',
        latest_physical_trial=physical['source_trial'],physical_desired_rotation_success=False,
        user_reports_no_visible_rotation=True,latest_camera_normal=True,
        camera_only_recapture_needed=False,identical_loaded_repetition_requested=False,
        new_launcher='leap_supported_yaw_empty.cmd',cube_allowed=False,
        new_goal_count=len(candidate()),suffix_nominal_seconds=len(candidate())*.05,
        maximum_suffix_travel_ticks=int(abs(candidate()-model['base_goal_ticks']).max()),
        agent_motor_or_camera_starts=0,live_sac=False,algorithm='offline_CEM_fixed_trajectory_NOT_SAC',
        native_start_fixtures=3,static_cases_per_start=67,dynamic_gain_cases_per_start=3,
        existing_regression_tests_passed=142,new_empty_tests_passed=6,unique_tests_covered=148,
        motion_deadline_s=75,cycle_period_s=.05,maximum_goal_step_ticks=2,current_cap_ma=450,
        control_changes=['Only this longer EMPTY rehearsal uses a 75s bounded motion budget.',
                         'Current/voltage/temperature/tracking/progress trips and OFF restoration stay active.',
                         'No further palmward ring curl; ring DIP/PIP/MCP goals only extend from base.',
                         'Fixed suffix joints retain the settling-aware hold guard.',
                         'Black/blank camera frames block start and persistent blank frames stop the trial.'],
        loaded_goal_successes=model['successful_cases'],loaded_goal_cases=model['total_cases'],
        loaded_path_approved=False,
        limitation='Loaded model cases include tilt; a 15-degree pure horizontal rotation is not demonstrated. This release authorizes only the EMPTY commissioning observation.',
        next_user_action='Leave hand as-is with cube OUT. Run launcher; SPACE starts; confirm ring tip clear of palm with SPACE. Observe new ring-involving suffix, then automatic OFF. Reply 했어.')
    (OUT/'supported_empty_ready_review.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    section='''## CURRENT 2026-10-22 15:12 이후: 실물 목표 회전 미달 / 다음은 약지 참여 경로 빈손 검사

- 사용자 재실행 `cube_small_rotation_trial_20261022_151241_962114` 확인. 회전 목표80개가 정확히 실행됐고4.172초 구간을 완주. 최저4.1V, 최대84mA, 최대 추적0.07363rad.16개 OFF/설정 복원·큐브 제거 확인. 사용자는 손가락만 조금 움직이고 큐브는 그대로였다고 답했다. 원하는 실물 회전 성공으로 기록하지 않는다.
- 첫15:05 실행은 큐브 배치 대기 중 영상이 검어져 쥐기/회전 구간을 판단할 수 없었다.15:12 재실행은1384프레임 모두 정상 신호다. 추가 카메라 촬영이나 같은 적재 경로 반복은 요청하지 않는다. 마지막 회전/유지42개 원본 프레임을 `recorded_small_rotation.mp4`로 저장했다. 작은 ID5의두 평면 해에서6~7도 국소 법선 변화 추정이 나오지만, 원근/반사광·중력 기준 미등록 때문에 목표 세계좌표 수평 회전의 증거로 사용하지 않는다.
- 카메라 연결 상태만 확인하던 문제 수정: 검거나 평탄한 영상에서 시작SPACE/사진 저장 차단. 정상 장면3개 연속 신호가 필요하고 동작/대기 중0.5초 이상 빈 영상이면 중단하여 기존 OFF 복원 경로로 들어간다. 장면 신호 검사는 손/큐브 검출 인증이 아니다. 원본 실행 결과는 보존하고 별도 검토 JSON에 결론 기록.
- 약지를 고정한 더 큰 CEM 후보는15도 목표를7개 합성 시작 중0개에서 달성. 약지가 펴지는 방향도 허용한 후보는 일부15도 회전과 기울기가 함께 나와2/7개만 목표 기준을 충족. 이를 적재 실물용으로 내보내지 않았다. SAC 학습/실시간 정책 성공으로 표현하지 않는다.
- 새 `leap_supported_yaw_empty.cmd`는 **큐브 없이** 기존 분리·준비·파지 모양을 거쳐 새 약지 참여 경로256개(명목12.8초)를 따라가고 사진/OFF한다. 약지 끝/PIP/뿌리는 기준보다 손바닥 쪽으로 더 접히지 않는 목표만 허용. 전류450mA/PID·목표당2틱/50ms·전압/추종/진행/온도·카메라·OFF 복원을 유지. 고정 목표 관절은 기존 held guard 유지. 긴 빈손 경로만 시간 한도75초로 명시하고, 목표 수에60ms 지연 모델·마지막 hold·2초 여유를 더한 값이75초 안에 드는 시작만 허용. 기존 v6/작은 회전 한도55초는 유지.
- 최종 새 경로3개 저장 시작에서 각각67개 정적/세 강도 동역학 검사 통과. 기존/새 코드 경로17개 Windows→WSL 재검사. 기존142개와 최종 빈손6개, 합계148개 고유 검사로 시작/중단/읽기·쓰기·카메라 오류/무재시도/OFF 복원/75초 중단 및 주기 중 JSON 재읽기 방지를 확인. 모터/카메라는 조수가 실행하지 않았다.
- 지금 사용자 할 일: 큐브를 빼두고 현재 손을 손으로 다시 만들지 않은 채 새 빈손 cmd 실행. 첫SPACE 시작, 약지 끝이 손바닥과 떨어진 것을 확인한 뒤SPACE. 새 경로 끝에서 자동OFF. 다시 긴 로그 없이 했어만 알리면 저장된 영상/각도를 검토한다. 적재 회전 시험은 이 새 움직임을 실물 확인한 뒤 별도로 준비한다.
- 근거: `right_hand_rotation_20261022/supported_empty_ready_review.json`, `supported_empty_integration_validation.json`, `camera_signal_fix_tests.json`, `supported_empty_tests.json`, 새15:12 실행의 `completion_review.json`·`marker_rotation_review.json`·`relative_marker_pose_review.json`.

'''
    for name in ('professor_progress_20261015.md','sac_current_status.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if not old.startswith(section):path.write_text(section+old,encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('status','new_launcher','cube_allowed','unique_tests_covered','loaded_path_approved')},indent=2))


if __name__=='__main__':main()
