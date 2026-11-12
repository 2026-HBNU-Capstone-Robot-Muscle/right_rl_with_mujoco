"""Record the user's actual quarter-turn objective and diagnostic limitations."""
import datetime as dt
import json
from pathlib import Path
root=Path(__file__).resolve().parent
stamp=dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).isoformat()
goal=dict(updated_at=stamp,user_confirmed_target_degrees=90,
    objective='Right hand reorients the WHOLE cube 90 degrees in the requested direction while retaining it.',
    not_a_cube_layer_turn=True,small_3_degree_probe_is_diagnostic_only=True,
    small_probe_does_not_establish_90_degree_reachability=True,
    current_small_angle_measurement_validated=False,current_physical_90_degree_success_verified=False,
    success_evidence='Continuous physical video shows the intended quarter-turn face change, without a person turning the cube or relying on torque-OFF motion.',
    manual_initial_placement_allowed=True,
    orientation_error_must_be_reported_when_estimated=True,
    no_unvalidated_subdegree_precision_claims=True,
    next_planning_requirement='Evaluate the complete 90-degree route, including any necessary regrasp, with joint bounds and actual support/self-contact geometry.',
    stagewise_physical_validation='Short sections may validate an end-to-end quarter-turn candidate; repeated tiny motions alone are not goal completion.',
    pending_readonly_side_photo_purpose='Compare actual middle/ring housing geometry with the model; not a 3-degree rotation measurement.',
    pending_command='leap_middle_ring_side_readonly.cmd',
    actual_motor_current_PID_and_voltage_tracking_stops_unchanged=True)
(root/'right_hand_90_degree_objective.json').write_text(json.dumps(goal,ensure_ascii=False,indent=2),encoding='utf-8')
entry=f'''## CURRENT {stamp[:16].replace('T',' ')}: 사용자 목표는 오른손 큐브 전체90도 회전 /3도 진단 반복은 성공 기준에서 제외

- 사용자가 "90도 돌리는게 목표"라고 명확히 정정했다. 목표는 오른손으로 큐브 전체를 원하는 방향으로90도 재정렬하고 놓치지 않는 것이다. 왼손의 레이어 회전과 구분한다. 최종 실물 근거는 시작과 끝의 면 방향 변화가 보이는 연속 영상이며 사람의 큐브 회전이나 토크 OFF 뒤 내려앉음은 성공 근거가 아니다.
- 기존3도 후보는 힘이 큐브에 전달되는지 보기 위한 접촉 진단이었다. 현재 촬영과 좌표 정합으로는 그3도 자체도 신뢰성 있게 검증하지 못했다. 작은 변화나 명령 완료를90도 가능성 또는 목표 완료 진척으로 표기하지 않는다. 작은 회전 경로를 단순히 크게 만들거나 반복하면90도가 된다고 가정하지 않는다.
- 다음 오프라인 계획은90도 전체 경로와 필요한 재파지까지 평가한다. 시뮬레이션의 손 강제 궤적 가정과 실제 모터 응답 및 접촉 차이를 구분해야 한다. 실물은 전체 경로 후보를 나눠 검증할 수 있으나3도 진단만을 반복하는 계획으로 돌아가지 않는다. 수치 각도 오차를 추정하면 함께 기록하고 검증되지 않은 미세 각도 정밀도를 주장하지 않는다.
- 대기 중인 `leap_middle_ring_side_readonly.cmd`의 옆면 사진은3도 회전 측정용이 아니라, 이전 회전 검사에서 빠졌던 중지와 약지 모터 외형을 실제 손과 비교하기 위한 입력이다. 손가락과 받침은 바꾸지 않고 카메라만 옆으로 옮기며 목표나 토크 및 설정 쓰기는 없다. 이 입력을 받기 전 실제 접촉을 단정하거나 같은 작은 회전 명령을 재요청하지 않는다.
- 현재 실물90도 성공은 미확인이다. 세부 기준은 `right_hand_90_degree_objective.json`에 기록했다. 전시 문구와 실제 전류 및 PID와 중단 기준은 변경하지 않았다.

'''
for name in ('sac_current_status.md','professor_progress_20261015.md'):
    path=root/name;prior=path.read_text(encoding='utf-8')
    if prior.startswith('## CURRENT'):prior=prior.replace('## CURRENT','## PREVIOUS',1)
    path.write_text(entry+prior,encoding='utf-8')
print(json.dumps(dict(target_degrees=90,small_probe_is_not_success=True,
    physical_quarter_turn_verified=False,pending_input='existing-hand middle/ring side view; no motion')))
