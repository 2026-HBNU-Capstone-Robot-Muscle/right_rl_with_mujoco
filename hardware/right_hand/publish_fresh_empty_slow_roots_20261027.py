"""Record the three voltage stops and the reviewed slower-root revision."""
import datetime,hashlib,json
from fresh_empty_grasp_plan_20261027 import ROOT,REQUEST,PROOF,FreshEmptyGraspPlan
from fresh_empty_grasp_gate_20261027 import validate_proof
def main():
    plan=FreshEmptyGraspPlan(REQUEST);proof=validate_proof(plan,PROOF);rows=plan.plan(plan.source_position());folder=REQUEST.parent;diagnostic=folder/'voltage_stop_review_20261027/review.json';review=json.loads(diagnostic.read_bytes());assert len(review['trials'])==3 and all(r['all_torque_off_verified'] and r['settings_restored'] for r in review['trials']);now=datetime.datetime.now().astimezone().isoformat(timespec='seconds');command='& "'+str(ROOT/'leap_fresh_empty_grasp.cmd')+'"'
    protected={'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b','right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716','exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0'}
    for name,sha in protected.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==sha
    state=dict(status='three_voltage_stops_reviewed_slower_root_v5_operator_trial_pending',timestamp=now,updated_local=now,plan_version=plan.PLAN_VERSION,latest_review=diagnostic.relative_to(ROOT).as_posix(),latest_user_trials=[r['source'] for r in review['trials']],last_three_stop_reason='3.6V during index/middle root lift; recovery criteria not met',all_three_torque_off_and_settings_restored=True,next_action='Operator performs one revised EMPTY-only run; assistant reads saved evidence.',next_user_action='Cube out. Leave fingers as they are. Show all fingertips in camera. Run same updated command once and press terminal SPACE.',next_user_command=command,next_user_command_ready=True,next_motion_command_ready=True,ready_for_operator_empty_trial=True,operator_empty_execution_ready=True,pending_user_input=None,hardware_execution_allowed=False,assistant_motor_execution_allowed=False,assistant_motor_commands_sent=0,motor_writes_sent_this_turn=0,camera_opened_this_turn=False,serial_ports_opened_this_turn=False,full_sensitivity_complete=True,empty_route_goals=len(rows),empty_route_nominal_seconds=len(rows)*.075,root_initial_average_step_ticks=1.5,root_initial_travel_ticks=120,root_tail_goal_step_ticks=1,previous_root_maximum_step_ticks=2,sample_period_s=.075,goal_endpoints_unchanged=True,voltage_thresholds_and_budget_unchanged=True,slower_route_voltage_improvement_physically_verified=False,no_device_regression_tests_passed=21,empty_review=PROOF.relative_to(ROOT).as_posix(),empty_review_sha256=hashlib.sha256(PROOF.read_bytes()).hexdigest(),empty_review_cases=dict(static=201,dynamic=9,camera_hold=9,all_passed=True),ready_for_real_yaw=False,horizontal_loaded_execution_ready=False,physical_horizontal_90_verified=False,horizontal_quarter_turn_verified=False,physical_vertical_successes_total=2,additional_marker_measurement_required=False,additional_capture_required=False,protected_hashes=protected)
    paths=[folder/'progress_20261027.json',ROOT/'right_hand_rotation_20261026/horizontal_next_step_20261026.json',ROOT/'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json']
    for path in paths:
        old=json.loads(path.read_bytes());old.update(state);path.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (folder/'slow_roots_v5_handoff.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block=f'''## CURRENT 2026-10-27: 반복 중단 3회 원인 확인, 느린 뿌리 경로 v5 검증 완료

- 사용자 실행 02:40:55, 02:43:12, 02:48:14의 저장 결과를 모두 확인했다. 약지 분리 중 최소 4.2V였고 검지/중지 뿌리 이동 중 3.6V로 내려갔다. 최종 회복 읽기 최소값은 각각 3.6/3.7/3.7V로, 150ms 내 전체 ID가 3.8V 이상 두 번이라는 기존 기준을 만족하지 못했다. 두 번째 실행은 한 차례 정상 회복 뒤 다시 중단했다. 세 실행 모두 토크 OFF 및 설정 복구가 확인됐고 검은 영상은 0장이다. 시작 자세나 새 경로 검사가 원인은 아니다.
- 저장 전압을 기존 검사기에 재생해도 지속 저전압은 거부됐고 두 번의 정상 전압이면 회복했다. 3.5V 즉시 중단도 유지했다. 따라서 이번 세 번은 단순 확인 지연 오류로 분류하지 않는다. 전원 경로의 정확한 저항/접속 원인을 실측한 것은 아니다.
- v5는 약지 분리, 목표 끝 자세, 450mA/PID, 관절 범위 및 모든 전압/전류/추종 중단 기준을 유지한다. 뿌리 초기 120틱은 평균 1.5틱/75ms(기존보다 25% 느림), 나머지는 1틱/75ms(50% 느림)로 바꿨다. 총 {len(rows):,}개 목표, 명목 {len(rows)*.075:.3f}초다. 전압 개선은 아직 실물 검증 전이며 모델에는 배선 전압 강하가 포함되지 않는다.
- 초반부터 전부 1틱으로 줄인 v4는 받침 앞끝 -5mm 조건에서 초기 겹침 해소가 92개 목표로 늦어 기존 70개 제한을 넘었다. v4는 거부됐고 실물로 실행하지 않았다. v5는 기존 제한을 늘리지 않고 초기 분리 속도를 조절해 201개 기하 검사, 9개 추종 검사, 9개 영상 공백 고정 검사 전부를 통과했다. 현재 코드/경로/근거 연결 및 전압 재생을 포함한 장치 없는 검사 21개가 통과했다.
- 다음 작업은 수정된 같은 명령의 빈손 실행 1회다. 큐브는 손과 화면 밖에 두고 손가락은 그대로 둔다. 현재 저장 영상은 손끝 일부가 화면 왼쪽에서 잘리므로 웹캠만 조금 뒤로 빼 손끝 전체를 보여준다. 터미널 SPACE 한 번으로 새 현재각 검사 후 약 2분 20초 동작하고 자동 OFF한다. 사용자는 '했어'만 보내면 된다. 다시 같은 이유로 멈추면 같은 실행을 반복시키지 않고 저장값으로 다음 조치를 정한다.

```powershell
{command}
```

- 증거: `{diagnostic.relative_to(ROOT).as_posix()}`. v5 검토: `{PROOF.relative_to(ROOT).as_posix()}`. 이번 작업에서 실제 장치 열기/목표/토크/설정 쓰기는 0회다. 실물 수직 뒤집기 성공 총 2회와 수평 90도 미검증 상태를 유지한다. 전시 족자 및 원본 관절 범위/기록 자세 파일은 수정하지 않았다.

<!-- END FRESH EMPTY SLOW ROOTS 20261027 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END FRESH EMPTY SLOW ROOTS 20261027 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip()
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(block+old,encoding='utf-8')
    print(json.dumps(dict(status=state['status'],nominal_seconds=state['empty_route_nominal_seconds'],model_cases=219,tests=21,physical_voltage_improvement_verified=False,motor_writes=0,next_user_command=command),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
