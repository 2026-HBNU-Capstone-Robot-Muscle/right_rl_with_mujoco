"""Publish passed EMPTY-only checks and the next operator action. No devices."""
import datetime,hashlib,json
from pathlib import Path
from fresh_empty_grasp_plan_20261027 import ROOT,REQUEST,PROOF,FreshEmptyGraspPlan
from fresh_empty_grasp_gate_20261027 import validate_proof

def main():
    plan=FreshEmptyGraspPlan(REQUEST);proof=validate_proof(plan,PROOF);out=REQUEST.parent;latest=json.loads((ROOT/'right_hand_rotation_20261026/fresh_empty_start_latest.json').read_bytes());readonly=json.loads((ROOT/latest['result']).read_bytes());rows=plan.plan(plan.source_position());cmd='& "'+str(ROOT/'leap_fresh_empty_grasp.cmd')+'"';now=datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    protected={'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b','right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716','exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0'}
    for name,sha in protected.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==sha,name
    state=dict(status='new_empty_grasp_route_review_passed_operator_trial_pending',timestamp=now,updated_local=now,power_confirmation_received=True,cube_removed_user_confirmed=True,pending_user_input=None,next_action='Operator runs the reviewed EMPTY-only posture check; then assistant reviews saved logs/video.',next_user_action='Keep cube out of the hand and camera, aim camera at the whole hand, run the new command and press terminal SPACE once.',next_user_command=cmd,next_user_command_ready=True,next_motion_command_ready=True,operator_empty_execution_ready=True,ready_for_operator_empty_trial=True,hardware_execution_allowed=False,assistant_motor_execution_allowed=False,assistant_motor_commands_sent=0,motor_writes_sent_this_turn=0,camera_opened_this_turn=False,latest_readonly=latest['result'],readonly_source=latest,readonly_snapshots=3,readonly_max_drift_ticks=readonly['max_drift_ticks'],full_sensitivity_complete=True,empty_route_goals=len(rows),empty_route_nominal_seconds=len(rows)*.075,early_ring_release_ticks=80,index_root_post_release_minimum_ticks=2048,ready_for_real_yaw=False,horizontal_loaded_execution_ready=False,physical_vertical_successes_total=2,physical_vertical_successes_consecutive=False,physical_horizontal_90_verified=False,horizontal_quarter_turn_verified=False,arbitrary_direction_rotation_verified=False,additional_marker_measurement_required=False,additional_capture_required=False,geometry_model_only=True,physical_calibration_verified=False,empty_review=PROOF.relative_to(ROOT).as_posix(),empty_review_sha256=hashlib.sha256(PROOF.read_bytes()).hexdigest(),empty_review_cases=dict(static=201,dynamic=9,camera_hold=9,all_passed=True),default_no_device_check_passed=True,no_device_regression_tests_passed=15,preview=(out/'preview_v3/current_and_target.png').relative_to(ROOT).as_posix(),protected_hashes=protected)
    progress=out/'progress_20261027.json'
    if progress.exists() and not (out/'progress_20261027_before_ready.json').exists():(out/'progress_20261027_before_ready.json').write_bytes(progress.read_bytes())
    progress.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    for name in ('horizontal_next_step_20261026.json','current_camera_and_yaw_review_20261026.json'):
        path=ROOT/'right_hand_rotation_20261026'/name;old=json.loads(path.read_bytes());old.update(state);path.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block=f'''## CURRENT 2026-10-27: 새 빈손 파지 경로 검증 완료, 실물 자세 확인 대기

- 새 경로 v3는 약지 PIP(ID15)를 먼저 80틱 펴고, 검지/중지 뿌리를 받침에서 분리한 뒤 약지를 열고 엄지/중지를 새 파지 자세로 옮긴다. 검지는 이 파지에 사용하지 않으므로 분리 이후 뿌리 ID8을 2048틱 이상으로 유지해 앞쪽 받침을 향해 뒤로 꺾이지 않게 했다. 큐브는 전 과정에서 제외하며 회전/뒤집기/SAC는 실행하지 않는다.
- 시작값 +/-4틱 및 받침 위치 3조건의 기하 검사 201개, 추종 반응 검사 9개, 5초씩 총 10초 영상 공백 고정 자세 검사 9개가 모두 통과했다. 초기 약지/손바닥의 기존 분리 범위는 그대로 유지하며 분리 뒤 새로운 손 자체/받침 겹침과 관절 범위 위반은 없었다. 이는 모델 검사 결과이며 실물 접촉이나 파지를 확정하지 않는다.
- 실행 파일은 약지가 실제 분리 위치에 도달했다는 건강한 관절 읽기 없이 다음 뿌리 목표를 보내지 않는다. 전체 목표는 {len(rows):,}개, 75ms 기준 {len(rows)*.075:.3f}초다. 450mA/PID, 전압/전류/추종 중단 기준은 유지한다. 실제 시작각이 달라지면 새 읽기로 전체 경로를 다시 검사하며 예전 자세로 수동 복원하도록 요구하지 않는다.
- 기본 실행이 카메라/모터/시뮬레이터를 열지 않는 것을 확인했다. 정확한 경로, 검토 자료, 원본, 코드 연결과 변경/불완전 자료 거부, 중복 재전송 거부, 실제 약지 분리 가드를 포함한 장치 없는 검사 15개가 통과했다. 실제 손의 목표/토크/설정 쓰기는 여전히 0회다.
- 다음 사용자 작업: 큐브를 손과 카메라 밖에 두고 웹캠을 손 전체에 맞춘 뒤 아래 명령을 실행한다. 터미널 SPACE 한 번으로 검사와 3초 카운트다운 후 약 2분간 빈손으로 이동하고 자동 OFF한다. 손가락을 다시 맞출 필요는 없다. 완료 후 '했어'만 보내면 저장 결과와 영상을 확인한다.

```powershell
{cmd}
```

- 검토: `{PROOF.relative_to(ROOT).as_posix()}`. 비교 그림: `{(out/'preview_v3/current_and_target.png').relative_to(ROOT).as_posix()}`. 저장 읽기 3회의 관절값 차이 0틱 및 5V 전원/큐브 제거 확인은 이미 완료됐다. 추가 측정이나 정지 사진은 요청하지 않는다.
- 실물 수직 뒤집기 성공 총 2회, 수평 90도 회전 미검증 상태를 유지한다. 이번 동작은 다음 회전에 사용할 새 파지 자세 확인이다. 전시 족자 파일과 원본 자세/관절 범위 파일의 SHA256은 변하지 않았다.

<!-- END FRESH EMPTY READY 20261027 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END FRESH EMPTY READY 20261027 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip()
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(block+old,encoding='utf-8')
    print(json.dumps(dict(status=state['status'],cases=state['empty_review_cases'],tests=15,next_user_command=cmd,assistant_motor_writes=0,protected_files_unchanged=True),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
