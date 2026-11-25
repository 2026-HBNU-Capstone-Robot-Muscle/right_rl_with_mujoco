"""Publish changed-pose diagnosis and matched v7 recovery evidence."""
import datetime,hashlib,json
from fresh_empty_grasp_plan_20261027 import ROOT,REQUEST,PROOF,FreshEmptyGraspPlan
from fresh_empty_grasp_gate_20261027 import validate_proof
def main():
    plan=FreshEmptyGraspPlan(REQUEST);nominal=validate_proof(plan,PROOF);base=ROOT/'right_hand_rotation_20261026/changed_off_pose_20261027';changed=FreshEmptyGraspPlan(base/'request_v7.json');current=validate_proof(changed,base/'review_v7.json');folders=['fresh_empty_grasp_20261027_030149_437983','fresh_empty_grasp_20261027_030441_709929','fresh_empty_grasp_20261027_030819_048059'];attempts=[]
    for folder in folders:
        path=ROOT/folder/'result.json';r=json.loads(path.read_bytes());assert not r.get('write_events') and r['all_torque_off_verified'];attempts.append(dict(source=path.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),error=r['error'],failed_phase=r['failed_phase'],motor_writes=0,all_torque_off_verified=True))
    before=json.loads((ROOT/'fresh_empty_grasp_20261027_024814_192218/result.json').read_bytes());previous=before['final_off_snapshots'][-1]['position'];delta=int(changed.source_position()[13])-previous[13];command='& "'+str(ROOT/'leap_fresh_empty_grasp.cmd')+'"';now=datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    protected={'sequence_flip_poses.json':'cc1e1841e5b441031fc61bbb4de2be7b424273863592385142ceb0d51bff9c6b','right_hand_rotation_20261023/native_motor_limits.json':'c2258197bcd016fcbfec3bac6c99d1f4da00d870b24bd2f337a988eec0e31716','exhibition_20261023/족자_붙여넣기.txt':'cba60e7523e729883d3e6859d4f9144a887c99883aeca6e838b292b071ab30f0'}
    for name,sha in protected.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==sha
    state=dict(status='changed_pose_v7_recovery_screened_operator_trial_pending',timestamp=now,updated_local=now,plan_version=plan.PLAN_VERSION,latest_user_trials=[r['source'] for r in attempts],last_three_stop_reason='Changed-pose model screen rejected before any motor writes; not undervoltage',latest_attempts=attempts,ring_rotation_id14_change_ticks=delta,ring_rotation_id14_change_degrees=delta*360/4096,touch_vs_gravity_cause_not_determined=True,latest_trials_settings_changed=False,latest_all_torque_off_verified=True,previous_voltage_trials_all_off_and_restored=True,model_support_initial_allowance_fix='For already-approved support pairs, apply max(initial envelope, existing0.15mm post-release allowance), rather than replacing0.15 with a near-zero envelope. Other pairs, geometry, forces and5mm initial bound unchanged.',early_ring_release_motor_ids=[13,15],early_ring_release_ticks=80,thumb_first_clearance_motor_id=4,thumb_first_clearance_ticks=16,thumb_first_clearance_degrees=16*360/4096,thumb_first_clearance_samples=8,physical_trial_of_v7_completed=False,physical_voltage_improvement_verified=False,physical_horizontal_90_verified=False,horizontal_quarter_turn_verified=False,ready_for_real_yaw=False,horizontal_loaded_execution_ready=False,empty_route_goals=len(plan.plan(plan.source_position())),empty_route_nominal_seconds=len(plan.plan(plan.source_position()))*.075,changed_pose_route_goals=len(changed.plan(changed.source_position())),changed_pose_route_nominal_seconds=len(changed.plan(changed.source_position()))*.075,empty_review=PROOF.relative_to(ROOT).as_posix(),empty_review_sha256=hashlib.sha256(PROOF.read_bytes()).hexdigest(),changed_pose_review=(base/'review_v7.json').relative_to(ROOT).as_posix(),changed_pose_review_sha256=hashlib.sha256((base/'review_v7.json').read_bytes()).hexdigest(),model_screened_starts=2,model_cases_total=438,no_device_regression_tests_passed=25,next_action='Operator runs the updated Revision7 EMPTY recovery once; assistant reviews saved results.',next_user_action='Leave fingers as they are. Cube stays OUT. Same command, one terminal SPACE. Check Revision7 header.',next_user_command=command,next_user_command_ready=True,next_motion_command_ready=True,operator_empty_execution_ready=True,ready_for_operator_empty_trial=True,pending_user_input=None,assistant_motor_commands_sent=0,motor_writes_sent_this_turn=0,camera_opened_this_turn=False,serial_ports_opened_this_turn=False,assistant_motor_execution_allowed=False,hardware_execution_allowed=False,full_sensitivity_complete=True,additional_marker_measurement_required=False,additional_capture_required=False,protected_hashes=protected)
    (base/'current_handoff_v7.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    for path in (REQUEST.parent/'progress_20261027.json',ROOT/'right_hand_rotation_20261026/horizontal_next_step_20261026.json',ROOT/'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
        old=json.loads(path.read_bytes());old.pop('all_three_torque_off_and_settings_restored',None);old.update(state);path.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    block=f'''## CURRENT 2026-10-27: 바뀐 시작 자세용 복귀 v7 검증 완료

- 최근 03:01:49, 03:04:41, 03:08:19는 앞선 저전압 3회와 다르다. 모두 새 현재각의 모델 검사에서 거부됐고 목표/토크/설정 쓰기는 0회다. 실제 모터 OFF는 읽기로 확인됐다. `settings_restored=false`는 설정을 바꾸지 않은 실행이므로 복원 실패를 뜻하지 않는다. 손상이나 고장이 확인된 기록은 없다.
- ID14 약지 옆방향이 직전 OFF 기록보다 {delta}틱, 약 {delta*360/4096:.2f}도 달라졌다. 손으로 만진 것과 토크 OFF 뒤 중력 이동을 로그만으로 구분하지 않는다. 엄지 뿌리 ID4도 모델 하한 가까운 1805틱이었다. 사용자가 손을 예전 자세로 되돌려야 한다고 요구하지 않는다.
- v5의 바뀐 자세 검사는 약지 PIP만 펴는 중 작은 재접촉, 약한 추종 가정의 엄지 하한 초과, 초기 받침 여유 계산 때문에 실패했다. v7은 약지 DIP(ID13)/PIP(ID15)를 각각 80틱 함께 펴고 엄지 CMC(ID4)를 첫 8개 목표 안에 16틱(약 1.4도) 안쪽으로 옮긴다. 실제 ID13/15가 모두 분리 위치를 통과했다는 새 건강 읽기가 없으면 뿌리 목표로 넘어가지 않는다. 이후 뿌리 속도 감소는 v5와 같다.
- 초기 허용 받침 쌍에 아주 작은 겹침 값(예: 0.011mm)이 있으면 이미 쓰던 분리 후 허용값 0.15mm보다 더 작게 덮어쓰던 계산을 바로잡았다. 승인된 쌍에서 `max(초기 관측 범위, 기존 0.15mm)`를 적용한다. 다른 쌍은 새로 허용하지 않았고 형상/힘/초기 5mm 상한/분리 후 0.15mm 상한은 유지했다. 전압/전류/추종 중단 기준, 실제 관절 범위 및 목표 2틱 상한은 바꾸지 않았다.
- v6의 현재각 검사는 정상 영상 조건은 통과했지만 초반 5초 영상 공백과 약한 추종에서 엄지가 아주 작게 하한을 넘었다. 기준을 넓히지 않고 엄지 분리를 앞당긴 v7을 채택했다. 이전 시작과 바뀐 현재 시작 각각 201개 기하, 9개 추종, 9개 공백 고정 검사, 합계 438개가 통과했다. 장치 없는 검사 25개와 기본 실행 무장치 확인도 통과했다. 물리적인 v7 실행과 전압 개선은 아직 미검증이다.
- 다음 사용자 작업: 손가락을 맞추지 않고 큐브를 밖에 둔 채 같은 명령을 1회 실행한다. 헤더 `Revision7`을 확인하고 터미널 SPACE 한 번을 누른다. 명목 동작 시간은 원래 시작 약 140초, 저장된 바뀐 시작 약 143초다. 자동 OFF 후 '했어'만 보내면 저장 자료를 읽는다. 이번은 빈손 복귀/새 자세 확인이며 큐브 회전은 실행하지 않는다.

```powershell
{command}
```

- 현재각 근거/검토는 `right_hand_rotation_20261026/changed_off_pose_20261027/`에 보존했다. 이전각 검토는 `{PROOF.relative_to(ROOT).as_posix()}`다. 이번 작업에서 실제 장치를 열거나 모터 쓰기를 하지 않았다. 실물 수직 뒤집기 성공 총 2회, 수평 90도 미검증 상태를 유지한다. 전시 족자 및 원본 자세/관절 범위 파일 SHA256은 변하지 않았다.

<!-- END CHANGED POSE RECOVERY V7 20261027 -->

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;old=path.read_text(encoding='utf-8-sig');marker='<!-- END CHANGED POSE RECOVERY V7 20261027 -->'
        if marker in old:old=old.split(marker,1)[1].lstrip()
        if old.startswith('## CURRENT '):old=old.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(block+old,encoding='utf-8')
    print(json.dumps(dict(status=state['status'],ring14_degrees=state['ring_rotation_id14_change_degrees'],model_cases=438,tests=25,motor_writes_this_turn=0,next_user_command=command),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
