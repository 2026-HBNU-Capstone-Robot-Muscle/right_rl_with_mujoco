"""Persist the completed EMPTY evidence and next operator-controlled trial."""
import json
import math
from pathlib import Path
from grasp_place_plan import GraspPlacePlan,SOURCE,SOURCE_SHA,completed_source,ROOT
from thumb_live_start_gate import validate_proof

out=ROOT/'prepared_cube_review_20261022';fixture=out/'grasp_trial_fixture'
plan=GraspPlacePlan(fixture/'request.json');proof=validate_proof(plan,fixture/'screen.json')
loaded=json.loads((out/'loaded_approach_sensitivity.json').read_bytes())
assert loaded['source_empty_trial_sha256']==SOURCE_SHA and loaded['clearance_tracking_screen_passed']
assert len(loaded['trials'])==21 and loaded['native_ranges_unchanged']
r=completed_source();samples=r['samples']
evidence=dict(timestamp=r['timestamp'],status=r['status'],source=str(SOURCE.relative_to(ROOT)),source_sha256=SOURCE_SHA,
    motion_samples=sum(s['phase']!='stage_2_fixed_hold' for s in samples),final_hold_samples=20,
    minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
    peak_current_ma=max(max(abs(v) for v in s['current_ma']) for s in samples),
    maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),
    final_on_position_ticks=r['final_on_state']['position'],final_on_goal_ticks=r['final_on_state']['goal'],
    final_off_position_ticks=r['final_off_snapshots'][-1]['position'],
    on_to_off_position_change_ticks=r['on_to_off_position_change_ticks'],
    final_off_reference_valid=r['final_off_reference_valid'],
    all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
    camera_frames=r['camera_recording']['frame_count'],camera_status=r['camera_recording']['status'],
    empty_grasp_approach_motor_motion_verified=True,physical_grasp_verified=False,
    physical_cube_rotation_verified=False,physical_no_contact_verified=False)
evidence['maximum_tracking_deg']=math.degrees(evidence['maximum_tracking_rad'])
(out/'completed_empty_approach_review.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')

command='& "'+str(ROOT/'leap_cube_grasp_trial.cmd')+'"'
user_task='큐브를 빼고 현재 손 모양 그대로 leap_cube_grasp_trial.cmd 실행. 터미널 첫 SPACE로 빈손 복귀. PLACE 후 초록 위/주황 카메라 쪽으로 큐브를 놓고 손을 뺀 뒤 두 번째 SPACE로 느린 접근. REMOVE 후 큐브를 빼고 세 번째 SPACE로 OFF. 끼워 넣어야 하면 Q. 결과 로그 전달.'
ready=dict(status='operator_cube_approach_observation_ready',launcher='leap_cube_grasp_trial.cmd',command=command,
    plan_version=plan.PLAN_VERSION,user_task=user_task,empty_trial=evidence,
    tests=dict(passed=87,duration_s=13.976,
        command='.runtime/python/python.exe -B -m unittest test_leap_thumb_prefix_empty test_empty_trial_camera test_thumb_continuation_empty test_thumb_current_start test_hand_preparation_timing test_prepared_placement_capture test_sequence_voltage test_grasp_approach_empty test_held_joint_guard test_grasp_place_trial'),
    actual_windows_wsl_subprocess_passed=True,static_start_cases=67,
    native_ranges_unchanged=True,
    initial_off_range_excess_rad=max(s['initial_native_range_excess_rad'] for s in proof['static_screens']),
    start_range_excess_is_only_monotonic_empty_reentry=True,
    max_unapproved_native_range_violation_rad=max(s['unapproved_native_range_violation_rad'] for s in proof['static_screens']),
    dynamics=proof['dynamic_sensitivity'],loaded_synthetic_cases=21,
    loaded_max_tracking_rad=loaded['max_tracking_rad'],loaded_max_self_overlap_mm=loaded['max_self_mm'],
    loaded_max_support_overlap_mm=loaded['max_support_mm'],loaded_max_cube_shift_mm=loaded['max_cube_shift_mm'],
    three_tip_grasp_not_established=True,physical_grasp_verified=False,physical_rotation_verified=False,
    live_sac=False,hardware_commands_sent_by_assistant=0,fresh_actual_start_screen_required_each_run=True,
    fixture_is_not_a_current_stable_pose=True,cube_placement_confirmation_required_before_loaded_goals=True,
    removal_confirmation_required_before_normal_off=True,
    current_pid_tracking_voltage_stops_unchanged=True,
    approach_seconds=16.1,fixture_empty_and_loaded_motion_seconds=len(plan.plan(plan.source_position()))*.05,
    placement_wait_limit_s=90,motion_time_limit_s=55,
    loaded_sensitivity='prepared_cube_review_20261022/loaded_approach_sensitivity.json',
    proof='prepared_cube_review_20261022/grasp_trial_fixture/windows_integration.json')
(out/'grasp_trial_ready.json').write_text(json.dumps(ready,ensure_ascii=False,indent=2),encoding='utf-8')

p=out/'current_status.json';state=json.loads(p.read_text(encoding='utf-8'))
state.update(status=ready['status'],next_user_task=user_task,next_motion_launcher=ready['launcher'],
    latest_completed_empty_trial=evidence,next_trial_evidence='prepared_cube_review_20261022/grasp_trial_ready.json',
    physical_cube_rotation_verified=False,live_sac=False)
p.write_text(json.dumps(state,ensure_ascii=False,indent=2),encoding='utf-8')
p=ROOT/'right_hand_start_20261019/objective_and_schedule.json';state=json.loads(p.read_text(encoding='utf-8'))
state.update(current_user_task=user_task,current_blocker='Empty preparation and new approach completed physically. Loaded cube retention/grasp and vertical/horizontal rotations remain unverified.',
    latest_user_hardware_trial=evidence,last_completed_user_hardware_trial=evidence,
    latest_offline_review=ready['proof'],next_trial_review='prepared_cube_review_20261022/grasp_trial_ready.json')
state['milestones'][0]['status']='empty_preparation_and_new_approach_passed_full_cube_trajectory_not_verified'
p.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

note='''## CURRENT 2026-10-22 02:07 빈손 접근 완료 / 큐브를 놓은 뒤 접근 시험 준비

- 사용자 실행 `grasp_approach_current_empty_20261022_020751_741219`: 616개 이동 목표와20개 최종 유지 읽기 완료. 최저4.3V, 최대91mA, 최대 추종0.07977rad(약4.57°). 토크 OFF/설정 복원/578프레임 카메라 저장 확인. 마지막 빈손 사진도 검토. 이는 빈손 관절 경로 완료이며 파지/뒤집기 성공이 아니다.
- OFF 뒤 엄지 ID4가2061→1802(-259틱, 약22.76°), 중지 뿌리 ID12가2584→2412(-172틱)로 내려감. 두 OFF 표본도 계속 변했으므로 마지막 ON 자세를 다음 시작값으로 취급하지 않는다. 출처 원본은 수정하지 않고 새 실행에서 안정 OFF 각도3개를 읽는다.
- 새 `leap_cube_grasp_trial.cmd`: 현재 빈손 OFF 자세 → 마지막 접근 자세까지 완만한 회복 → 방금 완료한 접근의 역방향으로 열기 → PLACE 대기 → 터미널 SPACE/새 배치 사진/3초 카운트다운 → 같은322개 목표 약16.1초 접근 → 유지/새 사진 → REMOVE/터미널 SPACE → OFF/복원. 카메라와 모터를 하나의 명령에서 기록. ID40/41 인식 gate 없음. 힘으로 끼워 넣거나 사람이 받쳐야 하는 배치는 Q로 중단.
- 새 복귀 경로의 첫 검사는 OFF 처짐이 원래 모델 범위를 약1.6° 벗어났음을 검출하여 실패했다. 범위 자체는 변경하지 않음. 새 계획만 첫 **빈손 회복** 단계에 한해 초기 초과0.06rad 이하, 매 관절 초과의 단조 감소, 큐브 배치 전 초과0을 요구한다. 이 조건의 위반/증가/미복귀/잘못된 단계/NaN은 proof 검증에서 거부. 67개 ±4틱 사례의 초기 최대 초과0.03450rad, 허용되지 않은 초과0, 자기/받침 겹침0. cube-loaded 목표에서는 이 예외를 사용하지 않음.
- 새 경로 및 기존4개 경로 Windows→WSL 실제 검증 통과. 새 경로 gain0.5/1/1.5 최대 추종0.05556/0.02793/0.02787rad, 자기/받침 겹침0. 55초 이동 제한은 유지하고 건강 상태를 계속 읽는 PLACE 대기만 별도90초 한도로 계산. 미확인 SPACE/Q/시간 초과에서는 첫 loaded 목표를 보내지 않으며, 쓰기 실패를 자동 재전송하지 않는다.
- 55mm/100g 큐브의 합성 배치7개×gain3개, 총21개 느린 접근 동역학 확인. 자기/받침 겹침0, 최대 추종0.05554rad. 일부 배치가 약15.90mm 밀리고 모두 세 손끝으로 잡히는 것은 아니므로 세 손끝 파지·회전 성공으로 부르지 않는다. 정확한 실물 큐브 좌표/마찰/파지 힘은 아직 검증되지 않았다. 다음 시험은 이 접근에서 실제 큐브가 손 안에 남는지 확인하는 관찰이다.
- 신규 PLACE/REMOVE 큐·시간 제한·Q·쓰기 실패·새 사진·정상 OFF 복원·native 복귀 proof 변조 차단 등을 포함 **87개 테스트 통과**(13.976초). CLI check-only는 장치/시뮬레이터를 열지 않음을 확인. 조수의 실제 카메라/모터 접근0, 회전/학습 정책 명령0. 기존450mA/PID/추종0.18rad/전압 보호 유지.
- 다음 사용자 작업: 큐브 없이 `leap_cube_grasp_trial.cmd` 실행 → 첫 터미널 SPACE. PLACE 후 초록 위/주황 카메라 쪽으로 자연스럽게 놓고 손을 빼서 두 번째 SPACE. 접근 종료/REMOVE 후 큐브를 빼고 세 번째 SPACE. 결과 로그를 전달. 회전은 아직 실행하지 않는다.
- 근거: `prepared_cube_review_20261022/completed_empty_approach_review.json`, `grasp_trial_ready.json`, `grasp_trial_fixture/windows_integration.json`, `loaded_approach_sensitivity.json`, `leap_cube_grasp_trial_README.md`.

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;content=p.read_text(encoding='utf-8')
    if note.splitlines()[0] not in content:p.write_text(note+content,encoding='utf-8')
print(json.dumps({k:ready[k] for k in ('status','tests','static_start_cases','loaded_synthetic_cases','approach_seconds')},ensure_ascii=False,indent=2))
