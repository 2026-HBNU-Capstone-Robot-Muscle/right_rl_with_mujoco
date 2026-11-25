"""Publish completed code/model checks and the remaining USB prerequisite."""
import datetime as dt,hashlib,json
from pathlib import Path
import calibrated_quarter_plan_20261028 as p
from calibrated_quarter_gate_20261028 import validate_review,validate_loaded
from calibrated_precise_clock_20261028 import review as clock_review
proof=validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF);loaded,geometry=validate_loaded()
unit=json.loads((p.OUT/'transport_unit_checks.json').read_bytes());operator=json.loads((p.OUT/'calibration_tests.json').read_bytes())
assert unit['tests']==172 and unit['status']=='device_free_checks_passed' and operator['tests']==16 and operator['status']=='passed'
run=p.ROOT/'calibrated_quarter_trial_20261028_055850_634642';raw=json.loads((run/'result.json').read_bytes());timing=json.loads((run/'timing_review.json').read_bytes())
out=p.ROOT/'right_hand_rotation_20261028/empty_open_deadline_055850';out.mkdir(exist_ok=True)
physical=dict(status='empty_open_stopped_before_placement_after_recovered_read',source_run=run.relative_to(p.ROOT).as_posix(),
    source_result_sha256=hashlib.sha256((run/'result.json').read_bytes()).hexdigest(),
    cube_placement_reached=False,new_cube_rotation_observed=False,all_torque_off_verified=raw['all_torque_off_verified'],settings_restored=raw['settings_restored'],
    read_failure=raw['read_failures'][0],confirmation=raw['read_recovery_confirmation_events'][0],
    clock_info=timing['clock_info'],prior_near90_held_observations_preserved=2,
    controller_change='QPC host timing and two complete healthy snapshots after the last bad read; fresh recovered full snapshot may count as first',
    partial_snapshot_never_accepted=True,health_budget_ms=200,voltage_current_temperature_tracking_goal_checks_unchanged=True,
    new_variant_not_yet_physically_run=True,assistant_motor_writes=0)
(out/'review.json').write_bytes((json.dumps(physical,indent=2)+'\n').encode())
from horizontal_quarter_plan_20261027 import OUT as baseline_out
baseline=json.loads((baseline_out/'verification.json').read_bytes())
for name,wanted in baseline['protected_hashes'].items():
    path=Path(name) if Path(name).is_absolute() else p.ROOT/name
    assert hashlib.sha256(path.read_bytes()).hexdigest()==wanted,name
verification=dict(status='code_and_saved_start_model_checks_complete',transport_unit_tests=172,operator_tests=16,offline_launcher_tests=3,
    total_tests=191,current_start_model_cases=210,loaded_physics_cases=len(loaded['cases']),loaded_geometry_cases=len(geometry['cases']),
    total_model_cases=438,precise_readonly_evidence=clock_review(),code_hashes=p.hashes(),
    protected_assets_unchanged=True,loaded_goal_path_preserved=True,driver_clock_patch_is_instance_local=True,
    default_cli_opens_no_devices=True,new_variant_not_yet_physically_run=True,assistant_motor_writes=0)
(p.OUT/'verification.json').write_bytes((json.dumps(verification,indent=2)+'\n').encode())
ready=dict(status='code_and_model_review_complete_waiting_for_robot_connection',timestamp=dt.datetime.now().astimezone().isoformat(),
    plan_version=p.HorizontalQuarterPlan.PLAN_VERSION,operator_code_model_review_accepted=True,
    physical_connection_verified_now=False,current_off_pose_read_pending=True,
    connection_probe='prefix638_off_readonly_20261028_165414_124323/result.json',
    available_ports_at_last_check=['COM3 Bluetooth','COM4 Bluetooth'],
    user_next_action='Keep cube out and fingers/base unchanged. Connect existing robot USB and5V power. Do not start a motion program yet.',
    after_connection='Assistant reads motor states only; refresh the current-start model if the posture differs.',
    next_motion_command_after_connection_review='& "'+str(p.ROOT/'leap_calibrated_quarter_trial.cmd')+'"',
    motor_program_not_started_by_assistant=True,assistant_motor_writes=0,
    model_review_executed=True,model_review_passed=True,previous_automatic_review_usage_block_resolved_on_new_resume=True,
    simulation_only_local_command_no_longer_needed=True,total_tests=191,model_cases=438,
    proof=p.PROOF.relative_to(p.ROOT).as_posix(),last_physical_review=(out/'review.json').relative_to(p.ROOT).as_posix(),
    near90_observation_degrees=[88.3,89.9],completed_same_route_runs=1,new_variant_not_yet_physically_run=True)
(p.OUT/'readiness.json').write_bytes((json.dumps(ready,indent=2)+'\n').encode())
archive=p.OUT/'before_connection_wait';archive.mkdir(exist_ok=True)
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=p.ROOT/name;old=archive/path.name
    if path.exists() and not old.exists():old.write_bytes(path.read_bytes())
    path.write_bytes((json.dumps(ready,indent=2)+'\n').encode())
text='2026-10-28 재개 후 검증 완료: 앞서 사용량 한도로 미실행됐던 WSL 시뮬레이션을 같은 승인 검토 경로로 다시 요청했고 이번에는 실행됐다. 저장된 새 시작 자세의 정적201/동적9조건 모두 통과. 고정 회전 경로의 동적27/정적201조건을 포함해438조건을 확인했고, 제어 코드172개와 심사/기본실행16개 및 오프라인 실행기3개, 총191개 검증을 마쳤다. 이전 승인 검토 사용량 차단은 이번 재개에서 해소됐으며 사용자에게 시뮬레이션 파일 실행을 요청할 필요가 없다.\n\n보완판은 host/SDK/건강 예산에 일관된perf_counter를 쓰고, 마지막 오류 뒤 완전히 새로 읽은 정상 상태를 첫 번째로 세어 총두 번의 완전한 정상 상태를 확인한다. 부분 상태는 버리고, 다시 오류가 나면 새 완전 상태부터 재확인한다.200ms 및 모든 기존 전압/전류/온도/추종/목표 기준, 재시도 제한과 회전 목표/힘/PID는 유지한다. 실제 보완판 동작은 아직 실행하지 않았다.\n\n현재 연결 목록에는COM3/COM4 Bluetooth만 있고 로봇 USB 어댑터가 없다. 읽기 전용 점검도 장치를 찾지 못해 각도를 읽지 않았다. 다음 사용자 작업은 큐브를 손 밖에 둔 채 기존 로봇 USB와5V 전원을 연결/켜는 것이다. 손가락과 받침은 그대로 두고 프로그램을 실행하지 않는다. 연결 후 내가 상태/각도만 읽고, 자세가 달라졌으면 복귀 모델 근거를 갱신한다. 기존 약88/90도 두 회전 관찰과 전체 완주1회를 보존했다. 현재 근거 `right_hand_rotation_20261028/observed90_prefix638/operator_review_v3_precise_two_full/readiness.json`.\n\n<!-- END PRECISE VERIFIED WAITING CONNECTION 20261028 -->\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=p.ROOT/name;old=path.read_text(encoding='utf-8-sig')
    if '<!-- END PRECISE VERIFIED WAITING CONNECTION 20261028 -->' not in old:path.write_bytes((text+old).encode())
validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
print(json.dumps(dict(status=ready['status'],tests=191,model_cases=438,approval_usage_block_resolved=True,motor_writes=0)))
