"""Record completed work and the exact remaining review, with no readiness claim."""
import datetime as dt,hashlib,json,subprocess,sys
from pathlib import Path
import calibrated_quarter_plan_20261028 as p
from calibrated_quarter_gate_20261028 import validate_loaded
from calibrated_precise_clock_20261028 import review as precise_review
unit=json.loads((p.OUT/'transport_unit_checks.json').read_bytes());assert unit['status']=='device_free_checks_passed' and unit['tests']==172
loaded,static=validate_loaded();evidence=precise_review()
run=p.ROOT/'calibrated_quarter_trial_20261028_055850_634642';raw=json.loads((run/'result.json').read_bytes())
assert raw['all_torque_off_verified'] and raw['settings_restored']
check=subprocess.run([sys.executable,'-X','utf8','-B',str(p.ROOT/'leap_calibrated_quarter_20261028.py')],cwd=p.ROOT,capture_output=True,text=True,encoding='utf-8',timeout=30)
assert check.returncode==2 and 'NOT READY:' in check.stdout and 'No devices opened.' in check.stdout
assert not p.PROOF.exists()
ready=dict(status='code_checks_passed_current_start_simulation_pending',timestamp=dt.datetime.now().astimezone().isoformat(),
    operator_trial_ready=False,plan_version=p.HorizontalQuarterPlan.PLAN_VERSION,
    last_run=run.relative_to(p.ROOT).as_posix(),last_failed_phase=raw['primary_failed_phase'],
    last_reason='recovered complete state plus first followup consumed the200ms budget before a third complete read',
    all_torque_off_verified=True,settings_restored=True,cube_placement_phase_not_reached=True,
    changes=['Use one high-resolution perf_counter clock for host timeouts, quiet wait and health budgets.',
      'After the last RX failure validate two full healthy snapshots; the wholly new recovered snapshot may be first.',
      'Keep all voltage/current/temperature/tracking/goal checks,200ms budget, retry limits and all motion goals.'],
    unit_tests_passed=172,offline_launcher_tests_passed=3,loaded_physics_cases_reused=len(loaded['cases']),
    loaded_geometry_cases_reused=len(static['cases']),precise_readonly_evidence=evidence,
    current_start_model_cases_pending=210,current_start_model_executed=False,
    model_execution_blocker=dict(source='automatic_approval_review',reason='usage limit prevented approval review; requested WSL action was not executed',unsafe_action_determination=False),
    default_motion_cli_denies_unreviewed_start=True,assistant_motor_writes=0,
    user_next_action='Run simulation-only check once; no robot power is needed.',
    simulation_only_command='& "'+str(p.ROOT/'leap_precise_simulation_check.cmd')+'"',
    no_device_access_in_simulation_launcher=True,proof=p.PROOF.relative_to(p.ROOT).as_posix(),
    near90_held_observations_preserved=2,full_project_or_other_axes_verified=False)
(p.OUT/'readiness.json').write_bytes((json.dumps(ready,indent=2)+'\n').encode())
archive=p.OUT/'prior_pointers';archive.mkdir(exist_ok=True)
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=p.ROOT/name;saved=archive/path.name
    if path.exists() and not saved.exists():saved.write_bytes(path.read_bytes())
    path.write_bytes((json.dumps(ready,indent=2)+'\n').encode())
text='2026-10-28 최신 상태:05:58 실행은 큐브 배치 전 빈손OPEN 중 통신 오류에서 복구했으나, 복구한 완전한 상태 뒤에 추가 정상 상태 두 번을 요구하는 과정이200ms를 넘어 중단됐다. 토크OFF와 설정복구는 정상이며 배치 단계에 도달하지 않았다. 이 PC의 GetTickCount64/GetSystemTimeAsFileTime 시간 해상도가15.625ms라3ms 무신호 대기가16ms로 늘어난 기록도 확인했다.\n\n보완판은 host/SDK timeout과 전체 읽기 제한에 일관된perf_counter를 쓰고, 마지막 오류 뒤 완전히 새로 읽은 정상 상태를 첫 번째로 세어 총두 번의 완전한 정상 상태를 검증한다. 불완전 상태는 버리고, 중간에 다시 오류가 나면 새 완전 상태부터 다시 확인한다. 모든 건강 기준과200ms, 재시도 제한 및 동작 목표는 유지한다. 회전은 기존 약88도와90도 두 관찰을 보존하며 이번 실행을 새 회전 성공으로 세지 않는다.\n\n독립 코드 검사172개와 로컬검사 실행기3개 통과, 실제 읽기 전용200회 오류0. 기존 고정 경로의 동적27/정적201조건은 보존했다. 현재 새 시작 자세210조건은 아직 실행하지 못했다. WSL 호출은 사용량 한도로 자동 승인 검토를 완료하지 못해 미실행이며 우회하지 않았다. 현재 동작 프로그램은 심사 파일이 없어 장치를 열기 전에NOT READY로 거부함을 확인했다. 다음 사용자 작업은 모터 프로그램 대신 `leap_precise_simulation_check.cmd`를 한 번 실행하는 것이다. 이 파일은 시뮬레이션과 검사만 수행하고 모터/카메라를 열지 않는다. 완료 후 ‘했어’만 보내면 결과를 확인한다. 현재 근거 `right_hand_rotation_20261028/observed90_prefix638/operator_review_v3_precise_two_full/readiness.json`.\n\n<!-- END PRECISE SIMULATION PENDING 20261028 -->\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=p.ROOT/name;old=path.read_text(encoding='utf-8-sig')
    if '<!-- END PRECISE SIMULATION PENDING 20261028 -->' not in old:path.write_bytes((text+old).encode())
print(json.dumps(dict(status=ready['status'],operator_trial_ready=False,unit_tests=172,launcher_tests=3,simulation_only_command=ready['simulation_only_command'],motor_writes=0)))
