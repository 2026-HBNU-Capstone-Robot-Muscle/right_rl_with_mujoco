"""Publish the stopped-repeat outcome and a fully reviewed transport update."""
import datetime as dt,hashlib,json,unittest
from pathlib import Path
import numpy as np
import calibrated_quarter_plan_20261028 as p
from calibrated_quarter_gate_20261028 import validate_review
from single_reply40_evidence_20261028 import review as timing_evidence
from test_single_reply40_20261028 import Fake,Reply40Tests
ROOT=p.ROOT;run=ROOT/'calibrated_quarter_trial_20261028_053806_737439';out=ROOT/'right_hand_rotation_20261028/stopped_calibrated_053806'
r=json.loads((run/'result.json').read_bytes());measured=json.loads((out/'same_marker_rotation_review.json').read_bytes());trace=json.loads((out/'rotation_trace.json').read_bytes())
angle=measured['comparisons']['placement_to_turn']['twist_total_tilt_p05_median_p95'][1][0]
physical=dict(status='rotation_completed_near90_but_hold_interrupted_by_read_failure',source_run=run.relative_to(ROOT).as_posix(),
    source_result_sha256=hashlib.sha256((run/'result.json').read_bytes()).hexdigest(),command_yaw_goals_completed=638,
    fixed_hold_samples_completed=30,automatic_open_started=False,whole_sequence_complete=False,
    held_camera_relative_yaw_estimate_deg=angle,angle_reference='placed cube before loaded grasp',exact_robot_frame90_certified=False,
    read_failure=r['read_failures'][0],all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
    cube_removed_after_stop_confirmed_by_user=True,near90_held_observations_same_route=2,completed_same_route_runs=1,
    robust_motion_link_reliability_verified=False,full_cube_solving_or_other_axes_verified=False,
    static_palm_feature_check=measured['static_palm_feature_check'],
    min_voltage_v=min(min(s['voltage_v']) for s in r['samples']),peak_abs_current_ma=max(max(abs(v) for v in s['current_ma']) for s in r['samples']),
    max_tracking_rad=max(max(s['tracking_rad']) for s in r['samples']),
    assistant_motor_writes=0,video_preview='right_hand_rotation_20261028/stopped_calibrated_053806/cube_rotation_preview_4x.mp4')
(out/'physical_review.json').write_bytes((json.dumps(physical,indent=2)+'\n').encode())
proof=validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
tests=unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromTestCase(Reply40Tests))
assert tests.wasSuccessful() and not tests.skipped
cal=json.loads((p.OUT/'calibration_tests.json').read_bytes());assert cal['status']=='passed' and cal['tests']==14
from horizontal_quarter_plan_20261027 import OUT as baseline_out,LOADED_PATH as original_path
baseline=json.loads((baseline_out/'verification.json').read_bytes());assert baseline['tests_passed']==150
for name,wanted in baseline['protected_hashes'].items():
    path=Path(name) if Path(name).is_absolute() else ROOT/name
    assert hashlib.sha256(path.read_bytes()).hexdigest()==wanted,name
assert hashlib.sha256(original_path.read_bytes()).hexdigest()=='631c4081f5bbf75490479f48a9a84e437c7f7280597cdc484722205b6a0b30df'
fake=Fake();harness=Reply40Tests();state,_=harness.run_fake(fake,fake.confirmed)
production_path=ROOT/'reply40_production_readonly_20261028_055358_224806/result.json';production=json.loads(production_path.read_bytes())
assert production['status']=='reply40_production_readonly_complete' and production['reads']==200 and production.get('read_failures',[])==[]
assert production['motor_writes']==production['torque_changes']==0 and production['bad_frames']==0
verification=dict(status='all_current_checks_passed',tests=174,core_tests=150,calibration_tests=14,new_window_fault_tests=10,
    current_empty_model_cases=210,reused_loaded_physics_cases=27,reused_loaded_geometry_cases=201,
    timed_logged_ID5_fault_recovery_ms=fake.t*1000,timed_fault_clean_confirmations=fake.bus.report['read_recovery_confirmation_events'][-1]['clean_reads'],
    production_readonly_source=production_path.relative_to(ROOT).as_posix(),production_readonly_reads=200,production_readonly_failures=0,
    production_max_sample_ms=max(production['latencies_ms']),production_OFF_goal_register_changes=len(production.get('off_goal_changes',[])),
    first_OFF_only_probe_stopped_on_uncommanded_goal_comparison=True,
    active_controller_exact_goal_validation_unchanged=True,timing_evidence=timing_evidence(),
    health_thresholds_unchanged=True,loaded_goals_unchanged=True,protected_assets_unchanged=True,
    code_hashes=p.hashes(),assistant_motor_writes=0,physical40ms_motion_recovery_not_yet_verified=True)
(p.OUT/'verification.json').write_bytes((json.dumps(verification,indent=2)+'\n').encode())
ready=dict(status='same_rotation_with_reply40_ready_for_one_operator_trial',timestamp=dt.datetime.now().astimezone().isoformat(),
    source_run=physical['source_run'],physical_review=(out/'physical_review.json').relative_to(ROOT).as_posix(),
    last_trial_stopped_after_all_yaw_goals=True,near90_held_observations_same_route=2,
    commanded_sequence_completion_runs=1,held_observation_degrees=[88.3,round(angle,1)],
    command='& "'+str(ROOT/'leap_calibrated_quarter_trial.cmd')+'"',proof=p.PROOF.relative_to(ROOT).as_posix(),
    plan_version=proof['plan_version'],yaw_goals=638,loaded_trajectory_unchanged=True,
    health_reply_timeout_ms=40,torque_off_readback_timeout_ms=80,whole_health_budget_ms=200,
    clean_snapshots_after_read_recovery=2,host_baud=1000000,hardware_health_thresholds_unchanged=True,
    tests=174,model_cases=438,assistant_motor_writes=0,
    physical40ms_motion_recovery_not_yet_verified=True,user_cube_removal_confirmed=True,
    user_steps=['Leave fingers, base, camera and USB as they are. Cube OUT at FIRST SPACE.',
        'At PLACE: GREEN up, ORANGE toward WRIST/ID41. Hands clear, SECOND SPACE.',
        'After automatic OPEN remove cube before THIRD SPACE = OFF; send done.'],
    video_preview=physical['video_preview'])
(p.OUT/'readiness.json').write_bytes((json.dumps(ready,indent=2)+'\n').encode())
archive=p.OUT/'prior_pointers';archive.mkdir(exist_ok=True)
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name;saved=archive/path.name
    if path.exists() and not saved.exists():saved.write_bytes(path.read_bytes())
    path.write_bytes((json.dumps(ready,indent=2)+'\n').encode())
text='2026-10-28 05:38 재현 실행:638개 회전 목표를 모두 마쳤고 파지 중 카메라 기준 약90도를 관찰했다. 앞선05:24의 약88도와 함께 같은 경로의90도 근처 회전이 두 번 관찰됐지만, 이번은 고정30/40샘플에서ID5 상태 패킷 오류로 멈춰 자동OPEN과 전체 완주를 확인하지 못했다. 토크OFF와 설정복구는 확인됐고, 사용자가 이후 큐브를 뺐다고 답했다.\n\n중단은 한 ID의 응답80ms를 기다리며 전체 읽기에110ms가 지나 기존 복구 예약120ms를 확보하지 못한 경로다.25ms 읽기 시험은200회 중5회 오류로 채택하지 않았고,40ms 단일 응답 시험은200회 전체 상태/3200개 응답에서 오류0이었다. 보완판은 건강 응답40ms, OFF 확인80ms를 사용하고 전체200ms 및 복구 후 두 번 새 정상 상태, 모든 기존 건강 기준을 유지한다. 기록과 같은 시간 형태의 ID5 오류 주입에서는 약170ms 안에 복구 및 정상 두 번 확인을 마쳤다. 별도 실제 장치 읽기 전용 검사200회도 오류0이었다. 첫 OFF 검사에서 목표 레지스터 값 변화로 종료된 기록을 보존했고, 읽기 전용 관찰의 OFF 목표와 실제 ON 제어 목표를 구분했다. ON 제어의 정확한 목표 확인 기준은 바꾸지 않았다.\n\n회전 목표/힘/PID/배선/1Mbps는 유지했다. 현재 복귀210조건, 기존 고정 경로의 동적27/정적201조건 및 제어174개 검증을 통과했다. 보완한 수신 대기를 실제 구동 중 시험한 것은 아직 아니므로, 다음 사용자 작업은 같은 `leap_calibrated_quarter_trial.cmd` 한 번이다. 빈손 첫SPACE, PLACE에서 초록위/주황손목ID41쪽 배치 후 손을 빼고 두 번째SPACE, 자동OPEN 후 큐브제거/세 번째SPACE. 현재 근거 `right_hand_rotation_20261028/observed90_prefix638/operator_review_v2_reply40/readiness.json`.\n\n<!-- END REPLY40 AFTER053806 READY 20261028 -->\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_text(encoding='utf-8-sig')
    if '<!-- END REPLY40 AFTER053806 READY 20261028 -->' not in old:path.write_bytes((text+old).encode())
validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
print(json.dumps(dict(status=ready['status'],tests=174,model_cases=438,timed_fault_recovery_ms=fake.t*1000,production_readonly_reads=200,
    production_OFF_goal_changes=len(production.get('off_goal_changes',[])),held_yaw_estimate_deg=angle,motor_writes=0)))
