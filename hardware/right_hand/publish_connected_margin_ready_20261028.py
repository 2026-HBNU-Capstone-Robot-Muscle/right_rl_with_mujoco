"""Publish the connected, model-checked start and unchanged loaded trial."""
import datetime as dt,hashlib,json
from pathlib import Path
import calibrated_quarter_plan_20261028 as p
from calibrated_quarter_gate_20261028 import validate_review,validate_loaded
proof=validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF);loaded,static=validate_loaded()
units=json.loads((p.OUT/'transport_unit_checks.json').read_bytes());cal=json.loads((p.OUT/'calibration_tests.json').read_bytes())
assert units['status']=='device_free_checks_passed' and units['tests']==172 and cal['status']=='passed' and cal['tests']==16
connection=json.loads((p.ROOT/'prefix638_off_readonly_20261028_170143_255609/result.json').read_bytes())
camera=json.loads((p.ROOT/'reconnected_camera_20261028_170322_794705/result.json').read_bytes())
assert connection['status']=='off_snapshots_saved' and connection['maximum_drift_ticks']<=2 and camera['status']=='fresh_camera_frame_saved'
from horizontal_quarter_plan_20261027 import OUT as baseline_out,LOADED_PATH as original
baseline=json.loads((baseline_out/'verification.json').read_bytes())
for name,wanted in baseline['protected_hashes'].items():
    path=Path(name) if Path(name).is_absolute() else p.ROOT/name
    assert hashlib.sha256(path.read_bytes()).hexdigest()==wanted,name
assert hashlib.sha256(original.read_bytes()).hexdigest()=='631c4081f5bbf75490479f48a9a84e437c7f7280597cdc484722205b6a0b30df'
verification=dict(status='all_connected_start_checks_passed',tests=199,transport_tests=172,operator_tests=16,margin_tests=8,offline_launcher_tests=3,
    empty_model_cases=210,loaded_model_cases=len(loaded['cases']),loaded_geometry_cases=len(static['cases']),total_model_cases=438,
    empty_ring_margin=proof['empty_ring_pip_margin'],native_joint_limits_preserved=True,loaded_goal_path_preserved=True,
    initial_support_policy='Only existing initial pairs, no increase, cleared by roots; later threshold unchanged.',
    coarse_and_fine_rejected_range_diagnostics_preserved=True,protected_assets_unchanged=True,
    code_hashes=p.hashes(),assistant_motor_writes=0,new_variant_not_yet_physically_run=True)
(p.OUT/'verification.json').write_bytes((json.dumps(verification,indent=2)+'\n').encode())
ready=dict(status='connected_current_start_and_precise_transport_ready_for_one_trial',timestamp=dt.datetime.now().astimezone().isoformat(),
    operator_trial_ready=True,plan_version=p.HorizontalQuarterPlan.PLAN_VERSION,
    connection_source='prefix638_off_readonly_20261028_170143_255609/result.json',camera_source='reconnected_camera_20261028_170322_794705/result.json',
    all16_respond_off_error0_watchdog0=True,existing_baud=1000000,mode=5,voltage_range_v=[5.0,5.1],
    old_start_maximum_change_ticks=445,manual_pose_reset_required=False,
    current_start_review='right_hand_rotation_20261028/reconnected_170143_margin_v4_checked/review.json',
    empty_margin_motor15_ticks=[1724,1735],empty_margin_travel_degrees=11*360/4096,
    native_range_extension=False,loaded_yaw_goals=638,loaded_path_unchanged=True,
    timing_clock='perf_counter',whole_health_budget_ms=200,full_healthy_states_after_last_bad_read=2,
    motor_current_and_PID_unchanged=True,all_health_checks_preserved=True,
    command='& "'+str(p.ROOT/'leap_calibrated_quarter_trial.cmd')+'"',
    user_steps=['Leave fingers, base, camera and USB as they are. Cube OUT at FIRST SPACE.',
       'Only at PLACE: GREEN up, ORANGE toward WRIST/ID41. Hands clear, SECOND SPACE.',
       'After automatic OPEN remove cube before THIRD SPACE = OFF. Send done.'],
    tests=199,model_cases=438,assistant_motor_writes=0,previous_near90_held_observations=2,
    physical_completion_of_new_variant_unverified=True,proof=p.PROOF.relative_to(p.ROOT).as_posix())
(p.OUT/'readiness.json').write_bytes((json.dumps(ready,indent=2)+'\n').encode())
archive=p.OUT/'previous_pointers';archive.mkdir(exist_ok=True)
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json','right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=p.ROOT/name;old=archive/path.name
    if path.exists() and not old.exists():old.write_bytes(path.read_bytes())
    path.write_bytes((json.dumps(ready,indent=2)+'\n').encode())
text='2026-10-28 연결 후 준비 완료:사용자가USB/5V 전원을 연결했다고 답했고,17:01 읽기 전용 확인에서16개 모두응답/모드5/기존1Mbps/토크OFF/오류0/워치독0/5.0~5.1V, 세 번 읽기의최대변화1틱을 확인했다. 웹캠 새 사진에서도 빈손과 전체손을 확인했다. 저장된 이전자세와최대445틱 차이가 있어 새 자세로 복귀를 검토했다.\n\n기존 복귀는 약지15번의 하한근처에서 약한응답모델이0.071틱 벗어나3조건이실패했고, 시간간격을줄인모델에서도 같은결과였다. 관절범위를 넓히지 않고,뿌리를올리기전에15번을1724→1735틱(약0.97도)모으는빈손구간을추가했다. 이구간도이미있던받침접촉쌍이깊어지지않는기존초기접촉조건을적용하고,뿌리이후0.15mm조건을유지했다.새경로정적201/동적9모두통과.고정회전경로동적27/정적201을포함해438조건,제어검증199개통과.모든실패진단은보존했다.\n\n고해상도host/SDK시계와마지막오류뒤완전한정상상태두번확인을유지하며,회전638개목표와전류/PID/전압/추종등조건은유지했다.모터구동/설정쓰기는하지않았다.기존약88/90도두관찰은보존했고,보완판실물완주는아직확인전이다.다음사용자작업은같은 `leap_calibrated_quarter_trial.cmd`한번.손을직접맞추지않고빈손첫SPACE→PLACE초록위/주황손목ID41쪽배치와손제거후두번째SPACE→자동OPEN후큐브제거/세번째SPACE.완료후‘했어’만보내면저장기록을확인한다.현재근거 `right_hand_rotation_20261028/observed90_prefix638/operator_review_v4_empty_margin/readiness.json`.\n\n<!-- END CONNECTED MARGIN READY 20261028 -->\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=p.ROOT/name;old=path.read_text(encoding='utf-8-sig')
    if '<!-- END CONNECTED MARGIN READY 20261028 -->' not in old:path.write_bytes((text+old).encode())
validate_review(p.HorizontalQuarterPlan(p.REQUEST),p.PROOF)
print(json.dumps(dict(status=ready['status'],tests=199,model_cases=438,manual_pose_reset_required=False,new_variant_not_yet_physically_run=True,motor_writes=0)))
