"""Record the actual stop, tested change and operator handoff."""
import datetime as dt,json,hashlib
from horizontal_quarter_plan_20261027 import ROOT,OUT,REQUEST,PROOF,HorizontalQuarterPlan,hashes,LOADED_PATH
from horizontal_quarter_gate_20261027 import validate_review
from horizontal_read_recovery_20261028 import POLICY
def read(p):return json.loads(p.read_bytes())
def save(p,v):p.write_bytes((json.dumps(v,ensure_ascii=False,indent=2)+'\n').encode('utf-8'))
def rel(p):return p.relative_to(ROOT).as_posix()
proof=validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
verification=read(OUT/'verification.json')
assert verification['tests_passed']==137 and verification['code_hashes']==hashes()
source=ROOT/'horizontal_quarter_trial_20261028_020027_500658/result.json';raw=source.read_bytes();r=json.loads(raw)
assert r['status']=='stopped' and r['failed_phase']=='stage_3_empty_open'
assert r['all_torque_off_verified'] and r['settings_restored']
assert not r.get('cube_placement_confirmed',False)
diagnosis=dict(timestamp=dt.datetime.now().astimezone().isoformat(),source=rel(source),
    source_sha256=hashlib.sha256(raw).hexdigest(),status='stopped_during_empty_open_before_cube',
    direct_stop_reason=r['error'],physical_rotation_stage_reached=False,
    read_failure_count=len(r['read_failures']),first_incomplete_ids=[f['first_incomplete_id'] for f in r['read_failures']],
    all_reported_read_failures_recovered=all(f.get('recovered') is True for f in r['read_failures']),
    confirmation_events=r.get('read_recovery_confirmation_events',[]),
    measured_min_voltage_v=min(v for s in r['samples'] for v in s['voltage_v']),
    measured_peak_current_ma=max(abs(v) for s in r['samples'] for v in s['current_ma']),
    measured_max_tracking_rad=max(v for s in r['samples'] for v in s['tracking_rad']),
    camera_frames=r['camera_recording']['frame_count'],camera_blank_frames=r['camera_recording']['image_signal_blank_frames'],
    all_torque_off_verified=True,settings_restored=True,
    corruption_hardware_root_cause_proven=False,
    software_findings='Legacy confirmation immediately stopped on any new receive failure even when the transport recovered it. The last two recovered reads took31ms and47ms; the stop occurred before two new clean confirmations could be tried.',
    correction=dict(POLICY),
    timing_replay_scope='Recorded31ms/47ms recovered READ durations followed by two injected16ms clean snapshots; simulated110ms total, not an actual successful v10 hardware run.',
    hardware_execution_after_change_verified=False,assistant_motor_writes=0)
save(OUT/'stop_020027_review.json',diagnosis)
current=read(ROOT/'repeat_off_readonly_20261028_020314_646667/result.json')
assert current['status']=='off_snapshots_saved' and current['maximum_drift_ticks']==0 and current['servo_writes']==0
ready=dict(timestamp=diagnosis['timestamp'],status='v10_bounded_confirmation_episode_ready',
    latest_physical_stop=rel(OUT/'stop_020027_review.json'),
    current_readonly='repeat_off_readonly_20261028_020314_646667/result.json',
    current_torque_off_verified=True,current_error_and_watchdog_zero=True,
    source_request=rel(REQUEST),operator_proof=rel(PROOF),model_conditions=210,tests_passed=137,
    read_recovery_episode_policy=dict(POLICY),transport_read_budget_ms=200,packet_window_maximum_ms=80,
    retries_per_ten_seconds_maximum=3,all_motor_health_thresholds_unchanged=True,
    loaded_goals_sha256=hashlib.sha256(LOADED_PATH.read_bytes()).hexdigest(),loaded_goals_unchanged=True,
    gains_current_limits_baud_eeprom_unchanged=True,
    first_observed_quarter_review='right_hand_rotation_20261028/completed_quarter_012305/physical_review.json',
    prior_v9_single_trial_horizontal_quarter_observed=True,
    v10_physical_motion_verified=False,repeatability_verified=False,
    corruption_hardware_root_cause_proven=False,assistant_motor_writes=0,
    next_user_command='leap_horizontal_quarter_trial.cmd',next_user_command_ready=True,pending_user_input=None,
    next_user_action='Leave fingers as now, close Wizard, no cube at first SPACE. PLACE green up/orange toward wrist ID41, hands clear then second SPACE. Remove cube after automatic OPEN, third SPACE for OFF; send done only.')
save(OUT/'readiness.json',ready)
guide='''# 통신 복구 확인 수정 후 같은 동작 재현

02:00 실행은 큐브를 놓기 전 빈손 OPEN 중 멈췄다. 상태 패킷 오류 세 번은 각각 재읽기에 성공했다. 마지막 복구 확인 중 다시 오류가 생기자, 기존 코드가 곧바로 종료했다. 토크 OFF와 원래 설정 복구는 확인됐다.

v10은 최초 읽기부터 복구 확인까지 하나의 200ms 제한을 적용한다. 재오류가 복구되면 정상 확인 횟수를 0으로 돌리고, 추가 오류 없이 정상 상태를 연속 두 번 읽어야 다음 목표를 보낸다. 손상된 데이터나 이전 상태는 승인하지 않는다. 정상 확인 최대 다섯 번, 통신 재시도 최대 3회/10초, 패킷 수신 창 최대 80ms를 유지한다. 목표 재전송이나 다음 목표 미리 보내기는 없다. 시간이 부족하거나 전압과 전류 및 추종오차 등이 범위를 벗어나면 종료한다.

오류 재현 및 경계 조건 테스트 137개와 현재 자세의 복귀 모델 210조건을 통과했다. 원래 성공한 실제 회전 목표는 유지했다. 수정본의 실물 동작과 반복 성공은 아직 다음 실행에서 확인해야 한다.

1. 손가락은 그대로 두고 위자드를 닫는다. 큐브는 손과 카메라 밖에 둔 뒤 `leap_horizontal_quarter_trial.cmd`를 실행하고 첫 SPACE를 누른다.
2. PLACE 때 엄지와 중지 사이 뿌리 쪽에 초록 위, 주황 손목 ID41 방향으로 놓는다. 사람 손을 뺀 뒤 두 번째 SPACE를 누른다.
3. 회전과 자동 OPEN 후 큐브를 빼고 세 번째 SPACE를 누른다. ‘했어’만 보내면 된다.
'''
(OUT/'operator_guide.md').write_bytes(guide.encode('utf-8'))
marker='<!-- END BOUNDED CONFIRMATION V10 20261028 -->'
header='''2026-10-28 02:00 재현 시험 중단과 수정: 큐브 배치 전 빈손 OPEN에서 통신 패킷 오류가 났다. 세 오류는 각각 재읽기에 성공했으나, 마지막 복구 확인 중 재오류가 생기면 즉시 종료하는 기존 코드가 실행됐다. OFF와 설정 복구는 확인됐다. v10은 최초 읽기부터 전체 확인까지 200ms를 공유하고, 재오류가 복구되면 확인 횟수를 0으로 되돌린 뒤 추가 오류 없이 정상 상태를 연속 두 번 확인해야 이어간다. 같은 목표를 고정하며 목표 재전송은 없다. 3회/10초 재시도 제한과 전압 및 전류 및 추종오차 기준은 유지한다.

현재 빈손 OFF 관절값 세 번의 차이는 0틱. 이 자세의 복귀 모델 210조건과 오류 재현 포함 테스트 137개를 통과했다. 첫 실물 약 89도 관측은 보존하고, 성공했던 회전 목표는 그대로 둔다. v10 실물과 반복성은 다음 실행에서 확인한다. 다음 명령은 같은 `leap_horizontal_quarter_trial.cmd`: 손가락 그대로, 빈손 첫 SPACE, PLACE 초록 위/주황 손목 ID41 쪽 후 손을 빼고 두 번째 SPACE, 자동 OPEN 후 큐브 제거/세 번째 SPACE. 결과는 ‘했어’만 보내면 된다. 현재 근거 `right_hand_rotation_20261028/horizontal_quarter_trial_v10_recovery_episode/readiness.json`.

'''+marker+'\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_bytes()
    if marker.encode('utf-8') not in old:path.write_bytes(header.encode('utf-8')+old)
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
             'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name;value=read(path)
    value.update(status=ready['status'],current_status=ready['status'],timestamp=ready['timestamp'],
        latest_physical_stop=rel(OUT/'stop_020027_review.json'),current_hand_review=rel(OUT/'readiness.json'),
        latest_readonly=ready['current_readonly'],next_user_command=ready['next_user_command'],
        next_user_command_ready=True,next_user_action=ready['next_user_action'],pending_user_input=None,
        v10_physical_motion_verified=False,repeatability_verified=False)
    save(path,value)
old_path=ROOT/'right_hand_rotation_20261028/horizontal_quarter_trial_v9_receive_window/readiness.json'
old=read(old_path);old.update(status='superseded_by_v10_bounded_confirmation_episode',next_user_command_ready=False,
    superseded_by=rel(OUT/'readiness.json'));save(old_path,old)
assert source.read_bytes()==raw and verification['code_hashes']==hashes()
validate_review(HorizontalQuarterPlan(REQUEST),PROOF)
print(json.dumps(dict(status=ready['status'],actual_stop_before_cube=True,all_torque_off_verified=True,
    tests=137,model_conditions=210,assistant_motor_writes=0,physical_v10_not_yet_run=True)))
