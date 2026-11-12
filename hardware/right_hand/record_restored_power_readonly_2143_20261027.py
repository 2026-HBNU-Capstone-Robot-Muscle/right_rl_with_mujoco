"""Record restored electrical responses without approving unknown wire routing."""
import datetime as dt,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
ping_path=ROOT/'all_ping_readonly_20261027_214320_869426/result.json'
bench_path=ROOT/'serial_benchmark_readonly_20261027_214345_675068/result.json'
pose_path=ROOT/'repeat_off_readonly_20261027_214344_043175/result.json'
ping=json.loads(ping_path.read_text(encoding='utf-8'));bench=json.loads(bench_path.read_text(encoding='utf-8'))
pose=json.loads(pose_path.read_text(encoding='utf-8'))
assert ping['responding_ids']==list(range(1,17)) and bench['communication_failures']==0 and len(bench['frames'])==100
assert pose['status']=='off_snapshots_saved'
run=json.loads((ROOT/'horizontal_quarter_trial_20261027_211439_985362/result.json').read_text(encoding='utf-8'))
samples=run['samples']
report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='responses_restored_terminal_confirmation_and_model_pending',
    ping_source=str(ping_path.relative_to(ROOT)),benchmark_source=str(bench_path.relative_to(ROOT)),
    fresh_pose_source=str(pose_path.relative_to(ROOT)),responding_ids=ping['responding_ids'],
    current_voltage_range_v=[min(v['registers']['144']['value']/10 for v in ping['devices']),max(v['registers']['144']['value']/10 for v in ping['devices'])],
    all_torque_off=True,hardware_errors=0,watchdog_errors=0,motor_baud_bps=3000000,
    successful_full_reads=100,read_errors=0,median_read_ms=bench['median_read_ms'],maximum_read_ms=bench['maximum_read_ms'],
    prior_trial_observed_motion_seconds=samples[-1]['t'],prior_trial_cube_placed=False,
    prior_trial_minimum_measured_voltage_v=min(min(v['voltage_v']) for v in samples),
    prior_trial_peak_measured_current_ma=max(max(abs(c) for c in v['current_ma']) for v in samples),
    prior_trial_maximum_tracking_rad=max(max(v['tracking_rad']) for v in samples),
    cable_fault_causal_status='User-reported loose supply wire and restored responses support power-connection involvement; exact final terminal has not been reported.',
    terminal_photo_received=False,wire_terminal_confirmed=False,assistant_motor_writes=0,
    next_motion_command_ready=False,pending_user_input='Which marked terminal(V+ orV-) now holds the refastened black wire?',
    current_model_request='right_hand_rotation_20261027/power_repaired_214344/request.json')
folder=ROOT/'right_hand_rotation_20261027/vplus_disconnect_211439'
(folder/'restored_responses_2143.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
ready_path=ROOT/'right_hand_rotation_20261027/horizontal_quarter_trial_v6_current/readiness.json'
ready=json.loads(ready_path.read_text(encoding='utf-8'));ready.update(status=report['status'],
    latest_connection_review=str((folder/'restored_responses_2143.json').relative_to(ROOT)),
    next_user_command=None,next_user_command_ready=False,pending_user_input=report['pending_user_input'])
ready_path.write_text(json.dumps(ready,indent=2)+'\n',encoding='utf-8')
header='''전원 응답 복구 확인(2026-10-27 21:43):사용자가 전원을 다시 켰다고 알린 뒤16개 모두 PING응답. 모두토크OFF,오류/워치독0,5.0~5.1V,3Mbps. 전체상태100회연속읽기 오류0,약15ms. 검은선을 최종적으로V+와V-중어느단자에고정했는지는 아직 질문 응답 대기. 배선 사진은 미제공. 현재 자세는 새로 읽어모델검증중이며 구동재실행은 아직 안내하지않음. 근거 `right_hand_rotation_20261027/vplus_disconnect_211439/restored_responses_2143.json`.

<!-- END POWER RESPONSES RESTORED 2143 20261027 -->

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    p=ROOT/name;old=p.read_text(encoding='utf-8')
    if '<!-- END POWER RESPONSES RESTORED 2143 20261027 -->' not in old:p.write_text(header+old,encoding='utf-8')
print(json.dumps({k:report[k] for k in ('status','median_read_ms','prior_trial_minimum_measured_voltage_v','prior_trial_peak_measured_current_ma','prior_trial_maximum_tracking_rad')},ensure_ascii=False))
