"""Record the unresolved moving-bus failure and the concrete baud-only trial."""
import datetime as dt,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261028/link_trial_after_040503';OUT.mkdir(exist_ok=True)
def read(path):return json.loads(path.read_bytes())
def save(path,v):path.write_bytes((json.dumps(v,ensure_ascii=False,indent=2)+'\n').encode('utf-8'))
source=ROOT/'horizontal_quarter_trial_20261028_040503_642362/result.json';r=read(source)
assert r['all_torque_off_verified'] and r['settings_restored'] and r['failed_phase']=='stage_0_cmc_return'
stress_path=ROOT/'sync_burst_readonly_20261028_040733_586087/result.json';stress=read(stress_path)
assert stress['status']=='readonly_burst_comparison_complete' and stress['motor_writes']==0
assert sum(b['complete'] for b in stress['blocks'])==1200 and not any(b['errors'] for b in stress['blocks'])
ready=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='awaiting_user_baud_only_1m_trial',
    failed_motion_source=source.relative_to(ROOT).as_posix(),failed_phase=r['failed_phase'],
    failure=r['error'],all_torque_off_verified=True,settings_restored=True,
    measured_min_voltage_v=min(v for s in r['samples'] for v in s['voltage_v']),
    measured_peak_current_ma=max(abs(v) for s in r['samples'] for v in s['current_ma']),
    measured_max_tracking_rad=max(v for s in r['samples'] for v in s['tracking_rad']),
    camera_usable_frames=r['camera_recording']['image_signal_usable_frames'],
    read_failures=r['read_failures'],recovery_confirmation_reached=False,
    readonly_comparison=stress_path.relative_to(ROOT).as_posix(),readonly_complete_snapshots=1200,
    readonly_errors=0,readonly_camera_frames=stress['camera_frames'],
    underlying_hardware_or_host_root_cause_proven=False,
    grouping_change_proven_beneficial=False,
    next_step_rationale='Moving-bus receive failures remain despite normal OFF reads. User previously authorized baud changes. Try supported1Mbps as an isolation experiment, not a proven fix.',
    official_control_table='https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#baud-rate8',
    tested_setup_program='hand_link_commission_1m_20261028.py',
    setup_program_sha256=hashlib.sha256((ROOT/'hand_link_commission_1m_20261028.py').read_bytes()).hexdigest(),
    setup_tests_passed=12,setup_default_mode_opens_no_devices=True,
    assistant_motor_or_setting_writes=0,baud_change_not_yet_executed=True,
    requires_user_started_setup=True,setup_changes_only_register8=True,
    requested_baud=1000000,baud_setting_persists_after_power_cycle=True,
    every_motor_must_already_be_torque_off=True,automatic_motion=False,
    rotation_program_must_be_matched_to_verified_baud_before_next_motion=True,
    next_user_command='leap_link_1m_setup.cmd',next_user_command_ready=True,pending_user_input=None,
    next_user_action='Keep cube OUT and hand unchanged, close Wizard, keep USB/5V connected. Run setup, SPACE once. No PLACE/movement. After1000 camera+READ checks send done; do not run rotation yet.',
    prior_first_observed_quarter='right_hand_rotation_20261028/completed_quarter_012305/physical_review.json',
    repeatability_verified=False)
save(OUT/'review.json',ready)
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
             'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name;value=read(path)
    value.update(status=ready['status'],current_status=ready['status'],timestamp=ready['timestamp'],
        current_hand_review=(OUT/'review.json').relative_to(ROOT).as_posix(),
        horizontal_loaded_execution_ready=False,next_user_command=ready['next_user_command'],
        next_user_command_ready=True,next_user_action=ready['next_user_action'],pending_user_input=None,
        repeatability_verified=False)
    save(path,value)
previous=ROOT/'right_hand_rotation_20261028/horizontal_quarter_trial_v10_recovery_episode/readiness.json'
value=read(previous)
if not (OUT/'previous_v10_readiness.json').exists():(OUT/'previous_v10_readiness.json').write_bytes(previous.read_bytes())
value.update(status='motion_retry_on_hold_pending_link_speed_trial',next_user_command_ready=False,
    latest_physical_stop=ready['failed_motion_source'],latest_link_review=(OUT/'review.json').relative_to(ROOT).as_posix(),
    v10_motion_started_and_stopped=True,v10_motion_completed=False,
    next_user_action=ready['next_user_action']);save(previous,value)
marker='<!-- END BAUD ONLY TRIAL PENDING 20261028 -->'
header='''2026-10-28 04:05 実행 중단: 엄지 CMC 복귀를 시작하고 약 2.2초, 28샘플 뒤 상태 패킷 수신이 연속 실패했다. 마지막 정상 전압은 5.0~5.1V, 전체 기록 최저 5.0V, 최대 전류 47mA, 추종 오차 최대 약 0.031rad. 토크 OFF와 설정 복구는 확인됐다. v10 복구 후 재확인 단계에 도달하기 전 읽기 자체가 실패했다. SDK -3002는 수신 손상 또는 일부 수신 후 타임아웃을 포함하므로 전기적 CRC 비트 오류라고 단정하지 않는다.

웹캠 동시 READ-only 비교는 16개씩, 8개씩, 4개씩 응답을 받는 조건을 앞뒤 순서로 비교해 총 1,200회 모두 정상, 카메라 369프레임이었다. 응답 묶음을 줄이는 방식의 개선 효과도 정지 상태에서는 입증하지 못했다. 구동 중 간헐적 통신 장애의 물리 원인은 미확정이다. 정지 기준을 더 늘리지 않고, 이전에 사용자가 허용한 bps 조정으로 공식 지원 1Mbps를 시험한다. 이는 해결 보장이 없는 통신 조건 비교다.

다음 사용자 명령은 `leap_link_1m_setup.cmd`다. 큐브를 밖에 두고 손가락은 그대로, 위자드를 닫고 USB와 5V 전원을 유지한 채 SPACE 한 번. 16개 모두 토크 OFF 확인 후 통신 속도 레지스터만 3Mbps에서 1Mbps로 변경하고, 원래 모드와 힘 및 PID 등의 보존을 확인한다. 이어 웹캠 동시 읽기 1,000회를 자동 검사한다. 설정은 전원을 꺼도 유지된다. 모터 구동과 목표 전송은 없다. 이 설정 프로그램은 장치 없는 검사 12개를 통과했고, 아직 실물에서는 실행하지 않았다. ‘했어’ 응답 후 새 속도를 확인하고 회전 프로그램의 호스트 설정과 현재 자세 근거를 맞춘 다음 실제 회전을 안내한다. 기존 회전 명령을 지금 바로 반복하도록 안내하지 않는다. 현재 근거 `right_hand_rotation_20261028/link_trial_after_040503/review.json`.

'''+marker+'\n\n'
# Keep the dated historical evidence, prepend the current handoff only.
header=header.replace('実행','실행')
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_bytes()
    if marker.encode('utf-8') not in old:path.write_bytes(header.encode('utf-8')+old)
print(json.dumps(dict(status=ready['status'],off=True,readonly_snapshots=1200,errors=0,
    setup_tests=12,motor_writes=0,next_user_command=ready['next_user_command'])))
