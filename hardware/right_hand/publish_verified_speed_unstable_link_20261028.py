"""Keep motion on hold after positive software tests but persistent byte loss."""
import datetime as dt,hashlib,json
from horizontal_quarter_plan_20261027 import ROOT,OUT,PROOF,hashes
def read(path):return json.loads(path.read_bytes())
def save(path,value):path.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode('utf-8'))
verification=read(OUT/'verification.json');assert verification['tests_passed']==150 and verification['code_hashes']==hashes()
latest='production_link_readonly_20261028_044024_000613/result.json';r=read(ROOT/latest)
assert r['status']=='production_decoder_readonly_complete' and r['reads']==999 and len(r['errors'])==1
assert all(not any(s[k]) for s in r['last_snapshots'] for k in ('torque','error','watchdog'))
dest=ROOT/'right_hand_rotation_20261028/verified_1m_usb_isolation';dest.mkdir(exist_ok=True)
report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='awaiting_usb_port_swap_readonly_recheck',
    user_added_motion_trials=[
        dict(source='horizontal_quarter_trial_20261028_041150_514710/result.json',failed_phase='place_cube',torque_off=True,settings_restored=True),
        dict(source='horizontal_quarter_trial_20261028_041403_825440/result.json',failed_phase='open_before_cube',torque_off=True,settings_restored=True)],
    user_baud_setup='link_commission_1m_20261028_041604_405637/result.json',
    user_baud_writes=16,user_baud_setup_status='stopped_at_final_ID13_preserved_register_comparison',
    exact_original_mismatched_register_not_recorded=True,
    independent_baud_confirmation='post_commission_readonly_20261028_041740_872756/result.json',
    all16_at_1Mbps=True,independent_two_reads_per_motor_all_preserved_values_match=True,
    detected_serial_device=dict(device='COM7',vid=1027,pid=24596,serial_number='FTBENPSAA'),
    wire_loss_capture='sync_burst_readonly_20261028_042302_708111/result.json',
    captured_missing_bytes=[135,45],capture_camera_on_conditions=[False,True],
    hardware_or_host_specific_root_cause_proven=False,
    grouped8_probe='sync_burst_readonly_20261028_042609_020976/result.json',grouped8_probe_errors=0,
    grouped8_production_with_simulation='production_link_readonly_20261028_042900_797572/result.json',grouped8_production_errors=8,
    single_reply_probe='sync_burst_readonly_20261028_043357_661811/result.json',single_reply_probe_errors=0,
    single_reply_production_with_simulation='production_link_readonly_20261028_043647_564583/result.json',single_reply_production_with_simulation_errors=6,
    latest_production_without_simulation=latest,latest_complete_snapshots=999,latest_receive_errors=1,
    latest_read_ms_median=r['median_ms'],latest_read_ms_maximum=r['maximum_ms'],
    concurrent_simulation_observed_error_association_is_not_proof_of_causation=True,
    last_observed_torque_off=True,last_observed_error_watchdog_zero=True,
    baud_or_grouping_fix_proven=False,hardware_readiness=False,
    latest_position_ticks=r['last_snapshots'][-1]['position'],
    software_tests_passed=150,current_pose_model_conditions_passed=210,
    loaded_goals_unchanged=True,hardware_health_limits_unchanged=True,assistant_motor_or_setting_writes=0,
    spare_usb_data_cable_available=False,
    current_configuration='hand_serial_link_20261028.json',
    next_user_command=None,next_user_command_ready=False,pending_user_input=None,
    next_user_action='Turn off5V motor power. With cube OUT and hand/base/camera angle unchanged, exchange the robot USB and webcam USB plugs between the laptop two ports. Turn5V back on, do not start programs, send connected. Identify adapter by serial FTBENPSAA and PING/READ at1Mbps only.',
    motion_retry_on_hold=True,
    previous_first_observed_quarter='right_hand_rotation_20261028/completed_quarter_012305/physical_review.json')
save(dest/'review.json',report)
# Preserve the passed software-only proof. The canonical entry must refuse
# movement until the unresolved physical receive path has been rechecked.
held=OUT/'review_software_verified_before_usb_recheck.json'
if not held.exists():held.write_bytes(PROOF.read_bytes())
proof=read(PROOF);proof.update(accepted_for_operator_trial=False,
    operator_hold_reason='Actual1Mbps production receive errors remain; USB port isolation pending.',
    software_verification=verification['tests_passed'],hardware_readiness=False)
save(PROOF,proof)
save(OUT/'readiness.json',dict(report,current_review=(dest/'review.json').relative_to(ROOT).as_posix(),
    verified_software_proof_held_at=held.relative_to(ROOT).as_posix()))
for name in ('right_hand_rotation_20261026/horizontal_next_step_20261026.json',
             'right_hand_rotation_20261026/current_camera_and_yaw_review_20261026.json'):
    path=ROOT/name;value=read(path)
    value.update(status=report['status'],current_status=report['status'],timestamp=report['timestamp'],
        current_hand_review=(dest/'review.json').relative_to(ROOT).as_posix(),
        horizontal_loaded_execution_ready=False,next_user_command=None,next_user_command_ready=False,
        next_user_action=report['next_user_action'],all16_at_1Mbps=True,
        spare_usb_data_cable_available=False,motion_retry_on_hold=True,pending_user_input=None)
    save(path,value)
marker='<!-- END VERIFIED1M LINK STILL UNSTABLE 20261028 -->'
header='''2026-10-28 최신 상태: 사용자가 기존 회전 명령을 두 번 더 실행했으나 04:11은 PLACE 대기 중, 04:14는 큐브 배치 전 사진 단계에서 통신 오류로 종료됐다. 두 실행 모두 OFF와 설정 복구 확인. 이후 사용자가 1Mbps 설정 프로그램을 실행해 16개 전부 속도 변경 및 두 번의 개별 확인을 마쳤으나, 마지막 전체 설정 비교에서 13번 값 차이로 종료됐다. 당시 어느 레지스터가 달랐는지는 로그에 없다. 내가 추가 설정 쓰기 없이 1Mbps에서 다시 읽었을 때 16개 모두 응답했고, 비교 대상 설정은 각 두 번 모두 변경 전 값과 일치했다. 현재 16개 전부 1Mbps이며 설정 프로그램을 다시 실행할 필요는 없다.

원본 바이트를 수집해 카메라 OFF와 ON에서 각각 135바이트와 45바이트가 수신 스트림 중간에 빠진 기록을 확보했다. 특정 케이블, USB 드라이버, 전기적 신호 중 어느 것이 원인인지는 미확정이다. 8개씩 읽기 단독 시험 1,000회는 오류 0이었지만 실제 디코더와 시뮬레이션을 함께 실행하면 1,000회 중 8회 오류가 남았다. 한 개씩 읽기도 단독 탐색 1,000회 오류 0 이후, 실제 디코더와 시뮬레이션 동시 실행에서 6회, 시뮬레이션 종료 뒤 실제 디코더 검사에서 1회 오류가 남았다. 따라서 성공한 시험만 선택해 해결됐다고 판단하지 않는다. 최신 999개 정상 상태는 토크 OFF, 오류와 워치독 0이었다. 소프트웨어 테스트 150개와 현재 복귀 모델 210조건은 통과했으나 실물 통신 문제는 해결되지 않았다.

사용자는 여분 USB 데이터 케이블이 없다고 답했다. 다음 실제 사용자 작업은 5V 모터 전원을 끄고, 기존 로봇 USB와 웹캠 USB를 노트북의 두 포트에서 서로 맞바꾼 뒤 5V 전원을 켜는 것이다. 손과 받침 및 카메라 각도는 그대로 두며, 프로그램은 실행하지 않고 연결했다고 알려주면 된다. 이후 어댑터 시리얼 FTBENPSAA(VID0403/PID6014)를 찾아 실제 포트 번호를 확인하고 1Mbps PING/READ만 수행한다. 추가 회전 시험은 보류한다. 검증된 소프트웨어 근거는 따로 보존하고 기본 회전 실행은 장치를 열기 전에 거부하도록 현재 심사 파일에 보류 상태를 기록했다. 현재 근거 `right_hand_rotation_20261028/verified_1m_usb_isolation/review.json`.

'''+marker+'\n\n'
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_bytes()
    if marker.encode('utf-8') not in old:path.write_bytes(header.encode('utf-8')+old)
print(json.dumps(dict(status=report['status'],all16_1m=True,torque_off=True,software_tests=150,
    actual_receive_errors_remaining=1,motor_writes=0,motion_on_hold=True)))
