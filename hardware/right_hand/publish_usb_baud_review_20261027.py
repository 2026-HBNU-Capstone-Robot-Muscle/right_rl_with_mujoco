"""Record baud/USB latency measurements and the pending Windows admin step."""
import datetime as dt,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
folder=ROOT/'right_hand_rotation_20261027'/'usb_baud_latency_review';folder.mkdir(exist_ok=True)
sample=ROOT/'serial_benchmark_readonly_20261027_190008_128041'/'result.json'
data=json.loads(sample.read_text(encoding='utf-8'))
files=['benchmark_current_serial_readonly_20261027.py','set_com7_usb_latency_20261027.py',
       'usb_latency_admin_workflow_20261027.py','test_usb_latency_workflow_20261027.py','leap_usb_latency_1ms.cmd']
report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='driver_latency_admin_step_ready_not_applied',
    motor_baud_bps=3000000,motor_baud_changed=False,motor_or_torque_writes=0,
    source=str(sample.relative_to(ROOT)),read_frames=100,communication_failures=0,
    median_read_ms=data['median_read_ms'],p95_read_ms=data['p95_read_ms'],maximum_read_ms=data['maximum_read_ms'],
    minimum_voltage_v=data['minimum_voltage_v'],all16_baud_registers_value5=True,
    current_driver_latency_ms=16,desired_driver_latency_ms=1,driver_latency_changed=False,
    reason_for_not_changing_baud='100 healthy full reads at3Mbps;4Mbps reduces estimated wire time only1.278ms for current packet size; prioritize official USB latency recommendation.',
    wire_time_estimates_ms=data['wire_time_ms'],wire_estimates_exclude_usb_delay_and_parsing=True,
    windows_error='WinError5 access denied; automatic sandbox review allowed execution, but Windows administrator token was absent.',
    automatic_approval_review_rejected=False,next_user_command='leap_usb_latency_1ms.cmd',
    next_user_action='Close Wizard; run prepared command once and approve Windows administrator dialog. No hand motion. Send done.',
    post_apply_checks='100 complete healthy OFF reads, unchanged16 motor baud registers, median improvement>=15%, max<75ms; otherwise registry rollback.',
    tests_passed=8,tests_used_mock_registry_and_motor_bus=True,
    physical_horizontal_90_verified=False,physical_loaded_voltage_after_rewire_verified=False,
    horizontal_motion_command_remains='leap_horizontal_quarter_trial.cmd',horizontal_control_sources_changed=False,
    source_hashes={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files},
    sources=['https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/#baud-rate8',
        'https://emanual.robotis.com/docs/en/parts/interface/u2d2/#usb-latency-setting',
        'https://ftdichip.com/Support/Knowledgebase/settingacustomdefaultlaten.htm',
        'https://learn.microsoft.com/en-us/windows-hardware/drivers/devtest/pnputil-command-syntax'])
(folder/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
text='''# USB 통신 설정 확인 2026-10-27

모터16개의 Baud Rate(8)는 모두5, 즉3Mbps다. 현재 설정에서 전체 상태100회 읽기에 오류가 없었으며 중앙값26.00ms,95백분위26.50ms,최대26.70ms,최저 정지 전압5.0V였다.

2Mbps/3Mbps/4Mbps는 데이터 전달 속도이며 목표 관절 속도나 전압 설정이 아니다. 현 상태 읽기의 추정 전송량1534바이트에 대한 순수 전송 시간은 각각7.67/5.11/3.84ms다. USB 지연,패킷 간 시간,드라이버와Python 처리 시간은 이 계산에 포함되지 않는다.4Mbps가 현재 정지 원인을 해결한다는 근거는 없어 모터 통신 속도는 바꾸지 않았다.

COM7 FTDI의 LatencyTimer는16ms로 확인됐다. ROBOTIS는 Windows USB Latency Timer1ms를 권장한다. 변경 실행은 Windows 관리자 권한 부족(WinError5)으로 중단됐으며 실제 레지스트리 변경은0회다. 자동 승인 검토가 거절한 상황은 아니다.

사용자 다음 작업: `leap_usb_latency_1ms.cmd`를 한 번 실행하고 Windows 관리자 창을 허용한다. 위자드 등 다른 모터 프로그램은 닫아둔다. 준비된 명령은 현재 토크OFF와16개3Mbps를 다시 읽고,COM7의LatencyTimer 한 항목만 백업 후1ms로 변경한다.100회 읽기로 적용을 확인하고 필요할 때 해당 COM7 장치만 한 번 재시작한다. 재부팅이나 모터 설정/목표/토크 쓰기는 없다. 개선이 검증되지 않으면 원래 레지스트리 값으로 복구하고 결과를 저장한다. 실제 드라이버의 복구 적용에는 USB 재연결이 필요할 수 있다.

결과 위치: `leap_usb_latency_result.json` 및 `usb_latency_change_*/workflow_result.json`. 준비 코드의 고장 조건 테스트8개 통과. 현재까지 적용 후 실측은 수행 전이다. 손 동작 경로와 v5 승인 코드는 변경하지 않았다.
'''
(folder/'review.md').write_text(text,encoding='utf-8')
header='''USB 통신 확인: 현재3Mbps에서16개 전체 상태100회 읽기 오류0,중앙값26.00ms. COM7 USB 수신 지연16ms를 공식 권장1ms로 조정하는 사용자 실행 명령 `leap_usb_latency_1ms.cmd` 준비. Windows 관리자 권한이 필요해 아직 적용 전이며, 모터 bps/동작/토크/경로는 변경하지 않음. 결과는 `right_hand_rotation_20261027/usb_baud_latency_review/review.json`. 모터 구동 중 전압과 수평90도는 여전히 미검증.

<!-- END USB BAUD REVIEW 20261027 -->

'''
for name in ('professor_progress_20261015.md','sac_current_status.md'):
    path=ROOT/name;old=path.read_text(encoding='utf-8')
    if '<!-- END USB BAUD REVIEW 20261027 -->' not in old:path.write_text(header+old,encoding='utf-8')
print(json.dumps(dict(report_file=str(folder/'review.json'),status=report['status'],tests_passed=8,hardware_or_registry_changed=False),ensure_ascii=False))
