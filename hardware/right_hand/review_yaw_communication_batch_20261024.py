"""Review saved attempts and OS port inventory only. Never opens a port."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np
from serial.tools.list_ports import comports

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024'
NAMES=('grasped_yaw_trial_20261024_033641_017566','grasped_yaw_trial_20261024_033813_387413',
       'grasped_yaw_trial_20261024_034249_800532','grasped_yaw_trial_20261024_034958_390218')


def main():
    attempts=[]
    for name in NAMES:
        path=ROOT/name/'result.json';blob=path.read_bytes();r=json.loads(blob);samples=r.get('samples',[])
        counts=Counter(s['phase'] for s in samples)
        assert r['status']=='stopped'
        assert not counts.get('stage_3_loaded_approach') and not counts.get('stage_5_grasped_small_yaw')
        row=dict(folder=name,source_sha256=hashlib.sha256(blob).hexdigest(),error=r['error'],
            failed_phase=r['failed_phase'],sample_counts=dict(counts),
            all_torque_off_verified=r['all_torque_off_verified'],settings_restored=r['settings_restored'],
            camera_restored=r['camera_exposure_restoration_verified'],
            accepted_release_confirmation=bool(r.get('contact_release_operator_confirmed')),
            cube_placement_confirmed=bool(r.get('cube_placement_confirmed')),
            read_failure_events=r.get('read_failures',[]),
            read_recovery_confirmation_events=r.get('read_recovery_confirmation_events',[]),
            rotation_attempted=False)
        if samples:
            assert row['all_torque_off_verified'] and row['settings_restored']
            row.update(minimum_voltage_v=min(min(s['voltage_v']) for s in samples),
                peak_current_ma=max(abs(v) for s in samples for v in s['current_ma']),
                maximum_tracking_rad=max(max(s['tracking_rad']) for s in samples),
                read_ms_quantiles=dict(zip(('p50','p95','p99','max'),np.quantile([s['read_ms'] for s in samples],[.5,.95,.99,1]).tolist())),
                phase_duration_from_samples_s={phase:max(s['t'] for s in samples if s['phase']==phase)-min(s['t'] for s in samples if s['phase']==phase) for phase in counts})
        attempts.append(row)
    ports=[dict(device=p.device,description=p.description,hwid=p.hwid,vid=p.vid,pid=p.pid,
                serial_number=p.serial_number) for p in comports()]
    report=dict(status='three_reconnected_attempts_stopped_before_placement_due_to_receive_failures',
        date_client='2026-10-24',attempts=attempts,current_OS_ports=ports,
        current_COM7_enumerated=any(p['device']=='COM7' for p in ports),
        confirmed_cause='receive failures and post-recovery confirmation limit',
        physical_cause_identified=False,specific_motor_fault_identified=False,
        current_or_tracking_fault_detected=False,software_limits_changed=False,
        motion_or_setting_commands_sent_by_review=0,serial_ports_opened_by_review=0,
        next_operator_action='Motor power OFF; reseat PC-to-U2D2 USB cable, use a direct PC USB port if currently on a hub. Report connected before any new motion trial.')
    (OUT/'yaw_communication_batch_review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    title='## CURRENT 2026-10-24 03:49: 三次通信中止'.replace('三次通信中止','3회 재실행 통신 중단 / 회전 미실행')
    section=f'''{title}

- 03:36에는 COM7 자체를 열 수 없어 모터 명령 전 중단됐다. 이후 03:38, 03:42, 03:49의 3개 저장 결과를 직접 확인했다. 세 실행 모두 새 시작 경로 검사는 통과했으나 PLACE와 파지 및 새 회전 구간에는 도달하지 않았다.
- 03:38과 03:42는 약지 분리 확인 SPACE를 기다리는 `release_contact_confirmation` 중 수신 패킷 손상 또는 복구 확인 중 재통신 오류로 중단됐다. 사용자 분리 확인이 수락된 기록은 없다. 03:49는 첫 약지 분리 동작 중 복구 뒤 새 상태 두 번 확인이 기존 200ms 예산을 넘겨 중단됐다. 실제 기록은 약 235ms다. 중단 기준이나 재시도 횟수를 늘리지 않았다.
- 이 세 실행은 모두 토크 OFF, 설정 복원과 카메라 노출 복원이 확인됐다. 최소 전압은 4.2V, 4.2V, 4.3V이고 최대 전류는 41mA, 49mA, 37mA다. 추종 오차는 모두 약 0.025rad 이하이다. 이번 기록의 중단 원인은 저전압이나 관절 걸림이 아니라 통신 읽기 실패다. 특정 모터 또는 물리 배선 원인까지 확정한 것은 아니다.
- 지금 OS 포트 목록에는 USB Serial COM7이 다시 있다. 최초 포트 소실과 이후 수신 실패는 구분한다. 기존 LED OFF 검사가 통과했더라도 지금 조건의 통신 안정성을 보증하지 않는다. 이전 23:47 실행의 회전 명령 완료와 이번 세 실행의 미실행도 구분한다.
- **지금 사용자 작업:** 모터 전원을 끈 뒤 PC와 U2D2 사이 USB 케이블을 다시 연결한다. USB 허브를 쓰고 있으면 PC 본체 포트에 직접 연결한다. 연결 후 알려주면 포트 재인식부터 확인한다. 같은 동작 명령의 추가 반복은 아직 요청하지 않는다.
- 근거: `right_hand_rotation_20261024/yaw_communication_batch_review.json`. 원본 결과와 사진, 모터 제어 코드, 전류 및 안전 제한은 수정하지 않았다. 검토 과정에서 실제 포트나 카메라를 열지 않았다.

'''
    for name in ('sac_current_status.md','professor_progress_20261015.md'):
        path=ROOT/name;text=path.read_text(encoding='utf-8')
        if text.startswith(title):
            boundary=text.find('## PREVIOUS ');text=text[boundary:] if boundary>=0 else ''
        else:text=text.replace('## CURRENT ','## PREVIOUS ',1)
        path.write_text(section+text,encoding='utf-8')
    print(json.dumps(dict(attempts=[dict(folder=r['folder'],failed_phase=r['failed_phase'],
        receive_failure_events=len(r['read_failure_events']),OFF=r['all_torque_off_verified'],
        settings_restored=r['settings_restored'],read_ms=r.get('read_ms_quantiles')) for r in attempts],
        current_COM7_enumerated=report['current_COM7_enumerated'],
        physical_cause_identified=False,serial_ports_opened_by_review=0),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
