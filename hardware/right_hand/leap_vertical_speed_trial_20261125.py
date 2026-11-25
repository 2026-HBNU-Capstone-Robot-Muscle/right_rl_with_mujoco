"""Separate, operator-started retimed vertical experiment; default checks only."""
import argparse
import datetime as dt
import json
import leap_measured_grip_hold_20261117 as pose
from vertical_speed_trial_plan_20261125 import VerticalSpeedTrial, PERIOD


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    try:
        pose.validate_sources()
        sequence = VerticalSpeedTrial()
        rows = sequence.rows(sequence.start)
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        print('NOT READY:', exc, 'No devices opened.')
        return 2
    if not args.execute:
        print(json.dumps(dict(status='device_free_check_passed',
            physical_execution_authorized=sequence.authorized(), goals=len(rows),
            nominal_seconds=len(rows)*PERIOD, maximum_step_ticks=6,
            communication_period_seconds=PERIOD, baseline_files_unchanged=True,
            devices_opened=False, physical_speedup_verified=False)))
        return 0
    if not sequence.authorized():
        print('NOT READY: 이 별도 파일의 최대6틱 변경 확인이 필요합니다. 장치를 열지 않았습니다.')
        return 2
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    from vertical_direct_grip_plan_20261118 import Terminal
    report = dict(status='started', holding=False, all_torque_off_verified=False,
        settings_restored=False, physical_task_success_verified=False,
        source_candidate_sha256=sequence.manifest['candidate_sha256'],
        baseline_candidate_sha256=sequence.manifest['baseline_candidate_sha256'])
    folder = pose.ROOT/('vertical_speed_trial_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    terminal = Terminal(sequence, report)
    bus = live = None
    try:
        with SingleInstance(pose.ROOT/'.final_grip_pose.lock'):
            print('별도 세로 속도 실험: G 준비 -> 세로 회전 -> G 복귀/계속 유지.', flush=True)
            print('기존75ms 통신/전체 상태 읽기/PID/450mA를 유지합니다.', flush=True)
            print('이 파일에서만 회전·복귀 목표 간격 최대6틱(약0.53도). 시작 준비는2틱/1배.', flush=True)
            print('G 근처의 명목 시간은 약2분30초이며 실제3배속·파지 안정성은 실물 확인 전입니다.', flush=True)
            print('손을 떼고 SPACE로 시작. 이동 중 SPACE/Q/Esc: 중단/OFF.', flush=True)
            print('마지막 G에서는 큐브를 받친 뒤 SPACE로 종료/OFF.', flush=True)
            terminal.wait()
            folder.mkdir()
            live = LiveState(folder, report)
            bus = pose.PoseBus(report, terminal.check)
            with sequence.runtime_scope():
                initial, start, _ = pose.preflight(bus, report, sequence.start)
                pose.run_pose(bus, initial, start, sequence.rows(start), report, terminal.check,
                              sequence=sequence, limit=float('inf'))
    except (pose.OperatorStop, KeyboardInterrupt):
        report['status'] = 'operator_finished' if report.get('holding') else 'operator_stopped'
    except BaseException as exc:
        report.update(status='stopped', error=str(exc))
        print('STOP:', exc, flush=True)
    finally:
        if bus is not None:
            bus.close()
        if live is not None:
            report['live_state_writer'] = live.close()
        report['serial_port_closed'] = True
        if folder.exists():
            (folder/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
            print('Saved:', folder, flush=True)
    return 0 if report['status'] in ('operator_finished', 'operator_stopped') and not report.get('cleanup_error') else 1


if __name__ == '__main__':
    raise SystemExit(main())
