"""Separate3x horizontal/vertical experiment; devices open only after user SPACE."""
import argparse
import datetime as dt
import json

import calibrated_two_full_health_20261028 as calibrated
import leap_measured_grip_hold_20261117 as pose
import leap_vertical_direct_grip_recovery_20261125 as recovery_entry
import leap_vertical_direct_grip_resume_20261125 as resume_entry
from grip_pause_resume_10s_trial_20261125 import sample, POLICY as PAUSE_POLICY
from speed_trial_state_observer_20261125 import ObservedPauseBus as PauseBus
from rotation_speed_resume_plan_20261125 import RotationSpeedTrial


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axis', choices=['horizontal', 'vertical'], required=True)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    try:
        resume_entry.validate_sources()
        pose.validate_sources()
        sequence = RotationSpeedTrial(args.axis)
        rows = sequence.rows(sequence.start)
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        print('NOT READY:', exc, 'No devices opened.')
        return 2
    if not args.execute:
        metadata = {k:v for k,v in sequence.metadata().items()
                    if k not in ('selected_source_indices', 'retained_mandatory_indices')}
        print(json.dumps(dict(status='device_free_check_passed', axis=args.axis,
            devices_opened=False, authorized=sequence.authorized(), goals=len(rows),
            communication_pause_maximum_seconds=PAUSE_POLICY['maximum_extra_pause_s'],
            **metadata), ensure_ascii=False))
        return 0
    if not sequence.authorized():
        print('NOT READY: New experimental6tick authorization missing. No devices opened.')
        return 2
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    report = dict(status='started', holding=False, all_torque_off_verified=False,
        settings_restored=False, physical_task_success_verified=False,
        source_candidate_sha256=sequence.manifest['candidate_sha256'],
        baseline_candidate_sha256=sequence.manifest['baseline_candidate_sha256'])
    folder = pose.ROOT/('rotation_speed_resume_'+args.axis+'_'+
        dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    stop = recovery_entry.StopRequest()
    terminal = recovery_entry.terminal_for(stop)(sequence, report)
    bus = live = None
    previous_sample = calibrated.sample
    try:
        with SingleInstance(pose.ROOT/'.final_grip_pose.lock'):
            stop.install()
            print('별도3배속 실험 / '+('가로' if args.axis=='horizontal' else '세로')+' 회전 → G 복귀/유지.', flush=True)
            print('통신75ms·450mA·PID·관절/추종/진행 검사와 별도 최대10초 READ 복구를 사용합니다.', flush=True)
            print('시작 G 준비는1배속/2틱. 회전·복귀만 최대6틱(약0.53도) 간격입니다.', flush=True)
            if args.axis == 'horizontal':
                print('약지 부하가 컸던 마지막 받침·파지 완성 구간은 기존1배속/2틱으로 진행합니다.', flush=True)
                print('복귀는 엄지 먼저 내림 → 검지·중지 받침/밀기 → G 완성 순서입니다.', flush=True)
            print('명목 '+f'{len(rows)*.075:.1f}'+'초(G 시작 기준). 실제3배속·큐브 안정성은 실물 확인 전입니다.', flush=True)
            print('손을 떼고 SPACE로 시작. 중간 확인은 자동, 이동 중 SPACE/Q/Esc는 중단/OFF.', flush=True)
            print('최종 G에서는 큐브를 받친 뒤 SPACE로 종료/OFF. 원래1배속 파일은 그대로입니다.', flush=True)
            terminal.wait()
            folder.mkdir()
            live = LiveState(folder, report)
            bus = PauseBus(report, terminal.check)
            report['communication_repair_policy'].update(maximum_goal_step_ticks=6,
                maximum_entry_step_ticks=2, separate_speed_experiment=True)
            with sequence.runtime_scope():
                calibrated.sample = sample
                initial, start, _ = pose.preflight(bus, report, sequence.start)
                pose.run_pose(bus, initial, start, sequence.rows(start), report, terminal.check,
                              sequence=sequence, limit=float('inf'))
    except (pose.OperatorStop, KeyboardInterrupt):
        report['status'] = 'operator_finished' if report.get('holding') else 'operator_stopped'
    except BaseException as exc:
        report.update(status='stopped', error=str(exc))
        print('STOP:', exc, flush=True)
    finally:
        calibrated.sample = previous_sample
        try:
            if bus is not None:
                bus.close()
        except BaseException as exc:
            report['port_close_error'] = str(exc)
        finally:
            stop.close()
        if live is not None:
            report['live_state_writer'] = live.close()
        report['serial_port_closed'] = 'port_close_error' not in report
        if folder.exists():
            (folder/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
            print('Saved:', folder, flush=True)
    return 0 if (report['status'] in ('operator_finished', 'operator_stopped')
                 and not report.get('cleanup_error') and not report.get('port_close_error')) else 1


if __name__ == '__main__':
    raise SystemExit(main())
