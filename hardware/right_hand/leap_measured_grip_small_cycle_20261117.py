"""Operator-started small-motion observation, not a90degree rotation cycle."""
from pathlib import Path
import argparse,datetime as dt,json
import leap_measured_grip_hold_20261117 as pose
from measured_grip_small_cycle_plan_20261117 import SmallCycle


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    if args.execute:
        print('소각도 실물 시험은 철회했습니다. 요청은90도급 회전과 확정 파지의 연결입니다. 장치에 접근하지 않습니다.',flush=True)
        return 2
    try:
        pose.validate_sources();sequence=SmallCycle()
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print('NOT READY:',exc,'. No devices opened.',flush=True);return 2
    if not args.execute:
        rows=sequence.rows(sequence.grip)
        print(json.dumps(dict(status='device_free_check_passed',goal_count=len(rows),
            stages=list(dict.fromkeys(sequence.labels)),devices_opened=False,
            fixed_grip_target_unchanged=True,requested_small_turn_degrees=[5,5],
            whole90degree_cycle=False),ensure_ascii=False));return 0
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    terminal=pose.Terminal();bus=live=None
    folder=pose.ROOT/('measured_grip_small_cycle_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    report=dict(status='started',holding=False,settings_restored=False,all_torque_off_verified=False,
        workflow='fixed_grip_then_two_small_turns_then_hold',rotation_included=True,
        whole90degree_cycle=False,final_fixed_grip_return_included=False,
        requested_small_turn_degrees=[5,5],cube_angle_measured=False,
        trajectory_sha256=sequence.manifest['trajectory_sha256'],automatic_open=False,
        reference_sha256=pose.sha(pose.REFERENCE),hardware_execution_by_operator=True)
    try:
        with SingleInstance(pose.ROOT/'.final_grip_pose.lock'):
            print('확정 파지 →5초 유지 → 작은 회전 두 번 → 현재 자세 유지.',flush=True)
            print('90도 전체 회전이나 확정 파지로 복귀하는 시험은 아직 아닙니다.',flush=True)
            print('큐브를 손 위에 놓고 손을 뗀 뒤 이 창에서 SPACE. SPACE/Q/Esc: 중단 및OFF.',flush=True)
            terminal.wait();folder.mkdir();live=LiveState(folder,report)
            bus=pose.PoseBus(report,terminal.check)
            initial,start,_=pose.preflight(bus,report,sequence.grip)
            rows=sequence.rows(start)
            pose.run_pose(bus,initial,start,rows,report,terminal.check,sequence=sequence)
            if report.get('holding') and report['status']=='started':report['status']='small_cycle_hold_finished'
    except (pose.OperatorStop,KeyboardInterrupt):
        report['status']='operator_finished' if report.get('holding') else 'operator_stopped'
    except BaseException as exc:
        report.update(status='stopped',error=str(exc));print('STOP:',exc,flush=True)
    finally:
        if bus is not None:bus.close()
        if live is not None:report['live_state_writer']=live.close()
        report['serial_port_closed']=True
        if folder.exists():
            (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            print('Saved:',folder,flush=True)
    return 0 if not report.get('cleanup_error') and report['status'] in ('operator_finished','operator_stopped','small_cycle_hold_finished') else 1


if __name__=='__main__':raise SystemExit(main())
