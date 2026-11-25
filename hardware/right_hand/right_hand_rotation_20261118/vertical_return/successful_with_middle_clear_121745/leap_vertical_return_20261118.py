"""Return-only test; empty setup avoids replaying the already observed pitch."""
import argparse,datetime as dt,json
import leap_measured_grip_hold_20261117 as pose
from vertical_return_plan_20261118 import VerticalReturn,Terminal

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--execute',action='store_true');args=ap.parse_args()
    try:pose.validate_sources();sequence=VerticalReturn()
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print('NOT READY:',exc,'. No devices opened.');return 2
    if not args.execute:
        rows=sequence.rows(sequence.start)
        print(json.dumps(dict(status='device_free_check_passed',goals=len(rows),return_to_same_G=True,
            pitch_rotation_replayed=False,devices_opened=False,physical_vertical_return_verified=False)));return 0
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    report=dict(status='started',holding=False,settings_restored=False,all_torque_off_verified=False,
        workflow='pitch_end_checked_return_to_fixed_G',source_candidate_sha256=sequence.manifest['candidate_sha256'],
        physical_vertical_return_verified=False,physical_task_success_verified=False,pitch_rotation_replayed=False)
    terminal=Terminal(sequence,report);bus=live=None
    folder=pose.ROOT/('vertical_return_trial_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    try:
        with SingleInstance(pose.ROOT/'.final_grip_pose.lock'):
            print('세로 회전 끝 -> 손바닥 이송 -> 중지 비움 -> 엄지로 검지·중지 사이로 밀기 -> 하부 받침 -> 확정 G 유지.',flush=True)
            print('엄지로 민 뒤 검지 쪽에만 남지 않고 검지·중지 사이로 옮겨졌는지 확인해야 합니다.',flush=True)
            print('먼저 큐브를 손 밖에 두고 빈손으로 SPACE. 준비가 끝난 PLACE 안내에서만 회전 직후와 같은 자리에 큐브를 놓습니다.',flush=True)
            print('안내마다 손을 뗀 뒤 SPACE. 움직이는 중 SPACE/Q/Esc: 중단/OFF. 최종 파지에서는 받친 뒤 SPACE로 종료/OFF.',flush=True)
            terminal.wait();folder.mkdir();live=LiveState(folder,report);bus=pose.PoseBus(report,terminal.check)
            initial,start,_=pose.preflight(bus,report,sequence.start)
            pose.run_pose(bus,initial,start,sequence.rows(start),report,terminal.check,sequence=sequence)
    except (pose.OperatorStop,KeyboardInterrupt):report['status']='operator_finished' if report.get('holding') else 'operator_stopped'
    except BaseException as exc:report.update(status='stopped',error=str(exc));print('STOP:',exc,flush=True)
    finally:
        if bus is not None:bus.close()
        if live is not None:report['live_state_writer']=live.close()
        report['serial_port_closed']=True
        if folder.exists():
            (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            print('Saved:',folder,flush=True)
    return 0 if report['status'] in ('operator_finished','operator_stopped') and not report.get('cleanup_error') else 1

if __name__=='__main__':raise SystemExit(main())
