"""User-started retaught transport and existing horizontal turn observation."""
import argparse,datetime as dt,json
import leap_measured_grip_hold_20261117 as pose
from retaught_transfer_scope_20261118 import limits
from retaught_transfer_plan_20261118 import RetaughtTransfer,Terminal

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--execute',action='store_true');ap.add_argument('--transfer-only',action='store_true');args=ap.parse_args()
    try:
        pose.validate_sources();sequence=RetaughtTransfer(args.transfer_only)
        with limits():
            rows=sequence.rows(sequence.start)
            if not args.execute:
                adopted=sequence.rows(sequence.taught)
                print(json.dumps(dict(status='device_free_check_passed',from_grip_goals=len(rows),from_taught_goals=len(adopted),
                    old_full_turn_preserved=not args.transfer_only,devices_opened=False,recorded_pose_clipped=False)))
                return 0
            return execute(sequence)
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print('NOT READY:',exc,'. No devices opened.');return 2

def execute(sequence):
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    report=dict(status='started',holding=False,settings_restored=False,all_torque_off_verified=False,
        workflow='retaught_combined_transfer_and_horizontal_turn',source_candidate_sha256=sequence.manifest['candidate_sha256'],
        physical_cube_angle_measured=False,physical_task_success_verified=False,final_fixed_grip_return_included=False)
    terminal=Terminal(sequence,report);bus=live=None
    folder=pose.ROOT/('retaught_transfer_trial_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    try:
        with SingleInstance(pose.ROOT/'.final_grip_pose.lock'):
            print('새로 기록한 엄지 후퇴·큐브 이송 자세 -> 회전 시작 파지 -> 기존 가로 회전 -> 유지.',flush=True)
            print('이미 기록한 자세에 있으면 그대로 이어받습니다. 다른 시작 자세라면 기존 파지 위치에 큐브를 준비하세요.',flush=True)
            print('손을 뗀 뒤 SPACE로 시작. 중간 확인 안내마다 SPACE. 움직이는 중 SPACE/Q/Esc: 중단/OFF.',flush=True)
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
