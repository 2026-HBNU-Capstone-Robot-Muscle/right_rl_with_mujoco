"""Operator-started G -> horizontal turn -> observed return -> same G hold."""
import argparse,datetime as dt,json
import leap_measured_grip_hold_20261117 as pose
from retaught_transfer_scope_20261118 import limits
from grip_rotation_return_plan_20261118 import GripRotationReturn,Terminal

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--execute',action='store_true');args=ap.parse_args()
    try:
        pose.validate_sources();sequence=GripRotationReturn()
        with limits():
            rows=sequence.rows(sequence.start)
            if not args.execute:
                print(json.dumps(dict(status='device_free_check_passed',goals=len(rows),nominal_seconds=len(rows)*.075,
                    fixed_grip_return_included=True,join_step_ticks=sequence.manifest['join_goal_step_ticks'],
                    devices_opened=False,physical_join_verified=False)));return 0
            return execute(sequence)
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print('NOT READY:',exc,'. No devices opened.');return 2

def execute(sequence):
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    report=dict(status='started',holding=False,settings_restored=False,all_torque_off_verified=False,
        workflow='fixed_grip_turn_checked_return_fixed_grip',source_candidate_sha256=sequence.manifest['candidate_sha256'],
        physical_join_verified=False,physical_cube_angle_measured=False,physical_task_success_verified=False,
        final_fixed_grip_return_included=True,automatic_open=False)
    terminal=Terminal(sequence,report);bus=live=None
    folder=pose.ROOT/('grip_rotation_return_trial_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    try:
        with SingleInstance(pose.ROOT/'.final_grip_pose.lock'):
            print('확정 파지 -> 새 이송 -> 가로 회전 -> 확인 -> 기존 복귀 경로 -> 같은 확정 파지 유지.',flush=True)
            print('전과 같은 파지 위치에 큐브를 준비하세요. 이미 새 이송 자세라면 그대로 이어받습니다.',flush=True)
            print('손을 떼고 SPACE로 시작. 처음 파지 확인과 회전 끝 확인에서 SPACE, 그 사이는 이어서 움직입니다.',flush=True)
            print('기존 최대500초 제한은 확인 대기까지 포함합니다. 이동 시간은 시작 자세에 따라 약7분 전후입니다.',flush=True)
            print('움직이는 중 SPACE/Q/Esc: 중단/OFF. 마지막 파지 유지에서는 큐브를 받친 뒤 SPACE로 종료/OFF.',flush=True)
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
