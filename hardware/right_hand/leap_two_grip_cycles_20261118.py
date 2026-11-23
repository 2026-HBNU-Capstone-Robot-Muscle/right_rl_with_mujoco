"""User-started two-cycle trial, same speed/guards, separate bounded time profile."""
import argparse,datetime as dt,json
import leap_measured_grip_hold_20261117 as pose
from retaught_transfer_scope_20261118 import limits
from two_grip_cycles_plan_20261118 import TwoGripCycles,Terminal,CycleDeadline,TOTAL_SECONDS

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--execute',action='store_true');args=ap.parse_args()
    try:
        pose.validate_sources();sequence=TwoGripCycles()
        with limits():
            rows=sequence.rows(sequence.start)
            if not args.execute:
                print(json.dumps(dict(status='device_free_check_passed',goals=len(rows),nominal_seconds=len(rows)*.075,
                    per_cycle_maximum_seconds=500,total_maximum_seconds=1000,devices_opened=False,
                    second_rotation_link_physically_verified=False)));return 0
            return execute(sequence)
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print('NOT READY:',exc,'. No devices opened.');return 2

def execute(sequence):
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    report=dict(status='started',holding=False,settings_restored=False,all_torque_off_verified=False,
        workflow='two_observed_grip_rotation_return_cycles',source_candidate_sha256=sequence.manifest['source_candidate_sha256'],
        physical_task_success_verified=False,physical_cube_angle_measured=False,automatic_open=False)
    sequence.bind(report);terminal=Terminal(sequence,report);bus=live=None
    folder=pose.ROOT/('two_grip_cycles_trial_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    try:
        with SingleInstance(pose.ROOT/'.final_grip_pose.lock'):
            print('파지 -> 회전 -> 파지 복귀 -> 큐브 그대로 확인 -> 다시 회전 -> 파지 복귀/유지.',flush=True)
            print('전과 같은 큐브 배치로 손을 뗀 뒤 SPACE. 처음 파지와 각 회전 끝에서 SPACE로 확인합니다.',flush=True)
            print('첫 파지 복귀에서 큐브를 빼거나 재배치하지 말고 SPACE로 두 번째 회전. 중간 토크 OFF는 하지 않습니다.',flush=True)
            print('속도는 그대로이며 약14~15분. 회당500초 제한/전체1000초 제한,전류·온도·추종 중단 기준을 유지합니다.',flush=True)
            print('움직이는 중 SPACE/Q/Esc는 중단/OFF. 마지막에는 큐브를 받치고 SPACE로 종료/OFF.',flush=True)
            terminal.wait();folder.mkdir();live=LiveState(folder,report);bus=pose.PoseBus(report,terminal.check)
            initial,start,_=pose.preflight(bus,report,sequence.start)
            pose.run_pose(bus,initial,start,sequence.rows(start),report,terminal.check,sequence=sequence,limit=TOTAL_SECONDS)
            if report['status']=='existing500second_limit_reached':report['status']='two_cycle1000second_limit_reached'
    except CycleDeadline as exc:report.update(status='per_cycle500second_limit_reached',time_limit_reason=str(exc));print('TIME LIMIT:',exc,flush=True)
    except (pose.OperatorStop,KeyboardInterrupt):report['status']='operator_finished' if report.get('holding') else 'operator_stopped'
    except BaseException as exc:report.update(status='stopped',error=str(exc));print('STOP:',exc,flush=True)
    finally:
        if bus is not None:bus.close()
        if live is not None:report['live_state_writer']=live.close()
        report['serial_port_closed']=True
        if folder.exists():
            (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            print('Saved:',folder,flush=True)
    return 0 if report['status'] in ('operator_finished','operator_stopped','per_cycle500second_limit_reached','two_cycle1000second_limit_reached') and not report.get('cleanup_error') else 1

if __name__=='__main__':raise SystemExit(main())
