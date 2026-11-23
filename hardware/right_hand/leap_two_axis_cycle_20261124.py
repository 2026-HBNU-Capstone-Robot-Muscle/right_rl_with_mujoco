"""User-started continuous G -> horizontal -> G -> vertical -> G hold."""
import argparse,datetime as dt,json
import leap_measured_grip_hold_20261117 as pose
from two_axis_cycle_plan_20261124 import TwoAxisCycle,Terminal,PERIOD,RUN_LIMIT_SECONDS

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    try:
        pose.validate_sources();seq=TwoAxisCycle();preview=seq.rows(seq.start)
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print('NOT READY:',exc,'. No devices opened.');return 2
    if not args.execute:
        print(json.dumps(dict(status='device_free_check_passed',goals=len(preview),
            nominal_seconds=len(preview)*PERIOD,devices_opened=False,
            horizontal_G_hold_seconds=seq.metadata()['horizontal_G_hold_seconds'],
            intermediate_SPACE_confirmations=0,intermediate_OFF=False,
            elapsed_time_limit_enabled=False,physical_two_axis_cycle_verified=False)));return 0
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    report=dict(status='started',holding=False,settings_restored=False,all_torque_off_verified=False,
        workflow='G_horizontal_G_vertical_G_one_controller',source_candidate_sha256=seq.manifest['candidate_sha256'],
        physical_task_success_verified=False,physical_cube_placement_handoff_verified=False)
    folder=pose.ROOT/('two_axis_cycle_trial_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    terminal=Terminal();bus=live=None
    try:
        with SingleInstance(pose.ROOT/'.final_grip_pose.lock'):
            print('G -> 가로 회전 -> G 약10초 유지 -> 세로 회전 -> G 계속 유지.',flush=True)
            print('큐브를 확정 파지 자리에 준비합니다. 시작 G 준비 뒤 전체가 자동 진행됩니다.',flush=True)
            print('손을 뗀 뒤 SPACE 한 번으로 시작. 중간 OFF·확인 SPACE·손으로 큐브 재배치는 없습니다.',flush=True)
            print('1배 속도75ms/최대2틱. 준비 이동과 통신·추종 대기를 제외한 경로는 약14분입니다.',flush=True)
            print('이 연결 파일은 전체 시간 제한 없이 진행합니다. SPACE/Q/Esc: 중단/OFF.',flush=True)
            print('마지막 G는 계속 유지합니다. 큐브를 받친 뒤 SPACE로 종료/OFF.',flush=True)
            terminal.wait();folder.mkdir();live=LiveState(folder,report)
            bus=pose.PoseBus(report,terminal.check)
            with seq.runtime_limits():
                initial,start,_=pose.preflight(bus,report,seq.start)
                pose.run_pose(bus,initial,start,seq.rows(start),report,terminal.check,
                    sequence=seq,limit=RUN_LIMIT_SECONDS)
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
