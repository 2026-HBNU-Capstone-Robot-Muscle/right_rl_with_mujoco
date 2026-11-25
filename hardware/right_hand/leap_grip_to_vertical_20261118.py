"""Operator-started vertical relocation -> thumb-first revised pitch -> hold."""
import argparse,datetime as dt,json
import leap_measured_grip_hold_20261117 as pose
from vertical_transfer_plan_20261118 import VerticalTransfer,Terminal

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--execute',action='store_true');ap.add_argument('--from-grip',action='store_true');args=ap.parse_args()
    try:pose.validate_sources();sequence=VerticalTransfer(force_grip=args.from_grip)
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print('NOT READY:',exc,'. No devices opened.');return 2
    if not args.execute:
        a=sequence.rows(sequence.start);b=sequence.rows(sequence.taught)
        print(json.dumps(dict(status='device_free_check_passed',from_G_goals=len(a),from_taught_goals=len(b),
            source_pitch_goals=1268,queued_pitch_goals=len(sequence.goals)-sequence.preparation,
            thumb_before_index_middle_raise=True,devices_opened=False,final_fixed_grip_return_included=False)));return 0
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    report=dict(status='started',holding=False,settings_restored=False,all_torque_off_verified=False,
        workflow='taught_vertical_relocation_then_thumb_first_pitch',source_candidate_sha256=sequence.manifest['candidate_sha256'],
        physical_task_success_verified=False,physical_cube_angle_measured=False,final_fixed_grip_return_included=False)
    terminal=Terminal(sequence,report);bus=live=None
    folder=pose.ROOT/('vertical_transfer_trial_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    try:
        with SingleInstance(pose.ROOT/'.final_grip_pose.lock'):
            print('파지 -> 검지·중지 먼저 비우기 -> 엄지로 밀면서 약지 비키기 -> 세로 시작 준비 -> 세로 회전/유지.',flush=True)
            print('뒤집기 구간은 엄지 내림을 먼저 끝내고,확인 SPACE 뒤 검지·중지를 세우도록 바꿨습니다.',flush=True)
            print('이번 파일은 G파지부터 시작합니다. 전과 같은 파지 위치에 큐브를 준비하세요.' if args.from_grip else '지금 기록한 이송 자세에 있으면 이어받습니다. 그 외에는 G파지에서 시작합니다.',flush=True)
            print('손을 뗀 뒤 SPACE로 시작. 각 확인 안내마다 SPACE,움직이는 중 SPACE/Q/Esc: 중단/OFF.',flush=True)
            print('이번 파일은 세로 회전 끝 유지까지입니다. G복귀는 세로 끝의 실제 상태를 보고 연결합니다.',flush=True)
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
