"""User-started fixed grasp -> turn preparation -> existing full-turn observation."""
import argparse,datetime as dt,json
import leap_measured_grip_hold_20261117 as pose
from grip_to_rotation_plan_20261118 import GripToRotation,GripTurnTerminal

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--execute',action='store_true');ap.add_argument('--transfer-only',action='store_true');args=ap.parse_args()
    try:pose.validate_sources();sequence=GripToRotation(transfer_only=args.transfer_only)
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print('NOT READY:',exc,'. No devices opened.');return 2
    if not args.execute:
        rows=sequence.rows(sequence.start)
        print(json.dumps(dict(status='device_free_check_passed',goals=len(rows),
            full_turn_goals_preserved=len(sequence.goals)-sequence.preparation,devices_opened=False,
            transfer_observation_only=args.transfer_only,
            new_handover_physically_verified=False,fixed_grip_return_included=False)));return 0
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    report=dict(status='started',holding=False,settings_restored=False,all_torque_off_verified=False,
        workflow='fixed_grip_then_measured_receiving_observation' if args.transfer_only else 'fixed_grip_then_existing_full_turn',source_candidate_sha256=sequence.manifest['candidate_sha256'],
        transfer_observation_only=args.transfer_only,
        physical_cube_angle_measured=False,physical_task_success_verified=False,
        final_fixed_grip_return_included=False,automatic_open=False)
    terminal=GripTurnTerminal(sequence,report);bus=live=None
    folder=pose.ROOT/('grip_to_rotation_trial_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    try:
        with SingleInstance(pose.ROOT/'.final_grip_pose.lock'):
            print('확정 파지 -> 엄지 후퇴 -> 기록한 검지·중지 받침 자세 -> 유지. 큐브 이송 관찰용입니다.' if args.transfer_only else '확정 파지 -> 회전 준비 -> 기존90도급 회전 -> 회전 끝 유지. 단계 관찰용입니다.',flush=True)
            print('큐브를 기존 파지 위치에 올려두고 손을 뗀 뒤 SPACE. Q/Esc: 취소.',flush=True)
            print('다른 로봇 실행창은 먼저 정상 종료하세요. 중간 안내마다 지지를 확인한 뒤 SPACE.',flush=True)
            if not args.transfer_only:
                print('앞서 실행한 검지·중지 받침까지는 같은 경로입니다. 이후 약지로 밀기 -> 엄지가 받기 -> 중지·약지 함께 인계 -> 회전입니다.',flush=True)
                print('약지 밀기와 회전 연결은 이번 실물 관찰 대상입니다.',flush=True)
                print('회전 시작 파지 안내에서 큐브가 엄지·중지 사이에 잡혔는지 확인한 뒤 SPACE. 끼거나 자리가 맞지 않으면 받치고 Q로 종료하세요.',flush=True)
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
