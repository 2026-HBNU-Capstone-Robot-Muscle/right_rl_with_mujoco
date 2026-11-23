"""User-started staged observation of full recorded turn -> receiving -> fixed grip."""
import argparse,datetime as dt,json
import leap_measured_grip_hold_20261117 as pose
from quarter_fixed_grip_plan_20261118 import QuarterFixedGrip,QuarterTerminal


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--execute',action='store_true');args=ap.parse_args()
    try:
        pose.validate_sources();sequence=QuarterFixedGrip()
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print('NOT READY:',exc,'. No devices opened.',flush=True);return 2
    if not args.execute:
        rows=sequence.rows(sequence.start)
        print(json.dumps(dict(status='device_free_check_passed',loaded_source_goals=len(sequence.goals),
            first_turn1417_preserved=True,fixed_grip_unchanged=True,period_s=.075,max_step_ticks=2,
            staged_operator_checks=True,loaded_model_full_task_passed=False,devices_opened=False,
            full_autonomous_cycle_verified=False),ensure_ascii=False));return 0
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    report=dict(status='started',holding=False,settings_restored=False,all_torque_off_verified=False,
        workflow='observed_quarter_turn_then_receiving_then_fixed_grip',automatic_open=False,
        staged_operator_observation=True,loaded_model_full_task_passed=False,
        physical_cube_angle_measured=False,physical_task_success_verified=False,
        full_autonomous_cycle_verified=False,source_candidate_sha256=sequence.manifest['candidate_sha256'])
    terminal=QuarterTerminal(sequence,report);bus=live=None
    folder=pose.ROOT/('quarter_fixed_grip_trial_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    try:
        with SingleInstance(pose.ROOT/'.final_grip_pose.lock'):
            print('기존 회전 -> 중지 빼기 -> 엄지 밀기 -> 엄지 후퇴/검지·중지 끝마디 준비 -> 아래 받침 밀기 -> 확정 파지 유지: 단계 확인 실기.',flush=True)
            print('먼저 큐브를 빼고 빈손으로 시작하세요. 준비 후 PLACE에서 큐브를 놓습니다.',flush=True)
            print('회전/중지 후퇴/엄지 밀기/끝마디 준비/아래 밀기 뒤에는 확인을 기다립니다. 안내마다 손을 뗀 뒤 SPACE로 다음 단계.',flush=True)
            print('전체 모델 과제는 미통과이며, 이번 파일은 새 파지 전환의 실물 단계 확인용입니다.',flush=True)
            print('손을 떼고 이 실행창에서 SPACE로 빈손 준비 시작. Q/Esc: 취소.',flush=True)
            terminal.wait();folder.mkdir();live=LiveState(folder,report)
            bus=pose.PoseBus(report,terminal.check)
            initial,start,_=pose.preflight(bus,report,sequence.start);rows=sequence.rows(start)
            pose.run_pose(bus,initial,start,rows,report,terminal.check,sequence=sequence)
            if report.get('holding') and report['status']=='started':report['status']='fixed_grip_observation_finished'
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
    return 0 if not report.get('cleanup_error') and report['status'] in ('operator_finished','operator_stopped','fixed_grip_observation_finished') else 1


if __name__=='__main__':raise SystemExit(main())
