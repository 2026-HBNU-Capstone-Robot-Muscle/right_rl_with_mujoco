"""User-started vertical turn and taught direct-thumb return trial."""
import argparse,datetime as dt,json
import leap_measured_grip_hold_20261117 as pose
from vertical_direct_grip_plan_20261118 import VerticalDirectGrip,Terminal,RUN_LIMIT_SECONDS
from vertical_direct_transport_20261124 import DirectPoseBus

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--execute',action='store_true');args=ap.parse_args()
    try:
        pose.validate_sources();seq=VerticalDirectGrip();preview=seq.rows(seq.start)
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print('NOT READY:',exc,'. No devices opened.');return 2
    if not args.execute:
        print(json.dumps(dict(status='device_free_check_passed',goals=len(preview),nominal_seconds=sum(seq.period(i,True) for i in range(len(preview))),
            move_period_seconds=seq.period(seq.labels.index('direct_return_final_taught_1'),True),
            requested_speed_multiplier=seq.metadata()['requested_speed_multiplier'],actual_speedup_verified=False,
            devices_opened=False,elapsed_time_limit_enabled=False,automatic_stage_transitions_enabled=True,
            operator_gated_observation_enabled=False,physical_direct_return_verified=False)));return 0
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    report=dict(status='started',holding=False,settings_restored=False,all_torque_off_verified=False,
        workflow='vertical_turn_direct_thumb_return_G',source_candidate_sha256=seq.manifest['candidate_sha256'],
        physical_task_success_verified=False,physical_direct_return_verified=False)
    terminal=Terminal(seq,report);bus=live=None
    folder=pose.ROOT/('vertical_direct_grip_trial_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    try:
        with SingleInstance(pose.ROOT/'.final_grip_pose.lock'):
            print('확정 G -> 세로 회전 -> 낮은 검지·중지 받침 -> 엄지 밀기 없이 후퇴 -> 1번/2번 -> 확정 G 유지.',flush=True)
            print('큐브는 확정 파지 자리에서 준비합니다. 시작과 회전 후 파지 모두 엄지를 먼저 내리고 끝마디/받침을 움직입니다.',flush=True)
            print('마지막 파지는 기록한 검지·중지 1번 -> 2번 -> 기존 G 순서입니다. 이때 엄지·약지는 그대로 유지합니다.',flush=True)
            print('손을 뗀 뒤 SPACE 한 번으로 시작합니다. 중간 확인 입력 없이 회전과 파지 복귀가 연속 진행됩니다.',flush=True)
            print('이동은 기존 75ms에서 37.5ms 간격의 2배 속도를 목표로 합니다. 모터 상태 읽기·추종 대기로 실제 속도는 느려질 수 있습니다.',flush=True)
            print('통신 부하를 줄이도록 전체 상태 읽기 사이에 최소 10ms 여유를 둡니다.',flush=True)
            print('전체 시간 제한 없음. 움직이는 중 SPACE/Q/Esc: 중단/OFF. 마지막 유지에서는 큐브를 받친 뒤 SPACE로 종료/OFF.',flush=True)
            terminal.wait();folder.mkdir();live=LiveState(folder,report);bus=DirectPoseBus(report,terminal.check)
            initial,start,_=pose.preflight(bus,report,seq.start)
            pose.run_pose(bus,initial,start,seq.rows(start),report,terminal.check,sequence=seq,limit=RUN_LIMIT_SECONDS)
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
