"""User-started joined rotation and return; no elapsed-time cutoff."""
import argparse,datetime as dt,json
import leap_measured_grip_hold_20261117 as pose
from vertical_rotation_return_plan_20261118 import VerticalRotationReturn,Terminal,RUN_LIMIT_SECONDS

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--execute',action='store_true');args=ap.parse_args()
    try:
        pose.validate_sources();sequence=VerticalRotationReturn();preview=sequence.rows(sequence.start)
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print('NOT READY:',exc,'. No devices opened.');return 2
    if not args.execute:
        print(json.dumps(dict(status='device_free_check_passed',goals=len(preview),
            nominal_seconds=len(preview)*.075,maximum_ON_seconds=None,elapsed_time_limit_enabled=False,
            join_endpoint_mismatch_ticks=sequence.manifest['join_endpoint_mismatch_ticks'],
            join_step_ticks=sequence.manifest['join_step_ticks'],
            devices_opened=False,physical_combined_execution_verified=False)));return 0
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    report=dict(status='started',holding=False,settings_restored=False,all_torque_off_verified=False,
        workflow='vertical_rotation_checked_return_same_G',source_candidate_sha256=sequence.manifest['candidate_sha256'],
        physical_task_success_verified=False,physical_cube_angle_measured=False,
        physical_combined_execution_verified=False,final_fixed_grip_return_included=True)
    terminal=Terminal(sequence,report);bus=live=None
    folder=pose.ROOT/('vertical_rotate_grip_trial_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    try:
        with SingleInstance(pose.ROOT/'.final_grip_pose.lock'):
            print('확정 파지 G -> 세로 회전 -> 확인 SPACE -> 같은 확정 파지 G 복귀/유지.',flush=True)
            print('큐브를 확정 파지 자리에서 준비합니다. 종료 후 처진 관절은 기존 속도로 G까지 먼저 복귀합니다. PLACE 단계는 없습니다.',flush=True)
            print('손을 뗀 뒤 SPACE로 시작. 파지 확인/회전 끝 확인에서 SPACE. 움직이는 중 SPACE/Q/Esc: 중단/OFF.',flush=True)
            print('중간 OFF/큐브 재배치 없이 이어집니다. 전체 실행/확인 대기/최종 유지의 시간 제한은 없습니다.',flush=True)
            print('전류·전압·온도·추종·통신 이상 중단과 Q/Esc 종료는 그대로 유지합니다.',flush=True)
            terminal.wait();folder.mkdir();live=LiveState(folder,report);bus=pose.PoseBus(report,terminal.check)
            initial,start,_=pose.preflight(bus,report,sequence.start)
            rows=sequence.rows(start)
            print(f"G 준비 {sequence.timing['G_recovery_seconds']:.1f}초 / 전체 명목 {sequence.timing['nominal_seconds']:.1f}초. 시간에 따른 자동 종료 없음.",flush=True)
            pose.run_pose(bus,initial,start,rows,report,terminal.check,sequence=sequence,limit=RUN_LIMIT_SECONDS)
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
