"""Operator-started 15s current-pose goal transmission test. No G/turn path.

Default mode opens no devices. The original runner supplies configuration,
OFF preload/enable checks, full health checks and OFF/restoration. Initial
accepted-pose correction is bounded by its existing two-tick goal guard.
"""
import argparse,datetime as dt,json,time
import numpy as np
import leap_measured_grip_hold_20261117 as pose
from measured_grip_small_cycle_plan_20261117 import validate_rows
from vertical_direct_grip_plan_20261118 import VerticalDirectGrip

DURATION=15.
PERIOD=.075

class CurrentGoal:
    def __init__(self,target):self.target=pose.encoder_vector(target).copy()
    def rows(self,current):
        # Stay moving in the runner's scheduling sense so it writes the SAME
        # goal every cycle, rather than switching to read-only final hold.
        rows=np.tile(self.target,(400,1));validate_rows(current,rows);return rows
    def phase(self,index,moving):return 'stationary_repeated_current_goal'
    def period(self,index,moving):return PERIOD
    def message(self,phase):
        return '현재 자세 목표만75ms마다 반복 전송합니다.15초 뒤 OFF를 시도합니다. SPACE/Q/Esc: 중단/OFF.'
    def metadata(self):
        return dict(workflow='stationary_repeated_current_goal_15s',
            diagnostic_target_ticks=self.target.tolist(),maximum_ON_seconds=DURATION,
            period_seconds=PERIOD,maximum_step_ticks=2,rotation_included=False,
            fixed_G_transition_included=False,automatic_OFF_attempt_after_test=True)

def run(bus,initial,start,report,cancel,*,now=time.monotonic,sleep=time.sleep):
    if np.any(initial['torque']):
        raise RuntimeError('Stationary test requires initial torque OFF; no new writes')
    sequence=CurrentGoal(start)
    pose.run_pose(bus,initial,start,sequence.rows(start),report,cancel,
        sequence=sequence,limit=DURATION,now=now,sleep=sleep)
    # The common runner's time-limit label says 500s even for a supplied limit.
    if report.get('status')=='existing500second_limit_reached':
        report['status']=('stationary_15second_test_complete'
            if report.get('all_torque_off_verified') and report.get('settings_restored')
            else 'stationary_cleanup_failed')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    try:
        pose.validate_sources();baseline=VerticalDirectGrip()
    except (OSError,ValueError,RuntimeError,KeyError) as exc:
        print('NOT READY:',exc,'. No devices opened.');return 2
    if not args.execute:
        print(json.dumps(dict(status='device_free_check_passed',devices_opened=False,
            maximum_ON_seconds=DURATION,period_seconds=PERIOD,
            rotation_included=False,fixed_G_transition_included=False,
            baseline_source_files=len(baseline.manifest['source_hashes']))));return 0
    from final_grip_single_instance_20261116 import SingleInstance
    from measured_grip_live_state_20261117 import LiveState
    report=dict(status='started',holding=False,settings_restored=False,
        all_torque_off_verified=False,workflow='stationary_repeated_current_goal_15s',
        baseline_candidate_sha256=baseline.manifest['candidate_sha256'])
    folder=pose.ROOT/('stationary_comm_trial_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    bus=live=None;terminal=pose.Terminal()
    try:
        with SingleInstance(pose.ROOT/'.final_grip_pose.lock'):
            print('현재 자세를15초 동안 힘을 줘 유지하며 같은 목표를 반복 전송한 뒤 OFF를 시도합니다.',flush=True)
            print('새 파지 G나 회전 자세로 이동하지 않습니다. 초기 유지 보정도 기존 최대2틱 이내입니다.',flush=True)
            print('큐브는 빼두고 손을 뗀 뒤 SPACE 한 번으로 시작. SPACE/Q/Esc: 중단/OFF.',flush=True)
            terminal.wait();folder.mkdir();live=LiveState(folder,report)
            bus=pose.PoseBus(report,terminal.check)
            initial,start,_=pose.preflight(bus,report,pose.reference()[2])
            run(bus,initial,start,report,terminal.check)
    except (pose.OperatorStop,KeyboardInterrupt):report['status']='operator_stopped'
    except BaseException as exc:
        report.update(status='stopped',error=str(exc));print('STOP:',exc,flush=True)
    finally:
        if bus is not None:bus.close()
        if live is not None:report['live_state_writer']=live.close()
        report['serial_port_closed']=True
        if folder.exists():
            (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            print('Saved:',folder,flush=True)
    return 0 if report.get('status') in ('stationary_15second_test_complete','operator_stopped') and not report.get('cleanup_error') else 1

if __name__=='__main__':raise SystemExit(main())
