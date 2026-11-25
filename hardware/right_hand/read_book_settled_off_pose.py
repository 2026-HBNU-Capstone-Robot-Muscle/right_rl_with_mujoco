"""Read three stable current OFF snapshots. All servo WRITE methods disabled."""
import argparse
import datetime as dt
import json
import time
import numpy as np
from pathlib import Path
from sequence_flip_checked import Bus as BaseBus
from book_grasp_trial_plan import BookGraspPlan,ROOT
from thumb_live_start_gate import stable_off
from hand_motor_mapping import SIM_TO_MOTOR_IDS


def readonly_health(state):
    raw=np.asarray(state['position'])
    if raw.shape!=(16,) or not np.isfinite(raw).all() or np.any(raw!=np.rint(raw)) or np.any(raw<0) or np.any(raw>4095):
        raise ValueError('Invalid raw OFF reading')
    if any(np.any(state[k]) for k in ('torque','error','watchdog')):
        raise ValueError('Read-only acquisition requires torque/error/watchdog OFF')
    if np.any(np.asarray(state['voltage'])<38) or np.any(np.asarray(state['voltage'])>65) or np.any(np.asarray(state['temperature'])>45):
        raise ValueError('Unhealthy OFF readings')
    return raw


class ReadOnlyBus(BaseBus):
    # Reading an out-of-model pose is allowed. It never grants motion admission.
    planner=staticmethod(lambda raw:np.asarray(raw,dtype=int))
    def write(self,*args,**kwargs):raise AssertionError('READ ONLY: writes disabled')
    def off(self):raise AssertionError('READ ONLY: torque changes disabled')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    if not args.execute:print('CHECK ONLY: no motor port opened');return 0
    folder=ROOT/('book_settled_off_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(status='not_started',timestamp=dt.datetime.now().astimezone().isoformat(),
        servo_write_commands_sent=0,torque_changes_requested=0)
    bus=None;opened=False
    try:
        import dynamixel_sdk as sdk
        bus=ReadOnlyBus(sdk,report)
        opened=bool(bus.port.openPort())
        if not opened:raise RuntimeError('COM7 unavailable; no device changes')
        if not bus.port.setBaudRate(3000000):raise RuntimeError('Host baud unavailable')
        bus.preflight()
        state=stable_off(bus,report,cancel=lambda:None,validator=readonly_health)
        try:BookGraspPlan.validate_off_state(state)
        except ValueError as exc:report['motion_admission_error']=str(exc)
        request=dict(plan_version=BookGraspPlan.PLAN_VERSION,motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
            origin='assistant_readonly_stable_off_snapshots_for_offline_review_only',
            hardware_execution_allowed=False,
            off_state={k:v.tolist() for k,v in state.items()})
        (folder/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
        report.update(status='stable_current_off_read',position=state['position'].tolist(),minimum_voltage_v=min(state['voltage'])/10)
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if opened:bus.port.closePort()
        (folder/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(dict(folder=str(folder),status=report['status'],error=report.get('error'),
            position=report.get('position'),servo_write_commands_sent=0)))
    return 0 if report['status']=='stable_current_off_read' else 1


if __name__=='__main__':raise SystemExit(main())
