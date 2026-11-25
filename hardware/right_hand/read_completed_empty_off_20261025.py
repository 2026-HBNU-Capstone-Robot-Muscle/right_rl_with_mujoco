"""Authorized read-only OFF acquisition after the user reports completion."""
import argparse,datetime,json
from pathlib import Path
import numpy as np
import sequence_flip_checked as checked
from observed_book_model_policy import admission,healthy_off
from thumb_live_start_gate import stable_off
from hand_readonly_check import require_serial_port
from hand_motor_mapping import SIM_TO_MOTOR_IDS
ROOT=Path(__file__).resolve().parent

class ReadOnlyBus(checked.Bus):
    planner=staticmethod(admission)
    def write(self,*a,**kw):raise RuntimeError('Read-only bus prohibits all servo writes')
    def off(self,*a,**kw):raise RuntimeError('Read-only bus prohibits torque writes')

def main():
    p=argparse.ArgumentParser();p.add_argument('--read',action='store_true');args=p.parse_args()
    if not args.read:print('CHECK ONLY: no motor port opened');return 0
    import dynamixel_sdk as sdk
    folder=ROOT/('completed_empty_readonly_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(timestamp=datetime.datetime.now().astimezone().isoformat(),servo_write_commands_sent=0,torque_changes_requested=0,status='not_started')
    bus=None;opened=False;code=1
    try:
        require_serial_port('COM7',report);bus=ReadOnlyBus(sdk,report)
        if not bus.port.openPort():raise RuntimeError('Cannot open COM7 for read-only check')
        opened=True
        if not bus.port.setBaudRate(3000000):raise RuntimeError('Cannot set host baud')
        bus.preflight();state=stable_off(bus,report,cancel=lambda:None,validator=healthy_off)
        request=dict(plan_version='completed_empty_readonly_v1_20261025',motor_ids_in_sim_order=list(SIM_TO_MOTOR_IDS),
            origin='assistant_read_only_after_user_reported_completed_empty_run',off_state={k:v.tolist() for k,v in state.items()})
        (folder/'request.json').write_text(json.dumps(request,indent=2),encoding='utf-8')
        report.update(status='stable_off_read_complete',request_file=str(folder/'request.json'),raw_position_ticks=state['position'].tolist());code=0
    except Exception as exc:report.update(status='stopped',error=str(exc))
    finally:
        if bus is not None and opened:bus.port.closePort()
        (folder/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('off_start_observations','preflight_state')},ensure_ascii=False));print('SAVED',folder)
    return code

if __name__=='__main__':raise SystemExit(main())
