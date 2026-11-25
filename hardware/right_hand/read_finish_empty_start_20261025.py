"""Read three OFF snapshots only. No goal/configuration/torque writes."""
import datetime,json,time
from pathlib import Path
import numpy as np
import sequence_flip_checked as checked
from hand_readonly_check import require_serial_port
ROOT=Path(__file__).resolve().parent
class ReadOnlyBus(checked.Bus):
    def write(self,*args,**kwargs):raise RuntimeError('All motor writes forbidden in read-only acquisition')
def main():
    stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f');folder=ROOT/f'finish_empty_readonly_{stamp}';folder.mkdir();r=dict(timestamp=datetime.datetime.now().astimezone().isoformat(),status='not_started',servo_write_commands_sent=0,torque_changes_requested=0,snapshots=[]);bus=None;opened=False
    try:
        require_serial_port('COM7',r)
        import dynamixel_sdk as sdk
        bus=ReadOnlyBus(sdk,r)
        if not bus.port.openPort():raise RuntimeError('Cannot open COM7')
        opened=True
        if not bus.port.setBaudRate(3000000):raise RuntimeError('Cannot set host baud')
        for _ in range(3):
            s=bus.snapshot()
            if any(np.any(s[k]) for k in ('torque','error','watchdog')):raise RuntimeError('Expected torque OFF and no motor error/watchdog')
            if np.any(s['voltage']<38) or np.any(s['voltage']>65) or np.any(s['temperature']>45):raise RuntimeError('OFF voltage/temperature check failed')
            r['snapshots'].append({k:v.tolist() for k,v in s.items()});time.sleep(.2)
        positions=np.array([x['position'] for x in r['snapshots']]);r.update(maximum_drift_ticks=int(np.ptp(positions,axis=0).max()),position_ticks=positions[-1].tolist(),status='off_snapshots_saved')
        if r['maximum_drift_ticks']>2:raise RuntimeError('OFF readings still moving; not a stable start')
        print(json.dumps({k:r[k] for k in ('status','position_ticks','maximum_drift_ticks','servo_write_commands_sent')}),flush=True)
    except Exception as exc:r.update(status='stopped',error=str(exc));print('READ ONLY STOP:',str(exc),flush=True)
    finally:
        if bus is not None and opened:bus.port.closePort()
        (folder/'result.json').write_text(json.dumps(r,indent=2),encoding='utf-8');print('SAVED',folder,flush=True)
    return 0 if r['status']=='off_snapshots_saved' else 1
if __name__=='__main__':raise SystemExit(main())
