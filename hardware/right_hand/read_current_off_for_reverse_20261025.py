"""Fresh existing OFF state. PING and READ only, no camera or servo writes."""
import argparse,contextlib,datetime,io,json,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    if not args.execute:print('CHECK ONLY: no device opened. --execute permits PING/READ only.');return 0
    import dynamixel_sdk as sdk
    from hand_readonly_check import require_serial_port,inspect_bus
    folder=ROOT/('reverse_start_readonly_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(status='starting',servo_write_commands_sent=0,torque_changes_requested=0,snapshots=[])
    log=io.StringIO()
    try:
        require_serial_port('COM7',report)
        for i in range(3):
            row={'motors':[],'timestamp':datetime.datetime.now().astimezone().isoformat()}
            report['snapshots'].append(row)
            with contextlib.redirect_stdout(log):inspect_bus(sdk,'COM7',row)
            time.sleep(.15)
        positions=np.array([[m['position_ticks_signed'] for m in r['motors']] for r in report['snapshots']])
        report['maximum_drift_ticks']=int(np.ptp(positions,axis=0).max())
        if report['maximum_drift_ticks']>2:raise RuntimeError('Fresh OFF readings are not stable')
        motors=report['snapshots'][-1]['motors'];fields=dict(position='position_ticks_signed',torque='torque_enabled',
            error='hardware_error',watchdog='bus_watchdog_raw',voltage='voltage_raw',temperature='temperature_c')
        report['off_state']={k:[m[v] for m in motors] for k,v in fields.items()}
        report['status']='stable_off_readings_saved'
    except Exception as exc:report.update(status='stopped',error=str(exc))
    (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (folder/'read_log.txt').write_text(log.getvalue(),encoding='utf-8')
    print(json.dumps(dict(folder=str(folder),status=report['status'],error=report.get('error'),
        maximum_drift_ticks=report.get('maximum_drift_ticks'),off_state=report.get('off_state'),
        servo_write_commands_sent=0,torque_changes_requested=0),ensure_ascii=False),flush=True)
    return 0 if report['status']=='stable_off_readings_saved' else 1

if __name__=='__main__':raise SystemExit(main())
