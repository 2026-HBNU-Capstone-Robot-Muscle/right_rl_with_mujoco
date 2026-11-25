"""Three current OFF snapshots for a source-bound candidate; READ/PING only."""
import argparse,datetime as dt,json,time
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parent
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--read',action='store_true');args=parser.parse_args()
    if not args.read:print('No devices opened. --read permits only PING and READ.');return
    import numpy as np
    import dynamixel_sdk as sdk
    from serial.tools.list_ports import comports
    from horizontal_sync_receive_window_20261028 import block_once
    out=ROOT/('prefix638_off_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',servo_writes=0,torque_changes=0,baud=1000000,snapshots=[],individual_checks=[])
    port=None;opened=False
    try:
        matches=[p for p in comports() if p.vid==1027 and p.pid==24596 and p.serial_number=='FTBENPSAA']
        if len(matches)!=1:raise RuntimeError('Known USB adapter missing or ambiguous')
        report['device']=matches[0].device;port=sdk.PortHandler(matches[0].device);packet=sdk.PacketHandler(2.0)
        if not port.openPort() or not port.setBaudRate(1000000):raise RuntimeError('Cannot open existing1Mbps')
        opened=True;original=port.writePort
        def readonly(data):
            if len(data)<8 or data[7] not in (1,2,0x82):raise RuntimeError('Motor write instruction blocked')
            return original(data)
        port.writePort=readonly
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} PING failed')
            checks={}
            for address,expected in ((8,3),(11,5),(64,0),(70,0),(98,0)):
                value,comm,error=packet.read1ByteTxRx(port,motor,address)
                if comm or error or value!=expected:raise RuntimeError(f'ID{motor} unexpected READ{address}')
                checks[str(address)]=int(value)
            report['individual_checks'].append(dict(motor_id=motor,model=model,registers=checks))
        bus=SimpleNamespace(sdk=sdk,port=port,packet=packet,report=report)
        fields=[('torque',64,1),('error',70,1),('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),('watchdog',98,1),('current_cap',102,2),('acceleration',108,4),('velocity',112,4),('goal',116,4),('current',126,2),('position',132,4),('voltage',144,2),('temperature',146,1)]
        for index in range(3):
            state,comm,error,detail=block_once(bus,64,83,fields)
            if comm or error:raise RuntimeError('Incomplete snapshot: '+str(detail))
            if any(state[k].any() for k in ('torque','error','watchdog')):raise RuntimeError('Expected healthy OFF state')
            if min(state['voltage'])<38 or max(state['voltage'])>65 or max(state['temperature'])>45:raise RuntimeError('OFF voltage/temperature abnormal')
            report['snapshots'].append({k:v.tolist() for k,v in state.items()});time.sleep(.1)
        positions=np.asarray([s['position'] for s in report['snapshots']]);report['maximum_drift_ticks']=int(np.max(np.ptp(positions,axis=0)))
        if report['maximum_drift_ticks']>2:raise RuntimeError('OFF posture not stationary across snapshots')
        report['status']='off_snapshots_saved'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if opened:port.closePort()
        (out/'result.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
    print(json.dumps(dict(folder=out.name,status=report['status'],error=report.get('error'),snapshots=len(report['snapshots']),maximum_drift_ticks=report.get('maximum_drift_ticks'),servo_writes=0,torque_changes=0)))
if __name__=='__main__':main()
