"""PING and sync READ only: stable OFF reference for repeat preparation."""
import datetime as dt,json,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
REGS=dict(torque=(64,1),error=(70,1),d_gain=(80,2),i_gain=(82,2),p_gain=(84,2),watchdog=(98,1),
    current_cap=(102,2),acceleration=(108,4),velocity=(112,4),goal=(116,4),current=(126,2),position=(132,4),voltage=(144,2),temperature=(146,1))
def main():
    if '--read' not in sys.argv:print('CHECK ONLY: no port opened. --read uses PING/READ only.');return
    import dynamixel_sdk as sdk
    folder=ROOT/('repeat_off_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(status='started',device='COM7',servo_writes=0,torque_changes=0,snapshots=[],precise_read_times_ms=[])
    port=sdk.PortHandler('COM7');packet=sdk.PacketHandler(2.0);opened=False
    try:
        if not port.openPort():raise RuntimeError('Cannot open COM7')
        opened=True
        if not port.setBaudRate(3000000):raise RuntimeError('Cannot set host baud')
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm!=sdk.COMM_SUCCESS or error or model!=1240:raise RuntimeError(f'ID{motor} PING/model failed')
        group=sdk.GroupSyncRead(port,packet,64,83)
        for motor in range(1,17):
            if not group.addParam(motor):raise RuntimeError('Could not add sync READ ID')
        for _ in range(3):
            began=time.perf_counter();comm=group.txRxPacket();report['precise_read_times_ms'].append(1000*(time.perf_counter()-began))
            if comm!=sdk.COMM_SUCCESS or not all(group.isAvailable(motor,64,83) for motor in range(1,17)):raise RuntimeError('Incomplete READ')
            state={key:[int(group.getData(motor,address,size)) for motor in range(1,17)] for key,(address,size) in REGS.items()}
            report['snapshots'].append(state)
            if any(state['torque']) or any(state['error']) or any(state['watchdog']):raise RuntimeError('OFF/error/watchdog condition failed; no writes')
            time.sleep(.1)
        q=np.asarray([s['position'] for s in report['snapshots']]);report['maximum_drift_ticks']=int(np.ptp(q,axis=0).max())
        if report['maximum_drift_ticks']>2:raise RuntimeError('OFF readings not stable')
        report['status']='off_snapshots_saved'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if opened:port.closePort()
        (folder/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=folder.name,status=report['status'],error=report.get('error'),maximum_drift_ticks=report.get('maximum_drift_ticks'),
        position=report['snapshots'][-1]['position'] if report['snapshots'] else None,read_ms=report['precise_read_times_ms'],servo_writes=0)),flush=True)
if __name__=='__main__':main()
