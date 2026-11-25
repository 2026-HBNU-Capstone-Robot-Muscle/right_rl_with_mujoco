"""Fresh stable OFF reference with verified Fast READ; no motor writes."""
import argparse,datetime as dt,json,time
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parent
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--read',action='store_true');args=parser.parse_args()
    if not args.read:print('No devices opened. --read permits documented PING/READ/Fast Sync Read.');return
    import numpy as np,dynamixel_sdk as sdk
    from serial.tools.list_ports import comports
    from fast_health_read_20261029 import block_once,POLICY
    from combined_quiet_reserve_20261029 import receive
    from calibrated_precise_clock_20261028 import install
    from observed_book_model_policy import healthy_off
    folder=ROOT/('resume_off_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(status='started',servo_writes=0,torque_changes=0,baud=1000000,snapshots=[],individual_checks=[],instruction_counts={},fast_health_policy=POLICY)
    port=None
    try:
        devices=[p for p in comports() if p.vid==1027 and p.pid==24596 and p.serial_number=='FTBENPSAA']
        if len(devices)!=1:raise RuntimeError('Known USB adapter missing or ambiguous')
        report['device']=devices[0].device;port=sdk.PortHandler(devices[0].device);packet=sdk.PacketHandler(2.)
        if not port.openPort() or not port.setBaudRate(1000000):raise RuntimeError('Cannot open verified1Mbps')
        original=port.writePort
        def readonly(data):
            if len(data)<8 or data[7] not in (1,2,0x8a):raise RuntimeError('Motor write instruction blocked')
            if data[7]==0x8a and (data[8:12]!=[64,0,83,0] or list(data[12:-2]) not in [list(range(1,9)),list(range(9,17))]):raise RuntimeError('Fast READ outside validated scope')
            key=str(data[7]);report['instruction_counts'][key]=report['instruction_counts'].get(key,0)+1;return original(data)
        port.writePort=readonly
        bus=SimpleNamespace(sdk=sdk,packet=packet,port=port,report=report,_read_recovery_times=[]);install(bus)
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} PING failed')
            fw,comm,error=packet.read1ByteTxRx(port,motor,6)
            if comm or error or fw<46:raise RuntimeError('Fast READ firmware unsupported')
            checks={6:int(fw)}
            for address,expected in ((8,3),(11,5),(64,0),(70,0),(98,0)):
                value,comm,error=packet.read1ByteTxRx(port,motor,address)
                if comm or error or value!=expected:raise RuntimeError(f'ID{motor} unexpected register{address}')
                checks[address]=int(value)
            report['individual_checks'].append(dict(motor_id=motor,model=model,registers=checks))
        fields=[('torque',64,1),('error',70,1),('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),('watchdog',98,1),('current_cap',102,2),('acceleration',108,4),('velocity',112,4),('goal',116,4),('current',126,2),('position',132,4),('voltage',144,2),('temperature',146,1)]
        for index in range(3):
            bus._horizontal_sample_deadline=time.perf_counter()+.2
            state=receive(bus,lambda:block_once(bus,64,83,fields),'Fast OFF reference',dict(kind='sync_read',address=64,length=83))
            report['snapshots'].append({k:v.tolist() for k,v in state.items()})
            try:healthy_off(state)
            except Exception as error:
                report.setdefault('pose_health_validation_errors',[]).append(str(error))
            time.sleep(.1)
        positions=np.asarray([s['position'] for s in report['snapshots']]);report['maximum_drift_ticks']=int(np.max(np.ptp(positions,axis=0)))
        if report['maximum_drift_ticks']>2:raise RuntimeError('OFF position not stationary across three snapshots')
        report['status']='readonly_data_saved';report['motion_admission_granted']=False;report['all_torque_off_read']=all(all(x==0 for x in row['torque']) for row in report['snapshots'])
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if port is not None:port.closePort()
        (folder/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=folder.name,status=report['status'],error=report.get('error'),snapshots=len(report['snapshots']),maximum_drift_ticks=report.get('maximum_drift_ticks'),instructions=report['instruction_counts'],servo_writes=0,torque_changes=0)))
if __name__=='__main__':main()
