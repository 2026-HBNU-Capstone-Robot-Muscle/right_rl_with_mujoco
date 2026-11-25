"""No-motion serial and Windows wait timing measurements; PING/READ only."""
import argparse,datetime as dt,json,time,sys,statistics
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parent
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--read',action='store_true');args=parser.parse_args()
    if not args.read:print('No devices opened. --read allows PING/READ only.');return
    import numpy as np,dynamixel_sdk as sdk
    from serial.tools.list_ports import comports
    from calibrated_precise_clock_20261028 import install
    from calibrated_precise_window_20261028 import block_once
    from calibrated_precise_receive_20261028 import quiet
    folder=ROOT/('readonly_quiet_timing_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(status='started',python=sys.version,servo_writes=0,torque_changes=0,
        instruction_counts={},snapshots=[],old_quiet_probes=[],sleep_probes=[])
    port=None
    try:
        devices=[p for p in comports() if p.vid==1027 and p.pid==24596 and p.serial_number=='FTBENPSAA']
        if len(devices)!=1:raise RuntimeError('Known USB adapter missing or ambiguous')
        port=sdk.PortHandler(devices[0].device);packet=sdk.PacketHandler(2.)
        if not port.openPort() or not port.setBaudRate(1000000):raise RuntimeError('Cannot open verified1Mbps')
        original=port.writePort
        def readonly(data):
            if len(data)<8 or data[7] not in (1,2,0x82):raise RuntimeError('Motor write instruction blocked')
            k=str(data[7]);report['instruction_counts'][k]=report['instruction_counts'].get(k,0)+1
            return original(data)
        port.writePort=readonly
        bus=SimpleNamespace(sdk=sdk,port=port,packet=packet,report=report);install(bus)
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} ping failed')
            for address,expected in ((8,3),(11,5),(64,0),(70,0),(98,0)):
                value,comm,error=packet.read1ByteTxRx(port,motor,address)
                if comm or error or value!=expected:raise RuntimeError(f'ID{motor} unexpected register{address}')
        # Measure the requested sleep used by the old quiet loop on this host.
        for requested in (.001,0.):
            spans=[]
            for _ in range(30):
                began=time.perf_counter();time.sleep(requested);spans.append(1000*(time.perf_counter()-began))
            report['sleep_probes'].append(dict(requested_ms=requested*1000,minimum_ms=min(spans),median_ms=statistics.median(spans),maximum_ms=max(spans)))
        # Old function, including its original deadline ordering and sleep1ms.
        for i in range(30):
            trace=[];bus._horizontal_sample_deadline=time.perf_counter()+.2
            def measured_sleep(seconds):
                began=time.perf_counter();time.sleep(seconds);trace.append(dict(requested_ms=seconds*1000,actual_ms=1000*(time.perf_counter()-began)))
            before=time.perf_counter()
            try:row=dict(index=i,passed=True,result=quiet(bus,sleep=measured_sleep))
            except RuntimeError as exc:row=dict(index=i,passed=False,error=str(exc))
            row.update(elapsed_ms=1000*(time.perf_counter()-before),sleep_trace=trace);report['old_quiet_probes'].append(row)
        fields=[('torque',64,1),('error',70,1),('watchdog',98,1),('goal',116,4),('current',126,2),('position',132,4),('voltage',144,2),('temperature',146,1)]
        for i in range(160):
            began=time.perf_counter();bus._horizontal_sample_deadline=began+.2
            state,comm,error,detail=block_once(bus,64,83,fields)
            if comm or error:raise RuntimeError(f'Raw read failed index{i}: '+str(dict(comm=comm,error=error,detail=detail)))
            if any(state[k].any() for k in ('torque','error','watchdog')):raise RuntimeError('Expected healthy torque OFF')
            if min(state['voltage'])<38 or max(state['voltage'])>65 or max(state['temperature'])>45:raise RuntimeError('OFF voltage/temperature abnormal')
            report['snapshots'].append(dict(index=i,read_ms=1000*(time.perf_counter()-began),state={k:v.tolist() for k,v in state.items()}))
            elapsed=time.perf_counter()-began
            if elapsed<.05:time.sleep(.05-elapsed)
        report['status']='readonly_timing_saved'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if port is not None:port.closePort()
        (folder/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=folder.name,status=report['status'],error=report.get('error'),python=report['python'],
        sleep_probes=report['sleep_probes'],old_quiet_failures=sum(not r['passed'] for r in report['old_quiet_probes']),
        snapshots=len(report['snapshots']),instruction_counts=report['instruction_counts'],servo_writes=0,torque_changes=0)))
if __name__=='__main__':main()
