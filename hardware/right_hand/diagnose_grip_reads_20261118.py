"""Record failed raw status packets using the current1Mbps read-only connection."""
from pathlib import Path
import argparse,datetime as dt,json,time
import numpy as np
from read_receiving_waypoint_20261118 import ReadOnlyBus
from final_grip_single_instance_20261116 import SingleInstance

ROOT=Path(__file__).resolve().parent

def install_trace(bus):
    original=bus.packet.rxPacket
    def receive(port,fast):
        began=time.perf_counter();raw,comm=original(port,fast)
        if comm!=bus.sdk.COMM_SUCCESS:
            events=bus.report.setdefault('failed_raw_packets',[])
            if len(events)<64:
                data=bytes(raw);declared=7+int.from_bytes(data[5:7],'little') if len(data)>=7 and data[:4]==b'\xff\xff\xfd\x00' else None
                complete=declared is not None and len(data)==declared
                events.append(dict(comm=int(comm),received_bytes=len(data),declared_bytes=declared,
                    complete=complete,raw_hex=data.hex(),elapsed_ms=1000*(time.perf_counter()-began),
                    crc_matches=bool(bus.packet.updateCRC(0,list(data[:-2]),len(data)-2)==int.from_bytes(data[-2:],'little')) if complete else None))
        return raw,comm
    bus.packet.rxPacket=receive

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--record',action='store_true')
    ap.add_argument('--samples',type=int,default=160);args=ap.parse_args()
    if not args.record:print('CHECK ONLY. No devices opened.');return 0
    if not 1<=args.samples<=400:raise ValueError('Expected1..400 snapshots')
    folder=ROOT/('grip_read_diagnosis_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=9))).strftime('%Y%m%d_%H%M%S_%f'))
    report=dict(status='started',motor_writes=0,torque_changes=0,samples=0,latencies_ms=[])
    bus=None;folder.mkdir()
    try:
        with SingleInstance(ROOT/'.final_grip_pose.lock'):
            bus=ReadOnlyBus(report,lambda:None)
            if not hasattr(bus.packet.rxPacket,'__name__') or bus.packet.rxPacket.__name__!='receive':install_trace(bus)
            for motor in range(1,17):
                model,comm,error=bus.packet.ping(bus.port,motor)
                if comm or error or model!=1240:raise RuntimeError('PING/model mismatch ID'+str(motor))
            for _ in range(args.samples):
                began=time.perf_counter();state=bus.snapshot()
                if np.any(state['torque']) or np.any(state['error']):
                    raise RuntimeError('Diagnostic requires already OFF/error-free state; no changes sent')
                report['samples']+=1;report['latencies_ms'].append(1000*(time.perf_counter()-began))
                time.sleep(max(0.,.075-(time.perf_counter()-began)))
            report['status']='readonly_diagnosis_complete'
    except Exception as exc:report.update(status='stopped',error=str(exc))
    finally:
        if bus is not None:bus.close()
        report['serial_port_closed']=True
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=folder.name,**{k:v for k,v in report.items() if k in ('status','error','samples','failed_raw_packets','read_failures','motor_writes','torque_changes')}),ensure_ascii=False))
    return 0 if report['status']=='readonly_diagnosis_complete' else 1

if __name__=='__main__':raise SystemExit(main())
