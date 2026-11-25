"""No-motion ordinary/Fast READ comparison and host-only corrupt-frame checks."""
import argparse,datetime as dt,hashlib,json,statistics,time
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parent
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--read',action='store_true');a=parser.parse_args()
    if not a.read:print('No devices opened. --read permits documented PING/READ/Fast Sync Read only.');return
    import numpy as np,dynamixel_sdk as sdk
    from serial.tools.list_ports import comports
    from calibrated_precise_clock_20261028 import install
    from calibrated_precise_window_20261028 import block_once as ordinary
    from fast_health_read_20261029 import block_once as fast,POLICY
    from combined_quiet_reserve_20261029 import receive
    from observed_book_model_policy import healthy_off
    folder=ROOT/('fast_health_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(status='started',servo_writes=0,torque_changes=0,firmware=[],instruction_counts={},comparison=[],
        fast_snapshots=[],host_corruption_episodes=[],fast_policy=POLICY,
        transport_source_sha256=hashlib.sha256((ROOT/'fast_health_read_20261029.py').read_bytes()).hexdigest(),
        ordinary_source_sha256=hashlib.sha256((ROOT/'calibrated_precise_window_20261028.py').read_bytes()).hexdigest(),
        official_reference='https://emanual.robotis.com/docs/en/dxl/protocol2/#fast-sync-read-0x8a')
    port=None
    try:
        devices=[p for p in comports() if p.vid==1027 and p.pid==24596 and p.serial_number=='FTBENPSAA']
        if len(devices)!=1:raise RuntimeError('Known USB adapter missing or ambiguous')
        port=sdk.PortHandler(devices[0].device);packet=sdk.PacketHandler(2.)
        if not port.openPort() or not port.setBaudRate(1000000):raise RuntimeError('Cannot open verified1Mbps')
        original_write=port.writePort
        def readonly(data):
            if len(data)<8 or data[7] not in (1,2,0x82,0x8a):raise RuntimeError('Motor write instruction blocked')
            if data[7]==0x8a:
                if data[8:12]!=[64,0,83,0] or list(data[12:-2]) not in [list(range(1,9)),list(range(9,17))]:raise RuntimeError('Fast READ outside approved health span/IDs')
            key=str(data[7]);report['instruction_counts'][key]=report['instruction_counts'].get(key,0)+1;return original_write(data)
        port.writePort=readonly
        bus=SimpleNamespace(sdk=sdk,packet=packet,port=port,report=report,_read_recovery_times=[]);install(bus)
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} PING failed')
            fw,comm,error=packet.read1ByteTxRx(port,motor,6)
            if comm or error or fw<46:raise RuntimeError('Live firmware does not support documented Fast READ')
            for addr,expected in ((8,3),(11,5),(64,0),(70,0),(98,0)):
                value,comm,error=packet.read1ByteTxRx(port,motor,addr)
                if comm or error or value!=expected:raise RuntimeError(f'ID{motor} unexpected register{addr}')
            report['firmware'].append(dict(id=motor,model=model,firmware=fw))
        fields=[('torque',64,1),('error',70,1),('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),('watchdog',98,1),('current_cap',102,2),('acceleration',108,4),('velocity',112,4),('goal',116,4),('current',126,2),('position',132,4),('voltage',144,2),('temperature',146,1)]
        def direct(reader):
            began=time.perf_counter();bus._horizontal_sample_deadline=began+.2
            state,comm,error,detail=reader(bus,64,83,fields)
            if comm or error:raise RuntimeError('Read failed: '+str(dict(comm=comm,error=error,detail=detail)))
            healthy_off(state);return state,1000*(time.perf_counter()-began)
        for i in range(20):
            old,old_ms=direct(ordinary);new,new_ms=direct(fast);differences={}
            for name,_,_ in fields:
                maximum=int(np.max(abs(new[name]-old[name])));differences[name]=maximum
                allowed=3 if name in ('position','goal','current') else 1 if name in ('voltage','temperature') else 0
                if maximum>allowed:raise RuntimeError('Ordinary/Fast value mismatch: '+name)
            report['comparison'].append(dict(index=i,ordinary_ms=old_ms,fast_ms=new_ms,field_differences=differences))
        for i in range(160):
            state,elapsed=direct(fast);report['fast_snapshots'].append(dict(index=i,elapsed_ms=elapsed,state={k:v.tolist() for k,v in state.items()}))
        # Discard/flip received bytes only in HOST memory, never transmit bad
        # instructions or change a register. The second group fails after the
        # first half was received; the retry must re-read all16 devices.
        native_rx=packet.rxPacket
        for episode_index in range(3):
            calls=0;bus._read_recovery_times=[];began=time.perf_counter();bus._horizontal_sample_deadline=began+.2
            def injected_rx(*args,**kwargs):
                nonlocal calls
                raw,comm=native_rx(*args,**kwargs);calls+=1
                if calls==2 and not comm:
                    raw=list(raw);raw[-1]^=1
                return raw,comm
            packet.rxPacket=injected_rx
            try:
                start=len(report.get('read_failures',[]))
                state=receive(bus,lambda:fast(bus,64,83,fields),'Fast READ only host fault',dict(kind='sync_read',address=64,length=83));healthy_off(state)
                fresh=receive(bus,lambda:fast(bus,64,83,fields),'Fast READ only confirmation',dict(kind='sync_read',address=64,length=83));healthy_off(fresh)
                elapsed=1000*(time.perf_counter()-began)
                if elapsed>=200 or calls!=6:raise RuntimeError('Corrupt half snapshot did not recover as two complete fresh16-device reads')
                failures=report['read_failures'][start:]
                if len(failures)!=1 or failures[0]['completed_ids']!=list(range(1,9)) or not failures[0]['recovered']:raise RuntimeError('Partial response handling mismatch')
                report['host_corruption_episodes'].append(dict(index=episode_index,host_only_corruption=True,elapsed_ms=elapsed,
                    partial_snapshot_discarded=True,two_complete_healthy_OFF_reads=True,reply_groups_received=calls))
            finally:packet.rxPacket=native_rx
        old=[r['ordinary_ms'] for r in report['comparison']];new=[r['fast_ms'] for r in report['comparison']]
        report['timing']=dict(ordinary_median_ms=statistics.median(old),fast_median_ms=statistics.median(new),
            fast_maximum_ms=max(r['elapsed_ms'] for r in report['fast_snapshots']),median_ratio=statistics.median(new)/statistics.median(old))
        report['status']='fast_health_readonly_passed'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if port is not None:port.closePort()
        (folder/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=folder.name,status=report['status'],error=report.get('error'),timing=report.get('timing'),
        comparison_pairs=len(report['comparison']),fast_snapshots=len(report['fast_snapshots']),
        host_corruption_episodes=report['host_corruption_episodes'],instructions=report['instruction_counts'],servo_writes=0,torque_changes=0)))
if __name__=='__main__':main()
