"""Actual preview/JPEG load, original8 versus smaller4 replies. PING/READ only."""
import argparse,datetime as dt,hashlib,json,statistics,time
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parent
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--read',action='store_true');a=parser.parse_args()
    if not a.read:print('No devices opened. --read permits camera and PING/READ only.');return
    import numpy as np,dynamixel_sdk as sdk
    from serial.tools.list_ports import comports
    from fast_health_read_20261029 import block_once as fast8
    from fast_health_read4_20261029 import block_once as fast4,GROUPS,POLICY
    from calibrated_precise_clock_20261028 import install
    from observed_book_model_policy import healthy_off
    from empty_trial_camera import EmptyTrialCamera
    folder=ROOT/('fast4_compare_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(status='started',servo_writes=0,torque_changes=0,instruction_counts={},firmware=[],comparison=[],blocks=[],host_corruption_episodes=[],
        candidate_policy=POLICY,candidate_source_sha256=hashlib.sha256((ROOT/'fast_health_read4_20261029.py').read_bytes()).hexdigest(),
        original_source_sha256=hashlib.sha256((ROOT/'fast_health_read_20261029.py').read_bytes()).hexdigest(),recording_requested=True,
        phase='READ ONLY: torque OFF; no movement',camera_instruction='READ ONLY | NO MOTOR COMMANDS')
    port=camera=None
    try:
        matches=[p for p in comports() if p.vid==1027 and p.pid==24596 and p.serial_number=='FTBENPSAA']
        if len(matches)!=1:raise RuntimeError('Known USB adapter missing/ambiguous')
        port=sdk.PortHandler(matches[0].device);packet=sdk.PacketHandler(2.)
        if not port.openPort() or not port.setBaudRate(1000000):raise RuntimeError('Cannot open known1Mbps')
        original=port.writePort
        def readonly(data):
            if len(data)<8 or data[7] not in (1,2,0x8a):raise RuntimeError('Motor write instruction blocked')
            if data[7]==0x8a and (data[8:12]!=[64,0,83,0] or tuple(data[12:-2]) not in GROUPS+(tuple(range(1,9)),tuple(range(9,17)))):raise RuntimeError('Fast READ outside approved health span/IDs')
            key=str(data[7]);report['instruction_counts'][key]=report['instruction_counts'].get(key,0)+1;return original(data)
        port.writePort=readonly;bus=SimpleNamespace(sdk=sdk,packet=packet,port=port,report=report,_read_recovery_times=[]);install(bus)
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError('PING failed')
            fw,comm,error=packet.read1ByteTxRx(port,motor,6)
            if comm or error or fw<46:raise RuntimeError('Unsupported live firmware')
            report['firmware'].append(dict(id=motor,model=model,firmware=fw))
            for address,value in ((8,3),(11,5),(64,0),(70,0),(98,0)):
                actual,comm,error=packet.read1ByteTxRx(port,motor,address)
                if comm or error or actual!=value:raise RuntimeError('Unexpected mode/OFF state')
        fields=[('torque',64,1),('error',70,1),('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),('watchdog',98,1),('current_cap',102,2),('acceleration',108,4),('velocity',112,4),('goal',116,4),('current',126,2),('position',132,4),('voltage',144,2),('temperature',146,1)]
        def direct(reader):
            began=time.perf_counter();bus._horizontal_sample_deadline=began+.2
            state,comm,error,detail=reader(bus,64,83,fields)
            if comm or error:raise RuntimeError('READ failed '+str(dict(comm=comm,error=error,detail=detail)))
            healthy_off(state);return state,1000*(time.perf_counter()-began)
        for i in range(20):
            old,old_ms=direct(fast8);new,new_ms=direct(fast4);differences={}
            for name,_,_ in fields:
                diff=int(np.max(abs(old[name]-new[name])));differences[name]=diff
                allowed=4 if name in ('position','goal','current') else 1 if name in ('voltage','temperature') else 0
                if diff>allowed:raise RuntimeError('8/4 reply value mismatch: '+name)
            report['comparison'].append(dict(index=i,fast8_ms=old_ms,fast4_ms=new_ms,differences=differences))
        camera=EmptyTrialCamera(folder/'camera',report)
        # ABBA order reduces bias from warming the camera/driver. No health data
        # from a failed OFF-only diagnostic sample is reused by another sample.
        for block_index,size in enumerate((8,4,4,8)):
            reader=fast8 if size==8 else fast4;row=dict(block=block_index,group_size=size,requested=150,complete=0,failures=[],read_ms=[]);report['blocks'].append(row)
            report['phase']=f'READ ONLY group{size} block{block_index}'
            for i in range(150):
                began=time.perf_counter();bus._horizontal_sample_deadline=began+.2
                state,comm,error,detail=reader(bus,64,83,fields);elapsed=1000*(time.perf_counter()-began);row['read_ms'].append(elapsed)
                if comm or error:row['failures'].append(dict(index=i,comm=comm,error=error,elapsed_ms=elapsed,detail=detail))
                else:healthy_off(state);row['complete']+=1
                camera.check();remaining=.075-(time.perf_counter()-began)
                if remaining>0:time.sleep(remaining)
            row.update(median_ms=statistics.median(row['read_ms']),maximum_ms=max(row['read_ms']))
            print(json.dumps({k:v for k,v in row.items() if k not in ('read_ms','failures')},ensure_ascii=False),flush=True)
        native_rx=packet.rxPacket
        from combined_quiet_reserve_20261029 import receive
        for episode in range(3):
            calls=0;bus._read_recovery_times=[];began=time.perf_counter();bus._horizontal_sample_deadline=began+.2
            def corrupt(*args,**kwargs):
                nonlocal calls
                raw,comm=native_rx(*args,**kwargs);calls+=1
                if calls==2 and not comm:raw=list(raw);raw[-1]^=1
                return raw,comm
            packet.rxPacket=corrupt
            try:
                count=len(report.get('read_failures',[]))
                first=receive(bus,lambda:fast4(bus,64,83,fields),'READ ONLY HOST CRC fault',dict(kind='sync_read',address=64,length=83));healthy_off(first)
                second=receive(bus,lambda:fast4(bus,64,83,fields),'READ ONLY second full snapshot',dict(kind='sync_read',address=64,length=83));healthy_off(second)
                failures=report['read_failures'][count:];elapsed=1000*(time.perf_counter()-began)
                if calls!=10 or elapsed>=200 or len(failures)!=1 or failures[0]['completed_ids']!=[1,2,3,4] or not failures[0]['recovered']:raise RuntimeError('Incomplete groups did not recover as two fresh16-device reads')
                report['host_corruption_episodes'].append(dict(index=episode,host_only_crc_corruption=True,elapsed_ms=elapsed,reply_groups_received=calls,
                    incomplete_snapshot_discarded=True,two_fresh_complete_OFF_snapshots=True))
            finally:packet.rxPacket=native_rx
        report['status']='fast4_readonly_comparison_complete'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if port is not None:port.closePort()
        if camera is not None:report['camera_recording']=camera.finish()
        (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=folder.name,status=report['status'],error=report.get('error'),comparison_pairs=len(report['comparison']),
        blocks=[{k:v for k,v in x.items() if k not in ('failures','read_ms')}|dict(failed_reads=len(x['failures'])) for x in report['blocks']],
        host_fault_episodes=report['host_corruption_episodes'],servo_writes=0,torque_changes=0),ensure_ascii=False))
if __name__=='__main__':main()
