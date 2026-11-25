"""Actual READs plus explicit host-only faults; no motion or setting writes."""
import argparse,datetime as dt,json,time
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parent
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--read',action='store_true');a=parser.parse_args()
    if not a.read:print('No devices opened. --read permits PING/READ only.');return
    import numpy as np,dynamixel_sdk as sdk
    from serial.tools.list_ports import comports
    from calibrated_precise_clock_20261028 import install
    from calibrated_precise_window_20261028 import block_once
    from combined_quiet_reserve_20261029 import receive,POLICY
    from observed_book_model_policy import healthy_off
    out=ROOT/('readonly_recovery_v2_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',servo_writes=0,torque_changes=0,transport_policy=POLICY,
        samples=[],episodes=[],instruction_counts={},host_faults_are_synthetic=True,camera_opened=False,
        diagnostic_scope='two healthy OFF snapshots; ON exact-command check remains unchanged and is tested separately')
    port=None
    try:
        matches=[p for p in comports() if p.vid==1027 and p.pid==24596 and p.serial_number=='FTBENPSAA']
        if len(matches)!=1:raise RuntimeError('Known USB adapter missing or ambiguous')
        port=sdk.PortHandler(matches[0].device);packet=sdk.PacketHandler(2.)
        if not port.openPort() or not port.setBaudRate(1000000):raise RuntimeError('Cannot open verified1Mbps')
        original=port.writePort
        def readonly(data):
            if len(data)<8 or data[7] not in (1,2,0x82):raise RuntimeError('Motor write instruction blocked')
            key=str(data[7]);report['instruction_counts'][key]=report['instruction_counts'].get(key,0)+1;return original(data)
        port.writePort=readonly
        bus=SimpleNamespace(sdk=sdk,port=port,packet=packet,report=report,_read_recovery_times=[]);install(bus)
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} ping failed')
            for address,expected in ((8,3),(11,5),(64,0),(70,0),(98,0)):
                value,comm,error=packet.read1ByteTxRx(port,motor,address)
                if comm or error or value!=expected:raise RuntimeError(f'ID{motor} unexpected register{address}')
        fields=[('torque',64,1),('error',70,1),('watchdog',98,1),('goal',116,4),('current',126,2),('position',132,4),('voltage',144,2),('temperature',146,1)]
        # Independent READ-only episodes. Clearing the retry history here only
        # isolates synthetic diagnostic cases; the motion program never does it.
        for index,pause in enumerate([None]*80+[.04,.06,.08,.13]):
            bus._read_recovery_times=[];began=time.perf_counter();bus._horizontal_sample_deadline=began+.2
            episode=dict(index=index,synthetic_host_pause_s=pause,expected='stop_at_original_deadline' if pause==.13 else 'two_full_healthy_snapshots',state_reads=0)
            report['episodes'].append(episode);operations=0
            def operation():
                nonlocal operations
                operations+=1;state,comm,error,detail=block_once(bus,64,83,fields)
                if pause is not None and operations==1 and not comm and not error:
                    time.sleep(pause)
                    return None,-3002,0,dict(host_injected_discard_of_complete_snapshot=True,host_pause_s=pause)
                return state,comm,error,detail
            def full_read():
                state=receive(bus,operation,'READ ONLY health',dict(kind='sync_read',address=64,length=83))
                healthy_off(state);episode['state_reads']+=1
                report['samples'].append(dict(phase='readonly',state={k:v.tolist() for k,v in state.items()}))
                return state,np.zeros(16)
            try:
                state,errors=full_read();clean=1
                # Goal Position follows Present Position while OFF on these
                # drives (saved1tick evidence). Do not change the ON checker or
                # fabricate fixed goals to force this OFF diagnostic through it.
                for _ in range(5):
                    before=len(report.get('read_failures',[]));fresh,errors=full_read()
                    if time.perf_counter()>=began+.2:raise RuntimeError('Two OFF snapshots exceeded200ms')
                    clean=1 if len(report.get('read_failures',[]))>before else clean+1
                    if clean>=2:break
                if clean<2:raise RuntimeError('Two consecutive healthy OFF snapshots missing')
                episode['outcome']='two_full_healthy_snapshots'
            except RuntimeError as exc:
                episode.update(outcome='stopped',error=str(exc))
                if pause!=.13:raise
            episode['elapsed_ms']=1000*(time.perf_counter()-began)
            if pause==.13:
                if episode['outcome']!='stopped' or episode['elapsed_ms']<195:raise RuntimeError('Original200ms stop did not behave as expected')
            elif episode['elapsed_ms']>=200:raise RuntimeError('Healthy confirmation exceeded original200ms')
        report['status']='readonly_recovery_v2_passed'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if port is not None:port.closePort()
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=out.name,status=report['status'],error=report.get('error'),episodes=len(report['episodes']),
        final_episodes=report['episodes'][-4:],instructions=report['instruction_counts'],servo_writes=0,torque_changes=0)))
if __name__=='__main__':main()
