"""Compare80/25ms single-reply windows, PING/READ only, with camera running."""
import argparse,datetime as dt,json,time
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parent
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--read',action='store_true')
    ap.add_argument('--windows',type=float,nargs='+',default=[80.,25.]);ap.add_argument('--counts',type=int,nargs='+',default=[50,200]);args=ap.parse_args()
    if len(args.windows)!=len(args.counts) or any(not 1<=v<=80 for v in args.windows) or any(not 1<=n<=500 for n in args.counts):raise ValueError('Invalid bounded read-only comparison')
    if not args.read:print('CHECK ONLY. --read permits only PING/READ; no devices opened.');return
    import numpy as np
    import dynamixel_sdk as sdk
    from serial.tools.list_ports import comports
    from camera_stream import CameraStream
    from camera_frame_signal import frame_signal
    from horizontal_sync_receive_window_20261028 import block_once
    out=ROOT/('single_reply_profile_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',motor_writes=0,torque_changes=0,baud=1000000,observations=[],packet_latencies=[],frames=0,unusable_frames=0)
    port=None;camera=None;opened=False;window=80.;snapshot=0
    try:
        matches=[p for p in comports() if p.vid==1027 and p.pid==24596 and p.serial_number=='FTBENPSAA']
        if len(matches)!=1:raise RuntimeError('Known USB adapter missing or ambiguous')
        report['device']=matches[0].device;port=sdk.PortHandler(matches[0].device);packet=sdk.PacketHandler(2.0)
        if not port.openPort() or not port.setBaudRate(1000000):raise RuntimeError('Cannot open existing1Mbps')
        opened=True;original_write=port.writePort
        def readonly(data):
            if len(data)<8 or data[7] not in (1,2,0x82):raise RuntimeError('Non-read motor instruction blocked')
            return original_write(data)
        port.writePort=readonly
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} PING failed')
            for address,expected in ((8,3),(11,5),(64,0),(70,0),(98,0)):
                value,comm,error=packet.read1ByteTxRx(port,motor,address)
                if comm or error or value!=expected:raise RuntimeError(f'ID{motor} unexpected READ{address}')
        original_timeout=port.setPacketTimeoutMillis
        port.setPacketTimeoutMillis=lambda value:original_timeout(min(value,window))
        class TimedGroup(sdk.GroupSyncRead):
            def rxPacket(self):
                began=time.perf_counter();comm=super().rxPacket()
                report['packet_latencies'].append(dict(snapshot=snapshot,requested_ids=list(self.data_dict),window_ms=window,receive_ms=(time.perf_counter()-began)*1000,comm=int(comm)))
                return comm
        shim=SimpleNamespace(COMM_SUCCESS=sdk.COMM_SUCCESS,GroupSyncRead=TimedGroup)
        bus=SimpleNamespace(sdk=shim,port=port,packet=packet,report=report)
        camera=CameraStream(report)
        def frame():
            p=camera.read_packet()
            if p is not None:
                signal=frame_signal(p[1]);report['frames']+=1;report['unusable_frames']+=int(not signal['usable'])
        fields=[('torque',64,1),('error',70,1),('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),('watchdog',98,1),('current_cap',102,2),('acceleration',108,4),('velocity',112,4),('goal',116,4),('current',126,2),('position',132,4),('voltage',144,2),('temperature',146,1)]
        report['snapshots']=[]
        for window,count in zip(args.windows,args.counts):
            failures=0
            for _ in range(count):
                snapshot+=1;began=time.perf_counter();frame()
                state,comm,error,detail=block_once(bus,64,83,fields)
                row=dict(snapshot=snapshot,window_ms=window,read_ms=(time.perf_counter()-began)*1000,comm=int(comm),error=int(error),detail=detail)
                report['observations'].append(row)
                if comm or error:failures+=1
                else:
                    if any(state[k].any() for k in ('torque','error','watchdog')):raise RuntimeError('Expected healthy OFF state')
                    if min(state['voltage'])<38 or max(state['voltage'])>65 or max(state['temperature'])>45:raise RuntimeError('OFF voltage/temperature abnormal')
                    report['snapshots']=(report['snapshots']+[{k:v.tolist() for k,v in state.items()}])[-3:]
                while time.perf_counter()-began<.075:frame();time.sleep(.002)
            rows=[v for v in report['packet_latencies'] if v['window_ms']==window and v['comm']==0]
            print(json.dumps(dict(window_ms=window,snapshots=count,failures=failures,packet_receive_ms_p50_p95_max=np.percentile([v['receive_ms'] for v in rows],[50,95,100]).tolist() if rows else None)),flush=True)
        report['status']='readonly_window_comparison_complete'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if opened:port.closePort()
        if camera is not None:camera.release()
        (out/'result.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
    print(json.dumps(dict(folder=out.name,status=report['status'],error=report.get('error'),snapshots=len(report['observations']),frames=report['frames'],bad_frames=report['unusable_frames'],motor_writes=0)))
if __name__=='__main__':main()
