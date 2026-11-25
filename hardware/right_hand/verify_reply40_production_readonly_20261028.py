"""Exercise the actual40ms decoder/quiet-recovery/deadline while OFF."""
import argparse,datetime as dt,json,time
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parent
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--read',action='store_true');args=ap.parse_args()
    if not args.read:print('CHECK ONLY. No devices opened. --read permits PING/READ only.');return
    import numpy as np
    import dynamixel_sdk as sdk
    from serial.tools.list_ports import comports
    from camera_stream import CameraStream
    from camera_frame_signal import frame_signal
    from horizontal_single_reply40_20261028 import block_once
    from horizontal_receive_resync_20261027 import receive
    from horizontal_read_recovery_20261028 import sample_deadline,confirm_recovered_read
    out=ROOT/('reply40_production_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',motor_writes=0,torque_changes=0,reads=0,samples=[],frames=0,bad_frames=0,latencies_ms=[],snapshots=[])
    port=None;camera=None;opened=False;command=None
    fields=[('torque',64,1),('error',70,1),('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),('watchdog',98,1),('current_cap',102,2),('acceleration',108,4),('velocity',112,4),('goal',116,4),('current',126,2),('position',132,4),('voltage',144,2),('temperature',146,1)]
    try:
        matches=[p for p in comports() if p.vid==1027 and p.pid==24596 and p.serial_number=='FTBENPSAA']
        if len(matches)!=1:raise RuntimeError('Known USB adapter missing or ambiguous')
        report['device']=matches[0].device;port=sdk.PortHandler(matches[0].device);packet=sdk.PacketHandler(2.0)
        if not port.openPort() or not port.setBaudRate(1000000):raise RuntimeError('Cannot open existing1Mbps')
        opened=True;original_write=port.writePort
        def readonly(data):
            if len(data)<8 or data[7] not in (1,2,0x82):raise RuntimeError('Motor write instruction blocked')
            return original_write(data)
        port.writePort=readonly
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} PING failed')
            for address,expected in ((8,3),(11,5),(64,0),(70,0),(98,0)):
                value,comm,error=packet.read1ByteTxRx(port,motor,address)
                if comm or error or value!=expected:raise RuntimeError(f'ID{motor} unexpected READ{address}')
        bus=SimpleNamespace(sdk=sdk,port=port,packet=packet,report=report,_read_recovery_times=[])
        camera=CameraStream(report)
        def observe_frame():
            p=camera.read_packet()
            if p is not None:report['frames']+=1;report['bad_frames']+=int(not frame_signal(p[1])['usable'])
        def validate(state):
            if any(state[k].any() for k in ('torque','error','watchdog')):raise RuntimeError('Expected healthy OFF state')
            if min(state['voltage'])<38 or max(state['voltage'])>65 or max(state['temperature'])>45:raise RuntimeError('OFF voltage/temperature abnormal')
            # OFF observations have no commanded motion target. Preserve any
            # register changes as data; the live ON controller's exact-goal
            # checks are unchanged. Recovery below still requires the same
            # fresh latched goal in both confirmation snapshots.
        def read():
            state=receive(bus,lambda:block_once(bus,64,83,fields),'full health READ',dict(kind='sync_read',address=64,length=83))
            validate(state);report['samples'].append(dict(goal=state['goal'].tolist()))
            return state,np.zeros(16)
        for _ in range(200):
            began=time.monotonic();failures=len(report.get('read_failures',[]));observe_frame()
            with sample_deadline(bus,began):
                state,errors=read()
                if command is not None and not np.array_equal(state['goal'],command):
                    report.setdefault('off_goal_changes',[]).append(dict(before=command.tolist(),after=state['goal'].tolist(),position=state['position'].tolist(),maximum_delta_ticks=int(np.max(abs(state['goal']-command)))))
                command=state['goal'].copy()
                if len(report.get('read_failures',[]))>failures:
                    state,errors=confirm_recovered_read(bus,command,state,errors,report,'readonly_link_observation',began=began,read_sample=read,validate=validate)
            report['reads']+=1;report['latencies_ms'].append((time.monotonic()-began)*1000)
            report['snapshots']=(report['snapshots']+[{k:v.tolist() for k,v in state.items()}])[-3:]
            while time.monotonic()-began<.075:observe_frame();time.sleep(.002)
        report['status']='reply40_production_readonly_complete'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if opened:port.closePort()
        if camera is not None:camera.release()
        (out/'result.json').write_bytes((json.dumps(report,indent=2)+'\n').encode())
    print(json.dumps(dict(folder=out.name,status=report['status'],error=report.get('error'),reads=report['reads'],
        raw_failures=len(report.get('read_failures',[])),recovery_events=len(report.get('read_recovery_confirmation_events',[])),frames=report['frames'],bad_frames=report['bad_frames'],
        max_sample_ms=max(report['latencies_ms'],default=None),motor_writes=0)))
if __name__=='__main__':main()
