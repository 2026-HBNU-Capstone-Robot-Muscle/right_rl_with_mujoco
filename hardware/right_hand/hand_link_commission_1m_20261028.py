"""User-started 3Mbps -> 1Mbps test. Only EEPROM baud(8) may be written.

Default and imports open no devices. Torque must already be OFF everywhere.
No torque, goal, current, gain, mode, offset or watchdog write is available.
"""
import argparse,datetime as dt,hashlib,json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
CONFIG=ROOT/'hand_serial_link_20261028.json'
RATES={3000000:5,1000000:3}
PRESERVED={64:1,70:1,98:1,11:1,10:1,20:4,9:1,38:2,80:2,82:2,84:2,102:2,108:4,112:4,116:4}

def save(path,value):
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode('utf-8'));temporary.replace(path)

class Link:
    def __init__(self,sdk,report):
        self.sdk=sdk;self.port=sdk.PortHandler('COM7');self.packet=sdk.PacketHandler(2.0);self.report=report
    def rate(self,value):
        if value not in RATES or not self.port.setBaudRate(value):raise RuntimeError('Host baud selection failed')
        self.port.clearPort()
    def read(self,motor,address,size=1):
        fn={1:self.packet.read1ByteTxRx,2:self.packet.read2ByteTxRx,4:self.packet.read4ByteTxRx}[size]
        value,comm,error=fn(self.port,motor,address)
        if comm or error:raise RuntimeError(f'ID{motor} READ{address} failed: comm={comm}, error={error}')
        return int(value)
    def state(self,motor):
        state={str(a):self.read(motor,a,n) for a,n in PRESERVED.items()}
        state['baud_value']=self.read(motor,8);state['voltage']=self.read(motor,144,2);state['temperature']=self.read(motor,146)
        if any(state[str(a)] for a in (64,70,98)):raise RuntimeError(f'ID{motor} torque/error/watchdog must already be zero; no correction sent')
        if state['11']!=5 or state['10']!=0 or state['20']!=0:raise RuntimeError(f'ID{motor} mode/offset differs; no correction sent')
        if not 38<=state['voltage']<=65 or state['temperature']>45:raise RuntimeError(f'ID{motor} OFF voltage/temperature outside checks')
        return state
    def locate(self,motor):
        found=[]
        for rate in RATES:
            self.rate(rate);model,comm,error=self.packet.ping(self.port,motor)
            if comm==0:
                if error or model!=1240:raise RuntimeError(f'ID{motor} model/error mismatch')
                state=self.state(motor)
                if state['baud_value']!=RATES[rate]:raise RuntimeError('Baud readback disagrees with responding speed')
                found.append(dict(rate=rate,state=state))
        if len(found)!=1:raise RuntimeError(f'ID{motor} must respond at exactly one supported speed; found {len(found)}')
        return found[0]
    def write_baud_once(self,motor,address,value):
        if motor not in range(1,17) or address!=8 or value!=3:raise RuntimeError('Only ID1..16 baud register8 ->3 permitted')
        # Recheck this motor immediately before the sole allowed write.
        current=self.state(motor)
        if current['baud_value']!=5:raise RuntimeError('Only existing3Mbps devices may be converted')
        event=dict(motor_id=motor,address=8,size=1,value=3,old_value=5,transmissions=1)
        self.report.setdefault('baud_write_events',[]).append(event)
        comm=self.packet.write1ByteTxOnly(self.port,motor,8,3)
        event['comm']=int(comm)
        if comm:raise RuntimeError(f'ID{motor} baud transmission failed; no automatic resend')

def migrate(link,report,checkpoint=lambda:None,sleep=time.sleep):
    # Finish all16 OFF checks before changing even the first device.
    before={str(m):link.locate(m) for m in range(1,17)}
    report['before']=before;checkpoint()
    for motor in range(1,17):
        record=before[str(motor)]
        if record['rate']==3000000:
            link.rate(3000000);link.write_baud_once(motor,8,3);checkpoint()
            sleep(.05)
        link.rate(1000000)
        first=link.state(motor);second=link.state(motor)
        if first['baud_value']!=3 or second['baud_value']!=3:raise RuntimeError(f'ID{motor} new baud not confirmed; no write resend')
        for address in PRESERVED:
            key=str(address)
            if first[key]!=record['state'][key] or second[key]!=record['state'][key]:
                raise RuntimeError(f'ID{motor} preserved register{address} changed; stopped without correction')
        report.setdefault('confirmed_ids',[]).append(motor);checkpoint()
        print(f'ID{motor}: 1Mbps confirmed; torque OFF; other settings unchanged.',flush=True)
    # A final scan at the common new speed catches partial/mixed outcomes.
    link.rate(1000000);after={}
    for motor in range(1,17):
        after[str(motor)]=link.state(motor)
        if after[str(motor)]['baud_value']!=3:raise RuntimeError('Final common speed check failed')
        if any(after[str(motor)][str(a)]!=before[str(motor)]['state'][str(a)] for a in PRESERVED):
            raise RuntimeError(f'ID{motor} final preserved-register comparison failed')
    report['after']=after;report['status']='all_16_baud_1m_confirmed';checkpoint()
    return after

def stress(link,report):
    import statistics
    from camera_stream import CameraStream
    camera=None;times=[];review=dict(status='started',requested_reads=1000,complete_reads=0,errors=[],camera_frames=0)
    report['readonly_check']=review
    try:
        camera=CameraStream(report)
        for index in range(1000):
            group=link.sdk.GroupSyncRead(link.port,link.packet,64,83)
            try:
                for motor in range(1,17):
                    if not group.addParam(motor):raise RuntimeError('Cannot add all motor IDs')
                began=time.perf_counter();comm=group.txPacket()
                if not comm:link.port.setPacketTimeoutMillis(80.);comm=group.rxPacket()
                elapsed=(time.perf_counter()-began)*1000
                if comm or not all(group.isAvailable(m,64,83) for m in range(1,17)):
                    review['errors'].append(dict(index=index,comm=int(comm),elapsed_ms=elapsed));time.sleep(.02)
                    if len(review['errors'])>=20:raise RuntimeError('Repeated receive errors; no motion performed')
                    continue
                if elapsed>200:raise RuntimeError('Read exceeds existing200ms limit')
                for motor in range(1,17):
                    if any(group.getData(motor,a,1) for a in (64,70,98)):raise RuntimeError('OFF/error/watchdog changed; no motion performed')
                    if not 38<=group.getData(motor,144,2)<=65 or group.getData(motor,146,1)>45:raise RuntimeError('OFF health abnormal')
                times.append(elapsed);review['complete_reads']+=1
                if camera.read_packet() is not None:review['camera_frames']+=1
                if (index+1)%200==0:print(f'Camera + READ only: {index+1}/1000; errors {len(review["errors"])}',flush=True)
            finally:group.clearParam()
        review['status']='complete'
    finally:
        if camera is not None:camera.release()
        if times:review.update(median_ms=statistics.median(times),maximum_ms=max(times))

def main(argv=None):
    parser=argparse.ArgumentParser();parser.add_argument('--apply',action='store_true');args=parser.parse_args(argv)
    if not args.apply:
        print('CHECK ONLY: no port/camera opened. User --apply + SPACE changes baud3Mbps ->1Mbps only.');return 0
    print('COMMUNICATION SETUP ONLY. No robot movement. Cube stays OUT throughout.',flush=True)
    print('Changes motor communication3Mbps ->1Mbps, saved across power cycles. Torque must already be OFF.',flush=True)
    print('Close Wizard and motor programs; leave hand in place. SPACE here starts; Q/Esc cancels.',flush=True)
    import msvcrt
    while True:
        key=msvcrt.getwch()
        if key==' ':break
        if key.lower()=='q' or key in ('\x1b','\x03'):return 1
    import dynamixel_sdk as sdk
    folder=ROOT/('link_commission_1m_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(timestamp=dt.datetime.now().astimezone().isoformat(),status='preflight',
        instruction='user_start_apply_and_SPACE',target_baud=1000000,torque_writes=0,goal_writes=0,
        current_gain_mode_offset_writes=0,configuration_persistent=True,baud_write_events=[],confirmed_ids=[],
        program_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    link=Link(sdk,report);opened=False
    try:
        if not link.port.openPort():raise RuntimeError('Cannot open COM7')
        opened=True
        migrate(link,report,checkpoint=lambda:save(folder/'result.json',report))
        # Publish host settings only after every device passed at the new speed.
        save(CONFIG,dict(status='all_16_verified',baud=1000000,baud_register_value=3,port='COM7',
            motor_ids=list(range(1,17)),torque_off_verified=True,evidence=(folder/'result.json').relative_to(ROOT).as_posix(),
            program_sha256=report['program_sha256']))
        print('All16 at1Mbps, torque OFF. Starting camera +1000 READ-only checks; no SPACE/PLACE needed.',flush=True)
        stress(link,report)
        report['status']='baud_1m_and_readonly_check_complete'
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc) or 'Interrupted')
        print('STOP:',report['error'],flush=True)
        print('No movement was requested. Do not start the rotation program yet; send done for saved-file review.',flush=True)
    finally:
        if opened:link.port.closePort()
        save(folder/'result.json',report)
    print('Saved:',folder,flush=True)
    print('Send done only. The rotation program will be matched to the verified speed before the next motion.',flush=True)
    return 0 if report['status']=='baud_1m_and_readonly_check_complete' else 1
if __name__=='__main__':raise SystemExit(main())
