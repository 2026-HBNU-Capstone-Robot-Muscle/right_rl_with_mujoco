"""User-executed original RAM P restoration. No torque, goal or EEPROM writes."""
import argparse,datetime as dt,hashlib,json,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'retained_slow_yaw_20261029_032447_137236/result.json'
SOURCE_SHA='6d994f1af3f8c115703f3c7fa4cbd50aa7620cfa7b1a36598e17d966acf9e1a8'
PROFILE=np.full(16,600,dtype=int);PROFILE[[7,11,15]]=450
def specification():
    if hashlib.sha256(SOURCE.read_bytes()).hexdigest()!=SOURCE_SHA:raise ValueError('Saved original configuration changed')
    r=json.loads(SOURCE.read_bytes());s=r['preflight_state'];p=np.asarray(s['p_gain'])
    if p.shape!=(16,) or not np.array_equal(p,np.full(16,2100)):raise ValueError('Unexpected saved original P values')
    return s
def validate_state(state,original):
    from observed_book_model_policy import healthy_off
    healthy_off(state)
    for k in ('d_gain','i_gain','current_cap','acceleration','velocity'):
        if not np.array_equal(state[k],original[k]):raise RuntimeError('Other configuration differs: '+k+'; no writes')
    current=np.asarray(state['p_gain']);expected=np.asarray(original['p_gain'])
    if current.shape!=(16,) or not np.all((current==expected)|(current==PROFILE)):raise RuntimeError('Unexpected P setting; no writes')
def validate_packet(data,original,armed):
    data=list(data)
    if len(data)<8:raise RuntimeError('Short packet blocked')
    instruction=data[7]
    if instruction in (1,2):return False
    if instruction==0x8a:
        if data[8:12]!=[64,0,83,0] or data[12:-2] not in [list(range(1,9)),list(range(9,17))]:raise RuntimeError('Fast READ outside health scope')
        return False
    expected=[]
    for motor,value in enumerate(original['p_gain'],1):expected.extend([motor,int(value)&255,int(value)>>8])
    if instruction!=0x83 or not armed or data[4]!=254 or data[8:12]!=[84,0,2,0] or data[12:-2]!=expected:
        raise RuntimeError('Only one exact saved P84 RAM restoration is allowed')
    return True
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true');args=parser.parse_args();original=specification()
    print('OFF RAM RESTORE: restore saved P2100 only. No torque enable, goal, mode, baud or EEPROM writes.',flush=True)
    if not args.execute:print('CHECK ONLY: no devices opened.');return 0
    print('큐브는 손 밖에 둡니다. 모터는 OFF 상태로 유지하고 원래 P값만 복구합니다.',flush=True)
    print('SPACE를 누르면 OFF 확인 후 복구합니다. Q/Esc/Ctrl+C는 중단합니다.',flush=True)
    import msvcrt
    while True:
        key=msvcrt.getwch()
        if key in ('q','Q','\x1b','\x03'):return 0
        if key==' ':break
    import dynamixel_sdk as sdk
    from serial.tools.list_ports import comports
    from types import SimpleNamespace
    from fast_health_read_20261029 import block_once
    from combined_quiet_reserve_20261029 import receive
    from calibrated_precise_clock_20261028 import install
    out=ROOT/('restore_off_P_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=SOURCE_SHA,torque_changes=0,
        goal_writes=0,EEPROM_writes=0,p_write_packets=0,snapshots=[],instruction_counts={},operator_SPACE_confirmed=True)
    port=None;armed=False
    try:
        matches=[p for p in comports() if p.vid==1027 and p.pid==24596 and p.serial_number=='FTBENPSAA']
        if len(matches)!=1:raise RuntimeError('Known USB adapter missing or ambiguous')
        port=sdk.PortHandler(matches[0].device);packet=sdk.PacketHandler(2.)
        if not port.openPort() or not port.setBaudRate(1000000):raise RuntimeError('Cannot open verified1Mbps')
        raw_write=port.writePort
        def guarded(data):
            nonlocal armed
            setting=validate_packet(data,original,armed)
            if setting:armed=False;report['p_write_packets']+=1
            key=str(data[7]);report['instruction_counts'][key]=report['instruction_counts'].get(key,0)+1;return raw_write(data)
        port.writePort=guarded;bus=SimpleNamespace(sdk=sdk,port=port,packet=packet,report=report,_read_recovery_times=[]);install(bus)
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError('PING/model mismatch')
            for address,expected in ((8,3),(11,5),(64,0),(70,0),(98,0)):
                value,comm,error=packet.read1ByteTxRx(port,motor,address)
                if comm or error or value!=expected:raise RuntimeError(f'ID{motor} state mismatch at{address}')
            fw,comm,error=packet.read1ByteTxRx(port,motor,6)
            if comm or error or fw<46:raise RuntimeError('Fast READ firmware unsupported')
        fields=[('torque',64,1),('error',70,1),('d_gain',80,2),('i_gain',82,2),('p_gain',84,2),('watchdog',98,1),('current_cap',102,2),('acceleration',108,4),('velocity',112,4),('goal',116,4),('current',126,2),('position',132,4),('voltage',144,2),('temperature',146,1)]
        def fresh(role):
            bus._horizontal_sample_deadline=time.perf_counter()+.2
            state=receive(bus,lambda:block_once(bus,64,83,fields),'OFF P restore READ',dict(kind='sync_read',address=64,length=83))
            validate_state(state,original);report['snapshots'].append(dict(role=role,state={k:v.tolist() for k,v in state.items()}));return state
        first=fresh('before1');second=fresh('before2')
        if not np.array_equal(second['p_gain'],original['p_gain']):
            group=sdk.GroupSyncWrite(port,packet,84,2)
            for motor,value in enumerate(original['p_gain'],1):
                if not group.addParam(motor,[value&255,value>>8]):raise RuntimeError('Cannot build exact saved P restoration')
            armed=True;report['status']='P_write_requested'
            try:
                if group.txPacket()!=sdk.COMM_SUCCESS:raise RuntimeError('P write transmission failed; no automatic resend')
            finally:armed=False;group.clearParam()
        for role in ('after1','after2'):
            state=fresh(role)
            if not np.array_equal(state['p_gain'],original['p_gain']):raise RuntimeError('P restoration not verified; no automatic resend')
        report.update(status='original_P_restored_OFF_verified',all_torque_off_verified=True,original_configuration_restored=True)
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if port is not None:port.closePort()
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=out.name,status=report['status'],error=report.get('error'),p_write_packets=report['p_write_packets'],torque_changes=0,goal_writes=0)),flush=True)
    return 0 if report['status']=='original_P_restored_OFF_verified' else 2
if __name__=='__main__':raise SystemExit(main())
