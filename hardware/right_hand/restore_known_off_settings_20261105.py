"""User-run, source-bound RAM settings restoration with torque staying OFF.

Default opens no devices. No position, torque, mode, baud or EEPROM writes.
"""
import argparse,datetime as dt,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'forward_hold_observe_20261105_030930_516359/result.json'
SOURCE_SHA='1429c406b250648d64f70dcec1417fb12a9545c9e906e43446388b4ce670bc98'
FIELDS=((98,1,'watchdog'),(102,2,'current_cap'),(108,4,'acceleration'),(112,4,'velocity'),(80,2,'d_gain'),(82,2,'i_gain'),(84,2,'p_gain'))

def baseline():
    if hashlib.sha256(SOURCE.read_bytes()).hexdigest()!=SOURCE_SHA:raise RuntimeError('Original settings evidence changed')
    state=json.loads(SOURCE.read_bytes())['initial']
    expected={'d_gain':1900,'i_gain':0,'p_gain':2100,'current_cap':2352,'acceleration':0,'velocity':0,'watchdog':0}
    if any(state[k]!=[value]*16 for k,value in expected.items()) or state['torque']!=[0]*16 or state['error']!=[0]*16:raise RuntimeError('Unexpected original OFF settings')
    return state

def check_off(io):
    for motor in range(1,17):
        if io.read(motor,64,1)!=0 or io.read(motor,70,1)!=0 or io.read(motor,11,1)!=5:raise RuntimeError(f'ID{motor}: OFF/error/mode check failed; no further setting writes')

def restore(io,state,report):
    check_off(io);report['writes']=[];report['already_matching']=[]
    for address,size,key in FIELDS:
        check_off(io)
        for motor in range(1,17):
            expected=int(state[key][motor-1]);actual=io.read(motor,address,size)
            if actual==expected:
                report['already_matching'].append(dict(id=motor,address=address));continue
            if io.read(motor,64,1)!=0:raise RuntimeError(f'ID{motor}: torque changed; no write')
            io.write(motor,address,size,expected)
            report['writes'].append(dict(id=motor,address=address,value=expected))
            if io.read(motor,address,size)!=expected:raise RuntimeError(f'ID{motor}: setting readback mismatch; no retry')
    check_off(io)
    for address,size,key in FIELDS:
        for motor in range(1,17):
            if io.read(motor,address,size)!=int(state[key][motor-1]):raise RuntimeError('Final original settings verification failed')
    report.update(status='original_off_RAM_settings_restored',all_torque_off_verified=True,original_settings_verified=True,torque_changes=0,position_writes=0,mode_writes=0,EEPROM_writes=0)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--restore',action='store_true');args=ap.parse_args();state=baseline()
    print('원래 RAM 설정 복구: 토크 OFF 유지. 손 움직임/목표각/모드/통신속도 변경 없음.',flush=True)
    if not args.restore:print('CHECK ONLY: no devices opened. --restore is required.');return 0
    import dynamixel_sdk as sdk
    from serial.tools.list_ports import comports
    folder=ROOT/('off_settings_restore_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir();report=dict(status='started',source=SOURCE.relative_to(ROOT).as_posix(),source_sha256=SOURCE_SHA,torque_changes=0,position_writes=0,EEPROM_writes=0,instruction_counts={});port=None
    try:
        ports=[p for p in comports() if p.vid==1027 and p.pid==24596 and p.serial_number=='FTBENPSAA']
        if len(ports)!=1:raise RuntimeError('Known adapter missing or ambiguous')
        port=sdk.PortHandler(ports[0].device);packet=sdk.PacketHandler(2.)
        if not port.openPort() or not port.setBaudRate(1000000):raise RuntimeError('Cannot open verified1Mbps')
        original=port.writePort;allowed={address:(size,key) for address,size,key in FIELDS}
        def guard(data):
            if len(data)<10:raise RuntimeError('Invalid packet')
            instruction=int(data[7]);motor=int(data[4])
            if instruction not in (1,2,3):raise RuntimeError('Instruction outside restore scope')
            if instruction==3:
                address=int.from_bytes(bytes(data[8:10]),'little')
                if address not in allowed or not 1<=motor<=16:raise RuntimeError('Forbidden setting write')
                size,key=allowed[address];value=int(state[key][motor-1])
                if bytes(data[10:-2])!=value.to_bytes(size,'little'):raise RuntimeError('Write differs from original source')
            counts=report['instruction_counts'];counts[str(instruction)]=counts.get(str(instruction),0)+1;return original(data)
        port.writePort=guard
        class IO:
            def read(self,motor,address,size):
                value,comm,error=getattr(packet,f'read{size}ByteTxRx')(port,motor,address)
                if comm or error:raise RuntimeError(f'ID{motor} READ{address}: comm={comm} error={error}')
                return int(value)
            def write(self,motor,address,size,value):
                comm,error=getattr(packet,f'write{size}ByteTxRx')(port,motor,address,value)
                if comm or error:raise RuntimeError(f'ID{motor} restore{address}: comm={comm} error={error}')
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor}: unexpected/missing motor')
        restore(IO(),state,report)
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc) or 'Interrupted')
    finally:
        if port is not None:port.closePort()
        (folder/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=folder.name,status=report['status'],error=report.get('error'),setting_writes=len(report.get('writes',[])),all_torque_off_verified=report.get('all_torque_off_verified'),original_settings_verified=report.get('original_settings_verified'),position_writes=0,torque_changes=0),ensure_ascii=False),flush=True)
    return 0 if report['status']=='original_off_RAM_settings_restored' else 2
if __name__=='__main__':raise SystemExit(main())
