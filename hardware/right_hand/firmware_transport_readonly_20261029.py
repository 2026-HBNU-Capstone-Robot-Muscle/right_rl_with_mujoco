"""Read supported-device firmware and communication settings; never writes."""
import argparse,datetime as dt,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--read',action='store_true');a=parser.parse_args()
    if not a.read:print('No devices opened.');return
    import dynamixel_sdk as sdk
    import importlib.metadata as metadata
    from serial.tools.list_ports import comports
    out=ROOT/('firmware_transport_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',servo_writes=0,torque_changes=0,devices=[],sdk_file=sdk.__file__,
        sdk_version=metadata.version('dynamixel-sdk'),fast_members=[s for s in dir(sdk) if 'fast' in s.lower()],instruction_counts={})
    port=None
    try:
        devices=[p for p in comports() if p.vid==1027 and p.pid==24596 and p.serial_number=='FTBENPSAA']
        if len(devices)!=1:raise RuntimeError('Known USB adapter missing or ambiguous')
        port=sdk.PortHandler(devices[0].device);packet=sdk.PacketHandler(2.)
        if not port.openPort() or not port.setBaudRate(1000000):raise RuntimeError('Cannot open verified1Mbps')
        original=port.writePort
        def readonly(data):
            if len(data)<8 or data[7] not in (1,2):raise RuntimeError('Only PING and READ allowed')
            key=str(data[7]);report['instruction_counts'][key]=report['instruction_counts'].get(key,0)+1;return original(data)
        port.writePort=readonly
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor)
            if comm or error or model!=1240:raise RuntimeError(f'ID{motor} ping failed')
            values={}
            for name,address in (('firmware',6),('baud',8),('return_delay',9),('mode',11),('torque',64),('status_return_level',68),('error',70),('watchdog',98)):
                value,comm,error=packet.read1ByteTxRx(port,motor,address)
                if comm or error:raise RuntimeError(f'ID{motor} READ{address} failed')
                values[name]=int(value)
            if values['torque'] or values['error'] or values['watchdog']:raise RuntimeError('Expected healthy torque OFF')
            report['devices'].append(dict(id=motor,model=int(model),**values))
        report['status']='firmware_transport_saved'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if port is not None:port.closePort()
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=out.name,**report)))
if __name__=='__main__':main()
