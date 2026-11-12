"""Inspect every motor without writes, even when ID1 is disconnected."""
import argparse,datetime as dt,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--read',action='store_true');args=parser.parse_args()
    if not args.read:print('No devices opened. --read permits PING/READ only.');return
    import dynamixel_sdk as sdk
    from serial.tools.list_ports import comports
    folder=ROOT/('connection_ping16_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));folder.mkdir()
    report=dict(instruction_counts={},motors=[],servo_writes=0,torque_changes=0,baud=1000000);port=None
    try:
        devices=[p for p in comports() if (p.vid,p.pid,p.serial_number)==(1027,24596,'FTBENPSAA')]
        if len(devices)!=1:raise RuntimeError('Known USB adapter missing or ambiguous')
        report['device']=devices[0].device;port=sdk.PortHandler(devices[0].device);packet=sdk.PacketHandler(2.)
        if not port.openPort() or not port.setBaudRate(1000000):raise RuntimeError('Cannot open verified1Mbps')
        original=port.writePort
        def readonly(data):
            if len(data)<8 or data[7] not in (1,2):raise RuntimeError('Blocked instruction other than PING/READ')
            key=str(data[7]);report['instruction_counts'][key]=report['instruction_counts'].get(key,0)+1
            return original(data)
        port.writePort=readonly
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor);row=dict(id=motor,model=model,comm=comm,error=error,responded=comm==0)
            if comm==0 and model==1240:
                checks={}
                for name,address,size in (('torque',64,1),('hardware_error',70,1),('watchdog',98,1),('voltage_raw',144,2)):
                    value,rc,err=(packet.read1ByteTxRx if size==1 else packet.read2ByteTxRx)(port,motor,address)
                    checks[name]=dict(value=value,comm=rc,error=err)
                row['readings']=checks
            report['motors'].append(row)
        report['status']='individual_ping_sweep_complete'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if port is not None:port.closePort()
        (folder/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=folder.name,status=report['status'],responding_ids=[r['id'] for r in report['motors'] if r['responded']],
        instructions=report['instruction_counts'],servo_writes=0,torque_changes=0)))
if __name__=='__main__':main()
