"""PING every known ID at existing3Mbps; optional health READs, never WRITE."""
import datetime as dt,json,sys
from pathlib import Path
def main():
    if '--read' not in sys.argv:print('CHECK ONLY: no motor port opened.');return 0
    import dynamixel_sdk as sdk
    root=Path(__file__).resolve().parent;out=root/('all_ping_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',host_bps=3000000,motor_writes=0,servo_setting_writes=0,devices=[])
    port=sdk.PortHandler('COM7');packet=sdk.PacketHandler(2.);opened=False
    try:
        if not port.openPort():raise RuntimeError('Cannot open COM7; close other motor programs')
        opened=True
        if not port.setBaudRate(3000000):raise RuntimeError('Cannot set existing host baud')
        for motor in range(1,17):
            model,comm,error=packet.ping(port,motor);row=dict(id=motor,model=int(model),comm=int(comm),error=int(error));report['devices'].append(row)
            if comm==sdk.COMM_SUCCESS and error==0:
                row['registers']={}
                for address,size in ((64,1),(70,1),(98,1),(144,2),(8,1)):
                    fn=packet.read1ByteTxRx if size==1 else packet.read2ByteTxRx
                    value,result,device_error=fn(port,motor,address)
                    row['registers'][str(address)]=dict(value=int(value),comm=int(result),error=int(device_error))
        report['responding_ids']=[d['id'] for d in report['devices'] if d['comm']==0 and d['error']==0]
        report['status']='all_known_ids_pinged_read_only'
    except Exception as exc:report.update(status='stopped',error=str(exc))
    finally:
        if opened:port.closePort()
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=out.name,status=report['status'],responding_ids=report.get('responding_ids',[]),
        responding_states=[d for d in report['devices'] if d['comm']==0],error=report.get('error'),motor_writes=0),ensure_ascii=False))
if __name__=='__main__':raise SystemExit(main())
