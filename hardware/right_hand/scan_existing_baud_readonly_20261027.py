"""Known-ID PING at supported HOST rates only; never changes motor baud."""
import datetime as dt,json,sys
from pathlib import Path
def main():
    if '--scan' not in sys.argv:print('CHECK ONLY: no port opened.');return
    import dynamixel_sdk as sdk
    root=Path(__file__).resolve().parent;out=root/('baud_scan_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',motor_writes=0,torque_or_setting_writes=0,rates=[]);port=sdk.PortHandler('COM7');packet=sdk.PacketHandler(2.0);opened=False
    try:
        if not port.openPort():raise RuntimeError('Cannot open COM7')
        opened=True
        for rate in (3000000,1000000,2000000,4000000,115200,57600,9600):
            if not port.setBaudRate(rate):raise RuntimeError('Cannot set host baud')
            row=dict(host_bps=rate,responding_ids=[])
            for motor in range(1,17):
                model,comm,error=packet.ping(port,motor)
                if comm==sdk.COMM_SUCCESS and error==0 and model==1240:row['responding_ids'].append(motor)
            report['rates'].append(row);print(json.dumps(row),flush=True)
        report['status']='host_rate_ping_scan_complete'
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        if opened:port.closePort()
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=out.name,status=report['status'],motor_writes=0)))
if __name__=='__main__':main()
