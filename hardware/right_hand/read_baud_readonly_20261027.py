"""Read EEPROM Baud Rate(8) for all16; never changes it."""
import datetime as dt,json,sys
from pathlib import Path

def main():
    if '--read' not in sys.argv:print('CHECK ONLY: no port opened.');return
    import dynamixel_sdk as sdk
    from leap_camera_readonly_check import ReadonlyBus,validate_off
    root=Path(__file__).resolve().parent;out=root/('baud_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',host_bps=3000000,register_address=8,register_values={},motor_writes=0,torque_or_setting_writes=0);bus=ReadonlyBus(sdk,'COM7')
    try:
        bus.open();report['before']=bus.snapshot();validate_off(report['before'])
        for motor in range(1,17):
            value,comm,error=bus.packet.read1ByteTxRx(bus.port,motor,8)
            if comm!=sdk.COMM_SUCCESS or error:raise RuntimeError(f'ID{motor} baud READ failed')
            report['register_values'][str(motor)]=value
        report['after']=bus.snapshot();validate_off(report['after']);report['status']='baud_registers_read_only_saved';report['all16_value5_3mbps']=len(report['register_values'])==16 and set(report['register_values'].values())=={5}
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        bus.close();(out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=out.name,status=report['status'],all16_3mbps=report.get('all16_value5_3mbps'),values=report['register_values'],error=report.get('error'),motor_writes=0)))
if __name__=='__main__':main()
