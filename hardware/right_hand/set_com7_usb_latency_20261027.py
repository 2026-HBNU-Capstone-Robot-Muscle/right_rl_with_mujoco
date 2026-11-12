"""Change only COM7 Windows FTDI LatencyTimer after fresh torque-OFF reads.

No actuator WRITE/torque/goal calls. Default opens no motor port and changes
nothing. Administrator permission may be required for the driver registry key.
"""
import datetime as dt,json,sys,winreg
from pathlib import Path

def main():
    if '--apply' not in sys.argv:print('CHECK ONLY: no ports opened, no settings changed.');return 0
    import dynamixel_sdk as sdk
    from leap_camera_readonly_check import ReadonlyBus,validate_off
    from benchmark_current_serial_readonly_20261027 import inspect_usb_latency
    root=Path(__file__).resolve().parent
    out=(Path(sys.argv[sys.argv.index('--out')+1]).resolve() if '--out' in sys.argv else
         root/('usb_latency_change_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
    if out.parent!=root or not out.name.startswith('usb_latency_change_'):raise ValueError('Output must stay in workspace')
    out.mkdir(exist_ok=True)
    report=dict(status='started',motor_writes=0,baud_writes=0,driver_setting='COM7 LatencyTimer only',registry_writes=0,
                target_latency_ms=1,rollback_required=False)
    bus=ReadonlyBus(sdk,'COM7');applied=False;entry=None
    try:
        matches=inspect_usb_latency()['matches']
        if len(matches)!=1:raise RuntimeError('Expected exactly one FTDI COM7 device')
        entry=matches[0];old=entry['latency_timer_ms'];report['original']=entry
        if old not in (1,16):raise RuntimeError('Unexpected driver latency; not changed')
        bus.open();report['before']=bus.snapshot();validate_off(report['before'])
        report['baud_registers']={}
        for motor in range(1,17):
            value,comm,error=bus.packet.read1ByteTxRx(bus.port,motor,8)
            if comm!=sdk.COMM_SUCCESS or error or value!=5:raise RuntimeError('Expected unchanged3Mbps on every motor')
            report['baud_registers'][str(motor)]=value
        bus.close()
        # Save the original value before the single, device-specific driver write.
        (out/'backup.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        if old!=1:
            with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,entry['registry_path'],0,winreg.KEY_QUERY_VALUE|winreg.KEY_SET_VALUE) as key:
                actual,kind=winreg.QueryValueEx(key,'LatencyTimer')
                if actual!=old or kind!=winreg.REG_DWORD:raise RuntimeError('Driver setting changed or not DWORD; no write')
                winreg.SetValueEx(key,'LatencyTimer',0,winreg.REG_DWORD,1);applied=True;report['registry_writes']=1
                if winreg.QueryValueEx(key,'LatencyTimer')[0]!=1:raise RuntimeError('Driver latency readback failed')
        report.update(status='driver_registry_updated',driver_latency_readback_ms=1,
            measured_read_speed_improvement_verified=False,motor_baud_changed=False,
            note='Reopen COM7 and benchmark; registry readback alone does not certify active-driver timing.')
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc))
        if applied:
            try:
                with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,entry['registry_path'],0,winreg.KEY_SET_VALUE) as key:
                    winreg.SetValueEx(key,'LatencyTimer',0,winreg.REG_DWORD,old)
                report['rolled_back_to_ms']=old;report['registry_writes']+=1
            except OSError as restore:report.update(rollback_required=True,rollback_error=str(restore))
    finally:
        bus.close();(out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('before','original','baud_registers')},ensure_ascii=False))
    print('Saved:',out)
    return 0 if report['status']=='driver_registry_updated' else 1
if __name__=='__main__':raise SystemExit(main())
