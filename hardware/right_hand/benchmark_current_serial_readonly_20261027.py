"""Benchmark existing3Mbps + inspect Windows USB latency; no motor writes."""
import datetime as dt,json,sys,time
from pathlib import Path

def inspect_usb_latency():
    import winreg
    found=[];base=r'SYSTEM\CurrentControlSet\Enum\FTDIBUS'
    try:root=winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,base,0,winreg.KEY_READ)
    except OSError as exc:return dict(matches=[],error=str(exc))
    with root:
        for i in range(winreg.QueryInfoKey(root)[0]):
            device=winreg.EnumKey(root,i)
            with winreg.OpenKey(root,device) as key:
                for j in range(winreg.QueryInfoKey(key)[0]):
                    endpoint=winreg.EnumKey(key,j);path=base+'\\'+device+'\\'+endpoint+r'\Device Parameters'
                    try:
                        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,path,0,winreg.KEY_READ) as params:
                            port,_=winreg.QueryValueEx(params,'PortName')
                            if port.upper()!='COM7':continue
                            try:timer,_=winreg.QueryValueEx(params,'LatencyTimer')
                            except OSError:timer=None
                            found.append(dict(port=port,latency_timer_ms=timer,registry_path=path))
                    except OSError:continue
    return dict(matches=found,registry_writes=0)

def main():
    if '--read' not in sys.argv:print('CHECK ONLY: no devices opened.');return
    import numpy as np
    import dynamixel_sdk as sdk
    from leap_camera_readonly_check import ReadonlyBus,validate_off
    root=Path(__file__).resolve().parent;out=root/('serial_benchmark_readonly_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
    report=dict(status='started',host_bps=3000000,frames=[],baud_registers={},return_delay_registers={},usb_latency=inspect_usb_latency(),motor_writes=0,torque_or_setting_writes=0,registry_writes=0);bus=ReadonlyBus(sdk,'COM7')
    try:
        bus.open();report['before']=bus.snapshot();validate_off(report['before'])
        for motor in range(1,17):
            for address,key in ((8,'baud_registers'),(9,'return_delay_registers')):
                value,comm,error=bus.packet.read1ByteTxRx(bus.port,motor,address)
                if comm!=sdk.COMM_SUCCESS or error:raise RuntimeError(f'ID{motor} register{address} READ failed')
                report[key][str(motor)]=value
        for _ in range(100):
            began=time.perf_counter();state=bus.snapshot();duration=(time.perf_counter()-began)*1000;validate_off(state)
            report['frames'].append(dict(read_ms=duration,minimum_voltage_v=min(state['voltage_v']),position_ticks=state['position_ticks']))
        report['after']=bus.snapshot();validate_off(report['after']);durations=[v['read_ms'] for v in report['frames']]
        report.update(status='100_full_off_reads_complete',all16_baud_value5_3mbps=set(report['baud_registers'].values())=={5},
            median_read_ms=float(np.median(durations)),p95_read_ms=float(np.percentile(durations,95)),maximum_read_ms=max(durations),
            reads_over_75ms=sum(v>75 for v in durations),reads_over_150ms=sum(v>150 for v in durations),communication_failures=0,
            minimum_voltage_v=min(v['minimum_voltage_v'] for v in report['frames']),
            packet_bytes=16*(83+11)+30,wire_time_ms={str(rate):(16*(83+11)+30)*10/rate*1000 for rate in (2000000,3000000,4000000)})
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped',error=str(exc))
    finally:
        bus.close();(out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(folder=out.name,**{k:v for k,v in report.items() if k not in ('frames','before','after','usb_latency')} ,usb_latency_ms=[v['latency_timer_ms'] for v in report['usb_latency']['matches']]),ensure_ascii=False))
if __name__=='__main__':main()
