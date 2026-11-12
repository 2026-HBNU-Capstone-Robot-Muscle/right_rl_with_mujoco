"""Operator-started Windows COM7 latency adjustment, with measurement/rollback.

Only the driver LatencyTimer is written. All DYNAMIXEL traffic is PING/READ.
No reboot, driver install, broad device restart, or actuator setting changes.
"""
import argparse,ctypes,datetime as dt,json,os,subprocess,sys,time,winreg
from pathlib import Path
ROOT=Path(__file__).resolve().parent
BASELINE=ROOT/'serial_benchmark_readonly_20261027_190008_128041'/'result.json'
LATEST=ROOT/'leap_usb_latency_result.json'

def accepted(data,baseline):
    return (data.get('status')=='100_full_off_reads_complete' and data.get('communication_failures')==0
        and data.get('all16_baud_value5_3mbps') is True and len(data.get('frames',[]))==100
        and data.get('maximum_read_ms',float('inf'))<75 and data.get('minimum_voltage_v',0)>=3.8
        and data.get('median_read_ms',float('inf'))<baseline['median_read_ms']*.85)

def device_instance(entry):
    path=entry['registry_path'];prefix='SYSTEM\\CurrentControlSet\\Enum\\';suffix=r'\Device Parameters'
    if entry['port']!='COM7' or not path.startswith(prefix+'FTDIBUS\\') or not path.endswith(suffix):
        raise RuntimeError('Not the exact FTDI COM7 endpoint')
    instance=path[len(prefix):-len(suffix)]
    if any(c in instance for c in '*?\r\n') or not instance.endswith(r'\0000'):
        raise RuntimeError('Unsafe or unexpected device instance')
    return instance

def benchmark(out,label):
    child=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'benchmark_current_serial_readonly_20261027.py'),'--read'],
        cwd=str(ROOT),capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=25)
    (out/(label+'.log')).write_text(child.stdout+'\n'+child.stderr,encoding='utf-8')
    rows=[json.loads(line) for line in child.stdout.splitlines() if line.startswith('{')]
    if not rows:raise RuntimeError('Benchmark did not save a readable result')
    folder=ROOT/rows[-1]['folder']
    if folder.parent!=ROOT or not folder.name.startswith('serial_benchmark_readonly_'):raise RuntimeError('Invalid benchmark folder')
    return json.loads((folder/'result.json').read_text(encoding='utf-8'))

def restart_one(entry,out,label):
    # Exact COM7 instance only, no /deviceid, wildcard, /force or /reboot.
    command=[str(Path(os.environ['WINDIR'])/'System32'/'pnputil.exe'),'/restart-device',device_instance(entry)]
    child=subprocess.run(command,capture_output=True,timeout=20)
    (out/(label+'.log')).write_bytes(child.stdout+b'\n'+child.stderr)
    if child.returncode!=0:raise RuntimeError('COM7 driver restart did not complete; reboot not requested')
    from serial.tools import list_ports
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        if any(p.device.upper()=='COM7' for p in list_ports.comports()):return
        time.sleep(.1)
    raise RuntimeError('COM7 did not return after its driver restart')

def run_admin(out):
    report=dict(status='started',motor_writes=0,motor_baud_changed=False,driver_registry_changed=False,
        rollback_required=False,baseline_file=str(BASELINE),actuator_motion_requested=False)
    original=None;change=None
    try:
        if not ctypes.windll.shell32.IsUserAnAdmin():raise RuntimeError('Windows administrator permission required')
        baseline=json.loads(BASELINE.read_text(encoding='utf-8'))
        # The setter independently rechecks all 16 torque-OFF states and baud registers.
        child=subprocess.run([sys.executable,'-X','utf8','-B',str(ROOT/'set_com7_usb_latency_20261027.py'),
            '--apply','--out',str(out)],cwd=str(ROOT),capture_output=True,timeout=20)
        (out/'setter.log').write_bytes(child.stdout+b'\n'+child.stderr)
        change=json.loads((out/'result.json').read_text(encoding='utf-8'));report['setter']=change
        original=change.get('original');report['driver_registry_changed']=bool(change.get('registry_writes'))
        if child.returncode!=0 or change['status']!='driver_registry_updated':raise RuntimeError(change.get('error','Driver adjustment failed'))
        data=benchmark(out,'after_reopen');report['after_reopen']=data
        if data['status']!='100_full_off_reads_complete':raise RuntimeError(data.get('error','Fresh OFF read failed; no driver restart attempted'))
        if not accepted(data,baseline):
            # This benchmark has just reconfirmed torque OFF on all16 before restart.
            report['exact_com7_restart_attempted']=True;restart_one(original,out,'com7_restart')
            data=benchmark(out,'after_restart');report['after_restart']=data
        if not accepted(data,baseline):raise RuntimeError('No verified latency benefit or full healthy read; restoring original driver value')
        report.update(status='usb_latency_improved',latency_timer_ms=1,median_before_ms=baseline['median_read_ms'],
            median_after_ms=data['median_read_ms'],p95_after_ms=data['p95_read_ms'],maximum_after_ms=data['maximum_read_ms'],
            read_frames=100,communication_failures=0,physical_loaded_voltage_verified=False)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped',error=str(exc))
        if original and report['driver_registry_changed']:
            try:
                with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,original['registry_path'],0,winreg.KEY_SET_VALUE) as key:
                    winreg.SetValueEx(key,'LatencyTimer',0,winreg.REG_DWORD,original['latency_timer_ms'])
                report['registry_restored_to_ms']=original['latency_timer_ms']
                # Never restart a port after an unhealthy/failed motor read.
                report['active_driver_restore_may_require_usb_reconnect']=True
            except OSError as restore:report.update(rollback_required=True,rollback_error=str(restore))
    finally:
        report['folder']=str(out);payload=json.dumps(report,indent=2)+'\n'
        (out/'workflow_result.json').write_text(payload,encoding='utf-8');LATEST.write_text(payload,encoding='utf-8')
    return 0 if report['status']=='usb_latency_improved' else 1

def elevate(out):
    from ctypes import wintypes as w
    class ExecuteInfo(ctypes.Structure):
        _fields_=[('cbSize',w.DWORD),('fMask',w.ULONG),('hwnd',w.HWND),('lpVerb',w.LPCWSTR),('lpFile',w.LPCWSTR),
            ('lpParameters',w.LPCWSTR),('lpDirectory',w.LPCWSTR),('nShow',ctypes.c_int),('hInstApp',w.HINSTANCE),
            ('lpIDList',ctypes.c_void_p),('lpClass',w.LPCWSTR),('hkeyClass',w.HKEY),('dwHotKey',w.DWORD),
            ('hIcon',w.HANDLE),('hProcess',w.HANDLE)]
    shell=ctypes.WinDLL('shell32',use_last_error=True);kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    shell.ShellExecuteExW.argtypes=[ctypes.POINTER(ExecuteInfo)];shell.ShellExecuteExW.restype=w.BOOL
    kernel.WaitForSingleObject.argtypes=[w.HANDLE,w.DWORD];kernel.WaitForSingleObject.restype=w.DWORD
    kernel.CloseHandle.argtypes=[w.HANDLE];kernel.CloseHandle.restype=w.BOOL
    info=ExecuteInfo();info.cbSize=ctypes.sizeof(info);info.fMask=0x40;info.lpVerb='runas';info.lpFile=sys.executable
    info.lpParameters=subprocess.list2cmdline(['-X','utf8','-B',str(Path(__file__).resolve()),'--apply-and-check','--out',str(out)])
    info.lpDirectory=str(ROOT);info.nShow=0
    if not shell.ShellExecuteExW(ctypes.byref(info)):
        code=ctypes.get_last_error();raise RuntimeError('Windows administrator request was cancelled' if code==1223 else f'Administrator launch failed: {code}')
    try:
        for seconds in range(120):
            if kernel.WaitForSingleObject(info.hProcess,1000)==0:break
            if seconds%10==9:print('Waiting for Windows admin approval / READ-only verification...',flush=True)
        else:raise RuntimeError('Administrator task still pending; do not launch motion. Its output folder is '+str(out))
    finally:kernel.CloseHandle(info.hProcess)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--request-admin',action='store_true');parser.add_argument('--apply-and-check',action='store_true');parser.add_argument('--out')
    args=parser.parse_args()
    if not args.request_admin and not args.apply_and_check:
        print('CHECK ONLY: no device/port opened; no registry change or UAC request.');return 0
    out=(Path(args.out).resolve() if args.out else ROOT/('usb_latency_change_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')))
    if out.parent!=ROOT or not out.name.startswith('usb_latency_change_'):raise RuntimeError('Output must stay in workspace')
    out.mkdir(exist_ok=True)
    if args.apply_and_check:return run_admin(out)
    print('USB LATENCY ONLY: close Dynamixel Wizard and other motor programs. Cube OUT. No hand motion.',flush=True)
    print('Approve Windows administrator dialog: COM7 USB latency16 ->1ms. Motor baud stays3Mbps.',flush=True)
    print('Original value saved;100 fresh PING/READ checks after change. COM7 driver may restart once; no PC reboot.',flush=True)
    try:
        elevate(out);result=json.loads((out/'workflow_result.json').read_text(encoding='utf-8'))
        print(json.dumps({k:v for k,v in result.items() if k not in ('setter','after_reopen','after_restart')},ensure_ascii=False),flush=True)
        return 0 if result['status']=='usb_latency_improved' else 1
    except Exception as exc:print('STOP:',exc,flush=True);print('Saved:',out);return 1
if __name__=='__main__':raise SystemExit(main())
