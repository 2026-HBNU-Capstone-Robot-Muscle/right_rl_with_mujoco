"""Read current COM7 FTDI driver settings and access permissions only."""
import json,os,winreg
from pathlib import Path
from benchmark_current_serial_readonly_20261027 import inspect_usb_latency
report=inspect_usb_latency()
report['system_d2xx_dll_exists']=(Path(os.environ['WINDIR'])/'System32'/'ftd2xx.dll').exists()
for entry in report['matches']:
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,entry['registry_path'],0,winreg.KEY_SET_VALUE):pass
        entry['can_set_latency']=True
    except OSError as exc:entry.update(can_set_latency=False,permission_error=str(exc))
    entry.pop('registry_path',None)
print(json.dumps(report,ensure_ascii=False))
