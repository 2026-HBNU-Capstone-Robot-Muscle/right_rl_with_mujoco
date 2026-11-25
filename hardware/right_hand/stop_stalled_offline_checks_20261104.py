"""Stop only this task's device-free GPU-startup processes before CPU restart."""
import os,signal,json
from pathlib import Path
names=('screen_yaw_roundtrip_model_20261030.py','screen_forward_hold_hand_20261104.py','screen_forward_hold_prefix_only_20261104.py','diagnose_forward_hold_startup_20261104.py')
stopped=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    pid=int(p.name)
    if pid==os.getpid():continue
    try:
        args=(p/'cmdline').read_bytes().split(b'\0')
        if not args or not any(b'venv/bin/python' in a for a in args[:1]):continue
        if not any(Path(os.fsdecode(a)).name in names for a in args[1:] if a):continue
        env=(p/'environ').read_bytes().split(b'\0')
        if b'JAX_PLATFORMS=cpu' in env:continue
        os.kill(pid,signal.SIGTERM);stopped.append(pid)
    except (OSError,ProcessLookupError):continue
print(json.dumps({'stopped_task_offline_processes':stopped,'physical_devices_accessed':False}))
