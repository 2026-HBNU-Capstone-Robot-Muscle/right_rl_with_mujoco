import json
from pathlib import Path
from prepare_github_hardware_update_20261112 import ROOT,git

base=ROOT/'github_desktop'
repo=base/'right_rl_with_mujoco'
status=base/'DESKTOP_WORKSPACE.json'
print('completion_record:',status.read_text(encoding='utf-8') if status.exists() else 'not_yet_written',flush=True)
try:
    import psutil
    for p in psutil.process_iter(['pid','name','cmdline']):
        info=p.info
        args=info.get('cmdline') or []
        if any('prepare_github_desktop_workspace_20261112.py' in s for s in args):
            print('preparation_process:',info['pid'],info['name'],flush=True)
except ImportError:
    print('process_inspection: psutil unavailable',flush=True)
if (repo/'.git').is_dir():
    for label,args in [('origin',('remote','get-url','origin')),('branch',('branch','--show-current')),('head',('rev-parse','HEAD')),('tracked_changes',('diff','--name-only'))]:
        print(label,git(repo,*args),flush=True)
    print('imported_hardware_files',sum(1 for p in (repo/'hardware/right_hand').rglob('*') if p.is_file()),flush=True)
