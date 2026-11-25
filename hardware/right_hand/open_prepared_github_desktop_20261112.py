"""Verify the prepared import and open it in the user's GitHub Desktop. No commit or push."""
import hashlib
import json
import os
import subprocess
import zipfile
from pathlib import Path
from prepare_github_hardware_update_20261112 import ROOT,git

repo=ROOT/'github_desktop/right_rl_with_mujoco'
archive=ROOT/'full_project_delivery_20261112/RIGHT_HAND_FULL_GITHUB_UPLOAD_20261112.zip'
status=repo.parent/'DESKTOP_WORKSPACE.json'
assert git(repo,'remote','get-url','origin')=='https://github.com/2026-HBNU-Capstone-Robot-Muscle/right_rl_with_mujoco.git'
assert git(repo,'branch','--show-current')=='feature/hardware'
before_head=git(repo,'rev-parse','HEAD')
missing=[];total=0;size=0
with zipfile.ZipFile(archive) as z:
    for entry in z.infolist():
        if entry.is_dir():continue
        p=repo/entry.filename
        if not p.is_file() or p.stat().st_size!=entry.file_size:missing.append(entry.filename)
        total+=1;size+=entry.file_size
    for name in ['hardware/right_hand/tip_first_transfer_20261112.py','hardware/right_hand/right_hand_rotation_20261112/tip_first_trial_v2/candidate.npz','hardware/right_hand/.gitattributes']:
        assert hashlib.sha256((repo/name).read_bytes()).digest()==hashlib.sha256(z.read(name)).digest(),name
if missing:raise RuntimeError('Import is incomplete: '+repr(missing[:10]))
tracked=git(repo,'diff','--name-only').splitlines()
assert not any(p.startswith('apps/monitoring_ui/') for p in tracked)
untracked=git(repo,'ls-files','--others','--exclude-standard').splitlines()
assert git(repo,'rev-parse','HEAD')==before_head
result=dict(repository_url=git(repo,'remote','get-url','origin'),local_path=str(repo),branch='feature/hardware',imported_files=total,imported_bytes=size,untracked_files=len(untracked),modified_tracked_files=tracked,head=before_head,remote_push_performed=False,commit_performed=False,import_file_sizes_verified=True)
status.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False),flush=True)
exe=Path(os.environ['LOCALAPPDATA'])/'GitHubDesktop/app-3.6.4/GitHubDesktop.exe'
assert exe.is_file()
process=subprocess.Popen([str(exe),'--cli-open='+str(repo)],stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,creationflags=subprocess.DETACHED_PROCESS|subprocess.CREATE_NEW_PROCESS_GROUP,close_fds=True)
print('DESKTOP_LAUNCH_REQUESTED',process.pid,flush=True)
ps=Path(os.environ['SystemRoot'])/'System32/WindowsPowerShell/v1.0/powershell.exe'
check=subprocess.run([str(ps),'-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'github_desktop_window_check_20261112.ps1')],capture_output=True)
print(check.stdout.decode('utf-8',errors='replace'),flush=True)
if check.returncode:print('WINDOW_CHECK_ERROR',check.stderr.decode('utf-8',errors='replace')[-1600:],flush=True)
