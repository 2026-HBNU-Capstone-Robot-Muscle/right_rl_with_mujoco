import os
import subprocess
from pathlib import Path
root=Path(__file__).resolve().parent
exe=Path(os.environ['LOCALAPPDATA'])/'GitHubDesktop/app-3.6.4/GitHubDesktop.exe'
repo=root/'github_desktop/right_rl_with_mujoco'
desktop_env=os.environ.copy()
had_node_mode=desktop_env.pop('ELECTRON_RUN_AS_NODE',None) is not None
print('CLEARED_INHERITED_ELECTRON_NODE_MODE',had_node_mode,flush=True)
try:
    r=subprocess.run([str(exe),'--cli-open='+str(repo)],env=desktop_env,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,timeout=20)
    print('DESKTOP_OPEN_REQUEST_EXIT',r.returncode,flush=True)
    if r.returncode:print('DESKTOP_ERROR',r.stderr.decode('utf-8',errors='replace')[:1200],flush=True)
except subprocess.TimeoutExpired:
    print('DESKTOP_OPEN_REQUEST_RUNNING',flush=True)
ps=Path(os.environ['SystemRoot'])/'System32/WindowsPowerShell/v1.0/powershell.exe'
for name in ['github_desktop_window_check_20261112.ps1','capture_github_desktop_window_20261112.ps1']:
    r=subprocess.run([str(ps),'-NoProfile','-ExecutionPolicy','Bypass','-File',str(root/name)],capture_output=True)
    print(r.stdout.decode('utf-8',errors='replace')[:6000],flush=True)
