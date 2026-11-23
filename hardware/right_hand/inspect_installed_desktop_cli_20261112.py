import os
import subprocess
from pathlib import Path
base=Path(os.environ['LOCALAPPDATA'])/'GitHubDesktop'
for name in ['bin/github.cmd','bin/github']:
    p=base/name
    if p.is_file():print(name,p.read_text(encoding='utf-8',errors='replace')[:7000],flush=True)
ps=Path(os.environ['SystemRoot'])/'System32/WindowsPowerShell/v1.0/powershell.exe'
r=subprocess.run([str(ps),'-NoProfile','-Command',"Get-Process -Name GitHubDesktop | Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object Id,Path,MainWindowTitle | ConvertTo-Json -Compress"],capture_output=True)
print(r.stdout.decode('utf-8',errors='replace'),flush=True)
resources=base/'app-3.6.4/resources'
print('RESOURCE_PATHS',*[str(p.relative_to(base)) for p in resources.glob('*')],sep='\n')
app=resources/'app'
print('APP_PATHS',*[str(p.relative_to(app)) for p in app.glob('*')],sep='\n')
for name in ['static/github.sh','cli.js','main.js']:
    p=app/name
    if not p.is_file():continue
    content=p.read_text(encoding='utf-8',errors='replace')
    if name.endswith('.sh'):print('INSTALLED_SHELL',content[:5000])
    else:
        for needle in ['cli-open','cli-clone','open-repo','add-repo']:
            pos=content.find(needle)
            if pos>=0:print(name,needle,content[max(0,pos-180):pos+450])
