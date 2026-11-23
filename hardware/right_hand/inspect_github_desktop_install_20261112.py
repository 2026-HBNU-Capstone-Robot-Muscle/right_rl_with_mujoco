"""Read GitHub Desktop install/protocol metadata only; do not inspect accounts."""
import json
import os
from pathlib import Path
import winreg

out={'protocols':[],'known_paths':[],'installations':[]}
for hive,label in ((winreg.HKEY_CURRENT_USER,'HKCU'),(winreg.HKEY_LOCAL_MACHINE,'HKLM')):
    for scheme in ('x-github-client','github-windows'):
        key='Software\\Classes\\'+scheme+'\\shell\\open\\command'
        try:
            with winreg.OpenKey(hive,key) as h:command=winreg.QueryValueEx(h,'')[0]
            out['protocols'].append(dict(hive=label,scheme=scheme,command=command))
        except OSError:pass
    for root in ('Software\\Microsoft\\Windows\\CurrentVersion\\Uninstall','Software\\WOW6432Node\\Microsoft\\Windows\\CurrentVersion\\Uninstall'):
        try:
            with winreg.OpenKey(hive,root) as base:
                for i in range(winreg.QueryInfoKey(base)[0]):
                    name=winreg.EnumKey(base,i)
                    try:
                        with winreg.OpenKey(base,name) as sub:
                            display=winreg.QueryValueEx(sub,'DisplayName')[0]
                            if 'github desktop' not in display.lower():continue
                            row={'display_name':display}
                            for field in ('InstallLocation','DisplayIcon','DisplayVersion'):
                                try:row[field]=winreg.QueryValueEx(sub,field)[0]
                                except OSError:pass
                            out['installations'].append(row)
                    except OSError:pass
        except OSError:pass
for root in (Path(os.environ['LOCALAPPDATA']),Path(os.environ.get('PROGRAMFILES','C:/Program Files')),Path(os.environ.get('PROGRAMFILES(X86)','C:/Program Files (x86)'))):
    if not root.is_dir():continue
    for name in ('GitHubDesktop','GitHub Desktop'):
        folder=root/name
        try:
            if folder.exists():out['known_paths'].append(dict(path=str(folder),children=[p.name for p in folder.iterdir()][:15]))
        except PermissionError:out['known_paths'].append(dict(path=str(folder),access_denied=True))
print(json.dumps(out,ensure_ascii=False))
