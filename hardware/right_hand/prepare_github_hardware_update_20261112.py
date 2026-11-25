"""Prepare selected code/evidence backups and a small public team review package.

No credentials, runtime installations, raw videos or camera-frame directories
are copied. Git credentials are delegated to the existing gh credential helper.
"""
import ast
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

ROOT=Path(__file__).resolve().parent
PRIVATE=ROOT/'cloud_handoff_20261106/upload_repo'
TEAM=ROOT/'team_integration_20261112/right_rl_with_mujoco'
WORK=ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle'
OUT=ROOT/'team_integration_20261112'

def wsl_path(path):return '/mnt/c/'+str(path.resolve()).replace('\\','/')[3:]
def git(repo,*args):
    helper="!'"+wsl_path(ROOT/'.runtime/github-cli/gh.exe')+"' auth git-credential"
    command=['wsl','--exec','/usr/bin/git','-C',wsl_path(repo),'-c','core.quotePath=false','-c','credential.helper=',
             '-c','credential.helper='+helper,'-c','user.name=Codex','-c','user.email=codex@local.invalid',*args]
    r=subprocess.run(command,capture_output=True,text=True,encoding='utf-8',errors='replace')
    if r.returncode:raise RuntimeError('git '+args[0]+' failed: '+r.stderr[-2000:])
    return r.stdout.strip()

def main():
    chosen=set();queue=[]
    def include(path):
        path=Path(path).resolve()
        try:relative=path.relative_to(ROOT)
        except ValueError:return
        if not path.is_file() or path in chosen:return
        if any(part in ('.runtime','.git','.aws','.codex','.agents','__pycache__','camera') for part in relative.parts):return
        if any(part.startswith(('lower_grip_trial_20','nominal_two_turn_trial_20','support_first_trial_20')) for part in relative.parts):return
        if path.suffix.lower() not in ('.py','.cmd','.json','.npz','.mjb','.md','.txt','.sh','.xml','.stl','.obj'):return
        chosen.add(path);queue.append(path)
    for pattern in ('*lower_grip*20261112.py','*tip_first*20261112.py','*support_first*20261112.py','*three_finger*20261112.py','*low_friction*20261112.py','*thumb_under_ring*20261112.py'):
        for path in ROOT.glob(pattern):include(path)
    for name in ('leap_cycle_lower_grip.cmd','tip_first_transfer_20261112.py','cloud_handoff_20261106/CLOUD_START_PROMPT.md','professor_progress_20261015.md'):
        include(ROOT/name)
    for folder in ('lower_grip_trial','lower_grip_actual_review','tip_first_transfer_v1','tip_first_transfer_v2'):
        for path in (ROOT/'right_hand_rotation_20261112'/folder).rglob('*'):include(path)
    def referenced(value,parent):
        if isinstance(value,dict):
            for k,v in value.items():
                if isinstance(k,str) and '/' in k:
                    for base in (ROOT,WORK,parent):
                        p=base/k
                        if p.is_file():include(p);break
                referenced(v,parent)
        elif isinstance(value,list):
            for v in value:referenced(v,parent)
        elif isinstance(value,str) and len(value)<500 and ('/' in value or '\\' in value):
            for base in (ROOT,WORK,parent):
                p=base/value
                if p.is_file():include(p);break
    while queue:
        path=queue.pop()
        if path.suffix=='.json':
            try:referenced(json.loads(path.read_bytes()),path.parent)
            except (ValueError,UnicodeError):pass
        elif path.suffix=='.py':
            try:tree=ast.parse(path.read_text(encoding='utf-8-sig'))
            except (SyntaxError,UnicodeError):continue
            for node in ast.walk(tree):
                names=[n.name for n in node.names] if isinstance(node,ast.Import) else ([node.module] if isinstance(node,ast.ImportFrom) and node.module else [])
                for name in names:
                    for base in (path.parent,ROOT,WORK):
                        p=base/(name.replace('.','/')+'.py')
                        if p.is_file():include(p);break
    changed=[];backup=OUT/'private_preupdate_backups';bindings={}
    for path in sorted(chosen):
        rel=path.relative_to(ROOT);target=PRIVATE/rel;data=path.read_bytes();digest=hashlib.sha256(data).hexdigest();bindings[rel.as_posix()]=digest
        if target.exists() and target.read_bytes()==data:continue
        if target.exists():
            old=backup/rel
            if not old.exists():old.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(target,old)
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data);changed.append(rel.as_posix())
    manifest=PRIVATE/'cloud/hardware_update_manifest_20261112.json'
    manifest.write_text(json.dumps(dict(kind='selected_hardware_code_and_model_update',files=bindings,changed_files=changed,
        excludes=['credentials','installed runtimes','raw camera frames','raw videos','raw conversations'],
        full_backup=False,full_rotation_cycle_complete=False),indent=2,ensure_ascii=False),encoding='utf-8')
    current=PRIVATE/'CURRENT_STATE.md';old=current.read_text(encoding='utf-8-sig')
    marker='hardware_update_20261112'
    if marker not in old:current.write_text('# Latest hardware development update\n\n<!-- '+marker+' -->\nRead `cloud_handoff_20261106/CLOUD_START_PROMPT.md` first. Latest actual lower-grip run stopped on ID15 tracking. Distal-first thumb/middle repair is under offline review; the full rotation/hold/next-rotation cycle is not complete. `cloud/hardware_update_manifest_20261112.json` records the selected additions.\n\n'+old,encoding='utf-8')
    if not TEAM.exists():
        TEAM.parent.mkdir(exist_ok=True)
        r=subprocess.run(['wsl','--exec','/usr/bin/git','clone','https://github.com/2026-HBNU-Capstone-Robot-Muscle/right_rl_with_mujoco.git',wsl_path(TEAM)],capture_output=True,text=True,encoding='utf-8',errors='replace')
        if r.returncode:raise RuntimeError('Team clone failed: '+r.stderr[-2000:])
    if git(TEAM,'status','--porcelain'):raise RuntimeError('Preserve pre-existing team checkout changes')
    git(TEAM,'checkout','-b','feature/hardware')
    dest=TEAM/'hardware/right_hand';dest.mkdir(parents=True,exist_ok=True)
    for name in ('tip_first_transfer_20261112.py','test_tip_first_transfer_20261112.py'):shutil.copyfile(ROOT/name,dest/name)
    shutil.copyfile(OUT/'README.md',dest/'README.md')
    receipt=dict(private_selected_files=len(chosen),private_selected_bytes=sum(p.stat().st_size for p in chosen),
        private_changed_files=len(changed),private_manifest=manifest.relative_to(ROOT).as_posix(),
        team_files=['hardware/right_hand/'+n for n in ('tip_first_transfer_20261112.py','test_tip_first_transfer_20261112.py','README.md')],
        public_raw_photos_or_logs_included=False,apps_monitoring_ui_modified=False,remote_push_done=False)
    (OUT/'prepared_update.json').write_text(json.dumps(receipt,indent=2,ensure_ascii=False),encoding='utf-8');print(json.dumps(receipt,ensure_ascii=False))

if __name__=='__main__':main()
