"""Prepare a clean local review workspace for GitHub Desktop; never push."""
import json
import os
import shutil
import subprocess
import time
import zipfile
from pathlib import Path,PurePosixPath
from prepare_github_hardware_update_20261112 import ROOT,git,wsl_path

BASE=ROOT/'github_desktop'
REPO=BASE/'right_rl_with_mujoco'
URL='https://github.com/2026-HBNU-Capstone-Robot-Muscle/right_rl_with_mujoco.git'
ARCHIVE=ROOT/'full_project_delivery_20261112/RIGHT_HAND_FULL_GITHUB_UPLOAD_20261112.zip'

def main():
    BASE.mkdir(exist_ok=True)
    if not (REPO/'.git').is_dir():
        if REPO.exists() and any(REPO.iterdir()):raise RuntimeError('Preserve existing non-repository directory')
        r=subprocess.run(['wsl','--exec','/usr/bin/git','clone',URL,wsl_path(REPO)],capture_output=True,text=True,encoding='utf-8',errors='replace')
        if r.returncode:raise RuntimeError('Repository clone failed: '+r.stderr[-2000:])
    if git(REPO,'remote','get-url','origin')!=URL:raise RuntimeError('Existing checkout remote does not match requested repository')
    if git(REPO,'status','--porcelain'):raise RuntimeError('Preserve local uncommitted work; do not import automatically')
    git(REPO,'config','core.autocrlf','false')
    git(REPO,'config','core.longpaths','true')
    if git(REPO,'branch','--list','feature/hardware'):git(REPO,'switch','feature/hardware')
    elif git(REPO,'branch','--remotes','--list','origin/feature/hardware'):git(REPO,'switch','--track','-c','feature/hardware','origin/feature/hardware')
    else:git(REPO,'switch','-c','feature/hardware','origin/main')
    original_head=git(REPO,'rev-parse','HEAD');backup=BASE/'before_import';overwritten=[]
    count=0;bytes_written=0;last=time.monotonic()
    with zipfile.ZipFile(ARCHIVE) as z:
        entries=z.infolist()
        for entry in entries:
            name=PurePosixPath(entry.filename)
            if name.is_absolute() or '..' in name.parts or any(':' in part for part in name.parts):raise RuntimeError('Unsafe archive path')
            target=(REPO/Path(*name.parts)).resolve();target.relative_to(REPO.resolve())
            if entry.is_dir():target.mkdir(parents=True,exist_ok=True);continue
            if target.exists():
                old=backup/Path(*name.parts);old.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(target,old);overwritten.append(entry.filename)
            target.parent.mkdir(parents=True,exist_ok=True)
            with z.open(entry) as source,target.open('wb') as output:shutil.copyfileobj(source,output,1024*1024)
            count+=1;bytes_written+=entry.file_size
            if count%4000==0 or time.monotonic()-last>25:
                print(json.dumps(dict(imported=count,total=len(entries),written_MB=round(bytes_written/1e6))),flush=True);last=time.monotonic()
    if git(REPO,'rev-parse','HEAD')!=original_head:raise RuntimeError('Import must not create a commit')
    tracked=git(REPO,'diff','--name-only').splitlines()
    untracked=git(REPO,'ls-files','--others','--exclude-standard').splitlines()
    result=dict(repository_url=URL,local_path=str(REPO),branch=git(REPO,'branch','--show-current'),
                imported_files=count,imported_bytes=bytes_written,untracked_files=len(untracked),
                modified_tracked_files=tracked,overwritten_existing_files=overwritten,head_unchanged=original_head,
                remote_push_performed=False,commit_performed=False)
    (BASE/'DESKTOP_WORKSPACE.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
