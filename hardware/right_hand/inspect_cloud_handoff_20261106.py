"""Inspect project portability and save the latest run without reading credentials."""
import json,os,urllib.parse,configparser,time
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'cloud_handoff_20261106'
EXCLUDED={'.runtime','.git','.codex','.agents','__pycache__','node_modules','.venv','venv',O.name}
def main():
    O.mkdir(exist_ok=True);result={}
    pointer=R/'leap_middle_clearance_probe_result.json'
    if pointer.exists():
        raw=json.loads(pointer.read_bytes());folder=Path(raw.get('request_file',str(pointer))).parent
        result['last_robot_run']={k:raw.get(k) for k in ('status','error','failed_phase','all_torque_off_verified','settings_restored','middle_probe_photo','middle_probe_position_ticks','grasp_observation_photo','cube_removal_confirmed')};result['last_robot_run']['folder']=str(folder);result['last_robot_run']['review_pending']=True
        (O/'last_robot_run_summary.json').write_text(json.dumps(result['last_robot_run'],indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    repositories=[]
    for folder in (R,*R.parents):
        dot=folder/'.git'
        if not dot.exists():continue
        row=dict(root=str(folder),metadata_kind='directory' if dot.is_dir() else 'gitfile',remotes=[]);cfg=dot/'config'
        if cfg.is_file():
            p=configparser.ConfigParser(interpolation=None);p.read(cfg,encoding='utf-8')
            for section in p.sections():
                if not section.startswith('remote '):continue
                url=p.get(section,'url',fallback='');host=urllib.parse.urlsplit(url).hostname if '://' in url else (url.split('@')[-1].split(':')[0] if ':' in url else '')
                repo=urllib.parse.urlsplit(url).path.strip('/') if '://' in url else url.split(':',1)[-1]
                row['remotes'].append(dict(name=section,host=host,repository_path=repo))
        repositories.append(row)
    result['git_repositories']=repositories;files=0;total=0;groups={};largest=[];began=time.perf_counter()
    for folder,dirs,names in os.walk(R,followlinks=False):
        dirs[:]=[n for n in dirs if n not in EXCLUDED and not (Path(folder)/n).is_symlink()]
        for name in names:
            path=Path(folder)/name
            if path.is_symlink():continue
            try:size=path.stat().st_size
            except OSError:continue
            rel=path.relative_to(R);files+=1;total+=size;key=rel.parts[0] if len(rel.parts)>1 else '(root files)';g=groups.setdefault(key,dict(files=0,bytes=0));g['files']+=1;g['bytes']+=size
            if size>=10_000_000:largest.append(dict(path=rel.as_posix(),bytes=size))
    result.update(files_excluding_runtime=files,bytes_excluding_runtime=total,largest_files=sorted(largest,key=lambda r:r['bytes'],reverse=True)[:15],largest_groups=sorted([dict(path=k,**v) for k,v in groups.items()],key=lambda r:r['bytes'],reverse=True)[:15],root_python_files=len(list(R.glob('*.py'))),critical_binding_files=len(json.loads((R/'right_hand_rotation_20261106/middle_clearance_probe/ready.json').read_bytes())['code_hashes']),inventory_seconds=time.perf_counter()-began,credential_files_not_read=True)
    (O/'inspection.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8');print(json.dumps(result,ensure_ascii=False),flush=True)
if __name__=='__main__':main()
