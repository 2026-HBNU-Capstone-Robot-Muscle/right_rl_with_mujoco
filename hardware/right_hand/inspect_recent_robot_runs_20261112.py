"""Read recent saved robot runs only; no device access."""
import json
from datetime import datetime
from pathlib import Path

ROOT=Path(__file__).resolve().parent
progress=ROOT/'professor_progress_20261015.md'
if progress.is_file() and False:
    content=progress.read_text(encoding='utf-8-sig')
    print('LATEST_PROGRESS_PREFIX\n'+content[:10000],flush=True)
results=[]
for folder in ROOT.iterdir():
    if not folder.is_dir():continue
    result=folder/'result.json'
    if result.is_file():results.append((result.stat().st_mtime,result))
for stamp,path in sorted(results,reverse=True)[:5]:
    data=json.loads(path.read_text(encoding='utf-8-sig'))
    print('\nRUN',path.parent.name,datetime.fromtimestamp(stamp).isoformat(),flush=True)
    selected={k:data.get(k) for k in ['timestamp','status','plan_version','error','failed_phase','primary_motion_error','last_issued_phase','all_torque_off_verified','settings_restored','physical_two_turn_success','samples','camera_recording'] if k in data}
    if not isinstance(selected.get('samples'),(str,int,float,bool,type(None))):selected['samples_type']=type(selected.pop('samples')).__name__
    print(json.dumps(selected,ensure_ascii=False)[:2500],flush=True)
