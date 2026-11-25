"""Read local source and timestamped records; do not infer creation time from mtime."""
import json,re,hashlib,ast
from pathlib import Path
R=Path(__file__).resolve().parent;O=R/'full_mapping_review_20261018';O.mkdir(exist_ok=True)
sources=[]
for p in [R.parent/'work/Leap_Test/deploy/leap_hw.py',Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API/python/main.py')]:
    if p.exists():
        lines=p.read_text(encoding='utf-8-sig',errors='replace').splitlines()
        hits=[dict(line=i,text=l) for i,l in enumerate(lines,1) if any(s in l for s in ('SIM2ARR','역순','손바닥 쪽부터','motors =','SIDE_MOTOR_IDS'))]
        sources.append(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),hits=hits))
history=[]
def walk(value,path=''):
    if isinstance(value,dict):
        if 'motor_id' in value and ('joint' in value or 'joint_name' in value):
            yield dict(path=path,row=value)
        for k,v in value.items():
            if k.lower() in ('sim2arr','sim_to_id','sim_to_motor_ids','joint_map','motor_mapping','mapping','root_ids','root_motor_ids'):
                yield dict(path=path+'/'+k,row=v)
            elif isinstance(v,(dict,list)):yield from walk(v,path+'/'+k)
    elif isinstance(value,list):
        for i,v in enumerate(value):
            if isinstance(v,(dict,list)):yield from walk(v,path+'/'+str(i))
for p in sorted(R.glob('*.json')):
    if p.stat().st_size>5_000_000:continue
    if not (p.name.startswith(('hand','leap','sac','motor','encoder'))):continue
    try:d=json.loads(p.read_text(encoding='utf-8-sig'))
    except (ValueError,OSError):continue
    if not isinstance(d,dict):continue
    hits=list(walk(d))
    if hits:history.append(dict(file=p.name,timestamp=d.get('timestamp'),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),hits=hits[:8]))
history.sort(key=lambda x:str(x['timestamp'] or x['file']))
code_refs=[]
for p in list(R.glob('*.py'))+list(R.glob('*.md')):
    if p.name.startswith(('audit_motor_mapping_history','record_confirmed','update_middle_order','inspect_')):continue
    lines=p.read_text(encoding='utf-8-sig',errors='replace').splitlines()
    hits=[dict(line=i,text=t[:250]) for i,t in enumerate(lines,1) if ('손바닥 쪽부터' in t or '실기 배선' in t or 'SIM_TO_ID=' in t or 'SIM2ARR =' in t or '매핑 확정' in t)]
    if hits:code_refs.append(dict(file=p.name,hits=hits[:8]))
repo_info=[]
for folder in [R,R.parent,R.parent/'work/Leap_Test',Path('C:/Users/김준서/Documents/GitHub/LEAP_Hand_API')]:
    repo_info.append(dict(path=str(folder),git_metadata_present=(folder/'.git').exists()))
report=dict(status='source_and_saved_record_history_not_commit_history',sources=sources,dated_records=history,code_references=code_refs,
    repositories=repo_info,limitations=['File mtimes are not used as evidence of introduction date.','Existing comments claim user confirmation; comments alone do not establish what was actually verified.'])
(O/'history.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(dict(sources=sources,earliest_records=history[:10],code_refs=code_refs,repo_info=repo_info),ensure_ascii=True,indent=2))
