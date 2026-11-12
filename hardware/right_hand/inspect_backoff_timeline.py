from pathlib import Path
import json
p=Path(__file__).resolve().parent/'leap_id12_backoff_empty_20261016_144922_807163.json'
d=json.loads(p.read_text());print('KEYS',list(d))
for k,v in d.items():
    if isinstance(v,list):
        print(k,len(v),'first',str(v[0])[:950] if v else '')
    elif isinstance(v,dict):
        print(k,list(v))
    else:print(k,str(v)[:150])
for k in ('approach_hold','backoff','return_hold'):
    v=d.get(k,{})
    print(k,{a:b for a,b in v.items() if not isinstance(b,(dict,list))})
for name in ('trace','samples','path_samples'):
    rows=d.get(name,[])
    if not rows:continue
    groups={}
    for row in rows:
        phase=str(row.get('stage',row.get('phase','unknown')))
        groups.setdefault(phase,[]).append(row)
    for phase,rr in groups.items():
        print(name,phase,len(rr),{k:v for k,v in rr[0].items() if k in ('t','time','elapsed','elapsed_s','phase','stage')},
              {k:v for k,v in rr[-1].items() if k in ('t','time','elapsed','elapsed_s','phase','stage')})
