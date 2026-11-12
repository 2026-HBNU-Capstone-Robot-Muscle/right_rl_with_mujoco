"""Saved-result inventory only. No devices or motor writes."""
import json
from pathlib import Path
R=Path(__file__).resolve().parent
for F in sorted(R.glob('yaw_roundtrip_trial_*'))[-2:]:
 p=F/'result.json'
 if not p.exists():print(F.name,'NO RESULT');continue
 r=json.loads(p.read_bytes());samples=r.get('samples',[]);blocks=[]
 for j,s in enumerate(samples):
  phase=s.get('phase')
  if not blocks or blocks[-1]['phase']!=phase:blocks.append(dict(phase=phase,first=j,count=0))
  blocks[-1]['count']+=1
 fields={k:v for k,v in r.items() if not isinstance(v,(dict,list)) and (any(t in k for t in ('status','stop','error','off','restor','cube','voltage','current','tracking','fail','blank','photo','verified')) or k=='enabled_at')}
 print(json.dumps(dict(folder=F.name,keys=list(r.keys()),fields=fields,blocks=blocks,photos=[p.name for p in F.glob('*.jpg')],last_image=sorted((F/'camera').glob('frame_*.jpg'))[-1].name if (F/'camera').exists() and list((F/'camera').glob('frame_*.jpg')) else None),ensure_ascii=False))
