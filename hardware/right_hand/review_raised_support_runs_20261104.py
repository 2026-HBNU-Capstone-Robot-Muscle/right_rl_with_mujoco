import json,hashlib
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261104/raised_support_capture/actual_review';O.mkdir(exist_ok=True);rows=[]
for folder in sorted(R.glob('raised_support_capture_20261104_*')):
 p=folder/'result.json'
 if not p.exists():continue
 d=json.loads(p.read_bytes());samples=d.get('samples',[]);measure={k:[] for k in ('voltage','current','temperature')}
 def visit(value):
  if isinstance(value,dict):
   for k,v in value.items():
    if k in measure and isinstance(v,list) and v and all(isinstance(x,(int,float)) for x in v):measure[k].extend(v)
    elif k not in ('write_events','off_configuration','restore_configuration','before_settings','after_settings'):visit(v)
  elif isinstance(value,list):
   for v in value:visit(v)
 visit(samples)
 phases={}
 for s in samples:
  if isinstance(s,dict):phases[s.get('phase','unknown')]=phases.get(s.get('phase','unknown'),0)+1
 row=dict(folder=folder.name,status=d.get('status'),error=d.get('error'),all_torque_off_verified=d.get('all_torque_off_verified'),settings_restored=d.get('settings_restored'),cube_placement_confirmed=d.get('cube_placement_confirmed'),cube_removal_confirmed=d.get('cube_removal_confirmed'),cube_present=d.get('cube_present'),camera=d.get('camera_recording'),sample_count=len(samples),phases=phases,extrema_raw_units={k:[min(v),max(v)] if v else None for k,v in measure.items()},photo_fields={k:v for k,v in d.items() if 'photo' in k and isinstance(v,dict)},top_keys=list(d),result_sha256=hashlib.sha256(p.read_bytes()).hexdigest());rows.append(row)
(O/'run_summary.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps([{k:v for k,v in x.items() if k not in ('top_keys','camera','photo_fields')} for x in rows],ensure_ascii=False));print('PHOTO_KEYS',json.dumps([{k:v for k,v in x.items() if k in ('folder','photo_fields','camera')} for x in rows],ensure_ascii=False))
