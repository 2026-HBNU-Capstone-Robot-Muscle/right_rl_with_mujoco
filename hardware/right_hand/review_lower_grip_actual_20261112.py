"""Compact audit of the latest operator trial; no devices or settings writes."""
import argparse
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent

def brief(value,depth=0):
    if isinstance(value,list):return {'count':len(value),'first':brief(value[0],depth+1) if value and depth<2 else None,'last':brief(value[-1],depth+1) if value and depth<2 else None}
    if isinstance(value,dict):return {k:brief(v,depth+1) if isinstance(v,(list,dict)) else v for k,v in value.items() if depth<3}
    return value

def main():
    p=argparse.ArgumentParser();p.add_argument('--trial',default='lower_grip_trial_20261112_162806_610535');a=p.parse_args()
    trial=ROOT/a.trial;r=json.loads((trial/'result.json').read_bytes())
    scalars={k:v for k,v in r.items() if not isinstance(v,(dict,list))}
    summaries={k:brief(v) for k,v in r.items() if isinstance(v,(dict,list)) and k in ('samples','motion_samples','last_command','last_issued_goal','last_issued_command','pending','support_first_phases_seen')}
    print(json.dumps(dict(trial=a.trial,scalars=scalars,structured=summaries),ensure_ascii=False),flush=True)
    print(json.dumps(dict(saved_images=[str(p.relative_to(ROOT)) for p in trial.rglob('*.jpg')][:12],top_files=[p.name for p in trial.iterdir()]),ensure_ascii=False),flush=True)
    print(json.dumps(dict(lists={k:len(v) for k,v in r.items() if isinstance(v,list)}),ensure_ascii=False),flush=True)
    by_phase={}
    for s in r['samples']:
        phase=s['phase'];entry=by_phase.setdefault(phase,dict(first=s,last=s,count=0));entry['last']=s;entry['count']+=1
    focused={phase:dict(count=v['count'],start_t=v['first']['t'],end_t=v['last']['t'],last_goal=v['last']['goal'],last_actual=v['last']['actual'],
                       max_current_ma=max(abs(x) for s in r['samples'] if s['phase']==phase for x in s['current_ma'])) for phase,v in by_phase.items() if phase.startswith(('stage_17','stage_18','stage_19','stage_20','stage_21','stage_22','stage_23'))}
    out=ROOT/'right_hand_rotation_20261112/lower_grip_actual_review';out.mkdir(exist_ok=True)
    audit=dict(trial=a.trial,status=r['status'],error=r.get('primary_motion_error'),phases=focused,
               all_torque_off_verified=r.get('all_torque_off_verified'),settings_restored=r.get('settings_restored'),devices_opened=0)
    (out/'review.json').write_text(json.dumps(audit,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(dict(focused_phases=focused),ensure_ascii=False),flush=True)
    frames=json.loads((trial/'camera/frames.json').read_bytes())
    print(json.dumps(dict(frame_index_type=type(frames).__name__,frame_keys=list(frames)[:8] if isinstance(frames,dict) else None,
                          first_frame=frames[0] if isinstance(frames,list) and frames else None),ensure_ascii=False),flush=True)

if __name__=='__main__':main()
