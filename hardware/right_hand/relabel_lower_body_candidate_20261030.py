"""Apply descriptive phases without changing any goals or recomputing IK."""
import json,hashlib
from pathlib import Path
import numpy as np
def blend(a,b):
 n=max(1,int(np.ceil(max(abs(b-a))/2)));return np.rint([a+(b-a)*k/n for k in range(1,n+1)]).astype(int)
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261030/lower_body_regrasp';p=O/'candidate.npz';z=np.load(p);report=json.loads((O/'geometry_review.json').read_bytes());end=int(z['forward_end_sample']);previous=z['goals'][end-1];phases=z['phases'][:end].tolist();count=0
for row in report['records']:
 q=np.asarray(row['goal']);v=blend(previous,q);previous=q;phase={'initial':'stage_11_body_release','release':'stage_11_body_release','lower_thumb':'stage_12_body_reseat','reclose':'stage_13_body_close'}[row['phase']];phases.extend([phase]*len(v));count+=len(v)
phases.extend(['stage_14_body_hold']*107+['stage_9_loaded_open']*(len(z['goals'])-end-count-107-20)+['stage_10_remove_hold']*20);assert len(phases)==len(z['goals']);goals=z['goals'].copy();start=z['start'].copy();np.savez_compressed(p,start=start,goals=goals,phases=np.array(phases),forward_end_sample=np.array(end),body_hold_end_sample=np.array(end+count+107));report['candidate_sha256']=hashlib.sha256(p.read_bytes()).hexdigest();report['body_hold_end_sample']=end+count+107;(O/'geometry_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print('Relabelled with exact goals preserved:',len(goals),count)
