"""Read only the already-executed routes and checkpoint rotations."""
import json
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle'))
import mujoco
from native_cycle import assets,restore
from cube_task_frame_20261105 import upright_yaw_target,orientation_error_deg
m,initial,_,_,_=assets('active_ccd50'); out={}
for name,path in [('first','right_hand_rotation_20261104/forward_hold/candidate.npz'),
                  ('stage2','right_hand_rotation_20261112/stage2_work_grip/candidate.npz')]:
    z=np.load(ROOT/path,allow_pickle=False); phases=z['phases']; rows=[]
    for p in dict.fromkeys(phases.tolist()):
        idx=np.flatnonzero(phases==p);rows.append(dict(phase=p,start=int(idx[0]),end=int(idx[-1]+1),steps=len(idx)))
    out[name]={'steps':len(z['goals']),'phases':rows,'start':z['start'].tolist(),'end':z['goals'][-1].tolist()}
folder=ROOT/'right_hand_rotation_20261112/nominal_cycle_actual_review/support_transfer_fix_v3/baseline'
out['nominal_prefix']={}
for i in (1417,1700,1800,1900,2000,2050,2100,2150,2188,2300,2400,2600,3351):
    d=restore(m,dict(np.load(folder/f'state_{i}.npz',allow_pickle=False)))
    out['nominal_prefix'][i]={'target90_error_deg':orientation_error_deg(d.geom_xmat[m.geom('cube').id].reshape(3,3),upright_yaw_target(initial['initial_cube_rotation'],initial['up'],90))}
first=np.load(ROOT/'right_hand_rotation_20261104/forward_hold/candidate.npz',allow_pickle=False)['goals']
current=np.load(ROOT/'right_hand_rotation_20261112/nominal_cycle_trial_preparation/rf16_2177_v2/candidate.npz',allow_pickle=False)['goals']
out['first1417_different_commands']=int(np.count_nonzero(np.any(first!=current[:len(first)],axis=1)))
print(json.dumps(out,ensure_ascii=False,indent=2))
