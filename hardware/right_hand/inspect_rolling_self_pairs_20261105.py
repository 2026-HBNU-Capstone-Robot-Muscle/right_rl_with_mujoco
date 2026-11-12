"""Identify the actual saved simulated self-contact; no device access."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT
from research_rotation_body_hold_20261104 import restore,inspect_for
p=ROOT/'full_placement_v3_thumb12/front-10/g1_f0.3';m=mujoco.MjModel.from_binary_path(str(p/'model.mjb'));hand,_,check=inspect_for(m);rows=[]
for tag in ('rolling_depth1_fractional','rolling_depth1_integer_restart','tangential_depth1_fractional'):
    z=np.load(ROOT/'rolling_feedback'/tag/'end_state.npz');d=restore(m,z);_,sp=check(d);contacts=[]
    for c in d.contact:
        a,b=int(c.geom1),int(c.geom2)
        if int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand and c.dist<0:contacts.append(dict(geoms=[m.geom(a).name,m.geom(b).name],penetration_mm=-float(c.dist)*1000))
    rows.append(dict(case=tag,checked_self_pairs=sp,negative_hand_contacts=contacts))
(ROOT/'rolling_feedback/self_pairs.json').write_text(json.dumps(rows,indent=2)+'\n',encoding='utf-8');print(json.dumps(rows),flush=True)
