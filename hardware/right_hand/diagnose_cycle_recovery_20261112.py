"""Locate the saved empty-recovery collision, without new device access."""
import json
from pathlib import Path
import numpy as np
import mujoco
from hand_motor_mapping import ticks_to_sim
from screen_loaded_repeat_20261026 import cached_contacts
ROOT=Path(__file__).resolve().parent
folder=ROOT/'nominal_two_turn_trial_20261112_054129_121375/empty_recovery_screen'
q=dict(np.load(folder/'prefix_path.npz',allow_pickle=False))
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
destination=ROOT/'right_hand_rotation_20261112/nominal_cycle_actual_review/recovery_diagnostic_scene.xml'
m,_=build(env,destination,book_projection_mm=-10.);correct_model(m,q['start'])
mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand: hand.add(b)
supports={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
check=cached_contacts(m,hand,supports); d=mujoco.MjData(m); hits=[]
for offset in (np.zeros(16,dtype=int),np.full(16,4)):
    first=None; worst=None
    for i,command in enumerate(np.vstack([q['start'],q['goals']])+offset):
        d.qpos[:16]=ticks_to_sim(command);d.ctrl[:]=d.qpos[:16];d.qvel[:]=0;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
        _,own=check(d); value=max(own.values(),default=0.)
        if value>1e-6:
            row={'index':i,'phase':'initial' if i==0 else str(q['phases'][i-1]),'pairs_mm':own,'ticks':command.tolist()}
            if first is None:first=row
            if worst is None or value>max(worst['pairs_mm'].values()):worst=row
    hits.append({'offset':int(offset[0]),'first':first,'worst':worst})
out=ROOT/'right_hand_rotation_20261112/nominal_cycle_actual_review/recovery_collision_location.json'
out.write_text(json.dumps({'start':q['start'].tolist(),'rows':hits},indent=2)+'\n',encoding='utf-8')
print(json.dumps({'start':q['start'].tolist(),'rows':hits}))
