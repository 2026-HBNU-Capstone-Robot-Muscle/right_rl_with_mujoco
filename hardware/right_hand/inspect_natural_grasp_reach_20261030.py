from pathlib import Path
import json
import numpy as np,mujoco
from hand_motor_mapping import ticks_to_sim
R=Path(__file__).resolve().parent;B=R/'right_hand_rotation_20261030/stable_support_research';P=R/'right_hand_rotation_20261030/directional_contact_research/checkpoint_nominal';m=mujoco.MjModel.from_binary_path(str(P/'forward_checkpoint.mjb'));d=mujoco.MjData(m);s=np.load(B/'natural_recess_hypothesis_state.npz');src=np.load(R/'right_hand_rotation_20261030/single_forward/candidate.npz');d.qpos[:]=s['qpos'];d.ctrl[:]=s['ctrl'];mujoco.mj_forward(m,d);cube=m.geom('cube').id;Rcube=d.geom_xmat[cube].reshape(3,3);C=d.geom_xpos[cube];out=dict(C=C.tolist(),B=Rcube.tolist(),up=s['up'].tolist(),rows=[])
for i in (0,100,200,300,400,500,605,645):
 q=src['start'] if not i else src['goals'][i-1];d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d);out['rows'].append(dict(sample=i,tip_centers={name:d.geom_xpos[m.geom(name).id].tolist() for name in ('mf_tip','th_tip')},goal=q.tolist()))
(B/'grasp_reach_diagnostic.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8');print(json.dumps(out))
