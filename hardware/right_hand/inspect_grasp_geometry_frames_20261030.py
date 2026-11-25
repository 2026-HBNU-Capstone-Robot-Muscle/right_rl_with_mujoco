from pathlib import Path
import json,numpy as np,mujoco
from hand_motor_mapping import ticks_to_sim
R=Path(__file__).resolve().parent;CP=R/'right_hand_rotation_20261030/lower_three_contact_grasp/initial_checkpoint';m=mujoco.MjModel.from_binary_path(str(CP/'open_cube_checkpoint.mjb'));s=np.load(CP/'open_cube_checkpoint.npz');d=mujoco.MjData(m);d.qpos[:]=s['qpos'];d.qvel[:]=0
src=np.load(R/'right_hand_rotation_20261030/single_forward/candidate.npz');q=src['goals'][np.flatnonzero(src['phases']=='stage_6_grasp_hold')[-1]];d.qpos[:16]=ticks_to_sim(q);mujoco.mj_forward(m,d)
print('UP',s['up'].tolist(),'CUBE_ROT',d.geom_xmat[m.geom('cube').id].reshape(3,3).tolist())
for name in ('cube','if_tip','mf_tip','rf_tip','th_tip'):
 g=m.geom(name).id;print(json.dumps(dict(name=name,type=int(m.geom_type[g]),size=m.geom_size[g].tolist(),position=d.geom_xpos[g].tolist(),rotation=d.geom_xmat[g].reshape(3,3).tolist(),mesh_id=int(m.geom_dataid[g]))))
