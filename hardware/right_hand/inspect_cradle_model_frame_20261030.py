"""Read existing model axes and candidate geometry only."""
import json
from pathlib import Path
import numpy as np,mujoco
R=Path(__file__).resolve().parent;CP=R/'right_hand_rotation_20261030/directional_contact_research/checkpoint_nominal';m=mujoco.MjModel.from_binary_path(str(CP/'forward_checkpoint.mjb'));d=mujoco.MjData(m);z=np.load(R/'right_hand_rotation_20261030/single_forward/candidate.npz');s=np.load(CP/'forward_checkpoint.npz');d.qpos[:]=s['qpos'];d.qpos[:16]=(z['start'][np.array([8,6,7,5,12,10,11,9,16,14,15,13,4,3,2,1])-1]-2048)*2*np.pi/4096;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
body=m.body('palm').id;out=dict(palm_position=d.xpos[body].tolist(),palm_rotation=d.xmat[body].reshape(3,3).tolist(),up_world=s['up'].tolist(),gravity=m.opt.gravity.tolist(),tip_centers={n:d.geom_xpos[m.geom(n).id].tolist() for n in ('if_tip','mf_tip','rf_tip','th_tip')},body_root_positions={n:d.xpos[m.body(n).id].tolist() for n in ('if_bs','mf_bs','rf_bs','th_mp')},no_hardware=True)
(R/'right_hand_rotation_20261030/stable_support_research/model_frame.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8');print(json.dumps(out))
