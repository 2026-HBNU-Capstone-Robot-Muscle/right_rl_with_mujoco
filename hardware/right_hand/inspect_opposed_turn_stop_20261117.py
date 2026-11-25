from pathlib import Path
import sys, json, argparse
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
import numpy as np, mujoco
from research_rotation_body_hold_20261104 import restore,inspect_for,native
from hand_motor_mapping import SIM_TO_MOTOR_IDS, TICK
from audit_opposed_working_grip_20261117 import MODEL

m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()})
ap=argparse.ArgumentParser();ap.add_argument('--parent',default='next_turn5p0');args=ap.parse_args()
source=ROOT/'right_hand_rotation_20261117/opposed_lower_grip'/args.parent/'end_state.npz'
d=restore(m,np.load(source,allow_pickle=False))
hand,_,check=inspect_for(m)
su,sp=check(d)
rows=[]
for c in d.contact:
    a,b=int(c.geom1),int(c.geom2)
    if int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:
        rows.append(dict(pair=[m.geom(a).name,m.geom(b).name],distance_mm=float(c.dist*1000)))
lo,hi,nlo,nhi=native(m)
joints=[dict(motor_id=SIM_TO_MOTOR_IDS[i],actual_ticks=float(d.qpos[i]/TICK+2048),
             goal_ticks=float(d.ctrl[i]/TICK+2048),lower_ticks=float(nlo[i]/TICK+2048),
             upper_ticks=float(nhi[i]/TICK+2048)) for i in range(16)
        if min(d.qpos[i]-nlo[i],nhi[i]-d.qpos[i]) < 8*TICK]
out=dict(self_collisions=sp,support_collisions=su,hand_contacts=rows,joints_near_bounds=joints)
print(json.dumps(out),flush=True)
