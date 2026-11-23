"""Read-only model diagnosis of the optional proximity projection."""
from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np,mujoco
from feedback_opposed_regrasp_20261117 import ApproachPlanner,SOURCE
from audit_opposed_working_grip_20261117 import MODEL
from research_rotation_body_hold_20261104 import restore,native,motor
from hand_motor_mapping import TICK,SIM_TO_MOTOR_IDS
m=mujoco.MjModel.from_binary_path('model.mjb',assets={'model.mjb':MODEL.read_bytes()});d=restore(m,np.load(SOURCE,allow_pickle=False))
lo,hi,_,_=native(m);start=motor(d.ctrl);planner=ApproachPlanner(start,np.minimum(lo+8,start),np.maximum(hi-8,start),m,d)
planner.freeze_index_base=True;planner.freeze_middle_base=True
lower,upper=planner.increment_bounds();ids=np.asarray(SIM_TO_MOTOR_IDS)-1;rows=[]
pairs=[(m.geom(g).name,'th_tip') for g in range(m.ngeom) if (m.geom(g).name or '').startswith('palm_collision_')]+[('if_md_collision_5','mf_px_collision')]
for na,nb in pairs:
    a,b=m.geom(na).id,m.geom(nb).id;p=np.zeros(6);dist=float(mujoco.mj_geomDistance(m,d,a,b,.003,p))
    if not 1e-9<dist<.002:continue
    n=(p[3:]-p[:3])/dist;ja,jb=np.zeros((3,m.nv)),np.zeros((3,m.nv))
    mujoco.mj_jac(m,d,ja,None,p[:3],int(m.geom_bodyid[a]));mujoco.mj_jac(m,d,jb,None,p[3:],int(m.geom_bodyid[b]))
    row=n@(jb-ja);g=np.zeros(16);g[ids]=row[:16]*TICK
    wanted=.0002+max(0.,-float(row@d.qvel)*.075)-dist
    maximum=float(g@np.where(g>=0,upper,lower))
    rows.append(dict(pair=[na,nb],distance_mm=dist*1000,wanted_increment_mm=wanted*1000,
        maximum_increment_with_base_fixed_mm=maximum*1000,gradient_mm_per_tick=(g*1000).tolist(),
        masks=[[int(m.geom_contype[x]),int(m.geom_conaffinity[x])] for x in (a,b)]))
print(json.dumps(rows),flush=True)
