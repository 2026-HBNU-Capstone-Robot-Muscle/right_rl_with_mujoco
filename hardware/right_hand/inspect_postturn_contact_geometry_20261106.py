"""Saved contact normals and position-control directions, no motion."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json
import numpy as np,mujoco
from dynamic_common_grip_search_20261105 import ROOT,R
from research_rotation_body_hold_20261104 import restore
from hand_motor_mapping import SIM_TO_MOTOR_IDS

def main():
    p=ROOT/'thumb_global_workspace/p12_seeds6';r=json.loads((ROOT/'near_quarter_full_replay/review.json').read_bytes());m=mujoco.MjModel.from_binary_path(str(R/r['source_model']));cube=m.geom('cube').id;results=[]
    for label,source in [('open',ROOT/'postturn_thumb_clearance/id1_delta-64/end_state.npz'),('thumb_reached',p/'zpos_3/force1_end.npz')]:
        d=restore(m,np.load(source));C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);contacts=[]
        for ci,c in enumerate(d.contact):
            a,b=int(c.geom1),int(c.geom2)
            if cube not in (a,b):continue
            g=b if a==cube else a;normal=c.frame[:3]*(1 if a==cube else -1);f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,c.pos,int(m.geom_bodyid[g]));v=normal@jp[:,:16];contacts.append(dict(geom=m.geom(g).name,normal_force_N=float(f[0]),cube_to_hand_normal_world=normal.tolist(),normal_cube_local=(B.T@normal).tolist(),point_cube_local_mm=((c.pos-C)@B*1000).tolist(),normal_motion_mm_per_rad_by_motor={str(SIM_TO_MOTOR_IDS[i]):float(v[i]*1000) for i in range(16) if abs(v[i])>1e-7}))
        results.append(dict(label=label,cube_center_m=C.tolist(),cube_rotation=B.tolist(),contacts=contacts));print(json.dumps(results[-1]),flush=True)
    (p/'contact_geometry.json').write_text(json.dumps(dict(cases=results,devices_opened=False,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
