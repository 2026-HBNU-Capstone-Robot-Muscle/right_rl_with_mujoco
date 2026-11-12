"""Compare the assumed contact directions against the actual CAD contacts."""
import os
os.environ['JAX_PLATFORMS']='cpu'
import json
from pathlib import Path
import numpy as np,mujoco
from research_rotation_body_hold_20261104 import restore
from hand_motor_mapping import ticks_to_sim
R=Path(__file__).resolve().parent;O=R/'right_hand_rotation_20261105/canonical_body_grip';CP=O.parent/'preintegration/reference_end'

def main():
    m=mujoco.MjModel.from_binary_path(str(CP/'end_model.mjb'));z=np.load(CP/'end_state.npz');base=restore(m,z);cube=m.geom('cube').id;C0=base.geom_xpos[cube].copy();B0=base.geom_xmat[cube].reshape(3,3).copy();pd=base.ctrl-base.qpos[:16];original={}
    for name in ('mf_tip','th_tip'):
        g=m.geom(name).id;c=min((c for c in base.contact if {int(c.geom1),int(c.geom2)}=={cube,g}),key=lambda c:c.dist);n=c.frame[:3]*(1 if int(c.geom1)==cube else -1);G=base.geom_xmat[g].reshape(3,3);original[name]=(G.T@n,np.sign((B0.T@(c.pos-C0))[0]))
    ep=np.load(O/'supported_endpoint/lift5_root64/endpoint.npz');p=restore(m,z);p.qpos[:16]=ticks_to_sim(ep['goal'])-pd
    from scipy.spatial.transform import Rotation
    q=Rotation.from_matrix(ep['cube_rotation']).as_quat();p.qpos[-7:]=np.r_[ep['cube_center'],q[3],q[:3]];p.ctrl[:]=ticks_to_sim(ep['goal']);mujoco.mj_forward(m,p)
    states=[('reference_end',base),('pinned_proposal',p),('free_end',restore(m,np.load(O/'easy_suffix/supported_level_near_reference/end_g1_f0.3.npz'))),('lowered_stable_yaw',restore(m,np.load(O.parent/'preintegration/body_grip/lower_m20_t16_n10/end_state.npz')))]
    result=[]
    for label,d in states:
        C=d.geom_xpos[cube];B=d.geom_xmat[cube].reshape(3,3);contacts=[]
        for name,(local,sign) in original.items():
            g=m.geom(name).id;normal=d.geom_xmat[g].reshape(3,3)@local;angle=float(np.degrees(np.arccos(np.clip(normal@(B[:,0]*sign),-1,1))));actual=[]
            for c in d.contact:
                if {int(c.geom1),int(c.geom2)}!={cube,g}:continue
                n=c.frame[:3]*(1 if int(c.geom1)==cube else -1)
                actual.append(dict(local_point_mm=((c.pos-C)@B*1000).tolist(),normal_in_cube=(n@B).tolist(),depth_mm=-float(c.dist)*1000))
            contacts.append(dict(geom=name,assumed_surface_normal_error_deg=angle,actual_CAD_contacts=actual))
        result.append(dict(label=label,contacts=contacts))
    report=dict(status='contact_normal_diagnostic',cases=result,devices_opened=False,physical_cause_verified=False,hardware_execution_allowed=False)
    (O/'contact_normal_diagnostic.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps(report),flush=True)
if __name__=='__main__':main()
