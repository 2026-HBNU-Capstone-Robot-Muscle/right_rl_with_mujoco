"""Compact offline inspection of saved states, without devices."""
import json
import numpy as np,mujoco
from research_taught_grasp_20261106 import O,R,TEACH,details
from research_rotation_body_hold_20261104 import restore
from hand_motor_mapping import ticks_to_sim

def main():
    m=mujoco.MjModel.from_binary_path(str(O/'model.mjb'));z=np.load(O/'baseline_end.npz');d=restore(m,z)
    for path in []:
        r=json.loads(path.read_bytes());state=np.load(path.parent/'end.npz');v=details(m,state)
        print(json.dumps(dict(name=r['name'],staticgap=r['geometry_only']['minimum_upper_gap_mm'],dynamic_safety=r['dynamic']['safety_passed'],worst=r['dynamic']['worst'],self_pairs=v['self_contact_mm'],dynamicgap=v['upper_gaps_mm'][:3])),flush=True)
    s=O/'sliding_contact/review.json'
    if s.exists():
        for case in json.loads(s.read_bytes())['cases']:
            for r in case['free_replays']:
                v=r['endpoint'];dy=r['dynamic'];print(json.dumps(dict(name=case['name'],order=r['order'],goal=case['goal'],safety=dy['safety_passed'],yaw=dy['regrasp_yaw_change_deg'],tilt=dy['final_up_tilt_deg'],gap=dy['settled_tail']['minimum_upper_gap_sampled_75ms_mm'],closest=v['upper_gaps_mm'][:3],contacts=v['contacts'],self=v['self_contact_mm'])),flush=True)
    teach=np.array(json.loads(TEACH.read_bytes())['corrected_position_ticks']);q=ticks_to_sim(teach);shape=[]
    for w in (.25,.5,.75,1.):
        p=restore(m,z);p.qpos[4:8]=d.qpos[4:8]+w*(q[4:8]-d.qpos[4:8]);p.qvel[:]=0;mujoco.mj_forward(m,p)
        state={k:getattr(p,k).copy() for k in ('qpos','qvel','ctrl','act','qacc_warmstart')};state['time']=p.time
        v=details(m,state);v['contacts']=[{k:x for k,x in c.items() if k!='normal_N'} for c in v['contacts']]
        shape.append(dict(taught_middle_fraction=w,static_shape_only=True,details=v))
    (O/'taught_shape_comparison.json').write_text(json.dumps(shape,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
