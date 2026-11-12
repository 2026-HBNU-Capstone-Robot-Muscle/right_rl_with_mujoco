"""Identify new EMPTY hand contacts in the candidate suffix, no devices."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim

ROOT=Path(__file__).resolve().parent


def main():
    candidate=json.loads((ROOT/'right_hand_rotation_20261023/same_grasp_small_yaw_candidate.json').read_bytes())
    base=np.array(candidate['base_goal_ticks']);goals=np.array(candidate['goal_ticks'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);cube=m.geom('cube').id;m.geom_contype[cube]=m.geom_conaffinity[cube]=0
    palm=m.body('palm').id;hand={palm}
    for i in range(m.nbody):
        if int(m.body_parentid[i]) in hand:hand.add(i)
    def contacts(d):
        result=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2)
            if c.dist<0 and int(m.geom_bodyid[a]) in hand and int(m.geom_bodyid[b]) in hand:
                result.append((m.geom(a).name,m.geom(b).name,float(-c.dist*1000)))
        return result
    static=[];d=mujoco.MjData(m)
    for i,q in enumerate(np.vstack([base,goals])):
        d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
        found=contacts(d)
        if found:static.append(dict(sample=i,contacts=found))
    dynamics=[];gain=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy()
    for scale in (.5,1.,1.5):
        m.actuator_gainprm[:]=gain;m.actuator_biasprm[:]=bias
        m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(base);d.qpos[-7:]=[0,0,2,1,0,0,0]
        rows=[base]*60+list(goals)+[goals[-1]]*60;maxima={}
        for i,q in enumerate(rows):
            d.ctrl[:]=ticks_to_sim(q)
            for _ in range(round(.05/m.opt.timestep)):
                mujoco.mj_step(m,d)
                for a,b,depth in contacts(d):
                    key='|'.join(sorted((a,b)))
                    if depth>maxima.get(key,{}).get('depth_mm',0):maxima[key]=dict(sample=i,depth_mm=depth)
        dynamics.append(dict(gain_scale=scale,maximum_contacts=maxima))
    result=dict(static=static,dynamic=dynamics,hardware_commands_sent=0)
    (ROOT/'right_hand_rotation_20261023/grasped_yaw_contact_diagnostic.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(dict(static_count=len(static),static_ends=static[:2]+static[-2:],dynamic=dynamics),indent=2))


if __name__=='__main__':main()
