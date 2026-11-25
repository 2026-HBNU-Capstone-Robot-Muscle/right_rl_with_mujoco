"""No-cube native screen of a candidate suffix, not hardware authorization."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261022'


def main():
    source=OUT/'supported_yaw_result.json';r=json.loads(source.read_bytes())
    base=np.asarray(r['base_goal_ticks']);rows=np.asarray(r['candidate_goal_ticks'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build_with_middle_thumb_hulls(env);d=mujoco.MjData(m)
    cube=m.geom('cube').id;m.geom_contype[cube]=0;m.geom_conaffinity[cube]=0
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_glass','support_wood')}
    def contacts():
        self_depth=support_depth=0.
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);v=max(0.,-float(c.dist)*1000)
            ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:self_depth=max(self_depth,v)
            if a in support and hb or b in support and ha:support_depth=max(support_depth,v)
        return self_depth,support_depth
    peak=np.zeros(2)
    for raw in np.vstack([base,rows]):
        d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
        mujoco.mj_forward(m,d);peak=np.maximum(peak,contacts())
    gain0=m.actuator_gainprm.copy();bias0=m.actuator_biasprm.copy();dynamic=[]
    for gain in (.5,1.,1.5):
        m.actuator_gainprm[:]=gain0;m.actuator_biasprm[:]=bias0
        m.actuator_gainprm[:,0]*=gain;m.actuator_biasprm[:,1]*=gain
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(base);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
        d.ctrl[:]=ticks_to_sim(base);mujoco.mj_forward(m,d);worst=np.zeros(2);tracking=0.
        for raw in list(rows)+[rows[-1]]*20:
            goal=ticks_to_sim(raw);d.ctrl[:]=goal
            for _ in range(round(.05/m.opt.timestep)):
                mujoco.mj_step(m,d);worst=np.maximum(worst,contacts())
                tracking=max(tracking,float(np.max(abs(d.qpos[:16]-goal))))
        dynamic.append(dict(gain=gain,max_self_mm=worst[0],max_support_mm=worst[1],max_tracking_rad=tracking))
    passed=bool(peak[0]<=1e-6 and peak[1]<=.15 and all(x['max_self_mm']<=1e-6 and x['max_support_mm']<=.15 and x['max_tracking_rad']<=.12 for x in dynamic))
    result=dict(passed=passed,static_max_self_mm=peak[0],static_max_support_mm=peak[1],dynamic=dynamic,
                hardware_commands_sent=0,hardware_authorized=False)
    (OUT/'supported_yaw_empty_suffix_screen.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2));return 0 if passed else 2


if __name__=='__main__':raise SystemExit(main())
