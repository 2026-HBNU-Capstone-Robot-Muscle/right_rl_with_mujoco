"""Offline placement/short closure feasibility; does not authorize hardware."""
import sys,json
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from final_grip_pose_plan_20261116 import ROOT,REFERENCE,candidate
from final_pose_temporary_support_20261116 import build
from loaded_quarter_model_policy_20261025 import correct_model
from hand_motor_mapping import ticks_to_sim
from research_rotation_body_hold_20261104 import inspect_for
from cube_contact_material_policy_20261029 import apply_material
sys.path.insert(0,str(ROOT/'right_hand_rotation_20261109/prehardware_policy_cycle'))
from native_cycle import assets,restore

def main():
    out=ROOT/'right_hand_rotation_20261116/final_grip_attempt_review/cube_placement_probe'
    out.mkdir(exist_ok=True)
    old,*_=assets('active_ccd50');z=dict(np.load(REFERENCE));od=restore(old,z)
    op=od.xpos[old.body('palm').id];oR=od.xmat[old.body('palm').id].reshape(3,3)
    localC=oR.T@(od.qpos[-7:-4]-op)
    localR=oR.T@od.xmat[old.body('cube').id].reshape(3,3)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build(env,out/'scene.xml',book_projection_mm=-10)
    final=candidate()['goals'][-1];correct_model(m,final);m.opt.timestep=.001
    hand,support,contacts=inspect_for(m);cube=m.geom('cube').id
    d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(final);mujoco.mj_forward(m,d)
    p=d.xpos[m.body('palm').id];R=d.xmat[m.body('palm').id].reshape(3,3)
    cubeq=np.r_[p+R@localC,Rotation.from_matrix(R@localR).as_quat(scalar_first=True)]
    m.opt.gravity[:]=-9.81*(R@oR.T@z['up']);apply_material(m,.3,hand,cube)
    results=[]
    up=R@oR.T@z['up']
    for delta in (8,16,32,64):
        start=final.copy();start[[0,1,8,10]]-=delta
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(start);d.ctrl[:]=d.qpos[:16]
        d.qpos[-7:]=cubeq;mujoco.mj_forward(m,d)
        # A cube must be insertable from above, rather than spawned inside fingers.
        def depth(height):
            d.qpos[-7:-4]=cubeq[:3]+up*height;mujoco.mj_forward(m,d)
            return max([max(0.,-float(c.dist)*1000) for c in d.contact
                if cube in (int(c.geom1),int(c.geom2)) and int(m.geom_bodyid[int(c.geom2) if int(c.geom1)==cube else int(c.geom1)]) in hand]+[0.])
        previous=.06;lower=None
        for h in np.arange(.06,-.025,-.0005):
            if depth(float(h))>1e-5:lower=float(h);upper=previous;break
            previous=float(h)
        if lower is None:raise RuntimeError('No seat on the bounded vertical insertion line')
        for _ in range(24):
            mid=(lower+upper)/2
            if depth(mid)>1e-5:lower=mid
            else:upper=mid
        seat=upper+.0001;d.qpos[-7:-4]=cubeq[:3]+up*seat;mujoco.mj_forward(m,d)
        initial_support,initial_self=contacts(d)
        maxself=maxsup=maxcube=maxtrack=0.;longest=gap=0.;stop=None;before=None
        count=max(1,abs(delta)//2)
        rows=[('place',start,2.)]+[('close',np.rint(start+(final-start)*i/count).astype(int),.075) for i in range(1,count+1)]+[('hold',final,10.)]
        for phase,q,seconds in rows:
            d.ctrl[:]=ticks_to_sim(q)
            for _ in range(round(seconds/.001)):
                mujoco.mj_step(m,d);su,sp=contacts(d)
                maxself=max(maxself,max(sp.values(),default=0.));maxsup=max(maxsup,max(su.values(),default=0.))
                maxtrack=max(maxtrack,float(max(abs(d.qpos[:16]-d.ctrl))));inhand=False
                for c in d.contact:
                    a,b=int(c.geom1),int(c.geom2)
                    if cube not in (a,b):continue
                    other=b if a==cube else a
                    if int(m.geom_bodyid[other]) in hand:
                        maxcube=max(maxcube,max(0.,-float(c.dist)*1000));inhand|=c.dist<.0004
                    elif other in support and c.dist<.0004:stop='external_cube_support'
                gap=0. if inhand else gap+.001;longest=max(longest,gap)
                if maxself>1e-6 or maxsup>.15 or maxcube>1 or maxtrack>.18 or longest>.25:
                    stop='contact/tracking/retention';break
            if phase=='place':before=d.qpos[-7:-4].copy()
            if stop:break
        row=dict(open_delta_ticks=delta,seating_shift_mm=seat*1000,stop=stop,last_phase=phase,initial_self_mm=initial_self,
            initial_support_mm=initial_support,max_self_mm=maxself,max_support_mm=maxsup,
            max_cube_mm=maxcube,max_tracking_rad=maxtrack,longest_no_hand_s=longest,
            final_cube_shift_mm=float(np.linalg.norm(d.qpos[-7:-4]-cubeq[:3])*1000),
            start_ticks=start.tolist(),final_ticks=final.tolist())
        results.append(row);print(json.dumps(row),flush=True)
    (out/'review.json').write_text(json.dumps(dict(cases=results,devices_opened=False,hardware_execution_allowed=False),indent=2)+'\n')

if __name__=='__main__':main()
