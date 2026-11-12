"""Offline diagnosis of changed-pose model rejection; no device access."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fresh_empty_grasp_plan_20261027 import ROOT,FreshEmptyGraspPlan,interpolate
from loaded_quarter_model_policy_20261025 import correct_model
from fingertip_cad_model_20261025 import build
from screen_loaded_repeat_20261026 import cached_contacts
from book_support_contact_scope import derive_scope,allowed_initial_pairs
from book_empty_preparation_plan import offsets
from hand_motor_mapping import ticks_to_sim,sim_to_ticks,SIM_TO_MOTOR_IDS

def main():
    source=ROOT/'fresh_empty_grasp_20261027_030819_048059/fresh_off_request.json';plan=FreshEmptyGraspPlan(source);raw=plan.source_position();out=ROOT/'right_hand_rotation_20261026/changed_off_pose_20261027';out.mkdir(exist_ok=True);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);results=[];static=[]
    for front in (-15.,-10.,-5.):
        m,_=build(env,out/f'initial_front_{front:g}.xml',book_projection_mm=front);correct_model(m,raw);m.opt.timestep=.005;cube=m.geom_bodyid==m.body('cube').id;m.geom_contype[cube]=0;m.geom_conaffinity[cube]=0;hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};inspect=cached_contacts(m,hand,support);allowed=allowed_initial_pairs(derive_scope(m));joints=m.actuator_trnid[:,0];lo=np.maximum(m.jnt_range[joints,0],m.actuator_ctrlrange[:,0]);hi=np.minimum(m.jnt_range[joints,1],m.actuator_ctrlrange[:,1]);g=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();envelope={}
        def put(q):
            d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(q);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];d.ctrl[:]=ticks_to_sim(q);mujoco.mj_forward(m,d);return d
        for off in offsets():
            su,_=inspect(put(raw+off))
            for k,v in su.items():envelope[k]=max(envelope.get(k,0.),v)
        for scale in (.5,1.,1.5):
            m.actuator_gainprm[:]=g;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;d=put(raw);bad_support=[];bad_range=[]
            for i in range(100):
                mujoco.mj_step(m,d);su,sp=inspect(d)
                for k,v in su.items():
                    allowance=envelope.get(k,.15 if k in allowed else 0.)
                    if v>allowance+1e-6:bad_support.append(dict(time=(i+1)*.005,pair=k,depth_mm=v,allowance_mm=allowance))
                excess=np.maximum(lo-d.qpos[:16],d.qpos[:16]-hi)
                for j in np.flatnonzero(excess>1e-6):bad_range.append(dict(time=(i+1)*.005,joint=m.joint(int(joints[j])).name,motor_id=SIM_TO_MOTOR_IDS[int(j)],excess_rad=float(excess[j]),actual_radians=float(d.qpos[j]),lower_radians=float(lo[j]),upper_radians=float(hi[j])))
            results.append(dict(front=front,scale=scale,initial_support_envelope_mm=envelope,first_support_violation=bad_support[0] if bad_support else None,max_support_violation=max(bad_support,key=lambda v:v['depth_mm']-v['allowance_mm']) if bad_support else None,first_range_violation=bad_range[0] if bad_range else None,max_range_violation=max(bad_range,key=lambda v:v['excess_rad']) if bad_range else None,final_ticks=sim_to_ticks(d.qpos[:16]).tolist()))
        if front==-10.:
            for name,delta in [('pip_only',{14:-80}),('dip_only',{12:-80}),('pip_and_dip',{12:-80,14:-80}),('root_plus',{15:40,14:-80}),('rot_minus',{13:-40,14:-80})]:
                fail=0;worst=0.
                for off in offsets():
                    a=raw+off;b=a.copy()
                    for j,v in delta.items():b[j]+=v
                    previous=inspect(put(a))[1]
                    for q in interpolate(a,b):
                        current=inspect(put(q))[1];increase=max((v-previous.get(k,0.) for k,v in current.items()),default=0.);worst=max(worst,increase);previous=current
                    fail+=int(worst>1e-6)
                static.append(dict(candidate=name,maximum_step_increase_mm=worst))
    payload=dict(source=source.relative_to(ROOT).as_posix(),initial_hold_cases=results,early_ring_options=static,devices_opened=False);(out/'diagnostic.json').write_text(json.dumps(payload,indent=2)+'\n',encoding='utf-8');print(json.dumps(payload,indent=2),flush=True)
if __name__=='__main__':main()
