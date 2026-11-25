"""Test stopping a partial lateral sweep and lowering its supporting roots.

Cube motion is free, hand motion prescribed, no hardware execution permission.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import itertools,json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/yaw_stop_and_level_v1'
SOURCE=ROOT/'right_hand_rotation_20261026/yaw_original_clearance_search/raised_start/floor2250_flat1_rf1_th1-128_th2-64.npz'
def main():
    OUT.mkdir(parents=True,exist_ok=True);z=np.load(SOURCE);source=np.asarray(z['goals']);residual=np.asarray(z['response_residual_ticks']);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'scene.xml');set_cube_physics(m,.055,.1);dt=.002;m.opt.timestep=dt;m.dof_armature[:16]=1e6;positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    d=mujoco.MjData(m);mujoco.mj_forward(m,d);palm=m.body('palm').id;R=d.xmat[palm].reshape(3,3).copy();origin=d.xpos[palm].copy();up=-R[:,2]
    if up[2]<0:up=-up
    x=R[:,0].copy();x-=up*(x@up);x/=np.linalg.norm(x);y=np.cross(up,x);frame=np.column_stack([x,y,up]);CR=np.column_stack([x,-up,np.cross(x,-up)]);quat=Rotation.from_matrix(CR).as_quat()[[3,0,1,2]];m.opt.gravity[:]=-9.81*up
    cube=m.geom('cube').id;cb=int(m.geom_bodyid[cube]);hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def observe(d):
        contact=set();sp=sup=cp=0.;external=False
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);dep=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:sp=max(sp,dep)
            if (a in support and hb) or (b in support and ha):sup=max(sup,dep)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cp=max(cp,dep)
                    if c.dist<.0003:contact.add(other)
                elif c.dist<.0003 and (other in support or m.geom(other).name=='floor'):external=True
        return contact,sp,sup,cp,external
    def angles(d):
        C=d.xmat[cb].reshape(3,3);a=-CR[:,0].copy();b=-C[:,0].copy();a-=up*(up@a);b-=up*(up@b);a/=np.linalg.norm(a);b/=np.linalg.norm(b)
        return float(np.degrees(np.arctan2(up@np.cross(a,b),a@b))),float(np.degrees(np.arccos(np.clip((-C[:,1])@up,-1,1))))
    results=[];started=time.perf_counter()
    for stop_s in (70.,75.,80.,82.5,85.2,87.5):
        index=round(stop_s/.075);prefix=source[:index+1].copy();previous=prefix[-1].copy();end=previous.copy();end[[7,11]]=2048;n=int(np.ceil(np.max(abs(end-previous))/2));level=np.asarray([np.rint(previous+(end-previous)*i/n).astype(int) for i in range(1,n+1)])
        goals=np.vstack([prefix,level,np.repeat(end[None,:],40,axis=0)]);q=ticks_to_sim(goals+residual);probe=mujoco.MjData(m);bad=None
        for gi,pose in enumerate(q):
            probe.qpos[:16]=pose;probe.qpos[-7:-4]=[0,0,10];mujoco.mj_forward(m,probe);_,sp,sup,_,_=observe(probe)
            if sp>.01 or sup>.15:bad=dict(index=gi,self_mm=sp,support_mm=sup);break
        for friction in (.2,.3,.5):
            row=dict(stop_time_s=index*.075,friction=friction,goals=len(goals),nominal_seconds=(len(goals)-1)*.075,native_self_support_rejection=bad)
            if bad is None:
                m.geom_friction[cube,0]=friction;d=mujoco.MjData(m);d.qpos[:16]=q[0];d.ctrl[:]=q[0];d.qpos[-7:-4]=origin+frame@(np.array([-10,-15,63.5])*.001);d.qpos[-4:]=quat
                for _ in range(500):d.qpos[:16]=q[0];d.qvel[:16]=0;mujoco.mj_step(m,d)
                maximum_cube=gap=maxgap=0.;external=False;count=0;trace=[];first_large=None;stop=None
                for gi in range(1,len(q)):
                    target=round(gi*.075/dt);n=target-count;count=target;a=q[gi-1];b=q[gi]
                    for j in range(n):
                        d.qpos[:16]=a+(b-a)*(j+1)/n;d.qvel[:16]=(b-a)/.075;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
                        if j%5==0:
                            contact,sp,sup,cp,ext=observe(d);maximum_cube=max(maximum_cube,cp);external|=ext;gap=0 if contact else gap+.01;maxgap=max(maxgap,gap)
                            if cp>1 and first_large is None:first_large=(gi-1)*.075+j*dt
                        mujoco.mj_step(m,d)
                    if gi%5==0:trace.append(d.qpos.copy())
                    if external or maxgap>.3:stop='lost_cube';break
                mujoco.mj_forward(m,d);yaw,green=angles(d);contact,sp,sup,cp,ext=observe(d)
                accepted=bool(stop is None and abs(abs(yaw)-90)<=15 and green<=15 and contact and not external and maximum_cube<=1 and maxgap<=.1)
                row.update(yaw_deg=yaw,green_up_error_deg=green,maximum_cube_overlap_mm=maximum_cube,first_excessive_cube_overlap_s=first_large,maximum_no_hand_contact_s=maxgap,external_cube_contact=external,stopped=stop,conditional_candidate_accepted=accepted)
                if accepted:
                    file=f'stop{index}_mu{friction}.npz';np.savez_compressed(OUT/file,goals=goals,qpos=np.asarray(trace),response_residual_ticks=residual);row['trace']=file
            results.append(row);print('FINISH',json.dumps(row),flush=True)
            (OUT/'review.json').write_text(json.dumps(dict(model=manifest,cases=results,cube_start_hypothesis=dict(x_mm=-10,y_mm=-15,height_mm=63.5),hand_response_not_identified=True,
                hardware_execution_allowed=False,real_devices_opened=False,elapsed_seconds=time.perf_counter()-started),indent=2)+'\n',encoding='utf-8')
    print('SUMMARY',dict(cases=len(results),promising=sum(v.get('conditional_candidate_accepted',False) for v in results)),flush=True)
if __name__=='__main__':main()
