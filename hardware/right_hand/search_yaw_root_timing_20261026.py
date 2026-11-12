"""Restore original middle-root lowering after the thumb crosses its base."""
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
from loaded_thumb_rotation_plan_20261026 import motion_goals
from loaded_quarter_reentry_20261025 import limits
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/yaw_root_timing_v1'
def main():
    OUT.mkdir(parents=True,exist_ok=True);r=json.loads((ROOT/'loaded_repeat_20261026_170932_163649/result.json').read_bytes());sample=next(s for s in r['samples'] if s['phase']=='stage_6_loaded_quarter_reference');measured_base=motion_goals()[0].copy();residual=np.asarray(sample['actual'])-measured_base
    base=measured_base.copy();base[[7,11]]=2250;original=json.loads((ROOT/'sequence_flip_poses.json').read_bytes())['poses'];raw=[base.copy()];prev=base.copy()
    for v in original[3:]:
        goal=np.asarray(v['raw_ticks']).copy();goal[0]-=128;goal[1]-=64;goal[12:]=base[12:];n=max(1,int(np.ceil(np.max(abs(goal-prev))/2)))
        raw.extend(np.rint(prev+(goal-prev)*i/n).astype(int) for i in range(1,n+1));prev=goal
    raw=np.asarray(raw);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,OUT/'scene.xml');set_cube_physics(m,.055,.1);dt=.002;m.opt.timestep=dt;m.dof_armature[:16]=1e6
    pos=m.geom_solref[:,0]>0;m.geom_solref[pos,0]=np.minimum(m.geom_solref[pos,0],.004);d=mujoco.MjData(m);mujoco.mj_forward(m,d);palm=m.body('palm').id;R=d.xmat[palm].reshape(3,3).copy();origin=d.xpos[palm].copy();up=-R[:,2]
    if up[2]<0:up=-up
    x=R[:,0].copy();x-=up*(up@x);x/=np.linalg.norm(x);frame=np.column_stack([x,np.cross(up,x),up]);B=np.column_stack([x,-up,np.cross(x,-up)]);quat=Rotation.from_matrix(B).as_quat()[[3,0,1,2]];m.opt.gravity[:]=-9.81*up
    cube=m.geom('cube').id;cb=int(m.geom_bodyid[cube]);hand={palm};lo,hi=limits()
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def observe(d):
        contact=set();sp=sup=cp=0.;ext=False
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);depth=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:sp=max(sp,depth)
            if (a in support and hb) or (b in support and ha):sup=max(sup,depth)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cp=max(cp,depth)
                    if c.dist<.0003:contact.add(other)
                elif c.dist<.0003 and (other in support or m.geom(other).name=='floor'):ext=True
        return contact,sp,sup,cp,ext
    results=[];started=time.perf_counter()
    for start_s in (25.,35.,45.,55.,65.):
        goals=raw.copy();fraction=np.clip((np.arange(len(goals))*.075-start_s)/12.,0,1)
        for motor,bottom in ((7,2048),(11,2005)):goals[:,motor]=np.maximum(goals[:,motor],np.rint(2250+(bottom-2250)*fraction).astype(int))
        goals=np.vstack([goals,np.repeat(goals[-1][None,:],40,axis=0)]);assert np.max(abs(np.diff(goals,axis=0)))<=2 and np.all(goals>=lo) and np.all(goals<=hi);q=ticks_to_sim(goals+residual);probe=mujoco.MjData(m);bad=None
        for gi,a in enumerate(q):
            probe.qpos[:16]=a;probe.qpos[-7:-4]=[0,0,10];mujoco.mj_forward(m,probe);_,sp,sup,_,_=observe(probe)
            if sp>.01 or sup>.15:bad=dict(index=gi,self_mm=sp,support_mm=sup);break
        for friction in (.2,.3,.5):
            row=dict(lowering_starts_s=start_s,lowering_duration_s=12.,friction=friction,static_rejected=bad,goals=len(goals),seconds=(len(goals)-1)*.075)
            if bad is None:
                m.geom_friction[cube,0]=friction;d=mujoco.MjData(m);d.qpos[:16]=q[0];d.ctrl[:]=q[0];d.qpos[-7:-4]=origin+frame@(np.array([-10,-15,63.5])*.001);d.qpos[-4:]=quat
                for _ in range(500):d.qpos[:16]=q[0];d.qvel[:16]=0;mujoco.mj_step(m,d)
                count=0;gap=maxgap=maxcp=0.;external=False;trace=[];stopped=None;first_large=None
                for gi in range(1,len(q)):
                    target=round(gi*.075/dt);n=target-count;count=target;a=q[gi-1];b=q[gi]
                    for j in range(n):
                        d.qpos[:16]=a+(b-a)*(j+1)/n;d.qvel[:16]=(b-a)/.075;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
                        if j%5==0:
                            contact,sp,sup,cp,ext=observe(d);maxcp=max(maxcp,cp);external|=ext;gap=0 if contact else gap+.01;maxgap=max(maxgap,gap)
                            if cp>1 and first_large is None:first_large=(gi-1)*.075+j*dt
                        mujoco.mj_step(m,d)
                    if gi%5==0:trace.append(d.qpos.copy())
                    if external or maxgap>.3:stopped='lost_cube';break
                mujoco.mj_forward(m,d);C=d.xmat[cb].reshape(3,3);a=-B[:,0].copy();b=-C[:,0].copy();a-=up*(up@a);b-=up*(up@b);a/=np.linalg.norm(a);b/=np.linalg.norm(b);yaw=float(np.degrees(np.arctan2(up@np.cross(a,b),a@b)));green=float(np.degrees(np.arccos(np.clip((-C[:,1])@up,-1,1))));contact,*_=observe(d)
                accepted=bool(stopped is None and abs(abs(yaw)-90)<=15 and green<=15 and contact and not external and maxcp<=1 and maxgap<=.1)
                row.update(yaw_deg=yaw,green_up_error_deg=green,max_cube_overlap_mm=maxcp,first_excessive_contact_s=first_large,max_no_hand_contact_s=maxgap,external_cube_contact=external,stopped=stopped,conditional_candidate_accepted=accepted)
                if accepted:
                    name=f'lower{int(start_s)}_mu{friction}.npz';np.savez_compressed(OUT/name,goals=goals,qpos=np.asarray(trace),response_residual_ticks=residual);row['trace']=name
            results.append(row);print('TIMING',json.dumps(row),flush=True)
            (OUT/'review.json').write_text(json.dumps(dict(model=manifest,cases=results,placement_and_friction_assumed=True,hand_response_not_identified=True,hardware_execution_allowed=False,real_devices_opened=False,elapsed_seconds=time.perf_counter()-started),indent=2)+'\n',encoding='utf-8')
    print('SUMMARY',dict(cases=len(results),accepted=sum(v.get('conditional_candidate_accepted',False) for v in results)),flush=True)
if __name__=='__main__':main()
