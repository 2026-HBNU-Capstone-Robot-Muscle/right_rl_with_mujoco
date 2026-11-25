"""Free-cube approach diagnostic for adjacent fingertips; no robot interface."""
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
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/yaw_after_flip_v1'
def main():
    seed=json.loads((OUT/'seed.json').read_bytes());base=np.asarray(seed['commanded_hand_ticks']);residual=np.asarray(seed['response_residual_ticks']);cube_pose=np.asarray(seed['settled_cube_qpos']);up=np.asarray(seed['up_world'])
    source=json.loads((OUT/'pinch_upper_corner_endpoints.json').read_bytes());pairs=[v for v in source['adjacent_pairs'] if v['endpoint_promising']]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'pinch_approach_scene.xml');set_cube_physics(m,.055,.1);dt=.002;m.opt.timestep=dt;m.dof_armature[:16]=1e6;m.opt.gravity[:]=-9.81*up
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    cube=m.geom('cube').id;cb=int(m.geom_bodyid[cube]);m.geom_friction[cube,0]=.3;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};tips={f:m.geom(f+'_tip').id for f in ('if','mf')}
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
                    if c.dist<.0004:contact.add(other)
                elif c.dist<.0004 and (other in support or m.geom(other).name=='floor'):external=True
        return contact,sp,sup,cp,external
    def interpolate(a,b):
        n=max(1,int(np.ceil(np.max(abs(b-a))/2)))
        return [np.rint(a+(b-a)*i/n).astype(int) for i in range(1,n+1)]
    results=[];start=time.perf_counter()
    for pi,pair in enumerate(pairs):
        end=np.asarray(pair['goal_ticks'])
        for order in ('middle_first','index_first','together'):
            middle=base.copy();ids=[11,9,10,8] if order=='middle_first' else [7,5,6,4];middle[ids]=end[ids]
            goals=np.asarray(interpolate(base,end) if order=='together' else interpolate(base,middle)+interpolate(middle,end));probe=mujoco.MjData(m);maxself=maxsupport=0.;badindex=None
            for gi,g in enumerate(goals):
                probe.qpos[:16]=ticks_to_sim(g+residual);probe.qpos[-7:]=cube_pose;mujoco.mj_forward(m,probe);_,sp,sup,_,_=observe(probe);maxself=max(maxself,sp);maxsupport=max(maxsupport,sup)
                if sp>.01 or sup>.15:badindex=gi;break
            row=dict(pair=pi,order=order,goals=len(goals),seconds=.075*len(goals),static_self_overlap_mm=maxself,static_support_overlap_mm=maxsupport,static_rejected_at=badindex)
            if badindex is None:
                d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(base+residual);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=cube_pose
                for _ in range(500):d.qpos[:16]=ticks_to_sim(base+residual);d.qvel[:16]=0;mujoco.mj_step(m,d)
                initial=d.qpos[-7:].copy();previous=base.copy();stepcount=0;gap=maxgap=maxcp=0.;external=False;stopped=None;trace=[]
                for gi,g in enumerate(np.vstack([goals,np.repeat(end[None,:],27,axis=0)])):
                    a=ticks_to_sim(previous+residual);b=ticks_to_sim(g+residual);target=round((gi+1)*.075/dt);n=target-stepcount;stepcount=target
                    for j in range(n):
                        d.qpos[:16]=a+(b-a)*(j+1)/n;d.qvel[:16]=(b-a)/.075;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
                        if j%5==0:
                            contact,sp,sup,cp,ext=observe(d);maxcp=max(maxcp,cp);external|=ext;gap=0 if contact else gap+.01;maxgap=max(maxgap,gap)
                        mujoco.mj_step(m,d)
                    previous=g.copy()
                    if gi%5==0:trace.append(d.qpos.copy())
                    if external or maxgap>.2 or maxcp>1:stopped='lost_cube_or_excessive_model_contact';break
                mujoco.mj_forward(m,d);contact,sp,sup,cp,ext=observe(d);C=d.xmat[cb].reshape(3,3).copy();tilt=float(np.degrees(np.arccos(np.clip(C[:,0]@up,-1,1))));shift=float(1000*np.linalg.norm(d.qpos[-7:-4]-initial[:3]))
                actual_tips={f:bool(g in contact) for f,g in tips.items()};retained=bool(stopped is None and all(actual_tips.values()) and tilt<=15 and shift<=15 and not external and maxgap<=.1)
                row.update(stopped=stopped,red_up_error_deg=tilt,translation_mm=shift,tip_contact=actual_tips,max_cube_overlap_mm=maxcp,max_no_hand_contact_s=maxgap,external_cube_contact=external,promising_regrasp=retained,
                    end_goal_ticks=end.tolist(),end_cube_qpos=d.qpos[-7:].tolist())
                if retained:
                    name=f'pinch_pair{pi}_{order}.npz';np.savez_compressed(OUT/name,goals=goals,qpos=np.asarray(trace),end_qpos=d.qpos.copy());row['trace']=name
            results.append(row);print('APPROACH',json.dumps({k:v for k,v in row.items() if k not in ('end_goal_ticks','end_cube_qpos')}),flush=True)
            (OUT/'pinch_approach_replay.json').write_text(json.dumps(dict(status='conditional_after_flip_adjacent_regrasp_replay',model=manifest,cases=results,
                initial_cube_not_registered=True,following_residual_not_identified_for_new_goals=True,hardware_execution_allowed=False,real_devices_opened=False,
                elapsed_seconds=time.perf_counter()-start),indent=2)+'\n',encoding='utf-8')
    print('SUMMARY',dict(cases=len(results),promising=sum(v.get('promising_regrasp',False) for v in results)),flush=True)
if __name__=='__main__':main()
