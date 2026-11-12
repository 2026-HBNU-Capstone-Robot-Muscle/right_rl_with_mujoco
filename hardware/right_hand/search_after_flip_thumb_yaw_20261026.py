"""Offline ninety-degree yaw contact-path feasibility and free-cube replay.

The initial cube pose and constant following residual are hypotheses. No motor
or camera interface is imported, and no hardware-ready certificate is emitted.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,itertools,json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
from loaded_quarter_reentry_20261025 import limits
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261026/yaw_after_flip_v1'

def main():
    began=time.perf_counter();seed=json.loads((OUT/'seed.json').read_bytes())
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'thumb_yaw_scene.xml');set_cube_physics(m,.055,.1);m.opt.timestep=.002;m.dof_armature[:16]=1e6
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    up=np.asarray(seed['up_world']);m.opt.gravity[:]=-9.81*up;cube=m.geom('cube').id;cube_body=int(m.geom_bodyid[cube]);palm=m.body('palm').id
    hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    base=np.asarray(seed['commanded_hand_ticks']);residual=np.asarray(seed['response_residual_ticks']);initial=np.asarray(seed['settled_cube_qpos'])
    initialR=Rotation.from_quat(initial[-4:][[1,2,3,0]]).as_matrix();C=initial[:3]
    group=[v for v in seed['contacts'] if v['body']=='th_px']
    if not group:raise ValueError('No conditional thumb proximal contact')
    local=np.mean([v['point_local_m'] for v in group],axis=0);body=m.body('th_px').id;moving=np.array([1,2,3])
    lo,hi=limits();lower=lo[moving]+1e-5;upper=hi[moving]-1e-5
    d=mujoco.MjData(m)
    def put(goal,cube_pose=initial):
        d.qpos[:16]=ticks_to_sim(np.asarray(goal)+residual);d.qpos[-7:]=cube_pose;d.qvel[:]=0;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
    def point(goal):
        put(goal);return d.xpos[body]+d.xmat[body].reshape(3,3)@local
    start=point(base);Rpal=d.xmat[palm].reshape(3,3);x=Rpal[:,0].copy();x-=up*(up@x);x/=np.linalg.norm(x);y=np.cross(up,x)
    def contacts(data):
        names=set();selfpeak=supportpeak=cubepeak=0.;external=False
        for c in data.contact:
            a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand;dep=max(0.,-float(c.dist)*1000)
            if ha and hb:selfpeak=max(selfpeak,dep)
            if (a in support and hb) or (b in support and ha):supportpeak=max(supportpeak,dep)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cubepeak=max(cubepeak,dep)
                    if c.dist<.0003:names.add(m.geom(other).name)
                elif c.dist<.0003 and (other in support or m.geom(other).name=='floor'):external=True
        return names,selfpeak,supportpeak,cubepeak,external
    def simulate(goals,friction):
        m.geom_friction[cube,0]=friction;data=mujoco.MjData(m);data.qpos[:16]=ticks_to_sim(base+residual);data.ctrl[:]=data.qpos[:16];data.qpos[-7:]=initial
        for _ in range(500):
            data.qpos[:16]=ticks_to_sim(base+residual);data.qvel[:16]=0;mujoco.mj_step(m,data)
        mujoco.mj_forward(m,data);pose0=data.qpos[-7:].copy();R0=data.xmat[cube_body].reshape(3,3).copy()
        previous=base.copy();maximum_self=maximum_support=maximum_cube=gap=maxgap=0.;external=False;trace=[]
        for index,goal in enumerate(goals):
            a=ticks_to_sim(previous+residual);b=ticks_to_sim(goal+residual)
            count=round(.075/m.opt.timestep)
            for i in range(count):
                data.qpos[:16]=a+(b-a)*(i+1)/count;data.qvel[:16]=(b-a)/(.075);data.ctrl[:]=data.qpos[:16];mujoco.mj_forward(m,data)
                if i%5==0:
                    names,sp,sup,cp,ext=contacts(data);maximum_self=max(maximum_self,sp);maximum_support=max(maximum_support,sup);maximum_cube=max(maximum_cube,cp);external|=ext
                    gap=0 if names else gap+.01;maxgap=max(maxgap,gap)
                mujoco.mj_step(m,data)
            if index%5==0:trace.append(data.qpos.copy())
            previous=goal.copy()
        for _ in range(1000):
            data.qpos[:16]=ticks_to_sim(previous+residual);data.qvel[:16]=0;data.ctrl[:]=data.qpos[:16];mujoco.mj_step(m,data)
        mujoco.mj_forward(m,data);R1=data.xmat[cube_body].reshape(3,3).copy();a=-R0[:,1];b=-R1[:,1];a-=up*(up@a);b-=up*(up@b);a/=np.linalg.norm(a);b/=np.linalg.norm(b)
        yaw=float(np.degrees(np.arctan2(up@np.cross(a,b),a@b)));tilt=float(np.degrees(np.arccos(np.clip(R1[:,0]@up,-1,1))))
        names,*_=contacts(data)
        return dict(friction=friction,yaw_deg=yaw,red_up_error_deg=tilt,center_translation_mm=float(1000*np.linalg.norm(data.qpos[-7:-4]-pose0[:3])),
            maximum_self_overlap_mm=maximum_self,maximum_support_overlap_mm=maximum_support,maximum_cube_overlap_mm=maximum_cube,
            maximum_no_hand_contact_seconds=maxgap,external_cube_contact=external,final_hand_contacts=sorted(names)),np.asarray(trace)
    results=[]
    for sign,(dx,dy) in itertools.product((-1,1),((0.,0.),(0.,.008),(.008,0.),(.008,.008),(-.008,.008))):
        goals=[];previous=base.copy();errors=[];selfpeak=sup=0.
        for theta in np.linspace(0,sign*np.pi/2,37)[1:]:
            fraction=abs(theta)/(np.pi/2);target=C+Rotation.from_rotvec(up*theta).apply(start-C)+(dx*x+dy*y)*fraction
            def objective(value):
                q=base.astype(float).copy();q[moving]=value
                return np.r_[(point(q)-target)*1000,.003*(value-previous[moving])]
            fit=least_squares(objective,np.clip(previous[moving].astype(float),lower,upper),bounds=(lower,upper),diff_step=.00003,max_nfev=100)
            goal=base.copy();goal[moving]=np.rint(fit.x).astype(int);errors.append(float(np.linalg.norm(point(goal)-target)*1000))
            count=max(1,int(np.ceil(np.max(abs(goal-previous))/2)))
            for i in range(1,count+1):
                q=np.rint(previous+(goal-previous)*i/count).astype(int);goals.append(q);put(q);_,sp,supportpeak,_,_=contacts(d);selfpeak=max(selfpeak,sp);sup=max(sup,supportpeak)
            previous=goal
        row=dict(target_yaw_deg=sign*90,center_shift_target_mm=[1000*dx,1000*dy],goals=len(goals),duration_s=.075*len(goals),
                 maximum_contact_point_error_mm=max(errors),static_maximum_self_overlap_mm=selfpeak,static_maximum_support_overlap_mm=sup)
        row['sticking_contact_fit_within_2mm']=max(errors)<=2.
        if selfpeak<=.01 and sup<=.15 and len(goals)<=2200:
            replays=[]
            for friction in (.2,.3,.5):
                result,trace=simulate(np.asarray(goals),friction);replays.append(result)
                np.savez_compressed(OUT/f'yaw_sign{sign}_dx{dx}_dy{dy}_mu{friction}.npz',qpos=trace)
            row['free_cube_replays']=replays
            np.savez_compressed(OUT/f'goals_sign{sign}_dx{dx}_dy{dy}.npz',raw_ticks=np.asarray(goals))
            row['candidate_accepted']=all(abs(v['yaw_deg']-sign*90)<=15 and v['red_up_error_deg']<=15 and v['center_translation_mm']<=20
                and v['maximum_self_overlap_mm']<=.01 and v['maximum_support_overlap_mm']<=.15 and v['maximum_cube_overlap_mm']<=1
                and not v['external_cube_contact'] and v['maximum_no_hand_contact_seconds']<=.1 and v['final_hand_contacts'] for v in replays)
            if row['candidate_accepted']:(OUT/'conditional_yaw_goals.json').write_text(json.dumps(dict(goals=[q.tolist() for q in goals],summary=row,hardware_execution_allowed=False),indent=2),encoding='utf-8')
        else:row['candidate_accepted']=False
        results.append(row);print('CANDIDATE',json.dumps(row),flush=True)
        report=dict(status='offline_contact_point_feasibility_and_free_cube_replay',seed_sha256=hashlib.sha256((OUT/'seed.json').read_bytes()).hexdigest(),
            model=manifest,assumed_initial_cube_pose=True,constant_following_residual_assumed=True,cases=results,hardware_execution_allowed=False,real_devices_opened=False,elapsed_s=time.perf_counter()-began)
        report['contact_point_fit_is_not_a_physical_sticking_requirement']=True
        (OUT/'thumb_yaw_free_cube_diagnostic.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('DONE',dict(candidates=len(results),accepted=sum(v['candidate_accepted'] for v in results)),flush=True)
if __name__=='__main__':main()
