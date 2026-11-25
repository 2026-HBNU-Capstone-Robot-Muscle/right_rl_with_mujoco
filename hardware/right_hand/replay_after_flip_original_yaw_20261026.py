"""Revisit the user's original lateral-turn poses from the newly successful end."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import hashlib,json,time
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_reentry_20261025 import limits
from loaded_thumb_rotation_plan_20261026 import motion_goals
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261026/yaw_after_flip_v1'
def main():
    seed=json.loads((OUT/'seed.json').read_bytes());base=np.asarray(seed['commanded_hand_ticks']);residual=np.asarray(seed['response_residual_ticks']);pose=np.asarray(seed['settled_cube_qpos'])
    original=json.loads((ROOT/'sequence_flip_poses.json').read_bytes());poses=[np.asarray(v['raw_ticks']) for v in original['poses']]
    print('START_COMPARISON',json.dumps(dict(current=base.tolist(),successful_turn_start=motion_goals()[0].tolist(),original_regrasp=poses[1].tolist())),flush=True)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'original_yaw_scene.xml');set_cube_physics(m,.055,.1);dt=.002;m.opt.timestep=dt;m.dof_armature[:16]=1e6
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
    up=np.asarray(seed['up_world']);m.opt.gravity[:]=-9.81*up;cube=m.geom('cube').id;cb=int(m.geom_bodyid[cube]);hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def observe(d):
        contacts=set();sp=sup=cp=0.;ext=False
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);depth=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:sp=max(sp,depth)
            if (a in support and hb) or (b in support and ha):sup=max(sup,depth)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cp=max(cp,depth)
                    if c.dist<.0003:contacts.add(m.geom(other).name)
                elif c.dist<.0003 and (other in support or m.geom(other).name=='floor'):ext=True
        return contacts,sp,sup,cp,ext
    lo,hi=limits();goals=[];previous=base.copy()
    for stage,target in enumerate(poses[1:],1):
        count=max(1,int(np.ceil(np.max(abs(target-previous))/2)))
        for i in range(1,count+1):goals.append((stage,np.rint(previous+(target-previous)*i/count).astype(int)))
        previous=target.copy()
    assert np.all(np.asarray([q for stage,q in goals])>=lo) and np.all(np.asarray([q for stage,q in goals])<=hi)
    outcomes=[]
    for friction in (.2,.3,.5):
        m.geom_friction[cube,0]=friction;d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(base+residual);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=pose
        for _ in range(500):
            d.qpos[:16]=ticks_to_sim(base+residual);d.qvel[:16]=0;mujoco.mj_step(m,d)
        mujoco.mj_forward(m,d);initial=d.qpos[-7:].copy();R0=d.xmat[cb].reshape(3,3).copy();previous=base.copy();elapsed=0.;count_steps=0;maximum_self=maximum_support=maximum_cube=gap=maxgap=0.;external=False;trace=[];stages=[];stop=None
        for index,(stage,goal) in enumerate(goals):
            a=ticks_to_sim(previous+residual);b=ticks_to_sim(goal+residual);target_steps=round((index+1)*.075/dt);n=target_steps-count_steps;count_steps=target_steps
            for i in range(n):
                d.qpos[:16]=a+(b-a)*(i+1)/n;d.qvel[:16]=(b-a)/(.075);d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
                if i%5==0:
                    contact,sp,sup,cp,ext=observe(d);maximum_self=max(maximum_self,sp);maximum_support=max(maximum_support,sup);maximum_cube=max(maximum_cube,cp);external|=ext
                    gap=0 if contact else gap+.01;maxgap=max(maxgap,gap)
                mujoco.mj_step(m,d);elapsed+=dt
            if index%5==0:trace.append(d.qpos.copy());stages.append(stage)
            previous=goal.copy()
            if external or maxgap>.5:stop='cube_lost_or_external_contact';break
            if maximum_self>.1 or maximum_support>.15:stop='hand_self_or_support_overlap';break
        mujoco.mj_forward(m,d);R1=d.xmat[cb].reshape(3,3);a=-R0[:,1];b=-R1[:,1];a-=up*(up@a);b-=up*(up@b);a/=np.linalg.norm(a);b/=np.linalg.norm(b)
        yaw=float(np.degrees(np.arctan2(up@np.cross(a,b),a@b)));tilt=float(np.degrees(np.arccos(np.clip(R1[:,0]@up,-1,1))))
        row=dict(friction=friction,planned_goals=len(goals),planned_seconds=.075*len(goals),simulated_seconds=elapsed,last_stage=stage,stopped=stop,
            yaw_deg=yaw,red_up_error_deg=tilt,center_translation_mm=float(1000*np.linalg.norm(d.qpos[-7:-4]-initial[:3])),
            maximum_self_overlap_mm=maximum_self,maximum_support_overlap_mm=maximum_support,maximum_cube_overlap_mm=maximum_cube,
            maximum_no_hand_contact_seconds=maxgap,external_cube_contact=external)
        np.savez_compressed(OUT/f'original_reference_after_flip_mu{friction}.npz',qpos=np.asarray(trace),stages=np.asarray(stages))
        outcomes.append(row);print('ORIGINAL_REFERENCE',json.dumps(row),flush=True)
    report=dict(status='original_reference_from_new_after_flip_seed_diagnostic',source_original_sha256=hashlib.sha256((ROOT/'sequence_flip_poses.json').read_bytes()).hexdigest(),
        model=manifest,cases=outcomes,hardware_execution_allowed=False,real_devices_opened=False)
    (OUT/'original_reference_after_flip_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
