"""Check freely moving cube on an offline thumb/finger transition."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,time
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/yaw_after_flip_v1'
def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('--pair',type=int,default=0);parser.add_argument('--seed-folder');args=parser.parse_args()
    if args.seed_folder:OUT=(ROOT/args.seed_folder).resolve();OUT.relative_to(ROOT)
    seed=json.loads((OUT/'seed.json').read_bytes());p=json.loads((OUT/f'rrt_thumb_pair{args.pair}_review.json').read_bytes());data=np.load(OUT/f'rrt_thumb_pair{args.pair}_path.npz');path=data['goals'];base=data['start'];residual=np.asarray(seed['response_residual_ticks']);up=np.asarray(seed['up_world']);state=np.asarray(seed['settled_cube_qpos'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,OUT/f'replay_thumb_pair{args.pair}_scene.xml');set_cube_physics(m,.055,.1);dt=.002;m.opt.timestep=dt;m.dof_armature[:16]=1e6;m.opt.gravity[:]=-9.81*up
    positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);cube=m.geom('cube').id;cb=int(m.geom_bodyid[cube]);m.geom_friction[cube,0]=.3;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};tips={f:m.geom(f+'_tip').id for f in ('th',p['finger'])}
    def observe(d):
        contact=set();sp=sup=cp=0.;external=False
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);depth=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:sp=max(sp,depth)
            if (a in support and hb) or (b in support and ha):sup=max(sup,depth)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cp=max(cp,depth)
                    if c.dist<.00015:contact.add(other)
                elif c.dist<.0004 and (other in support or m.geom(other).name=='floor'):external=True
        return dict(self_overlap_mm=sp,support_overlap_mm=sup,cube_overlap_mm=cp,external_cube_contact=external,tip_contacts={f:g in contact for f,g in tips.items()},gaps_mm={f:float(mujoco.mj_geomDistance(m,d,cube,g,1.,None)*1000) for f,g in tips.items()})
    d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(base+residual);d.qpos[-7:]=state;d.ctrl[:]=d.qpos[:16]
    for _ in range(500):d.qpos[:16]=ticks_to_sim(base+residual);d.qvel[:16]=0;mujoco.mj_step(m,d)
    previous=base.copy();maxcp=maxsp=maxsup=0.;trace=[];stepcount=0;stopped=None;start=time.perf_counter();first_grasp=None
    for gi,g in enumerate(np.vstack([path,np.repeat(path[-1][None,:],27,axis=0)])):
        a=ticks_to_sim(previous+residual);b=ticks_to_sim(g+residual);n=round((gi+1)*.075/dt)-stepcount;stepcount+=n
        for j in range(n):
            d.qpos[:16]=a+(b-a)*(j+1)/n;d.qvel[:16]=(b-a)/.075;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
            if j%5==0:
                obs=observe(d);maxcp=max(maxcp,obs['cube_overlap_mm']);maxsp=max(maxsp,obs['self_overlap_mm']);maxsup=max(maxsup,obs['support_overlap_mm'])
                if maxcp>1 or maxsp>.01 or maxsup>.15 or obs['external_cube_contact']:stopped='contact_or_clearance_rejection';break
            mujoco.mj_step(m,d)
        previous=g.copy()
        if first_grasp is None:
            mujoco.mj_forward(m,d);grip=observe(d)
            if all(grip['tip_contacts'].values()) and grip['cube_overlap_mm']<=.5:
                first_grasp=dict(goal_ticks=g.tolist(),sample=gi+1,contacts=grip)
                np.savez_compressed(OUT/f'rrt_thumb_pair{args.pair}_first_contact.npz',qpos=d.qpos.copy(),goal_ticks=g,source_sample=gi+1)
        if gi%5==0:trace.append(d.qpos.copy())
        if stopped:break
    mujoco.mj_forward(m,d);obs=observe(d);R=d.xmat[cb].reshape(3,3);tilt=float(np.degrees(np.arccos(np.clip((R@np.asarray(seed.get('up_cube_normal',[1,0,0])))@up,-1,1))));out=dict(status='free_cube_thumb_opposed_transition',pair=args.pair,finger=p['finger'],stopped=stopped,samples=gi+1,nominal_seconds=(gi+1)*.075,max_cube_overlap_mm=maxcp,max_self_overlap_mm=maxsp,max_support_overlap_mm=maxsup,up_face=seed.get('up_face','red'),up_face_error_deg=tilt,final_contacts=obs,end_cube_qpos=d.qpos[-7:].tolist(),end_goal_ticks=previous.tolist(),promising_regrasp=bool(stopped is None and all(obs['tip_contacts'].values()) and tilt<=15),elapsed_wall_seconds=time.perf_counter()-start,initial_cube_not_registered=True,hardware_execution_allowed=False,real_devices_opened=False)
    out['first_two_tip_contact']=first_grasp
    (OUT/f'replay_thumb_pair{args.pair}_review.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8');np.savez_compressed(OUT/f'replay_thumb_pair{args.pair}.npz',qpos=np.asarray(trace),end_qpos=d.qpos.copy(),goals=path);print(json.dumps(out),flush=True)
if __name__=='__main__':main()
