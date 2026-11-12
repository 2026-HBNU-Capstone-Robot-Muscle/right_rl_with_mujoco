"""Free cube with the commanded and actually observed EMPTY grasp trajectory."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import mujoco,numpy as np
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent
def main():
    result=json.loads((ROOT/'fresh_empty_grasp_20261027_032949_661470/result.json').read_bytes());records=[s for s in result['samples'] if s['phase']=='stage_4_empty_grasp'];seed=json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes());path=np.load(ROOT/'right_hand_rotation_20261026/fresh_empty_start_20261026_235258/recovery_plan_v2_ring_first/path.npz');cmd=path['goals'][path['phases']=='empty_new_grasp'].copy();cmd[:,7]=np.maximum(cmd[:,7],2048);measured=np.array([s['actual'] for s in records]);out=ROOT/'right_hand_rotation_20261027/loaded_grasp_model';out.mkdir(exist_ok=True);cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,out/'scene.xml');set_cube_physics(m,.055,.1);m.opt.timestep=.002;m.dof_armature[:16]=1e6;up=np.array(seed['up_world']);m.opt.gravity[:]=-9.81*up;positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);cube=m.geom('cube').id;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    tips={f:m.geom(f+'_tip').id for f in ('th','mf')};support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};reports=[]
    def obs(d):
        contacts=set();sp=sup=cp=0.;external=False
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);depth=max(0,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:sp=max(sp,depth)
            if (a in support and hb) or (b in support and ha):sup=max(sup,depth)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cp=max(cp,depth)
                    if c.dist<.00015:contacts.add(other)
                elif other in support and c.dist<.0004:external=True
        return dict(self_mm=sp,support_mm=sup,cube_mm=cp,external=external,tips={f:g in contacts for f,g in tips.items()},gaps_mm={f:float(mujoco.mj_geomDistance(m,d,cube,g,1.,None))*1000 for f,g in tips.items()})
    for name,goals in (('commanded',cmd),('measured_empty',measured)):
        for friction in (.2,.3,.5):
            m.geom_friction[cube,0]=friction;d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(goals[0]);d.qpos[-7:]=seed['settled_cube_qpos'];d.ctrl[:]=d.qpos[:16]
            for _ in range(500):d.qpos[:16]=ticks_to_sim(goals[0]);d.qvel[:16]=0;mujoco.mj_step(m,d)
            settled=d.qpos[-7:].copy();previous=goals[0];steps=0;maxself=maxsup=maxcp=0.;stable=0;stop=None
            for i,q in enumerate(np.vstack([goals,np.repeat(goals[-1][None,:],40,axis=0)])):
                a=ticks_to_sim(previous);b=ticks_to_sim(q);n=round((i+1)*.075/.002)-steps;steps+=n
                for j in range(n):
                    d.qpos[:16]=a+(b-a)*(j+1)/n;d.qvel[:16]=(b-a)/.075;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
                    if j%5==0:
                        o=obs(d);maxself=max(maxself,o['self_mm']);maxsup=max(maxsup,o['support_mm']);maxcp=max(maxcp,o['cube_mm'])
                        if o['self_mm']>.01 or o['support_mm']>.15 or o['cube_mm']>1 or o['external']:stop='contact_or_drop';break
                    mujoco.mj_step(m,d)
                previous=q
                if stop:break
                o=obs(d);stable=stable+1 if all(o['tips'].values()) else 0
            row=dict(hand_path=name,friction=friction,stop=stop,samples=i+1,max_self_mm=maxself,max_support_mm=maxsup,max_cube_mm=maxcp,final=obs(d),two_tip_final_seconds=stable*.075,settled_cube=settled.tolist(),end_cube=d.qpos[-7:].tolist(),assumed_cube_pose=True,hardware_motion_allowed=False);reports.append(row);print(json.dumps(row),flush=True)
    (out/'measured_comparison.json').write_text(json.dumps(dict(cases=reports,source_actual_samples=len(records),devices_opened=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
