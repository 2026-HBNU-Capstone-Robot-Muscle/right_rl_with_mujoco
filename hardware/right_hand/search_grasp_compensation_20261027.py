"""Compare small measured-response compensation without changing contact limits."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json
from pathlib import Path
import mujoco,numpy as np
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--fine',action='store_true');args=parser.parse_args()
    rec=json.loads((ROOT/'fresh_empty_grasp_20261027_032949_661470/result.json').read_bytes());actual=np.array([s['actual'] for s in rec['samples'] if s['phase']=='stage_4_empty_grasp']);z=np.load(ROOT/'right_hand_rotation_20261026/fresh_empty_start_20261026_235258/recovery_plan_v2_ring_first/path.npz');cmd=z['goals'][z['phases']=='empty_new_grasp'].copy();cmd[:,7]=np.maximum(cmd[:,7],2048);error=cmd-actual;mask=np.zeros(16);mask[[0,1,2,3,8,9,10,11]]=1;seed=json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes());out=ROOT/'right_hand_rotation_20261027/loaded_grasp_model';cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,out/'compensated_scene.xml');set_cube_physics(m,.055,.1);m.opt.timestep=.002;m.dof_armature[:16]=1e6;m.opt.gravity[:]=-9.81*np.array(seed['up_world']);positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);cube=m.geom('cube').id;m.geom_friction[cube,0]=.3;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};tips={f:m.geom(f+'_tip').id for f in ('th','mf')};reports=[]
    def obs(d):
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
                elif other in support and c.dist<.0004:external=True
        return dict(self_mm=sp,support_mm=sup,cube_mm=cp,external=external,tips={f:g in contact for f,g in tips.items()},gaps_mm={f:float(mujoco.mj_geomDistance(m,d,cube,g,1.,None))*1000 for f,g in tips.items()})
    for prefix,kappa in (((557,1.),(558,.95),(558,.9),(557,.95)) if args.fine else ((558,1.),(562,.5),(565,.5),(565,.75))):
        ramp=np.clip((np.arange(prefix)-300)/200,0,1)[:,None];newcmd=cmd[:prefix]+np.rint(kappa*ramp*error[:prefix]*mask).astype(int)
        for fraction in (0.,1.):
            goals=newcmd-fraction*error[:prefix];d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(goals[0]);d.qpos[-7:]=seed['settled_cube_qpos'];d.ctrl[:]=d.qpos[:16]
            for _ in range(500):d.qpos[:16]=ticks_to_sim(goals[0]);d.qvel[:16]=0;mujoco.mj_step(m,d)
            prev=goals[0];steps=0;maximum=0.;stop=None;stable=0;queue=np.vstack([goals,np.repeat(goals[-1][None,:],40,axis=0)])
            for i,q in enumerate(queue):
                a=ticks_to_sim(prev);b=ticks_to_sim(q);n=round((i+1)*.075/.002)-steps;steps+=n
                for j in range(n):
                    d.qpos[:16]=a+(b-a)*(j+1)/n;d.qvel[:16]=(b-a)/.075;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
                    if j%5==0:
                        o=obs(d);maximum=max(maximum,o['cube_mm'])
                        if o['self_mm']>1e-6 or o['support_mm']>.15 or o['cube_mm']>1 or o['external']:stop='contact_or_drop';break
                    mujoco.mj_step(m,d)
                prev=q
                if stop:break
                o=obs(d);stable=stable+1 if all(o['tips'].values()) else 0
            row=dict(prefix=prefix,kappa=kappa,response_fraction=fraction,stop=stop,max_cube_mm=maximum,stable_seconds=stable*.075,final=obs(d),end_goal=newcmd[-1].tolist());reports.append(row);print(json.dumps(row),flush=True)
    (out/('compensation_fine_search.json' if args.fine else 'compensation_search.json')).write_text(json.dumps(dict(cases=reports,hardware_motion_allowed=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
