"""Finite-stiffness hand/cube model, unlike the diagnostic kinematic clamp."""
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
    z=np.load(ROOT/'right_hand_rotation_20261026/fresh_empty_start_20261026_235258/recovery_plan_v2_ring_first/path.npz');cmd=z['goals'][z['phases']=='empty_new_grasp'].copy();cmd[:,7]=np.maximum(cmd[:,7],2048);record=json.loads((ROOT/'fresh_empty_grasp_20261027_032949_661470/result.json').read_bytes());actual=np.array([s['actual'] for s in record['samples'] if s['phase']=='stage_4_empty_grasp']);seed=json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_bytes());out=ROOT/'right_hand_rotation_20261027/loaded_grasp_model';cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,out/'actuated_scene.xml');set_cube_physics(m,.055,.1);m.opt.timestep=.002;up=np.array(seed['up_world']);m.opt.gravity[:]=-9.81*up;positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);cube=m.geom('cube').id;m.geom_friction[cube,0]=.3;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'};tips={f:m.geom(f+'_tip').id for f in ('th','mf')};gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();reports=[]
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
        return dict(self_mm=sp,support_mm=sup,cube_mm=cp,external=external,tips={f:g in contact for f,g in tips.items()})
    for prefix in (562,605):
        for scale in (.5,1.,1.5):
            m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(actual[0]);d.qpos[-7:]=seed['settled_cube_qpos'];d.ctrl[:]=ticks_to_sim(cmd[0])
            for _ in range(500):mujoco.mj_step(m,d)
            path=cmd[:prefix];prev=path[0];steps=0;maxself=maxsup=maxcp=maxtrack=0.;stop=None;stable=0;hold=None;queue=np.vstack([path,np.repeat(path[-1][None,:],40,axis=0),path[-2::-1]])
            for i,q in enumerate(queue):
                goal=ticks_to_sim(q);d.ctrl[:]=goal;n=round((i+1)*.075/.002)-steps;steps+=n
                for j in range(n):
                    mujoco.mj_step(m,d)
                    if j%5==0:
                        o=obs(d);maxself=max(maxself,o['self_mm']);maxsup=max(maxsup,o['support_mm']);maxcp=max(maxcp,o['cube_mm']);maxtrack=max(maxtrack,float(max(abs(d.qpos[:16]-goal))))
                        if o['self_mm']>1e-6 or o['support_mm']>.15 or o['cube_mm']>1 or o['external'] or maxtrack>.18:stop='contact_or_tracking';break
                prev=q
                if stop:break
                o=obs(d);stable=stable+1 if all(o['tips'].values()) else 0
                if i==prefix+39:hold=dict(contacts=o,stable_seconds=stable*.075,actual_angles=d.qpos[:16].tolist(),cube_qpos=d.qpos[-7:].tolist())
            row=dict(prefix=prefix,gain_scale=scale,stop=stop,sample=i+1,max_self_mm=maxself,max_support_mm=maxsup,max_cube_mm=maxcp,max_tracking_rad=maxtrack,hold=hold,end=obs(d),model_forcerange=m.actuator_forcerange.tolist(),geometry_and_dynamics_not_calibrated=True,physical_grasp_verified=False);reports.append(row);print(json.dumps({k:v for k,v in row.items() if k!='model_forcerange'}),flush=True)
    (out/'actuated_comparison.json').write_text(json.dumps(dict(cases=reports,devices_opened=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
