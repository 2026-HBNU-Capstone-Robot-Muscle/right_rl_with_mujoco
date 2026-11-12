"""Replay the original measured P4->P6 with a free cube and photo-derived seed."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,time
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim
from sac_cube_model import set_cube_physics
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1'
def main():
    seed=json.loads((OUT/'seed.json').read_bytes());raw=json.loads((ROOT/seed['source']).read_bytes());samples=[s for s in raw['samples'] if s['phase'] in ('stage_5','stage_6')];up=np.asarray(seed['up_world']);up_local=np.asarray(seed['up_cube_normal']);initial=np.asarray(seed['actual_hand_ticks']);state=np.asarray(seed['settled_cube_qpos'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,manifest=build(env,OUT/'original_suffix_scene.xml');set_cube_physics(m,.055,.1);dt=.002;m.opt.timestep=dt;m.dof_armature[:16]=1e6;m.opt.gravity[:]=-9.81*up;positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004);cube=m.geom('cube').id;cb=int(m.geom_bodyid[cube]);m.geom_friction[cube,0]=.3;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    def obs(d):
        cp=sp=sup=0.;external=False;names=[]
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);dep=max(0.,-float(c.dist)*1000);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:sp=max(sp,dep)
            if (a in support and hb) or (b in support and ha):sup=max(sup,dep)
            if cube in (a,b):
                other=b if a==cube else a
                if int(m.geom_bodyid[other]) in hand:
                    cp=max(cp,dep)
                    if c.dist<.0003:names.append(m.geom(other).name)
                elif c.dist<.0003 and (other in support or m.geom(other).name=='floor'):external=True
        return dict(cube_overlap_mm=cp,self_overlap_mm=sp,support_overlap_mm=sup,external_cube_contact=external,contacts=names)
    d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(initial);d.qpos[-7:]=state;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d);B0=d.xmat[cb].reshape(3,3).copy();front=-B0[:,0];front-=up*(front@up);front/=np.linalg.norm(front);trace=[];maxcp=maxsp=maxsup=0.;start_t=samples[0]['t']-.03;previous=initial.copy();previous_t=start_t;stopped=None;stepcount=0;phase_ends={};yaw=0.;last_yaw=0.;unwrapped=0.
    for si,s in enumerate(samples):
        t=float(s['t']);n=max(1,round((t-start_t)/dt)-stepcount);stepcount+=n;a=ticks_to_sim(previous);b=ticks_to_sim(s['actual']);duration=max(dt,t-previous_t)
        for j in range(n):
            d.qpos[:16]=a+(b-a)*(j+1)/n;d.qvel[:16]=(b-a)/duration;d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
            if j%5==0:
                o=obs(d);maxcp=max(maxcp,o['cube_overlap_mm']);maxsp=max(maxsp,o['self_overlap_mm']);maxsup=max(maxsup,o['support_overlap_mm'])
                if maxcp>1 or maxsp>.01 or maxsup>.15 or o['external_cube_contact']:stopped='contact_or_clearance_rejection';break
            mujoco.mj_step(m,d)
        mujoco.mj_forward(m,d);B=d.xmat[cb].reshape(3,3);f=-B[:,0];f-=up*(f@up);f/=np.linalg.norm(f);yaw=float(np.degrees(np.arctan2(up@np.cross(front,f),front@f)));unwrapped+=(yaw-last_yaw+180)%360-180;last_yaw=yaw;tilt=float(np.degrees(np.arccos(np.clip((B@up_local)@up,-1,1))));phase_ends[s['phase']]=dict(yaw_change_deg=unwrapped,green_up_error_deg=tilt,cube_qpos=d.qpos[-7:].tolist(),contacts=obs(d));trace.append(d.qpos.copy());previous=np.asarray(s['actual']);previous_t=t
        if stopped:break
    out=dict(status='legacy_actual_P4_to_P6_free_cube_replay',stopped=stopped,source_samples=len(samples),replayed_samples=si+1,source_duration_seconds=t-start_t,final_yaw_change_deg=unwrapped,final_green_up_error_deg=tilt,max_cube_overlap_mm=maxcp,max_self_overlap_mm=maxsp,max_support_overlap_mm=maxsup,phase_ends=phase_ends,model_pose_conditional=True,whole_cube_90_degrees_verified=False,real_devices_opened=False,hardware_execution_allowed=False);(OUT/'original_suffix_replay.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8');np.savez_compressed(OUT/'original_suffix_replay.npz',qpos=np.asarray(trace));print(json.dumps(out,indent=2),flush=True)
if __name__=='__main__':main()
