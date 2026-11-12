import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from quarter_finish_empty_plan_20261025 import QuarterFinishEmptyPlan,FIXTURE
from loaded_quarter_trial_plan_20261025 import release_target,motion_goals
from hand_motor_mapping import ticks_to_sim
def main():
    plan=QuarterFinishEmptyPlan(FIXTURE/'request.json');raw=plan.source_position();q=motion_goals();anchor=int(np.argmin(np.max(abs(q-raw),axis=1)));join=q[anchor];release=release_target(raw,join)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,FIXTURE/'thumb_separation_scene.xml');m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0;d=mujoco.MjData(m)
    palm=m.geom('palm_collision_8').id;tip=m.geom('th_ds_collision_1').id;rows=[]
    for extra in (-96,-64,-32,0,32,64,96):
        target=release.copy();target[1]+=extra;maximum=0;depths=[]
        for k in range(33):
            pose=np.rint(raw+(target-raw)*k/32).astype(int);d.qpos[:16]=ticks_to_sim(pose);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
            dist=float(mujoco.mj_geomDistance(m,d,palm,tip,.1,None))*1000;depths.append(max(0,-dist));maximum=max(maximum,max(0,-dist))
        row=dict(extra_mcp2_ticks=extra,initial_mm=depths[0],max_mm=maximum,end_mm=depths[-1],largest_increase_mm=max(np.diff(depths)));rows.append(row)
    report=dict(rows=rows,raw=raw.tolist(),release=release.tolist());(FIXTURE/'thumb_separation_sweep.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report,indent=2),flush=True)
if __name__=='__main__':main()
