"""Separate initialization transients from driven-path model tracking."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from book_grasp_trial_plan import ROOT,BookGraspPlan,LOADED_PHASE
from book_support_model_20261024 import build_book
from observed_book_model_policy import correct_model
from hand_motor_mapping import ticks_to_sim

folder=ROOT/'right_hand_rotation_20261024/book20_current_review/support_1813_diagnostic'
folder.mkdir(exist_ok=True)
source=ROOT/'book_cube_grasp_trial_20261024_181338_435214/fresh_off_request.json'
plan=BookGraspPlan(source);raw=plan.source_position();rows=plan.plan(raw)
q0=ticks_to_sim(raw);seq=[('initial_hold',q0)]*10;inserted=False
for phase,q in rows:
    if phase==LOADED_PHASE and not inserted:
        seq.extend([('placement_hold',seq[-1][1])]*40);inserted=True
    seq.append((phase,ticks_to_sim(q)))
seq.extend([('final_hold',seq[-1][1])]*20)
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
env=registry.load('LeapCubeFaceUp',config=cfg);records=[]
for front in (-15.,-10.,-5.):
    m,_=build_book(env,folder/'scene.xml',book_projection_mm=front,
        book_ylo_mm=-500.,book_yhi_mm=500.,support_projection_mm=50.)
    correct_model(m)
    m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0
    gain,bias=m.actuator_gainprm.copy(),m.actuator_biasprm.copy()
    for scale in (.5,1.,1.5):
        m.actuator_gainprm[:]=gain;m.actuator_biasprm[:]=bias
        m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale
        d=mujoco.MjData(m);d.qpos[:16]=q0;d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0]
        d.ctrl[:]=q0;mujoco.mj_forward(m,d)
        maxima={};elapsed=0.;events=[]
        for phase,q in seq:
            d.ctrl[:]=q
            for _ in range(round(.05/m.opt.timestep)):
                mujoco.mj_step(m,d);elapsed+=m.opt.timestep
                error=float(max(abs(d.qpos[:16]-q)))
                maxima[phase]=max(maxima.get(phase,0.),error)
                if error>.14 and len(events)<5:
                    i=int(np.argmax(abs(d.qpos[:16]-q)))
                    events.append(dict(t=elapsed,phase=phase,sim_index=i,error=error))
        r=dict(front_mm=front,gain_scale=scale,tracking_by_phase=maxima,first_high_tracking=events)
        records.append(r);print(json.dumps(r),flush=True)
(folder/'result.json').write_text(json.dumps(dict(cases=records,hardware_execution_allowed=False,
    hardware_commands_sent=0),indent=2),encoding='utf-8')
