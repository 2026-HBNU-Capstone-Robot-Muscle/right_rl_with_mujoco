"""Small OFFLINE diagnosis of the exact failed fresh-start release."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json
from pathlib import Path
import numpy as np,mujoco
from mujoco_playground import registry
from loaded_quarter_trial_plan_20261025 import ROOT,LoadedQuarterTrialPlan
from loaded_quarter_model_policy_20261025 import correct_model
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import ticks_to_sim,sim_to_ticks
from screen_loaded_quarter_trial_20261025 import cached_contacts
from observed_book_model_policy import classify_self_contacts
from book_empty_preparation_plan import offsets
from book_grasp_trial_plan import RING_RELEASE
OUT=ROOT/'right_hand_rotation_20261024/rigid_contact_review_20261025/off_pose_214901'
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--thumb-counter',type=int,default=0);args=parser.parse_args()
    if args.thumb_counter:
        import loaded_quarter_trial_plan_20261025 as planmodule
        original=planmodule.release_target
        def release(a,join):
            q=original(a,join)
            if a[1]>3100:q[1]=int(a[1])-args.thumb_counter
            return q
        planmodule.release_target=release
    p=LoadedQuarterTrialPlan(OUT/'request.json');raw=p.source_position();rows=p.plan(raw)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=build(env,OUT/'initial_contact_diagnostic_scene.xml',book_projection_mm=-10);correct_model(m,raw)
    m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')};inspect=cached_contacts(m,hand,support)
    def init(q):
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d);return d
    envelope={}
    for off in offsets():
        for pair,depth in classify_self_contacts(inspect(init(raw+off))[1]).items():envelope[pair]=max(envelope.get(pair,0),depth)
    m.actuator_gainprm[:,0]*=.5;m.actuator_biasprm[:,1]*=.5
    d=init(raw);time=0;events=[];first_non_ring=next(i for i,(phase,q) in enumerate(rows) if phase!=RING_RELEASE)
    commands=[('initial_hold',raw)]*10+rows[:first_non_ring+10]
    maxima={}
    for phase,goal in commands:
        d.ctrl[:]=ticks_to_sim(goal)
        for _ in range(round(.05/m.opt.timestep)):
            mujoco.mj_step(m,d);time+=m.opt.timestep;pairs=classify_self_contacts(inspect(d)[1])
            for pair,depth in pairs.items():
                limit=envelope.get(pair,0.)
                if depth>limit+1e-6 and depth>maxima.get(pair,0.):
                    maxima[pair]=depth;events.append(dict(phase=phase,t=time,pair=pair,depth_mm=depth,initial_envelope_mm=limit,goal=goal.tolist(),actual_ticks=sim_to_ticks(d.qpos[:16]).tolist()))
    result=dict(initial_envelope_mm=envelope,events=events,hardware_commands_sent=0,diagnostic_only=True)
    name=f'initial_contact_counter{args.thumb_counter}.json' if args.thumb_counter else 'initial_contact_diagnostic.json'
    (OUT/name).write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result))
if __name__=='__main__':main()
