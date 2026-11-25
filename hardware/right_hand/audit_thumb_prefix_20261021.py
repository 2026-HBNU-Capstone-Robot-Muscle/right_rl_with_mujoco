"""Exact bounded preparation screen; isolated model sensitivity, no hardware."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from offline_housing_proxy_model import build_with_middle_thumb_hulls
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS
import thumb_prefix_plan as p

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_preparation_20261021'

def main(plan_module=p, output_path=None):
    p=plan_module
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax'
    env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build_with_middle_thumb_hulls(env);d=mujoco.MjData(m)
    palm=m.body('palm').id;hand={palm}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_glass','support_wood')}
    cube=m.geom('cube').id;m.geom_contype[cube]=0;m.geom_conaffinity[cube]=0
    def overlaps():
        sd=wd=0.
        for c in d.contact:
            a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb:sd=max(sd,-float(c.dist)*1000)
            if (a in support and hb) or (b in support and ha):wd=max(wd,-float(c.dist)*1000)
        return sd,wd
    def setpose(raw):
        d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];mujoco.mj_forward(m,d)
    raw=p.source_position();rng=np.random.default_rng(20261021)
    offsets=[np.zeros(16,dtype=int),np.full(16,-32),np.full(16,32)]
    for i in range(16):
        for sign in (-1,1):
            a=np.zeros(16,dtype=int);a[i]=32*sign;offsets.append(a)
    offsets += [rng.integers(-32,33,16) for _ in range(32)]
    screens=[]
    for case,offset in enumerate(offsets):
        a=raw+offset;rows=p.plan(a);setpose(a);_,initial_support=overlaps()
        maximum_self=0.;support_increase=0.;thumb_support=0.;maxstep=0;last=a.copy();previous_support=initial_support
        for phase,q in rows:
            maxstep=max(maxstep,int(max(abs(q-last))));last=q.copy();setpose(q);s,w=overlaps()
            maximum_self=max(maximum_self,s);support_increase=max(support_increase,w-previous_support)
            previous_support=w
            if phase=='stage_1_thumb_prefix':thumb_support=max(thumb_support,w)
        screens.append(dict(case=case,start_offset_ticks=offset.tolist(),max_self_overlap_mm=maximum_self,
            initial_support_overlap_mm=initial_support,max_support_increase_mm=support_increase,
            thumb_support_overlap_mm=thumb_support,max_goal_step_ticks=maxstep))
    print('Static',json.dumps(dict(cases=len(screens),max_self=max(x['max_self_overlap_mm'] for x in screens),
        max_support_increase=max(x['max_support_increase_mm'] for x in screens),thumb_support=max(x['thumb_support_overlap_mm'] for x in screens))),flush=True)
    # Native constraints contradict the observed ID3 starting configuration.
    # Compare the unchanged model with an isolated diagnostic range extension.
    # This extension is NEVER written to any deployment model or hardware file.
    jids=m.actuator_trnid[:,0];ax=SIM_TO_MOTOR_IDS.index(3);joint=int(jids[ax])
    native_joint=m.jnt_range.copy();native_ctrl=m.actuator_ctrlrange.copy()
    native_gain=m.actuator_gainprm.copy();native_bias=m.actuator_biasprm.copy()
    trials=[];initial_q=ticks_to_sim(raw)
    path=[('initial_hold',raw)]*10+p.plan(raw)+[('final_hold',p.plan(raw)[-1][1])]*20
    for variant in ('native_unchanged','observed_range_DIAGNOSTIC_ONLY'):
        for gain in ((1.,) if variant=='native_unchanged' else (.5,1.,1.5)):
            m.jnt_range[:]=native_joint;m.actuator_ctrlrange[:]=native_ctrl
            m.actuator_gainprm[:]=native_gain;m.actuator_biasprm[:]=native_bias
            if variant!='native_unchanged':
                bound=initial_q[ax]+(32+4)*2*np.pi/4096
                m.jnt_range[joint,1]=max(bound,m.jnt_range[joint,1])
                m.actuator_ctrlrange[ax,1]=max(bound,m.actuator_ctrlrange[ax,1])
            m.actuator_gainprm[:,0]*=gain;m.actuator_biasprm[:,1]*=gain
            d=mujoco.MjData(m);setpose(raw);d.ctrl[:]=initial_q
            maxerr=ms=mw=0.;init_movement=0.;phase_stats={}
            for phase,q in path:
                goal=ticks_to_sim(q);d.ctrl[:]=goal
                for _ in range(round(.05/m.opt.timestep)):
                    mujoco.mj_step(m,d);s,w=overlaps();ms=max(ms,s);mw=max(mw,w)
                    maxerr=max(maxerr,float(max(abs(d.qpos[:16]-goal))))
                    if phase=='initial_hold':init_movement=max(init_movement,float(max(abs(d.qpos[:16]-initial_q))))
                phase_stats[phase]=dict(max_error_rad=float(max(abs(d.qpos[:16]-goal))),qpos=d.qpos[:16].tolist())
            row=dict(variant=variant,gain_scale=gain,max_tracking_rad=maxerr,
                max_self_overlap_mm=ms,max_support_overlap_mm=mw,initial_hold_movement_rad=init_movement,
                phase_end=phase_stats,finite=bool(np.isfinite(d.qpos).all()))
            trials.append(row);print(json.dumps({k:v for k,v in row.items() if k!='phase_end'}),flush=True)
    m.jnt_range[:]=native_joint;m.actuator_ctrlrange[:]=native_ctrl
    m.actuator_gainprm[:]=native_gain;m.actuator_biasprm[:]=native_bias
    report=dict(source_sha256=p.SOURCE_SHA,source_path=str(p.SOURCE),model_manifest=manifest,
        start_ticks=raw.tolist(),last_goal_ticks=p.plan(raw)[-1][1].tolist(),
        static_screens=screens,dynamic_sensitivity=trials,
        calibration_verified=False,full_preparation_verified=False,hardware_execution_allowed=False,
        hardware_commands_sent=0,limitations=['Static start-neighborhood sampling is not an exhaustive clearance guarantee.',
        'Observed-range variant is an isolated diagnostic model; limits in production files are unchanged.',
        'Ideal actuator gains and contacts are not calibrated to real motor PID/current/friction.',
        'Candidate is a small EMPTY observation, not a cube action or SAC output.'])
    if hasattr(p,'PLAN_VERSION'):
        report.update(plan_version=p.PLAN_VERSION,plan_sha256=p.plan_digest(raw))
    destination=Path(output_path) if output_path else OUT/'thumb_prefix_screen.json'
    destination.parent.mkdir(exist_ok=True)
    destination.write_text(json.dumps(report,indent=2),encoding='utf-8')

if __name__=='__main__':main()
