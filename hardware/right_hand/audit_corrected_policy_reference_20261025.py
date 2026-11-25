"""Device-free audit of archived policy trajectory with today's ID mapping.

The archived simulation-order trajectory is used directly. No old ID-based
joint cap or old hardware authorization is inherited.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,hashlib,argparse
from pathlib import Path
import numpy as np
import mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from hand_motor_mapping import sim_to_ticks,ticks_to_sim
from sac_cube_model import set_cube_physics
from sac_forward_geometry import measure

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'right_hand_rotation_20261024/corrected_policy_reference'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--projected',action='store_true');args=parser.parse_args()
    OUT.mkdir(exist_ok=True)
    p=ROOT/'sac_reachable_trace_search_result.json';blob=p.read_bytes();source=json.loads(blob)
    row=next(r for r in source['results'] if r['compensation_rad']==.07 and r['time_scale']==1.)
    q=np.asarray(row['goals_sim_order']);reset=json.loads((ROOT/'sac_cube55_resets.json').read_bytes())['candidates'][0]
    parameters=np.arange(len(q),dtype=float);projected=None
    if args.projected:
        projected=json.loads((OUT/'projected_policy_reference_v2.json').read_bytes())
        if not projected['full_static_path_clear']:raise ValueError('Projected path is not statically clear')
        q=np.asarray(projected['projected_goals_sim_rad']);parameters=np.asarray(projected['sparse_source_parameters'])
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=build(env,OUT/'policy_CAD_diagnostic.xml');set_cube_physics(m,.055,.1)
    d=mujoco.MjData(m);cube=m.geom('cube').id;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    static_max=0.;static_rows=[]
    for i,qq in enumerate(q):
        d.qpos[:16]=qq;d.qpos[-7:]=[10,10,10,1,0,0,0];mujoco.mj_forward(m,d)
        for c in d.contact:
            if c.dist>=-1e-7:continue
            a,b=int(c.geom1),int(c.geom2);ha,hb=int(m.geom_bodyid[a]) in hand,int(m.geom_bodyid[b]) in hand
            if ha and hb or a in support and hb or b in support and ha:
                depth=-float(c.dist)*1000;static_max=max(static_max,depth)
                if len(static_rows)<12:static_rows.append(dict(index=i,pair=[m.geom(a).name,m.geom(b).name],model_overlap_mm=depth))
    print('POLICY_REFERENCE_STATIC',static_max,static_rows[:2],flush=True)
    results=[];schedules=[]
    for factor in (1.,1.5,2.,4.,8.):
        times=np.arange(round(parameters[-1]*factor)+1)/factor
        goals=np.array([np.interp(times,parameters,q[:,j]) for j in range(16)]).T
        schedules.append(('reference_time',factor,goals))
    if projected:
        dense=ticks_to_sim(np.asarray(projected['dense_raw_ticks']))
        for stride in (1,2,4,8):
            goals=dense[::stride]
            if not np.array_equal(goals[-1],dense[-1]):goals=np.r_[goals,dense[-1:]]
            schedules.append(('bounded_joint_step',stride,goals))
    for timing_kind,factor,goals in schedules:
        for friction in (.4,.8,1.2):
            mujoco.mj_resetData(m,d);d.qpos[:]=reset['qpos'];d.ctrl[:]=goals[0];m.geom_friction[cube,0]=friction;mujoco.mj_forward(m,d)
            initial=d.qpos[-7:].copy();initial_overlap=max([-float(c.dist)*1000 for c in d.contact if cube in (int(c.geom1),int(c.geom2))]+[0.])
            maxerror=0.;maxself=0.;frames=[];hold=[]
            for i,g in enumerate(np.r_[goals,np.repeat(goals[-1:],60,axis=0)]):
                d.ctrl[:]=g
                for _ in range(round(.05/m.opt.timestep)):
                    mujoco.mj_step(m,d);maxerror=max(maxerror,float(np.max(abs(d.qpos[:16]-g))))
                    for c in d.contact:
                        if c.dist>=0:continue
                        if int(m.geom_bodyid[c.geom1]) in hand and int(m.geom_bodyid[c.geom2]) in hand:maxself=max(maxself,-float(c.dist)*1000)
                if i%2==0 and friction==.8:frames.append(d.qpos.tolist())
                if i>=len(goals):
                    tip,axis,err=map(float,measure(initial[-4:],d.qpos[-4:],xp=np))
                    contacts=[];external=False
                    for c in d.contact:
                        if cube in (int(c.geom1),int(c.geom2)) and c.dist<.0003:
                            other=int(c.geom2) if int(c.geom1)==cube else int(c.geom1)
                            if int(m.geom_bodyid[other]) in hand:contacts.append(m.body(int(m.geom_bodyid[other])).name)
                            if other in support or m.geom(other).name=='floor':external=True
                    shift=float(np.linalg.norm(d.qpos[-7:-4]-initial[:3])*1000)
                    hold.append(dict(tip_deg=float(np.degrees(tip)),target_error_deg=float(np.degrees(err)),
                        center_shift_mm=shift,contacts=sorted(set(contacts)),external_cube_contact=external,
                        retained=bool(contacts and shift<100 and not external)))
            item=dict(timing_kind=timing_kind,time_scale_or_dense_stride=factor,friction=friction,nominal_seconds=len(goals)*.05,
                maximum_control_step_ticks=int(np.max(abs(np.diff(sim_to_ticks(goals),axis=0)))),
                initial_cube_overlap_mm=initial_overlap,maximum_tracking_rad=maxerror,maximum_self_model_overlap_mm=maxself,
                final=hold[-1],hold_samples=hold,frames=frames,
                simulated_target_held=all(h['retained'] and h['target_error_deg']<15 for h in hold))
            results.append(item)
            print('CORRECTED_POLICY',timing_kind,factor,friction,'error',round(hold[-1]['target_error_deg'],2),
                'retained',hold[-1]['retained'],'tracking',round(maxerror,3),'self',round(maxself,3),flush=True)
    out=dict(source=p.name,source_sha256=hashlib.sha256(blob).hexdigest(),
        legacy_ID_based_caps_not_applied=True,corrected_start_raw_ticks=sim_to_ticks(q[0]).tolist(),
        trajectory_sim_order_unchanged=True,native_range_maximum_violation_rad=float(np.maximum(m.jnt_range[:16,0]-q,q-m.jnt_range[:16,1]).max()),
        static_path_maximum_model_overlap_mm=static_max,static_examples=static_rows,
        initial_reset_source='sac_cube55_resets.json',model=manifest,runs=results,
        simulated_motor_parameters_not_identified=True,hardware_execution_allowed=False,
        hardware_commands_sent=0,physical_ninety_degree_rotation_verified=False)
    out['collision_projection_applied']=bool(projected)
    if projected:out['projection_sha256']=hashlib.sha256((OUT/'projected_policy_reference_v2.json').read_bytes()).hexdigest()
    (OUT/('projected_policy_reference_evaluation.json' if projected else 'corrected_policy_reference_evaluation.json')).write_text(json.dumps(out,indent=2),encoding='utf-8')

if __name__=='__main__':main()
