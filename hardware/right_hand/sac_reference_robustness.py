"""Offline native sensitivity audit of reference, not hardware certification."""
import json
from pathlib import Path
import mujoco
import numpy as np
from mujoco_playground import registry
from sac_wood_glass_model import build
from sac_forward_geometry import measure,eligible
from sac_support_separation import SupportSeparation
ROOT=Path(__file__).resolve().parent
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
reference=json.loads((ROOT/'sac_supported_motion_search_20hz_result.json').read_text())
reset=json.loads((ROOT/'sac_supported_reset_result.json').read_text());base=np.asarray(reset['candidates'][0]['ctrl'])
lower=np.asarray(env._safe_lowers).copy();lower[[0,4,8]]=np.maximum(lower[[0,4,8]],.65)
targets=np.clip(base+np.asarray(reference['validation'][0]['delta']).reshape(-1,16),lower,env._safe_uppers)
cases=[('nominal',34,6,1.,1.),('support_31_5',31,5,1.,1.),('support_37_7',37,7,1.,1.),
       ('friction_80pct',34,6,.8,1.),('friction_120pct',34,6,1.2,1.),('mass_80g',34,6,1.,.8),('mass_120g',34,6,1.,1.2)]
rows=[]
for name,projection,glass,friction,mass in cases:
    m,_=build(env,projection_mm=projection,glass_mm=glass);cube=m.geom('cube').id;body=m.geom_bodyid[cube]
    m.geom_friction[:,0]*=friction;m.body_mass[body]*=mass;m.body_inertia[body]*=mass
    support={m.geom(n).id for n in ('support_glass','support_wood')};sep=SupportSeparation(m)
    for offset,candidate in enumerate(reset['candidates']):
        d=mujoco.MjData(m);initial=np.asarray(candidate['qpos']);d.qpos[:]=initial;d.ctrl[:]=candidate['ctrl'];mujoco.mj_forward(m,d)
        hold=longest=0;fault=None;minimum=float('inf');max_pen=0.
        for k in range(len(targets)*120+130):
            if k%5==0:
                phase=min(k//120,len(targets)-1);before=base if phase==0 else targets[phase-1]
                frac=min((k-phase*120+5)/120,1.);d.ctrl[:]=before+(targets[phase]-before)*frac
            mujoco.mj_step(m,d)
            if k%5!=4:continue
            penetration=max([-float(c.dist) for c in d.contact if c.geom1 in support or c.geom2 in support]or[0]);max_pen=max(max_pen,penetration)
            minimum=min(minimum,min(sep.lower_bound(d,g,s) for g in sep.vertices for s in support))
            tip,axis,error=map(float,measure(initial[-4:],d.qpos[-4:],xp=np));shift=float(np.linalg.norm(d.qpos[-7:-4]-initial[-7:-4]))
            if penetration>.0005:fault='support_contact'
            elif not np.isfinite(d.qpos).all() or shift>.08:fault='invalid_or_drop'
            hold=hold+1 if eligible(tip,axis,error,shift,xp=np) and not fault else 0;longest=max(longest,hold)
            if fault:break
        rows.append(dict(case=name,offset=offset,success=longest>=23,fault=fault,final_tip_deg=float(np.rad2deg(tip)),longest_hold_steps=longest,
                         min_housing_clearance_lower_bound_mm=minimum*1000,max_support_penetration_mm=max_pen*1000))
    print(json.dumps(dict(case=name,successes=sum(r['success'] for r in rows if r['case']==name),trials=5)),flush=True)
result=dict(status='offline_native_reference_sensitivity',motor_commands_sent=0,rows=rows,successes=sum(r['success'] for r in rows),trials=len(rows),
            caveat='One-at-a-time assumed perturbations, not measured distributions or MJX/SAC success; root model/calibration uncertainties remain.')
(ROOT/'sac_reference_robustness_result.json').write_text(json.dumps(result,indent=2)+'\n')
