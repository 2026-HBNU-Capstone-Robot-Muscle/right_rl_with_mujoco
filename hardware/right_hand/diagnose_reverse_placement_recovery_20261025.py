"""Offline witnesses and timing variants, never accepted as a hardware proof."""
import json
from pathlib import Path
import numpy as np
import screen_reverse_place_capture_20261025 as s
from reverse_place_capture_plan_20261025 import ReversePlacePlan,ROOT,BASE_RELEASE,PHOTO_HOLD,TARGET

OUT=ROOT/'right_hand_rotation_20261024/reverse_empty_entry/recovery_timing_diagnostics'

def main():
    OUT.mkdir(exist_ok=True)
    plan=ReversePlacePlan(ROOT/'right_hand_rotation_20261024/reverse_empty_entry/placement_fixture_v3/request.json')
    raw=plan.source_position();original=plan.plan(raw)
    cfg=s.registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=s.registry.load('LeapCubeFaceUp',config=cfg)
    m,manifest=s.build_cad(env,OUT/'scene.xml',book_projection_mm=-5.)
    s.correct_model(m,raw);m.geom_contype[m.geom('cube').id]=m.geom_conaffinity[m.geom('cube').id]=0
    m.actuator_gainprm[:,0]*=.5;m.actuator_biasprm[:,1]*=.5
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')}
    inspect=s.cached_contacts(m,hand,supports);variants={'linear':original}
    rows=[];before_lift=next(q for p,q in reversed(original) if p==BASE_RELEASE)
    for p,q in original:
        if p!=s.LIFT:rows.append((p,q.copy()));continue
        k=sum(pp==s.LIFT for pp,qq in rows)+1
        last=next(qq for pp,qq in reversed(original) if pp==s.LIFT)
        rows.append((p,before_lift+np.clip(last-before_lift,-2*k,2*k)))
    variants['independent_root_steps']=rows
    rows=[]
    for p,q in original:
        q=q.copy()
        if p==BASE_RELEASE:
            k=sum(pp==BASE_RELEASE for pp,qq in rows)+1
            delta=TARGET[[7,11,15]]-raw[[7,11,15]]
            q[[7,11,15]]=raw[[7,11,15]]+np.clip(delta,-2*k,2*k)
        rows.append((p,q))
    # Rebuild the remaining root stage from the already advanced root position.
    end_base=next(q for p,q in reversed(rows) if p==BASE_RELEASE)
    for i,(p,q) in enumerate(rows):
        if p==s.LIFT:
            k=sum(pp==s.LIFT for pp,qq in rows[:i])+1
            rows[i]=(p,end_base+np.clip(TARGET-end_base,-2*k,2*k)*np.isin(np.arange(16),[7,11,15]))
    variants['roots_during_base_release']=rows
    results={}
    for name,rows in variants.items():
        d=s.mujoco.MjData(m);d.qpos[:16]=s.ticks_to_sim(raw);d.qpos[-7:]=[0,0,2,1,0,0,0]
        d.ctrl[:]=d.qpos[:16];s.mujoco.mj_forward(m,d)
        initial_support,initial_self=inspect(d);new=[];deep=[];after_max=0.;contact_details=[]
        dynamics=[('initial_hold',s.ticks_to_sim(raw))]*10
        for p,q in rows:
            if p==s.END_HOLD:dynamics.extend([('placement_hold',s.ticks_to_sim(q))]*40)
            dynamics.append((p,s.ticks_to_sim(q)))
        for p,q in dynamics:
            d.ctrl[:]=q
            for _ in range(round(.05/m.opt.timestep)):
                s.mujoco.mj_step(m,d);support,self_pairs=inspect(d)
                for pair,depth in support.items():
                    event=dict(time=float(d.time),phase=p,pair=pair,depth_mm=depth,model_joint_angles_rad=d.qpos[:16].tolist())
                    if pair not in initial_support:new.append(event)
                    elif depth>initial_support[pair]+1e-6:deep.append(event)
                if p not in ('initial_hold',BASE_RELEASE,s.LIFT):after_max=max(after_max,max(support.values(),default=0.))
        result=dict(new_contact_max_mm=max((a['depth_mm'] for a in new),default=0.),
            deeper_initial_max_mm=max((a['depth_mm'] for a in deep),default=0.),
            first_new_contact=new[:1],first_deeper_initial=deep[:1],max_support_after_lift_mm=after_max)
        results[name]=result;print(name,json.dumps(result),flush=True)
    (OUT/'comparison.json').write_text(json.dumps(results,indent=2),encoding='utf-8')

if __name__=='__main__':main()
