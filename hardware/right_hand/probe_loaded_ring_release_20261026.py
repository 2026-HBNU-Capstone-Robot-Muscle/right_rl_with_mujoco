"""Inspect a short offline ring-release response; no hardware."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import numpy as np,mujoco
from mujoco_playground import registry
import screen_loaded_thumb_rotation_20261026 as screen
from loaded_thumb_rotation_plan_20261026 import FIXTURE,LoadedThumbRotationPlan,TURN_PHASE,RETURN_PHASE,motion_goals
from loaded_thumb_return_clearance_20261026 import goals as return_goals
from hand_motor_mapping import ticks_to_sim
from confirmed_thumb_rest_policy_20261026 import classify_initial_palm
from book_grasp_trial_plan import RING_RELEASE
def main():
    p=LoadedThumbRotationPlan(FIXTURE/'request.json');raw=p.source_position();proof=json.loads((FIXTURE/'screen.json').read_bytes())
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    m,_=screen.build_cad(env,FIXTURE/'ring_probe_scene.xml',book_projection_mm=-10.);screen.correct_model(m,raw);m.opt.timestep=.005
    m.geom_contype[m.geom('cube').id]=0;m.geom_conaffinity[m.geom('cube').id]=0
    m.actuator_gainprm[:,0]*=.5;m.actuator_biasprm[:,1]*=.5
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    inspect_raw=screen.cached_contacts(m,hand,{m.geom(n).id for n in ('support_book','support_glass','support_wood')})
    bound=next(r['initial_self_contacts_mm'] for r in proof['dynamic_sensitivity'] if r['book_front_mm']==-10 and r['gain_scale']==.5)
    all_rows=p.plan(raw);end=next(i for i,(phase,q) in enumerate(all_rows) if phase==TURN_PHASE)
    rows=all_rows[:end]
    results=[]
    for extra in (0,4,8):
        anchor=int(np.argmin(np.max(abs(motion_goals()-raw),axis=1)))
        variant=[];inserted=False
        for phase,q in rows:
            if phase==RETURN_PHASE:
                if not inserted:variant.extend((phase,v.copy()) for v in return_goals(anchor,extra)[1:]);inserted=True
            else:variant.append((phase,q.copy()))
        d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(raw);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];d.ctrl[:]=d.qpos[:16];mujoco.mj_forward(m,d)
        trajectory=[('initial_hold',raw,.5)]+[(phase,q.copy(),.075) for phase,q in variant]
        events=[];elapsed=0.
        for index,(phase,q,duration) in enumerate(trajectory):
            q=q.copy()
            d.ctrl[:]=ticks_to_sim(q)
            for _ in range(round(duration/.005)):
                mujoco.mj_step(m,d);elapsed+=.005
                _,pairs=inspect_raw(d)
                pairs=screen.classify_self_contacts(pairs);pairs=screen.classify_photo_initial(pairs,initial_release=phase in ('initial_hold',RING_RELEASE));pairs=screen.classify_slow_seam(pairs)
                pairs=screen.classify_return_corner(pairs,phase=phase)
                pairs=classify_initial_palm(pairs,enabled=True,initial_release=phase==RING_RELEASE,phase=phase)
                bad={pair:dict(depth_mm=depth,initial_mm=bound.get(pair,0.)) for pair,depth in pairs.items() if depth>bound.get(pair,0.)+1e-6 or (phase not in ('initial_hold',RING_RELEASE) and depth>1e-6)}
                if bad and len(events)<6:events.append(dict(time=elapsed,phase=phase,bad=bad))
        results.append(dict(extra_return_thumb_tip_clearance_ticks=extra,bad_events=events))
        print(json.dumps(results[-1]),flush=True)
    (FIXTURE/'thumb_return_probe.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
if __name__=='__main__':main()
