"""Test numerical convergence of the identified palm6/tip corner contact."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json
import numpy as np,mujoco
from mujoco_playground import registry
import screen_thumb_clear_root_empty_20261026 as screen
from thumb_clear_root_empty_plan_20261026 import ThumbRootEmptyPlan,FIXTURE,original_return,motion_goals,RETURN_PHASE

PAIR='palm_collision_6|th_tip'
def main():
    p=ThumbRootEmptyPlan(FIXTURE/'request.json');raw=p.source_position();rows=p.plan(raw)
    i=next(i for i,(phase,q) in enumerate(rows) if phase==RETURN_PHASE)
    anchor=int(np.argmin(np.max(abs(motion_goals()-raw),axis=1)))
    old=original_return(anchor);rows=rows[:i]+[(RETURN_PHASE,q.copy()) for q in old[1:121]]
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
    results=[]
    for step in (.005,.0025,.001):
        m,_=screen.build_cad(env,FIXTURE/f'seam_step_{step}.xml',book_projection_mm=-10)
        screen.correct_model(m,raw);m.opt.timestep=step;cube=m.geom('cube').id;m.geom_contype[cube]=0;m.geom_conaffinity[cube]=0
        m.actuator_gainprm[:,0]*=.5;m.actuator_biasprm[:,1]*=.5
        hand={m.body('palm').id}
        for b in range(m.nbody):
            if int(m.body_parentid[b]) in hand:hand.add(b)
        supports={m.geom(n).id for n in ('support_book','support_glass','support_wood')};inspect=screen.cached_contacts(m,hand,supports)
        d=mujoco.MjData(m);d.qpos[:16]=screen.ticks_to_sim(raw);d.qpos[-7:-4]=[0,0,2];d.qpos[-4:]=[1,0,0,0];d.ctrl[:]=d.qpos[:16]
        for _ in range(round(.5/step)):mujoco.mj_step(m,d)
        peak=0.;at=None;other={}
        for phase,q in rows:
            d.ctrl[:]=screen.ticks_to_sim(q)
            for _ in range(round(.075/step)):
                mujoco.mj_step(m,d)
                if phase!=RETURN_PHASE:continue
                _,pairs=inspect(d);v=pairs.get(PAIR,0)
                if v>peak:peak=v;at=float(d.time)
                for pair,depth in pairs.items():
                    if pair!=PAIR:other[pair]=max(other.get(pair,0),depth)
        row=dict(timestep_s=step,palm6_tip_raw_peak_mm=peak,at_s=at,other_raw_contacts_mm=other)
        results.append(row);print('RESOLUTION',json.dumps(row),flush=True)
    report=dict(pair=PAIR,original_return_without_MCP_modification=True,source=FIXTURE.name,cases=results,hardware_commands_sent=0,contact_forces_preserved=True)
    (FIXTURE/'return_seam_resolution_diagnostic.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
if __name__=='__main__':main()
