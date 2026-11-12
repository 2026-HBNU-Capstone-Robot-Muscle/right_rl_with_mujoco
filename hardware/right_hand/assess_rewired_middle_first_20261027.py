"""Diagnostic only: exact initial pair, monotonic separation, actual threshold."""
import json,os
import numpy as np
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from screen_loaded_repeat_20261026 import cached_contacts
from hand_motor_mapping import ticks_to_sim
from book_empty_preparation_plan import offsets
from fresh_empty_grasp_plan_20261027 import interpolate
import horizontal_quarter_plan_20261027 as p

PAIR='cadtip_mf_rf_mf_ds_0|cadtip_mf_rf_rf_ds_0'
def main():
    plan=p.HorizontalQuarterPlan(p.REQUEST);raw=plan.source_position();out=p.OUT/'middle_first_diagnostic';out.mkdir(exist_ok=True)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,out/'scene.xml',book_projection_mm=-10);correct_model(m,raw)
    mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);d=mujoco.MjData(m)
    def put(q):
        d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];d.qvel[:]=0;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d);return inspect(d)
    cases=[]
    for offset in offsets():
        a=raw+offset;rows=[(ph,q) for ph,q in plan.plan(a) if ph in p.PREFIX_PHASES];ri=[i for i,(ph,q) in enumerate(rows) if ph==p.ROOTS];first,last=ri[0],ri[-1];q=rows[first-1][1].copy() if first else a.copy();end=rows[last][1];queue=[]
        for axis in (11,7,15):
            target=q.copy();target[axis]=end[axis];queue.extend((p.ROOTS,v) for v in interpolate(q,target,1.5));q=target
        queue=rows[:first]+queue+rows[last+1:];_,initial=put(a);previous=initial.get(PAIR,0.);maximum=previous;increase=0.;other=0.;after=0.;clear=None
        for i,(phase,q) in enumerate(queue):
            _,sp=put(q);depth=sp.get(PAIR,0.);other=max(other,max((v for k,v in sp.items() if k!=PAIR),default=0.))
            if clear is None:
                increase=max(increase,depth-previous);maximum=max(maximum,depth);previous=depth
                if depth<=1e-6:clear=dict(sample=i+1,mf_goal=int(q[11]))
            else:after=max(after,max(sp.values(),default=0.))
        cases.append(dict(offset=offset.tolist(),initial_pair_mm=initial.get(PAIR,0.),maximum_before_clear_mm=maximum,maximum_step_increase_mm=max(0,increase),other_pair_mm=other,maximum_after_clear_mm=after,clear=clear))
    report=dict(cases=cases,max_initial_pair_mm=max(v['initial_pair_mm'] for v in cases),max_step_increase_mm=max(v['maximum_step_increase_mm'] for v in cases),other_pair_mm=max(v['other_pair_mm'] for v in cases),max_after_clear_mm=max(v['maximum_after_clear_mm'] for v in cases),max_clear_sample=max(v['clear']['sample'] for v in cases),maximum_mf_clear_goal=max(v['clear']['mf_goal'] for v in cases),hardware_execution_allowed=False,devices_opened=False)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in report.items() if k!='cases'}),flush=True)
if __name__=='__main__':main()
