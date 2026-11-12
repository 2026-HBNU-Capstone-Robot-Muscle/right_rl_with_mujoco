"""Offline contact timing and six EMPTY root-order candidates, no devices."""
import itertools,json,os
from pathlib import Path
import numpy as np
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from screen_loaded_repeat_20261026 import cached_contacts
from hand_motor_mapping import ticks_to_sim
from fresh_empty_grasp_plan_20261027 import interpolate
import horizontal_quarter_plan_20261027 as p

def main():
    plan=p.HorizontalQuarterPlan(p.REQUEST);raw=plan.source_position();rows=[(ph,q) for ph,q in plan.plan(raw) if ph in p.PREFIX_PHASES]
    out=p.OUT/'root_order_diagnostic';out.mkdir(exist_ok=True)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,out/'scene.xml',book_projection_mm=-10);correct_model(m,raw)
    cmask=m.geom_bodyid==m.body('cube').id;m.geom_contype[cmask]=0;m.geom_conaffinity[cmask]=0
    hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);d=mujoco.MjData(m)
    def put(q):
        d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];d.qvel[:]=0;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d);return inspect(d)
    initial=put(raw);root_indices=[i for i,(ph,q) in enumerate(rows) if ph==p.ROOTS];first,last=root_indices[0],root_indices[-1];start=rows[first-1][1] if first else raw;target=rows[last][1]
    results=[]
    for order in itertools.permutations((7,11,15)):
        q=start.copy();queue=[]
        for axis in order:
            end=q.copy();end[axis]=target[axis];queue.extend((p.ROOTS,v) for v in interpolate(q,end,1.5));q=end
        candidate=rows[:first]+queue+rows[last+1:];worst=0.;first_contact=None;worst_at=None
        for i,(phase,q) in enumerate([('initial',raw)]+candidate):
            _,sp=put(q);depth=max(sp.values(),default=0.)
            detail=dict(index=i,phase=phase,depth_mm=depth,pairs=sp,roots=q[[7,11,15]].tolist(),ring_rotation=int(q[13]))
            if depth>1e-6 and first_contact is None:first_contact=detail
            if depth>worst:worst=depth;worst_at=detail
        row=dict(order_motor_ids=[i+1 for i in order],maximum_self_mm=worst,first_contact=first_contact,worst_at=worst_at);results.append(row);print(json.dumps(row),flush=True)
    report=dict(status='offline_order_diagnostic',initial_support_mm=initial[0],initial_self_mm=initial[1],cases=results,devices_opened=False,hardware_execution_allowed=False)
    (out/'review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
