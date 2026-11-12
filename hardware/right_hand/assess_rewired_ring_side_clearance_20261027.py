"""Offline bounded separation candidates; no thresholds or runtime changed."""
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
from loaded_quarter_reentry_20261025 import validate_path
import horizontal_quarter_plan_20261027 as p

PAIR='cadtip_mf_rf_mf_ds_0|cadtip_mf_rf_rf_ds_0';RING='diagnostic_ring_side_clearance'
def main():
    plan=p.HorizontalQuarterPlan(p.REQUEST);raw=plan.source_position();out=p.OUT/'ring_side_diagnostic';out.mkdir(exist_ok=True)
    cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg);m,_=build(env,out/'scene.xml',book_projection_mm=-10);correct_model(m,raw)
    mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support);d=mujoco.MjData(m)
    def put(q):
        d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];d.qvel[:]=0;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d);return inspect(d)
    results=[]
    for delta in (96,-96,160):
        cases=[]
        for offset in offsets():
            a=raw+offset;original=[(ph,q) for ph,q in plan.plan(a) if ph in p.PREFIX_PHASES];roots=[q for ph,q in original if ph==p.ROOTS];entry=[(ph,q) for ph,q in original if ph==p.ENTRY];q=entry[-1][1].copy() if entry else a.copy();side=q.copy();side[13]+=delta
            queue=entry+[(RING,v) for v in interpolate(q,side)];q=side
            for axis in (7,11,15):
                end=q.copy();end[axis]=roots[-1][axis];queue.extend((p.ROOTS,v) for v in interpolate(q,end,1.5));q=end
            opened=np.load(p.OPEN_PATH)['open_goal'];queue.extend((p.OPEN,v) for v in interpolate(q,opened));validate_path(a,queue,p.ENTRY)
            _,initial=put(a);previous=initial.get(PAIR,0.);increase=other=after=support_max=0.;clear=None
            for i,(phase,q) in enumerate(queue):
                su,sp=put(q);support_max=max(support_max,max(su.values(),default=0.));other=max(other,max((v for k,v in sp.items() if k!=PAIR),default=0.));depth=sp.get(PAIR,0.)
                if phase in (RING,p.ENTRY):
                    increase=max(increase,depth-previous);previous=depth
                    if phase==RING and clear is None and depth<=1e-6:clear=dict(sample=i+1,ring_goal=int(q[13]))
                else:after=max(after,max(sp.values(),default=0.))
            cases.append(dict(initial_pair_mm=initial.get(PAIR,0.),max_increase_mm=max(0,increase),other_self_mm=other,after_clearance_self_mm=after,support_mm=support_max,clear=clear))
        row=dict(delta_ticks=delta,cases=cases,max_initial_mm=max(v['initial_pair_mm'] for v in cases),max_increase_mm=max(v['max_increase_mm'] for v in cases),other_self_mm=max(v['other_self_mm'] for v in cases),after_clearance_self_mm=max(v['after_clearance_self_mm'] for v in cases),max_support_mm=max(v['support_mm'] for v in cases),all_cleared=all(v['clear'] is not None for v in cases))
        results.append(row);print(json.dumps({k:v for k,v in row.items() if k!='cases'}),flush=True)
    (out/'review.json').write_text(json.dumps(dict(status='offline_candidates_only',results=results,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
