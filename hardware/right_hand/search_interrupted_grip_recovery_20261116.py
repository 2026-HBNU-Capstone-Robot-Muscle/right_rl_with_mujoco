"""Find a collision-free empty recovery detour for saved interrupted starts."""
import json
import numpy as np
import mujoco
from copy import copy
from mujoco_playground import registry
from final_pose_temporary_support_20261116 import build
from final_pose_start_scope_20261116 import correct_model,validate_path
from final_grip_pose_plan_20261116 import ROOT,OUT,FinalGripPosePlan
from calibrated_empty_ring_margin_20261028 import interpolate
from screen_loaded_repeat_20261026 import cached_contacts
from new_pair_grasp_plan_20261027 import ENTRY

folder=ROOT/'final_grip_pose_20261116_160804_734562'
request=folder/'fresh_off_request.json';p=FinalGripPosePlan(request);raw=p.source_position()
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build(env,OUT/'recovery_search.xml',book_projection_mm=-10);correct_model(m,raw)
mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
inspect=cached_contacts(m,hand,support);d=mujoco.MjData(m)
from hand_motor_mapping import ticks_to_sim
def check(rows):
    d.qpos[:16]=ticks_to_sim(raw);d.ctrl[:]=d.qpos[:16];d.qvel[:]=0;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
    _,initial=inspect(d);pair='if_ds_collision_1|mf_md_collision_5';initial_depth=initial.get(pair,0.)
    worst=0.;first=None;cleared=False
    for i,(phase,q) in enumerate(rows):
        d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];d.qvel[:]=0;d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
        su,sp=inspect(d);sp=dict(sp);actual=sp.get(pair,0.)
        if actual<=1e-6:cleared=True
        elif not cleared and i<64 and actual<=initial_depth+1e-6 and initial_depth<=.5:sp.pop(pair,None)
        depth=max(sp.values(),default=0.)
        if depth>worst:worst=depth
        if first is None and depth>1e-6:first=dict(phase=phase,goal_index=i,pairs=sp,goal=q.tolist())
        if depth>1e-6:return dict(passed=False,first=first,worst_mm=worst)
    return dict(passed=True,worst_mm=worst)
baseline=check(p.plan(raw));print('BASELINE',json.dumps(baseline),flush=True)
results=[]
for motor in (5,7,6,11,10,9):
    for delta in (-32,32,-64,64,-96,96,-128,128):
        waypoint=raw.copy();waypoint[motor-1]+=delta
        proxy=copy(p);proxy.center=waypoint.copy()
        try:
            rows=[(ENTRY,q) for q in interpolate(raw,waypoint)]+proxy.plan(waypoint)
            validate_path(raw,rows,ENTRY)
            result=check(rows);result.update(motor=motor,delta=delta,goals=len(rows))
        except (ValueError,RuntimeError) as exc:result=dict(motor=motor,delta=delta,passed=False,error=str(exc))
        results.append(result)
passing=[r for r in results if r['passed']];passing.sort(key=lambda r:(abs(r['delta']),r['goals']))
report=dict(baseline=baseline,cases=results,passing=passing,devices_opened=False)
out=ROOT/'right_hand_rotation_20261116/final_grip_attempt_review/recovery_search.json'
out.write_text(json.dumps(report,indent=2)+'\n')
print('PASSING',json.dumps(passing),flush=True)
