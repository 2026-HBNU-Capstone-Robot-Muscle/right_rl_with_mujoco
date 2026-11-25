"""Find a small EMPTY-only joint offset, keeping the model pose as reference."""
import json
import numpy as np
import mujoco
from final_grip_pose_plan_20261116 import OUT,REQUEST,FinalGripPosePlan,candidate
from hand_motor_mapping import ticks_to_sim
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from mujoco_playground import registry
from fingertip_cad_model_20261025 import build
p=FinalGripPosePlan(REQUEST);raw=p.source_position();base=candidate()['goals'][-1];lo,hi=limits()
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
m,_=build(env,OUT/'search_scene.xml',book_projection_mm=-10);correct_model(m,raw);m.opt.timestep=.005
mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0
m.actuator_gainprm[:,0]*=.5;m.actuator_biasprm[:,1]*=.5
hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
inspect=cached_contacts(m,hand,support)
bad=json.loads((OUT/'contact_diagnostic.json').read_bytes())['worst'];pair=next(iter(bad));g1,g2=[m.geom(n).id for n in pair.split('|')]
actual=np.asarray(bad[pair]['actual_ticks']);results=[]
for magnitude in (1,2,4):
    for motor in (1,2,3,4,9,10,11,12):
        for sign in (-1,1):
            q=base.copy();q[motor-1]+=sign*magnitude
            if np.any(q<lo) or np.any(q>hi):continue
            d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(actual);d.ctrl[:]=ticks_to_sim(q);d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
            for _ in range(600):mujoco.mj_step(m,d)
            su,sp=inspect(d);gap=mujoco.mj_geomDistance(m,d,g1,g2,1.,None)*1000
            results.append(dict(motor=motor,delta=sign*magnitude,gap_mm=gap,self_mm=max(sp.values(),default=0.),support_mm=max(su.values(),default=0.)))
    passing=[r for r in results if r['self_mm']<=1e-6 and r['support_mm']<=.15 and r['gap_mm']>=.1]
    if passing:break
passing.sort(key=lambda r:(abs(r['delta']),-r['gap_mm']))
(OUT/'clearance_search.json').write_text(json.dumps(dict(cases=results,selected=passing[0] if passing else None),indent=2)+'\n')
print(json.dumps(dict(selected=passing[0] if passing else None,best=sorted(results,key=lambda r:-r['gap_mm'])[:3])))
