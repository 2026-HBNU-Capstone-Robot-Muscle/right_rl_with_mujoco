"""Static CAD check for observed OPEN pose plus the proposed ID12 boundary."""
import os,json
import numpy as np
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
from mujoco_playground import registry
from horizontal_quarter_plan_20261027 import ROOT
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from hand_motor_mapping import ticks_to_sim
from book_empty_preparation_plan import FRONTS,offsets
out=ROOT/'right_hand_rotation_20261027/open_settle_221539'
observed=json.loads((out/'measured_review.json').read_text(encoding='utf-8'));a=np.array(observed['last_actual'])
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
extra=[]
for sign in (-1,1):
    v=np.zeros(16,dtype=int);v[11]=sign*5;extra.append(v)
lo,hi=limits();rows=[]
for front in FRONTS:
    m,_=build(env,out/f'bound_scene_{front:g}.xml',book_projection_mm=front);correct_model(m,a)
    mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support)
    for delta in list(offsets())+extra:
        q=a+delta;d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(q);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d);su,sp=inspect(d)
        row=dict(book_front_mm=front,offset_ticks=delta.tolist(),native=bool(np.all(q>=lo) and np.all(q<=hi)),max_self_mm=max(sp.values(),default=0.),max_support_mm=max(su.values(),default=0.))
        row['passed']=bool(row['native'] and row['max_self_mm']<=1e-6 and row['max_support_mm']<=.15);rows.append(row)
report=dict(status='observed_open_boundary_geometry',source='horizontal_quarter_trial_20261027_221539_291945/result.json',
    observed_actual_ticks=a.tolist(),open_goal_ticks=observed['goal'],id12_maximum_error_ticks=72,
    cases=rows,all_passed=all(r['passed'] for r in rows),geometry_and_ranges_changed=False,motor_writes=0)
(out/'placement_bound_geometry.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(cases=len(rows),all_passed=report['all_passed'],max_self_mm=max(r['max_self_mm'] for r in rows),max_support_mm=max(r['max_support_mm'] for r in rows))),flush=True)
