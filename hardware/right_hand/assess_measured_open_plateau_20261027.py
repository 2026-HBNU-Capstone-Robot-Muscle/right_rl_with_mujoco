"""Check observed OPEN geometry and compare model holding errors; no devices."""
import json,os
import numpy as np
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
from mujoco_playground import registry
from horizontal_quarter_plan_20261027 import ROOT,LOADED_REVIEW,OPEN_PATH
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from book_empty_preparation_plan import FRONTS,offsets
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
out=ROOT/'right_hand_rotation_20261027/open_settle_221539';measured=json.loads((out/'measured_review.json').read_text(encoding='utf-8'))
actual=np.array(measured['last_actual']);goal=np.array(measured['goal']);simids=np.array(SIM_TO_MOTOR_IDS)-1
up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_text())['up_world'])
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
lo,hi=limits();geometry=[];dynamic=[]
for front in FRONTS:
    m,_=build(env,out/f'actual_scene_{front:g}.xml',book_projection_mm=front);correct_model(m,actual)
    mask=m.geom_bodyid==m.body('cube').id;m.geom_contype[mask]=0;m.geom_conaffinity[mask]=0;hand={m.body('palm').id}
    for b in range(m.nbody):
        if int(m.body_parentid[b]) in hand:hand.add(b)
    support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
    inspect=cached_contacts(m,hand,support)
    def put(d,q):
        d.qpos[:16]=ticks_to_sim(q);d.qvel[:]=0;d.ctrl[:]=ticks_to_sim(goal);d.qpos[-7:]=[0,0,2,1,0,0,0];mujoco.mj_forward(m,d)
    for offset in offsets():
        q=actual+offset;d=mujoco.MjData(m);put(d,q);su,sp=inspect(d)
        row=dict(book_front_mm=front,offset_ticks=offset.tolist(),max_self_mm=max(sp.values(),default=0.),max_support_mm=max(su.values(),default=0.),native=bool(np.all(q>=lo) and np.all(q<=hi)),self_pairs=sp,support_pairs=su)
        row['passed']=bool(row['native'] and row['max_self_mm']<=1e-6 and row['max_support_mm']<=.15);geometry.append(row)
    print('MEASURED GEOMETRY',front,sum(v['passed'] for v in geometry if v['book_front_mm']==front),'/67',flush=True)
    if front!=-10.:continue
    gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();act=m.actuator('mf_mcp_act').id;m.opt.timestep=.002
    for gravity_name,gravity in (('default',np.array([0.,0.,-9.81])),('loaded_up_world',-9.81*up)):
      for scale in (1.,.75,.5,.35):
        m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[act,0]*=scale;m.actuator_biasprm[act,1]*=scale;m.opt.gravity[:]=gravity
        d=mujoco.MjData(m);put(d,actual)
        for _ in range(1500):mujoco.mj_step(m,d)
        q=np.empty(16);q[simids]=d.qpos[:16]/TICK+2048;error=goal-q
        row=dict(gravity=gravity_name,MF_root_gain_scale=scale,error_ticks=error.tolist(),id12_error_ticks=float(error[11]),RMSE_from_measured_ticks=float(np.sqrt(np.mean((q-actual)**2))))
        dynamic.append(row);print(json.dumps(row),flush=True)
loaded=json.loads(LOADED_REVIEW.read_text(encoding='utf-8'))
nominal=next(v for v in loaded['cases'] if (v['book_front_mm'],v['gain_scale'],v['friction'])==(-10.,1.,.3))
report=dict(status='measured_open_model_diagnostic',geometry_cases=geometry,all_geometry_passed=all(v['passed'] for v in geometry),holding_comparisons=dynamic,
    original_loaded_maximum_tracking_rad=max(v['max_tracking_rad'] for v in loaded['cases']),nominal_loaded_case=nominal,motor_writes=0)
(out/'model_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(all_geometry_passed=report['all_geometry_passed'],loaded_max_tracking_rad=report['original_loaded_maximum_tracking_rad'],nominal_loaded_max_tracking_rad=nominal['max_tracking_rad'])),flush=True)
