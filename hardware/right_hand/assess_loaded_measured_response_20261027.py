"""Offline loaded response stress informed by measured OPEN sag; no devices."""
import os,json,time
import numpy as np
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
from scipy.spatial.transform import Rotation
from mujoco_playground import registry
from horizontal_quarter_plan_20261027 import ROOT,LOADED_PATH
from new_pair_grasp_plan_20261027 import OPEN_PATH
from fingertip_cad_model_20261025 import build
from loaded_quarter_model_policy_20261025 import correct_model
from loaded_quarter_reentry_20261025 import limits
from screen_loaded_repeat_20261026 import cached_contacts
from hand_motor_mapping import ticks_to_sim,SIM_TO_MOTOR_IDS,TICK
from sac_cube_model import set_cube_physics

out=ROOT/'right_hand_rotation_20261027/open_settle_221539/loaded_response_stress';out.mkdir(exist_ok=True)
cfg=registry.get_default_config('LeapCubeFaceUp');cfg.impl='jax';env=registry.load('LeapCubeFaceUp',config=cfg)
z=np.load(LOADED_PATH);opened=z['start'];goals=z['goals'];phases=z['phases']
up=np.array(json.loads((ROOT/'right_hand_rotation_20261026/legacy_stage4_seating_v1/seed.json').read_text())['up_world'])
prior=json.loads((ROOT/'right_hand_rotation_20261027/new_pair_grasp_v2/review_v2.json').read_text());seed=next(v for v in prior['loaded_physics'] if (v['book_front_mm'],v['gain_scale'],v['friction'])==(-10.,1.,.3))
lo,hi=limits();simids=np.array(SIM_TO_MOTOR_IDS)-1;nativelo=(lo[simids]-2048)*TICK;nativehi=(hi[simids]-2048)*TICK
m,_=build(env,out/'scene.xml',book_projection_mm=-10);correct_model(m,opened);set_cube_physics(m,.055,.1);m.opt.timestep=.002;m.opt.gravity[:]=-9.81*up
positive=m.geom_solref[:,0]>0;m.geom_solref[positive,0]=np.minimum(m.geom_solref[positive,0],.004)
cube=m.geom('cube').id;hand={m.body('palm').id}
for b in range(m.nbody):
    if int(m.body_parentid[b]) in hand:hand.add(b)
support={i for i in range(m.ngeom) if (m.geom(i).name or '').startswith(('support_','leap_mount_collision')) or m.geom(i).name=='floor'}
inspect=cached_contacts(m,hand,support);gains=m.actuator_gainprm.copy();bias=m.actuator_biasprm.copy();results=[];began=time.perf_counter()
for name,mf_scale in (('measured_P450_response',.25),('predicted_P600_response',1/3),('existing_half_gain_reference',.5)):
    scale=np.full(m.nu,.5)
    if name!='existing_half_gain_reference':
        scale[m.actuator('if_mcp_act').id]=.65;scale[m.actuator('mf_mcp_act').id]=mf_scale;scale[m.actuator('rf_mcp_act').id]=.35
    m.actuator_gainprm[:]=gains;m.actuator_biasprm[:]=bias;m.actuator_gainprm[:,0]*=scale;m.actuator_biasprm[:,1]*=scale;m.geom_friction[cube,0]=.3
    d=mujoco.MjData(m);d.qpos[:16]=ticks_to_sim(opened);d.ctrl[:]=d.qpos[:16];d.qpos[-7:]=seed['settled_cube_qpos'];d.qvel[:]=0
    for _ in range(1000):mujoco.mj_step(m,d)
    actual=np.empty(16);actual[simids]=d.qpos[:16]/TICK+2048
    maxself=maxsup=maxcp=maxtrack=maxrange=nohand=longest=0.;external=False;stopped=None;steps=0;turn=None;yawbase=None;start=d.qpos[-7:-4].copy()
    for sample,(phase,q) in enumerate(zip(phases,goals),1):
        d.ctrl[:]=ticks_to_sim(q);target=round(sample*.075/.002)
        for _ in range(target-steps):
            mujoco.mj_step(m,d);su,sp=inspect(d);maxself=max(maxself,max(sp.values(),default=0.));maxsup=max(maxsup,max(su.values(),default=0.));maxtrack=max(maxtrack,float(max(abs(d.qpos[:16]-d.ctrl))));maxrange=max(maxrange,float(max(np.maximum(nativelo-d.qpos[:16],d.qpos[:16]-nativehi))));inhand=False
            for c in d.contact:
                a,b=int(c.geom1),int(c.geom2)
                if cube in (a,b):
                    other=b if a==cube else a
                    if int(m.geom_bodyid[other]) in hand:maxcp=max(maxcp,max(0.,-float(c.dist)*1000));inhand|=c.dist<.0004
                    elif other in support and c.dist<.0004:external=True
            nohand=0. if inhand else nohand+.002;longest=max(longest,nohand)
            if maxself>1e-6 or maxsup>.15 or maxcp>1 or maxtrack>.18 or maxrange>1e-6 or external or longest>.25:
                stopped='existing_contact_range_tracking_or_retention_limit';break
        steps=target
        if phase=='grasp_hold':yawbase=d.xmat[m.body('cube').id].reshape(3,3).copy()
        if phase=='yaw_hold' and yawbase is not None:
            R=d.xmat[m.body('cube').id].reshape(3,3);delta=Rotation.from_matrix(R@yawbase.T).as_quat();turn=float(np.degrees(2*np.arctan2(delta[:3]@up,delta[3])))
        if stopped:break
    row=dict(name=name,middle_root_gain_scale=mf_scale,passed=stopped is None,stop=stopped,last_phase=str(phase),last_sample=sample,
        open_tracking_ticks=(opened-actual).tolist(),yaw_before_open_deg=turn,max_self_mm=maxself,max_support_mm=maxsup,max_cube_mm=maxcp,max_tracking_rad=maxtrack,
        native_range_excess_rad=max(0,maxrange),external_cube_contact=external,longest_no_hand_contact_s=longest,center_translation_mm=float(np.linalg.norm(d.qpos[-7:-4]-start)*1000))
    results.append(row);print(json.dumps(row),flush=True)
    (out/'review.json').write_text(json.dumps(dict(status='measured_response_stress_only',cases=results,elapsed_s=time.perf_counter()-began,motor_writes=0,hardware_execution_allowed=False),indent=2)+'\n',encoding='utf-8')
